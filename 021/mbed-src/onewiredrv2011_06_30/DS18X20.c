/**
* @file DS18x20.c
* @brief library of DS18X20 1-Wire digital thermometer (http://www.maxim-ic.com/datasheet/index.mvp/id/2812)
* @author Maciej Rajtar (Published 10 May 2010 www.mbed.org)
* @author Frederic BLANC
*/
#include "mbed.h"
#include "onewire.h"
#include "DS18X20.h"
#include "crc8.h"
#include "utils.h"
/**
*     @brief get power status of DS18x20
*    @param  [in] uint8_t id[] = rom_code
*    @return DS18X20_POWER_EXTERN or DS18X20_POWER_PARASITE
*     @date 20/06/2011
*/
uint8_t DS18X20_get_power_status(uint8_t id[]) {
    uint8_t pstat;
    ow_reset();
    ow_command(DS18X20_READ_POWER_SUPPLY, id);
    pstat=ow_bit_io(1); // pstat 0=is parasite/ !=0 ext. powered
    ow_reset();
    return (pstat) ? DS18X20_POWER_EXTERN:DS18X20_POWER_PARASITE;
}



/**
*   @brief start measurement (CONVERT_T) for all sensors if input id==NULL
   or for single sensor. then id is the rom-code
*    @param  [in] uint8_t with_power_extern
*    @param  [in] uint8_t id[] = rom_code
*    @return DS18X20_OK or DS18X20_START_FAIL
*     @date 20/06/2011
*/
uint8_t DS18X20_start_meas( uint8_t with_power_extern, uint8_t id[]) {
    ow_reset(); //**
    if ( ow_test_pin() ) { // only send if bus is "idle" = high
        ow_command( DS18X20_CONVERT_T, id );
        if (with_power_extern != DS18X20_POWER_EXTERN)
            ow_parasite_enable();
        return DS18X20_OK;
    }
    return DS18X20_START_FAIL;

}

/**
*   @brief reads temperature (scratchpad) of sensor with rom-code id
   output: subzero==1 if temp.<0, cel: full celsius, mcel: frac
   in millicelsius*0.1
   i.e.: subzero=1, cel=18, millicel=5000 = -18,5000&#65533;C
*    @param  [in] id[] = rom_code
*    @param  [out] subzero
*    @param  [out] cel
*    @param  [out] cel_frac_bits
*    @return DS18X20_OK or DS18X20_ERROR_CRC
*     @date 20/06/2011
*/
uint8_t DS18X20_read_meas(uint8_t id[], uint8_t *subzero,
                          uint8_t *cel, uint8_t *cel_frac_bits) {
    uint8_t i;
    uint8_t sp[DS18X20_SP_SIZE];

    ow_reset();
    ow_command(DS18X20_READ, id);
    for ( i=0 ; i< DS18X20_SP_SIZE; i++ )
        sp[i]=ow_byte_rd();
    if ( crc8( &sp[0], DS18X20_SP_SIZE ) ){
          if ((sp[DS18X20_SP_SIZE-1]==0xFF) && (sp[DS18X20_SP_SIZE-2]==0xFF))
            return OW_ERROR;    // bus error
        return DS18X20_ERROR_CRC;    // data error
    }
        
    DS18X20_meas_to_cel(id[0], sp, subzero, cel, cel_frac_bits);
    return DS18X20_OK;
}

/**
*   @brief convert raw value from DS18x20 to Celsius
   input is:
   - familycode fc (0x10/0x28 see header)
   - scratchpad-buffer
   output is:
   - cel full celsius
   - fractions of celsius in millicelsius*(10^-1)/625 (the 4 LS-Bits)
   - subzero =0 positiv / 1 negativ
   always returns  DS18X20_OK
   TODO invalid-values detection (but should be covered by CRC)
*    @param  [in] fc
*    @param  [in] sp
*    @param  [out] subzero
*    @param  [out] cel
*    @param  [out] cel_frac_bits
*    @return DS18X20_OK
*     @date 20/06/2011
*/
uint8_t DS18X20_meas_to_cel( uint8_t fc, uint8_t *sp,
                             uint8_t* subzero, uint8_t* cel, uint8_t* cel_frac_bits) {
    uint16_t meas;
    uint8_t  i;

    meas = sp[0];  // LSB
    meas |= ((uint16_t)sp[1])<<8; // MSB
    //meas = 0xff5e; meas = 0xfe6f;

    //  only work on 12bit-base
    if ( fc == DS18S20_ID ) { // 9 -> 12 bit if 18S20
        /* Extended measurements for DS18S20 contributed by Carsten Foss */
        meas &= (uint16_t) 0xfffe;    // Discard LSB , needed for later extended precicion calc
        meas <<= 3;                    // Convert to 12-bit , now degrees are in 1/16 degrees units
        meas += (16 - sp[6]) - 4;    // Add the compensation , and remember to subtract 0.25 degree (4/16)
    }

    // check for negative
    if ( meas & 0x8000 )  {
        *subzero=1;      // mark negative
        meas ^= 0xffff;  // convert to positive => (twos complement)++
        meas++;
    } else *subzero=0;

    // clear undefined bits for B != 12bit
    if ( fc == DS18B20_ID ) { // check resolution 18B20
        i = sp[DS18B20_CONF_REG];
        if ( (i & DS18B20_12_BIT) == DS18B20_12_BIT ) ;
        else if ( (i & DS18B20_11_BIT) == DS18B20_11_BIT )
            meas &= ~(DS18B20_11_BIT_UNDF);
        else if ( (i & DS18B20_10_BIT) == DS18B20_10_BIT )
            meas &= ~(DS18B20_10_BIT_UNDF);
        else { // if ( (i & DS18B20_9_BIT) == DS18B20_9_BIT ) {
            meas &= ~(DS18B20_9_BIT_UNDF);
        }
    }

    *cel  = (uint8_t)(meas >> 4);
    *cel_frac_bits = (uint8_t)(meas & 0x000F);

    return DS18X20_OK;
}

/**
*   @brief converts to decicelsius
   input is ouput from meas_to_cel
    i.e.: sz=0, c=28, frac=15 returns 289 (=28.9&#65533;C)
0    0    0
1    625    625    1
2    1250    250
3    1875    875    3
4    2500    500    4
5    3125    125
6    3750    750    6
7    4375    375
8    5000    0
9    5625    625    9
10    6250    250
11    6875    875    11
12    7500    500    12
13    8125    125
14    8750    750    14
15    9375    375
*    @param  [in]  subzero
*    @param  [in]  cel
*    @param  [in]  cel_frac_bits
*    @return absolute value of temperatur in decicelsius
*     @date 20/06/2011
*/
uint16_t DS18X20_temp_to_decicel(uint8_t subzero, uint8_t cel,
                                 uint8_t cel_frac_bits) {
    uint16_t h;
    uint8_t  i;
    uint8_t need_rounding[] = { 1, 3, 4, 6, 9, 11, 12, 14 };

    h = cel_frac_bits*DS18X20_FRACCONV/1000;
    h += cel*10;
    if (!subzero) {
        for (i=0; i<sizeof(need_rounding); i++) {
            if ( cel_frac_bits == need_rounding[i] ) {
                h++;
                break;
            }
        }
    }
    return h;
}
/**
*   @brief  compare temperature values (full celsius only)
*    @param  [in] subzero1
*    @param  [in] cel1
*    @param  [in] subzero2
*    @param  [in] cel2
*    @return -1 if param-pair1 < param-pair2
            0 if ==
            1 if >
*     @date 20/06/2011
*/
int8_t DS18X20_temp_cmp(uint8_t subzero1, uint16_t cel1,
                        uint8_t subzero2, uint16_t cel2) {
    int16_t t1 = (subzero1) ? (cel1*(-1)) : (cel1);
    int16_t t2 = (subzero2) ? (cel2*(-1)) : (cel2);

    if (t1<t2) return -1;
    if (t1>t2) return 1;
    return 0;
}