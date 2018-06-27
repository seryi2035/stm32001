/**
* @file onewire.c
* @brief library 1-Wire(www.maxim-ic.com)
* @author Maciej Rajtar (Published 10 May 2010 www.mbed.org)
* @author Frederic BLANC
*/
#include "mbed.h"
#include "onewire.h"
#include "DS2450.h"
#include "DS18X20.h"
#include "crc8.h"

DigitalInOut ow_pin(ONEWIRE_PIN);

/**
*     @brief PUL-UP bus OW
*    @return OW_OK
*     @date 20/06/2011
*/
uint8_t ow_PullUp(void)
{
    ow_pin.mode(PullUp); //PULL-UP bus OW
return OW_OK;
}
/**
*     @brief search_sensors
*    @param  [out] nSensors number of device onewire
*    @param  [out] gSensorIDs[][] array of id romcode
*    @return OW_OK or OW_PRESENCE_ERR or OW_DATA_ERR
*     @date 20/06/2011
*/
uint8_t search_sensors(uint8_t *nSensors,uint8_t *gSensorIDs ) {
    uint8_t i;
    uint8_t id[OW_ROMCODE_SIZE];
    uint8_t diff;
    printf( "Scanning Bus\n" );
    diff = OW_SEARCH_FIRST;
    for (*nSensors = 0 ; (diff != OW_LAST_DEVICE) && (*nSensors < MAXSENSORS) ;++(*nSensors) ) {
        ow_find_sensor( &diff, &id[0] );
        if ( diff == OW_PRESENCE_ERR ) {
            printf( "No Sensor found\n" );
            return diff;
        }
        if ( diff == OW_DATA_ERR ) {
            printf( "Bus Error\n" );
            return diff;
        }
        for (i=0;i<OW_ROMCODE_SIZE;i++)
            gSensorIDs[(*nSensors)*OW_ROMCODE_SIZE+i]=id[i];

    }
    return OW_OK;
}
/**
*     @brief show_id
*    @param  [in] id[] = rom_code
*    @param  [in] n number of id[n]
*    @param  [out] text id
*    @return OW_OK or OW_ERROR_CRC
*     @date 20/06/2011
*/
uint8_t ow_show_id( uint8_t id[], size_t n ,char *text) {
    size_t i;
    char hex[4];
    sprintf(text,"");
    
    for ( i = 0; i < n; i++ ) {
        if ( i == 0 ) strcat(text, " FC: " );
        else if ( i == n-1 ) strcat(text, "CRC: " );
        if ( i == 1 ) strcat(text, " SN: " );
        sprintf(hex,"%2.2X ",id[i]);
        strcat(text,hex);
        if ( i == 0 ) {
            if ( id[0] == DS18S20_ID ) strcat(text,"(18S)");
            else if ( id[0] == DS18B20_ID ) strcat(text,"(18B)");
            else if ( id[0] == DS2450_ID ) strcat(text,"(ADC)");
            else strcat(text,"( ? )");
        }
    }
    if ( crc8( id, OW_ROMCODE_SIZE) )
        return OW_ERROR_CRC;
    return OW_OK;
}

/**
*     @brief test pin onewire bus
*    @return etat pin ow
*     @date 20/06/2011
*/
uint8_t ow_test_pin (void){
    if (ow_pin)
        return 1;
return 0;   
}
/**
*     @brief onewire reset bus
*    @return pin ow or OW_SHORT_CIRCUIT
*     @date 20/06/2011
*/
uint8_t ow_reset(void) { // reset.  Should improve to act as a presence pulse
    uint8_t err;

    ow_pin.output();
    ow_pin = 0;     // bring low for 500 us
    wait_us(500);
    ow_pin.input();
    wait_us(60);
    err = ow_pin;
    wait_us(240);
    if ( ow_pin == 0 )    {    // short circuit
        err = OW_SHORT_CIRCUIT;
    }
    return err;
}

/**
*     @brief read write onewire
*    @param  [in/out] b data
*    @return data
*     @date 20/06/2011
*/
uint8_t ow_bit_io( uint8_t b ) {

    ow_pin.output(); // drive bus low
    ow_pin = 0;
    wait_us(1); // Recovery-Time wuffwuff was 1
    
    if ( b ) 
        ow_pin.input(); // if bit is 1 set bus high (by ext. pull-up)
    //  delay was 15uS-1 see comment above
    wait_us(15-1);
    if ( ow_pin == 0 ) b = 0; // sample at end of read-timeslot
    wait_us(60-15);
    ow_pin.input();
    return b;
}

/**
*     @brief byte write on onewire
*    @param  [in] b data
*    @return data
*     @date 20/06/2011
*/
uint8_t ow_byte_wr( uint8_t b ) {
    uint8_t i = 8, j;

    do {
        j = ow_bit_io( b & 1 );
        b >>= 1;
        if ( j )
            b |= 0x80;
    } while ( --i );
    return b;
}

/**
*     @brief byte read on onewire
*    @param  [in] uint8_t b
*    @return 
*     @date 20/06/2011
*/
uint8_t ow_byte_rd( void ) {
    // read by sending 0xff (a dontcare?)
    return ow_byte_wr( 0xFF );
}


/**
*     @brief search romcode
*    @param  [in] uint8_t diff
*    @param  [out] id romcode
*    @return next_diff or OW_LAST_DEVICE or OW_DATA_ERR or OW_PRESENCE_ERR
*     @date 20/06/2011
*/
uint8_t ow_rom_search( uint8_t diff, uint8_t id[] ) {
    uint8_t i, j, next_diff;
    uint8_t b;

    if ( ow_reset() )
        return OW_PRESENCE_ERR;    // error, no device found
    ow_byte_wr( OW_SEARCH_ROM );            // ROM search command
    next_diff = OW_LAST_DEVICE;            // unchanged on last device
    i = OW_ROMCODE_SIZE * 8;                    // 8 bytes
    do {
        j = 8;                                // 8 bits
        do {
            b = ow_bit_io( 1 );                // read bit
            if ( ow_bit_io( 1 ) ) {            // read complement bit
                if ( b )                    // 11
                    return OW_DATA_ERR;        // data error
            } else {
                if ( !b ) {                    // 00 = 2 devices
                    if ( diff > i || ((*id & 1) && diff != i) ) {
                        b = 1;                // now 1
                        next_diff = i;        // next pass 0
                    }
                }
            }
            ow_bit_io( b );                 // write bit
            *id >>= 1;
            if ( b ) 
                *id |= 0x80;            // store bit
            --i;
        } while ( --j );
        id++;                                // next byte
    } while ( i );
    return next_diff;                // to continue search
}

/**
*     @brief write command
*    @param  [in] command
*    @param  [in] id romcode
*     @date 20/06/2011
*/
uint8_t ow_command( uint8_t command, uint8_t id[] ) {
    uint8_t i;

    ow_reset();
    if ( id ) {
        ow_byte_wr( OW_MATCH_ROM );            // to a single device
        i = OW_ROMCODE_SIZE;
        do {
            ow_byte_wr( *id );
            ++id;
        } while ( --i );
    } else {
        ow_byte_wr( OW_SKIP_ROM );            // to all devices
    }
    ow_byte_wr( command );
    return 0;
}
/**
*     @brief parasite enable
*     @date 20/06/2011
*/
uint8_t ow_parasite_enable(void) {
    ow_pin.output();
    ow_pin = 1;
    return 0;
}
/**
*     @brief parasite disable
*     @date 20/06/2011
*/
uint8_t ow_parasite_disable(void) {

    ow_pin.input();
    return 0;
}


/**
*     @brief find Sensors on 1-Wire-Bus
*    @param  [in/out] diff is the result of the last rom-search
*    @param  [out] is the rom-code of the sensor found
*    @return  OW_OK or OW_ERROR 
*/
uint8_t ow_find_sensor(uint8_t *diff, uint8_t id[]) {
    for (;;) 
    {
        *diff = ow_rom_search( *diff, &id[0] );
        if ( *diff==OW_PRESENCE_ERR)
            return OW_ERROR;
        if ( *diff==OW_DATA_ERR )
            return OW_ERROR;
        if ( *diff == OW_LAST_DEVICE )
            return OW_OK ;
        if ( id[0] == DS18B20_ID || id[0] == DS18S20_ID ) 
            return OW_OK ;
        if ( id[0] == DS2450_ID ) 
            return OW_OK ;
    }
}

/**
*     @brief output byte d (least sig bit first)
*    @param  [in] d output byte (least sig bit first)
*    @return  OW_OK 
*/
uint8_t OneWireOutByte(uint8_t d) { // output byte d (least sig bit first).
    for (int n=8; n!=0; n--) {
        if ((d & 0x01) == 1) { // test least sig bit
            ow_pin.output();
            ow_pin = 0;
            wait_us(5);
            ow_pin.input();
            wait_us(80);
        } else {
            ow_pin.output();
            ow_pin = 0;
            wait_us(80);
            ow_pin.input();
        }
        d=d>>1; // now the next bit is in the least sig bit position.
    }
    return OW_OK;
}
/**
*     @brief read byte, least sig byte first
*    @return  byte, least sig byte first 
*/
uint8_t OneWireInByte(void) { // read byte, least sig byte first
    uint8_t d = 0, b;
    for (int n=0; n<8; n++) {
        ow_pin.output();
        ow_pin = 0;
        wait_us(5);
        ow_pin.input();
        wait_us(5);
        b =ow_pin;
        wait_us(50);
        d = (d >> 1) | (b << 7); // shift d to right and insert b in most sig bit position
    }
    return d;
}

