/**
* @file DS2450.c
* @brief library of DS2450 1-Wire Quad A/D Converter  (http://www.maxim-ic.com/datasheet/index.mvp/id/2921)
* @author Frederic BLANC
*/
#include "mbed.h"
#include "onewire.h"
#include "DS2450.h"
#include "crc8.h"
#include "crc16.h"
#include "utils.h"

/**
 *     @brief lancement lecture DS2450 ADC
 *    @param  [in] id[] tableau d'identifiant OW
 *    @param  [out] adc[] tableau des valeurs des adc
 *    @return OW_OK si erreur retourne OW_ERROR_CRC
 *     @date 20/06/2011
 *
 */
uint8_t DS2450_read_ADC(uint8_t id[], uint16_t adc[]) {
    uint8_t i,j;
    uint8_t error;
    uint8_t sp[DS2450_SP_SIZE];
    //waiting for convertion time ( nbchannel x resolution x 80&#65533;s +160&#65533;s)

    error=DS2450_read_page(&id[0],DS2450_PAGE0,&sp[0]); //read data
    if (error)
        return error;
    j=0;
    for (i=0;i<8;i+=2)
        adc[j++]=uint8_to_uint16(sp[i+3],sp[i+4]); //sp[i+3] LSB ,sp[i+4] MSB
    return OW_OK;
}
/**
 *     @brief lancement & lecture DS2450 ADC
 *    @param  [in] id[] tableau d'identifiant OW
 *    @param  [out] adc[] tableau des valeurs des adc
 *    @return OW_OK si erreur retourne OW_ERROR_CRC
 *     @date 20/06/2011
 *
 */
uint8_t DS2450_start_and_read_ADC(uint8_t id[], uint16_t adc[]) {
    uint8_t i,j;
    uint8_t error;
    uint8_t sp[DS2450_SP_SIZE];

    error=DS2450_convert(&id[0],0x0F,0x00); //start convert
    if (error)
        return error;

    wait_ms(15);                           //waiting for convertion time ( nbchannel x resolution x 80&#65533;s +160&#65533;s)

    error=DS2450_read_page(&id[0],DS2450_PAGE0,&sp[0]); //read data
    if (error)
        return error;

    j=0;
    for (i=0;i<8;i+=2)
        adc[j++]=uint8_to_uint16(sp[i+3],sp[i+4]); //sp[i+3] LSB ,sp[i+4] MSB
    return OW_OK;
}
/**
 *     @brief lancement lecture page DS2450 ADC
 *    @param  [in] uint8_t id[] tableau d'identifiant OW
 *    @param  [in] uint8_t adresse de la page a lire
 *    @param  [out] uint8_t uint16_t sp tableau des valeurs de la page
 *    @return OW_OK si erreur retourne OW_ERROR_CRC
 *     @date 20/06/2011
 *
 */
uint8_t DS2450_read_page(uint8_t id[], uint8_t adresse,
                         uint8_t *sp) {
    uint8_t i;
    if (id[0] == DS2450_ID) {
        if (ow_reset())                 //reset OW
            return OW_SHORT_CIRCUIT;
        sp[0]=DS2450_READ_MEMORY;   // command
        sp[1]=adresse;              //adress page LSB
        sp[2]=0;                    //adress page MSB
        ow_command(sp[0], &id[0]);
        ow_byte_wr(sp[1]);
        ow_byte_wr(sp[2]);

        for ( i=3 ; i< DS2450_SP_SIZE; i++ ) { //read 8xdata + CRC16
            sp[i]=ow_byte_rd();
        }
#ifdef DEBUG
        printf( "\n" );
        for ( i=0 ; i< DS2450_SP_SIZE; i++ )
            printf(":%2.2X",sp[i]);
        printf( "\n" );
#endif
        if (ctrl_crc16( &sp[0], DS2450_SP_SIZE ) ) { //CRC16 (command+adress page LSB+adress page MSB+8xdata)
            wait_ms(100);                   //wait 100ms if error
            if ((sp[DS2450_SP_SIZE-1]==0xFF) && (sp[DS2450_SP_SIZE-2]==0xFF))
                return OW_ERROR;    // bus error
            if ((sp[DS2450_SP_SIZE-1]==0x00) && (sp[DS2450_SP_SIZE-2]==0x00))
                return OW_BUSY;
#ifdef DEBUG_L1 
printf( "\n" );
for ( i=0 ; i< DS2450_SP_SIZE; i++ )
    printf(":%2.2X",sp[i]);
printf( "\n" ); 
#endif    
            return OW_ERROR_CRC;    // data error
        }
        return OW_OK;
    }
    return OW_ERROR_BAD_ID;
}

/**
 *     @brief lancement convertion DS2450 ADC
 *    @param  [in] uint8_t id[] tableau d'identifiant OW
 *    @param  [in] uint8_t input_select_mask
 *    @param  [in] uint8_t read_out_control
 *    @return OW_OK si erreur retourne OW_ERROR_CRC
 *     @date 20/06/2011
 *
 */
uint8_t DS2450_convert(uint8_t id[], uint8_t input_select_mask,uint8_t read_out_control) {
    uint8_t i;
    uint8_t sp[5];
    if (id[0] == DS2450_ID) {
        if (ow_reset())                 //reset OW
            return OW_SHORT_CIRCUIT;
        sp[0]=DS2450_CONVERT;       // command
        sp[1]=input_select_mask;    //mask
        sp[2]=read_out_control;     //control
        ow_command(sp[0], &id[0]);
        ow_byte_wr(sp[1]);
        ow_byte_wr(sp[2]);
        for ( i=3 ; i< 5; i++ ) {   // read CRC16
            sp[i]=ow_byte_rd();
        }
#ifdef DEBUG
        printf( "\n" );
        for ( i=0 ; i< 5; i++ )
            printf(":%2.2X",sp[i]);
        printf( "\n" );
#endif
        if (ctrl_crc16( &sp[0], 5 ) ) { //CRC16 (command+mask LSB+control)
            if ((sp[3]==0xFF) && (sp[3]==0xFF))
                return OW_ERROR;
            return OW_ERROR_CRC;
        }
        return OW_OK;
    }
    return OW_ERROR_BAD_ID;
}

/**
 *     @brief configure canal ADC  DS2450
 *    @param  [in] id[] tableau d'identifiant OW
 *    @param  [in] channel
 *  @param  [in] conflsb configuration OE-A OC-A 0 0 RC3-A RC2-A RC1-A RC0-A
 *  @param  [in] confmsb configuration POR 0 AFH-A AFL-A AEH-A AEL-A 0 IR-A
 *    @return OW_OK si erreur retourne OW_ERROR_CRC
 *     @date 20/06/2011
 *
 */
uint8_t DS2450_configure_channel_ADC(uint8_t id[],uint8_t channel,uint8_t conflsb,uint8_t confmsb) {
    uint8_t i;
    uint8_t sp[7];
    if (id[0] == DS2450_ID) {
        if (ow_reset())                 //reset OW
            return OW_SHORT_CIRCUIT;
        sp[0]=DS2450_WRITE_MEMORY;  // command
        sp[1]=DS2450_PAGE1+channel; //adress page LSB
        sp[2]=0x00;                 //adress page MSB
        sp[3]=conflsb;              //databyte
        ow_command(sp[0], &id[0]);
        ow_byte_wr(sp[1]);
        ow_byte_wr(sp[2]);
        ow_byte_wr(sp[3]);
        for ( i=4 ; i< 7; i++ ) {   //read CRC16+databyte
            sp[i]=ow_byte_rd();
        }
#ifdef DEBUG
        printf( "\n" );
        for ( i=0 ; i< 7; i++ )
            printf(":%2.2X",sp[i]);
        printf( "\n" );
#endif
        if (ctrl_crc16( &sp[0], 6 ) ) //CRC16 (command+adress page LSB+adress page MSB+databyte)
            return OW_ERROR_CRC;
        sp[3]=confmsb;                //databyte
        ow_byte_wr(sp[3]);
        for ( i=4 ; i< 7; i++ ) {   //read CRC16+databyte
            sp[i]=ow_byte_rd();
        }
#ifdef DEBUG
        printf( "\n" );
        for ( i=0 ; i< 7; i++ )
            printf(":%2.2X",sp[i]);
        printf( "\n" );
#endif
        if (sp[3]!=sp[6] ) //control data
            return OW_ERROR_CRC;
        return OW_OK;
    }
    return OW_ERROR_BAD_ID;
}
/**
 *     @brief configure PAGE
 *    @param  [in] id[] tableau d'identifiant OW
  *    @param  [in] uint8_t adresse de la page a ecrire
 *  @param  [in] config_page tableau de 8 byte
 *    @return OW_OK si erreur retourne OW_ERROR_CRC
 *     @date 20/06/2011
 *
 */
uint8_t DS2450_configure_page(uint8_t id[], uint8_t adresse,uint8_t configpage[]) {
    uint8_t i,j;
    uint8_t sp[7];
    if (id[0] == DS2450_ID) {
        if (ow_reset())                 //reset OW
            return OW_SHORT_CIRCUIT;
        sp[0]=DS2450_WRITE_MEMORY;      // command
        sp[1]=adresse;                  //adress page LSB
        sp[2]=0x00;                     //adress page MSB
        sp[3]=configpage[0];            //databyte
        ow_command(sp[0], &id[0]);
        ow_byte_wr(sp[1]);
        ow_byte_wr(sp[2]);
        ow_byte_wr(sp[3]);
        for ( i=4 ; i< 7; i++ ) {       //read CRC16+databyte
            sp[i]=ow_byte_rd();
        }
#ifdef DEBUG
        printf( "\n" );
        for ( i=0 ; i< 7; i++ )
            printf(":%2.2X",sp[i]);
        printf( "\n" );
#endif
        if (sp[3]!=sp[6] ) //control data
            return OW_ERROR_CRC;

        for ( j=1 ; j< 7; j++ ) {
            sp[3]=configpage[j];        //databyte
            ow_byte_wr(sp[3]);
            for ( i=4 ; i< 7; i++ ) {   //read CRC16+databyte
                sp[i]=ow_byte_rd();
            }
            if (sp[3]!=sp[6] ) //control data
                return OW_ERROR_CRC;

#ifdef DEBUG
            printf( "\n" );
            for ( i=0 ; i< 7; i++ )
                printf(":%2.2X",sp[i]);
            printf( "\n" );
#endif
        }
        return OW_OK;
    }
    return OW_ERROR_BAD_ID;
}