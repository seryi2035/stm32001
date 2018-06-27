/**
* @file onewire.h
* @brief library 1-Wire(www.maxim-ic.com)
* @author Maciej Rajtar (Published 10 May 2010 www.mbed.org)
* @author Frederic BLANC
*/

#ifndef _onewire_
#define _onewire_

//#define DEBUG 1
//#define DEBUG_L1 1
#define ONEWIRE_PIN  PB_11

#define MAXSENSORS 16

// rom-code size including CRC
#define OW_ROMCODE_SIZE 8

#define OW_OK          0x00
#define OW_ERROR       0x01
#define OW_START_FAIL  0x02
#define OW_ERROR_CRC   0x03
#define OW_ERROR_BAD_ID 0x04
#define OW_BUSY          0x05

#define OW_MATCH_ROM    0x55
#define OW_SKIP_ROM     0xCC
#define OW_SEARCH_ROM   0xF0
#define OW_READ_ROM     0x33
#define OW_CONDITIONAL_SEARCH     0xEC
#define OW_OVERDRIVE_SKIP_ROM     0x3C
#define OW_OVERDRIVE_MATCH_ROM    0x69

#define OW_SHORT_CIRCUIT   0xFF
#define OW_SEARCH_FIRST    0xFF        // start new search
#define OW_PRESENCE_ERR    0x01
#define OW_DATA_ERR        0xFE
#define OW_LAST_DEVICE     0x00        // last device found
//            0x01 ... 0x40: continue searching
uint8_t search_sensors(uint8_t *nSensors,uint8_t *gSensorIDs );
uint8_t ow_PullUp(void);
uint8_t ow_show_id( uint8_t id[], size_t n ,char *text);
uint8_t ow_test_pin (void);
uint8_t ow_reset(void);
uint8_t ow_rom_search( uint8_t diff, uint8_t id[] );
uint8_t ow_command( uint8_t command, uint8_t id[] );
uint8_t ow_find_sensor(uint8_t *diff, uint8_t id[]);
uint8_t ow_parasite_enable(void);
uint8_t ow_parasite_disable(void);
uint8_t ow_bit_io( uint8_t b );
uint8_t ow_byte_wr( uint8_t b );
uint8_t ow_byte_rd( void );


#endif
