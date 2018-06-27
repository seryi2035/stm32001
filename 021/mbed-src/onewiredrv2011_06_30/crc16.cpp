#include <inttypes.h>
#include "mbed.h"
#include "onewire.h"

#define CRC16INIT    0x0000
//#define CRC16POLY    0x8005; // Polynome =x^16 + x^15 + x^2 + x^0 = 0x18005
#define CRC16POLY    0xA001;

uint16_t crc16(uint8_t* octets, uint16_t nboctets)
{
uint16_t crc = CRC16INIT;
int i, done = 0;
uint8_t todo;
    if (nboctets != 0) {
        do {
            todo = octets[done];
            crc ^= todo;
            for (i = 0; i < 8; i++) {
                if (crc % 2 != 0) {
                    crc = (crc >> 1) ^ CRC16POLY;
                } else {
                    crc = crc >> 1;
                }
            }
            done++;
        } while (done < nboctets);
        
        
    }

return crc;
}
//CRC16 byte, always two bytes, bit inverted, LSByte first 
uint8_t ctrl_crc16(uint8_t* octets, uint16_t nboctets)
{
    uint16_t crc;
    uint8_t *ptr;
#ifdef DEBUG
    printf( "\nCRC16 : " );
    for ( uint8_t i=0 ; i< nboctets; i++ )
        printf(":%2.2X",octets[i]);
    printf( "\n" );
#endif
    crc =~crc16(octets, nboctets-2);
    ptr=(uint8_t*)&crc;
#ifdef DEBUG
    printf( "\n" );
    printf("CRC16:%X",crc);
    printf( "\n" );
#endif
    if(*ptr==octets[nboctets-2])
        if(*++ptr==octets[nboctets-1])
            return 0;

    return 1;
 } 