#include <inttypes.h>

uint16_t uint8_to_uint16(uint8_t lsb, uint8_t msb)
{
    uint16_t data;
    uint8_t *ptr;
    
    ptr=(uint8_t*)&data;
    *ptr=lsb;
    *++ptr=msb;
    return data;
}