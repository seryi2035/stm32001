#include "ds1820.h"
#include "001.h"
#include "tim2_delay.h"
#include "string.h"
#include "stdio.h"

//#define ONE_WIRE_IN PORTBbits.RB5
//#define ONE_WIRE_OUT LATBbits.LATB5
//#define ONE_WIRE_TRIS TRISBbits.TRISB5

//#define delay1mks asm("nop"); asm("nop"); asm("nop"); asm("nop")

/*void delay_us(uint16 period)
{
	 uint16 i;
	 for (i = 0; i < period; i++)
	 {
		nop(); nop(); nop(); nop();
                nop(); nop(); nop(); nop();
                nop(); nop(); nop(); nop();
                nop(); nop(); nop(); nop();
	 }
}*/

/*******************1-wire communication functions********************/

/************onewire_reset*************************************************/
/*This function initiates the 1wire bus */
/* */
/*PARAMETERS: */
/*RETURNS: */
/*********************************************************************/

void onewire_reset()  // OK if just using a single permanently connected device
{

 ONE_WIRE_OUT();
 //delay_us (400); // pull 1-wire low for reset pulse
 delay_us (400);
 ONE_WIRE_IN();
 delay_us (400); // wait-out remaining initialisation window.
}

/*********************** onewire_write() ********************************/
/*This function writes a byte to the sensor.*/
/* */
/*Parameters: byte - the byte to be written to the 1-wire */
/*Returns: */
/*********************************************************************/

void onewire_write(uint8 data)
{
 uint8 count;
 uint8 databuf;
 databuf = data;
 uint8 cond;

 for (count=0; count<8; ++count)
 {
   ONE_WIRE_OUT();
    delay_us(2);  // pull 1-wire low to initiate write time-slot.
    if (databuf & 0x01)
    {
      ONE_WIRE_IN();
    }
   databuf = databuf >> 1;
  delay_us( 60 ); // wait until end of write slot.
  ONE_WIRE_IN(); // set 1-wire high again,
  delay_us( 2 ); // for more than 1us minimum.
 }
}




/*********************** read1wire() *********************************/
/*This function reads the 8 -bit data via the 1-wire sensor. */
/* */
/*Parameters: */
/*Returns: 8-bit (1-byte) data from sensor */
/*********************************************************************/

uint8 onewire_read()
{
 uint8 count, data;
 data = 0;
 for (count=0; count<8; ++count)
 {
   ONE_WIRE_OUT();
   delay_us(2); // pull 1-wire low to initiate read time-slot.
   ONE_WIRE_IN(); // now let 1-wire float high,
 	// let device state stabilise,
   delay_us(6);
   if (ONE_WIRE_READ())
     data = data | 0x80;
   if (count < 7)
   {
   	data = (data >> 1) & 0x7f;
   }	
  delay_us( 80 ); // wait until end of read slot.  вообще-то в оригинале здесь 120мкс, но процедурка моя не выдерживает строго 1мкс
 }

 return( data );
}

/*

float ds1820_read()
{
 int8 busy=0;

 uint8 temp1, temp2;
 uint16 temp3;
 float result;

 onewire_reset();
 onewire_write(0xCC);
 onewire_write(0x44);

 while (busy == 0)
  busy = onewire_read();

 onewire_reset();
 onewire_write(0xCC);
 onewire_write(0xBE);
 temp1 = onewire_read();   // D2
 temp2 = onewire_read();  // 01

 temp3 = (uint16) temp2 * (uint16)0x0100L + (uint16) temp1;
 //result = (float) temp3 / 2.0;   //Calculation for DS18S20 with 0.5 deg C resolution
 result = (float)temp3 / 16.0; //Calculation for DS18B20 with 0.1 deg C resolution
 result = temp3/16.0;

 return(result);
}

*/

void ds1820_set9bit(void)
{
	onewire_reset();
	onewire_write(0xCC);
	onewire_write(0x4E);
	onewire_write(0x00);
	onewire_write(0x00);
	onewire_write(0x1F);
}

void ds1820_startconversion(void)
{
	onewire_reset();
	onewire_write(0xCC);
	onewire_write(0x44);
}
	
int16 ds1820_readtemp(void)
{
	u8 temp1, temp2, t3;
	int16 temp3;
	onewire_reset();
	onewire_write(0xCC);
	onewire_write(0xBE);
	temp1 = onewire_read();   // D2
	temp2 = onewire_read();  // 01
	t3 = onewire_read();
	t3 = onewire_read();
	t3 = onewire_read();
	
	temp3 = (uint16) temp2 * (uint16)0x0100L + (uint16) temp1;
	temp3 = temp3 / 16; // целочисленное деление на 16
	return temp3;
}	
