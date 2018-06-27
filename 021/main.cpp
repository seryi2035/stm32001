#include "mbed.h"
// DS18B20 converted to run on mbed
//
#define TEMP_PIN  PB_10
Serial pc(PA_9, PA_10); // tx, rx



void delayMicroseconds(int howmany)
{
  int n;
  Timer t;
  t.start();
  do
  {
    n=t.read_us();
  }
  while (n<= howmany);
}
void OneWireReset() // reset.  Should improve to act as a presence pulse
{
     DigitalOut temperature_pin(TEMP_PIN);
     temperature_pin = 0;     // bring low for 500 us
     delayMicroseconds(500);
    DigitalIn temperature_pin_in(TEMP_PIN);
     delayMicroseconds(500);
}

void OneWireOutByte(unsigned char d) // output byte d (least sig bit first).
{
   unsigned char n;

   for(n=8; n!=0; n--)
   {
      if ((d & 0x01) == 1)  // test least sig bit
      {
         DigitalOut temperature_pin(TEMP_PIN);
         temperature_pin = 0;
         delayMicroseconds(5);
         DigitalIn temperature_pin_in(TEMP_PIN);
         delayMicroseconds(60);
      }
      else
      {
         DigitalOut temperature_pin(TEMP_PIN);
         temperature_pin = 0;
         delayMicroseconds(60);
         DigitalIn temperature_pin_in(TEMP_PIN);
      }

      d=d>>1; // now the next bit is in the least sig bit position.
   }

}

unsigned char OneWireInByte() // read byte, least sig byte first
{
    unsigned char d, n, b;

    for (n=0; n<8; n++)
    {
        DigitalOut temperature_pin(TEMP_PIN);
         temperature_pin = 0;
        delayMicroseconds(5);
        DigitalIn temperature_pin_in(TEMP_PIN);
        delayMicroseconds(5);
        b= temperature_pin_in;
        delayMicroseconds(50);
        d = (d >> 1) | (b<<7); // shift d to right and insert b in most sig bit position
    }
    return(d);
}



int main()
{
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
   pc.printf("mbed 1 wire interface simple test!\r\n");
   DigitalOut temperature_pin(TEMP_PIN);
   temperature_pin = 0;
   DigitalIn temperature_pin_in(TEMP_PIN);  // sets the digital pin as input (logic 1) make sure external pullup resistor 4K7
    wait(1);
    pc.printf("temperature measurements:\r\n");

while(1) {
  OneWireReset();
  OneWireOutByte( 0xcc);  //Skip ROM command
  OneWireOutByte( 0x44); // perform temperature conversion, strong pullup for one sec

  OneWireReset();
  OneWireOutByte( 0xcc);
  OneWireOutByte( 0xbe);   //Read Scratchpad

  LowByte = OneWireInByte();
  HighByte = OneWireInByte();
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;


  if (SignBit) // If its negative
  {
     pc.printf("-");
  }
   pc.printf("%d", Whole);
   pc.printf(".");
  if (Fract < 10)
  {
      pc.printf("0");
  }
  pc.printf("%d", Fract);
  pc.printf("\r\n");

  wait (2);         // 5 second delay.  Adjust as necessary
}
}
