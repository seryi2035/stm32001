#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_bkp.h"
#include "stdio.h"
#include "misc.h"
#include "001.h"
#include "tim2_delay.h"
#include "onewire.h"
#include <string.h>
struct DHT11_Dev dev001;

char cifry[10];

int main(void)
{
  uint32_t RTC_Counter = 0;
  RTC_DateTimeTypeDef RTC_DateTime;

  GETonGPIO(); //led C13
  TIM2_init();
  usart1_init(); //A9 A10 //RS232
  usart3_init();//  B 10 TX DI //  B 11 RX RO //B1 RE //B0 DE
  OW_Init(); //usart2 А2 А3         B10 B11
  dev001.port = GPIOA;
  dev001.pin = GPIO_Pin_8;
  dev001.humidity = 0;
  dev001.temparature = 0;
  DHT11_init(&dev001, GPIOA, GPIO_Pin_8);
  wwdgenable();

  //GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1
  //GPIO_ResetBits(GPIOC, GPIO_Pin_13);   //C13 --0

  if (RTC_Init() == 1) {
      // Если первая инициализация RTC устанавливаем начальную дату, например 22.09.2016 14:30:00
      RTC_DateTime.RTC_Date = 8;
      RTC_DateTime.RTC_Month = 7;
      RTC_DateTime.RTC_Year = 2018;

      RTC_DateTime.RTC_Hours = 19;
      RTC_DateTime.RTC_Minutes = 49;
      RTC_DateTime.RTC_Seconds = 30;
      //После инициализации требуется задержка. Без нее время не устанавливается.
      delay_ms(500);
      RTC_SetCounter(RTC_GetRTC_Counter(&RTC_DateTime));

    }

  USARTSend("\n\rREADY!!!\n\r");
  //USART3Send("\n\rREADY!!!\n\r");
  while (1) {
      /*if(uart3.rxgap==1)
        {
          MODBUS_SLAVE(&uart3);
          net_tx3(&uart3);
        }
*/
      /*if(uart1.rxgap==1)
        {
          MODBUS_SLAVE(&uart1);
          net_tx1(&uart1);
        }*/

      if (RX_FLAG_END_LINE == 1) {
          // Reset RX_Flag end line
          RX_FLAG_END_LINE = 0;

          USARTSend("\n\rI has received a line:\n\r");
          USARTSend(RX_BUF);
          USARTSend("\n\r");
          /*USART3Send("\n\rI has received a line:\n\r");
          USART3Send(RX_BUF);
          USART3Send("\n\r");*/

          if (strncmp(RX_BUF, "ON\r", 3) == 0 || strncmp(RX_BUF, "on\r", 3) == 0) {
              USARTSend("\n\r THIS IS A COMMAND \"ON\"!!!\n\r");
              delay_ms(100);
              GPIO_ResetBits(GPIOC, GPIO_Pin_13); //ON
            }
          if (strncmp(RX_BUF, "OFF\r", 4) == 0 || strncmp(RX_BUF, "off\r", 4) == 0) {
              USARTSend("\n\rTHIS IS A COMMAND \"OFF\"!!!\n\r");
              delay_ms(100);
              GPIO_SetBits(GPIOC, GPIO_Pin_13); //OFF
            }
          if (strncmp(RX_BUF, "T\r", 1) == 0 || strncmp(RX_BUF, "t\r", 4) == 0) {
              RTC_Counter = RTC_GetCounter();
              sprintf(buffer, "\r\n\r\nCOUNTER: %d\r\n", (int)RTC_Counter);
              USARTSend(buffer);
              RTC_GetDateTime(RTC_Counter, &RTC_DateTime);
              sprintf(buffer, "\r\n\r\n%d.%d.%d  %d:%d:%d\r\n\r",
                      (int)RTC_DateTime.RTC_Date, (int)RTC_DateTime.RTC_Month, (int)RTC_DateTime.RTC_Year,
                      (int)RTC_DateTime.RTC_Hours, (int)RTC_DateTime.RTC_Minutes, (int)RTC_DateTime.RTC_Seconds);
              USARTSend(buffer);

              // Функция генерирует в буфере дату собственного формата
              RTC_GetMyFormat(&RTC_DateTime, buffer);
              USARTSend(buffer);
              USARTSend("\n\r");

            }
          if (strncmp(RX_BUF, "SCAN\r", 4) == 0) {
              for(int i = 0;i < RX_BUF_SIZE - 1; i++) RX_BUF08[i] = (u8) RX_BUF[i];
              OW_Scan(RX_BUF08, 1);
              for(int i = 0;i < RX_BUF_SIZE - 1; i++) RX_BUF[i] = (char) RX_BUF08[i];
              //char cifry[10];
              sendaddrow();
            }
          if (strncmp(RX_BUF, "0\r", 4) == 0) {
              schitatTemp("\x28\xee\x6c\x08\x1a\x16\x01\x30");
              //schitatTemp("\x28\xee\xcd\xa9\x19\x16\x01\x0c");
            }
          if (strncmp(RX_BUF, "1\r", 4) == 0) {
              schitatTemp("\x28\xee\x09\x03\x1a\x16\x01\x67");
            }
          if (strncmp(RX_BUF, "5\r", 4) == 0) {
              oprosite ();
              //USARTSend("oprosheno\n\r");
            }
          if(strncmp(RX_BUF, "2\r", 4) == 0) {
              delay_ms(100);
              schitatTemp("\x28\xee\x6c\x08\x1a\x16\x01\x30");
              //schitatTemp("\x28\xee\xcd\xa9\x19\x16\x01\x0c");
              delay_ms(100);
              schitatTemp("\x28\xee\x09\x03\x1a\x16\x01\x67");
              delay_ms(100);
            }
          if (strncmp(RX_BUF, "9\r", 4) == 0) {
              GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET);
            }
          if (strncmp(RX_BUF, "8\r", 4) == 0) {
              GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);
            }
          if (strncmp(RX_BUF, "7\r", 4) == 0) {
              sprintf(buffer, "\r\n\r\nint %d. float %d. double %d u8 %d u16 %d u32 %d\r\n\r",
                      (int)sizeof(int), (int)sizeof(float), (int)sizeof(double),
                      (int)sizeof(u8), (int)sizeof(u16), (int)sizeof(u32));
              USARTSend(buffer);
            }
          if (strncmp(RX_BUF, "6\r", 4) == 0) {
              int res003 = DHT11_read(&dev001);
              delay_ms(100);
              sprintf(cifry, "%d\r\n", res003);
              USARTSend(cifry);
              delay_ms(100);
              sprintf(cifry, "%d\r\n", dev001.temparature);
              USARTSend(cifry);
              delay_ms(100);
              sprintf(cifry, "%d\r\n", dev001.humidity);
              USARTSend(cifry);
              delay_ms(100);
            }

          clear_RXBuffer();
        }

    }
}
