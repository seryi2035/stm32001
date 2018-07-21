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
//#include "libmodbus.h"
#include "modbus.h"

void atSTART(void);
struct DHT11_Dev dev001;

int main(void) {
  uint32_t RTC_Counter01 = 0;
  uint32_t RTC_Counter02 = 0;
  uint32_t RTC_Counter03 = 0;
  u8 n = 0;

  //uint16_t res003;
  RTC_DateTimeTypeDef RTC_DateTime;
  SET_PAR[0] = 10; //адрес этого устройства 10 (modbus) 1-247

  GETonGPIO();
  TIM2_init(); // мс 0-49999 TIM2->CNT/2 25sec
  TIM3_init();
  TIM4_init(); // мкс 0-49999 TIM4->CNT
  usart1_init(); //A9 PP RXD A10 TXD жёлый //RS232 A11 ResetBits //485     //USART 1 and GPIO A (9/10/11) ON A11pp
  OW_Init(); //usart2 А2 А3
  dev001.port = GPIOA;
  dev001.pin = GPIO_Pin_12;
  dev001.humidity = 0;
  dev001.temparature = 0;
  DHT11_init(&dev001, dev001.port, dev001.pin);
  GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1 GDN set!
  uart1.delay=150; //modbus gap 9600
  uart1.rxtimer = 0;
  delay_ms(1000);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);   // C13 -- 0 VCC
  atSTART();
  oprosite();

  if (RTC_Init() == 1) {
      // Если первая инициализация RTC устанавливаем начальную дату, например 22.09.2016 14:30:00
      RTC_DateTime.RTC_Date = 21;
      RTC_DateTime.RTC_Month = 7;
      RTC_DateTime.RTC_Year = 2018;

      RTC_DateTime.RTC_Hours = 14;
      RTC_DateTime.RTC_Minutes = 8;
      RTC_DateTime.RTC_Seconds = 30;
      //После инициализации требуется задержка. Без нее время не устанавливается.
      delay_ms(500);
      RTC_SetCounter(RTC_GetRTC_Counter(&RTC_DateTime));
    }
  iwdg_init();

  while (1) {
      if (Coils_RW[8] == 0) {
          IWDG_ReloadCounter();
        }
      if(uart1.rxgap==1) {
          //delay_ms(1);
          //GPIO_ResetBits(GPIOC, GPIO_Pin_13);   // C13 -- 0 VCC
          GPIO_SetBits(USART1PPport, USART1PPpin);
          MODBUS_SLAVE(&uart1);
          net_tx1(&uart1);
          //delay_ms(1);
          GPIO_ResetBits(USART1PPport, USART1PPpin);
          //GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1 GDN set!
          //USARTSend("\n\rREADY!!!\n\r");
          //delay_ms(50);
        }
      if ( ((RTC_Counter02 = RTC_GetCounter()) - RTC_Counter01) >= 4) {
          RTC_Counter01 = RTC_Counter02;
          //read_Coils_RW();
          //setCOILS(Coils_RW);
          ds18b20Value = schitatU16Temp("\x28\xee\x6c\x08\x1a\x16\x01\x30");
          input_reg.tmp_u16[3] = ds18b20Value >> 4;                    //Number STM10DS001 "DS001Temperature [%d °C]"   (smt32modbus10RO)     {modbus="<[slave10_4:3]"}
          input_reg.tmp_float[9] = (float) (ds18b20Value / 16.0);     //Number STM10DS01f "DS01 floatTemp [%.2f °C]"   (smt32modbus10RO)     {modbus="<[slave10_402:1]"}
          hold_reg.tmp_float[1] = (float) (ds18b20Value / 16.0);           //Number STMfloatDS001 "DS001Temperature [%.2f °C]"   (smt32modbus10RW)     {modbus="slave10_3:1"}
          ds18b20Value = schitatU16Temp("\x28\xee\x09\x03\x1a\x16\x01\x67");
          input_reg.tmp_u16[4] = ds18b20Value >> 4;                    //Number STM10DS002 "DS002Temperature [%d °C]"   (smt32modbus10RO)     {modbus="<[slave10_4:4]"}
          input_reg.tmp_float[10] = (float) (ds18b20Value / 16.0);     //Number STM10DS02f "DS02 floatTemp [%.2f °C]"   (smt32modbus10RO)     {modbus="<[slave10_402:2]"}
          hold_reg.tmp_float[2] = (float) (ds18b20Value / 16.0);           //Number STMfloatDS002 "DS002Temperature [%.2f °C]"   (smt32modbus10RW)     {modbus="slave10_3:2"}
          input_reg.tmp_u16[2] = DHT11_read(&dev001);                  //Number STM10DHTres "DHTstatus [%d]"            (smt32modbus10RO)     {modbus="<[slave10_4:2]"}
          hold_reg.tmp_float[3] = (float) RTC_Counter01;                   //Number STMfloatcount "STM10countRW [%.2f]"          (smt32modbus10RW)     {modbus="<[slave10_3:3], >[slave10_3:4]"}
          if (input_reg.tmp_u16[2] == DHT11_SUCCESS) {
              input_reg.tmp_u16[0] = dev001.humidity;                  //Number STM10DHThum "humidity [%d %%]"          (smt32modbus10RO)     {modbus="<[slave10_4:0]"}
              input_reg.tmp_u16[1] = dev001.temparature;               //Number STM10DHTtemp "DHTtemp [%d °C]"          (smt32modbus10RO)     {modbus="<[slave10_4:1]"}
            }
          if ( (RTC_Counter02 - RTC_Counter03) >= 60) {
              n++;
              if (n > 6) {
                  if ((hold_reg.tmp_u16[24] - hold_reg.tmp_u16[25]) > 5) {
                      Coils_RW[8] = 1;
                    }
                }else if (n > 100) {
                  n = 0;
                }
              hold_reg.tmp_u16[24] = hold_reg.tmp_u16[24] + 1;
              RTC_Counter03 = RTC_Counter02;
              RTC_GetDateTime(RTC_Counter01, &RTC_DateTime);
              input_reg.tmp_u16[5] = RTC_DateTime.RTC_Hours;          //Number STM10hour   "hour [%d]"                 (smt32modbus10RO)     {modbus="<[slave10_4:5]"}
              input_reg.tmp_u16[6] = RTC_DateTime.RTC_Minutes;        //Number STM10minute   "minute [:%d]"            (smt32modbus10RO)     {modbus="<[slave10_4:6]"}
              input_reg.tmp_u16[7] = RTC_DateTime.RTC_Seconds;        //Number STM10second  "seconds [:%d]"            (smt32modbus10RO)     {modbus="<[slave10_4:7]"}
              input_reg.tmp_u16[8] = RTC_DateTime.RTC_Year;           //Number STM10year  "year [%d]"                  (smt32modbus10RO)     {modbus="<[slave10_4:8]"}
              input_reg.tmp_u16[9] = RTC_DateTime.RTC_Month;          //Number STM10month  "month [%d]"                (smt32modbus10RO)     {modbus="<[slave10_4:9]"}
              input_reg.tmp_u16[10] = RTC_DateTime.RTC_Date;          //Number STM10date  "date [%d]"                  (smt32modbus10RO)     {modbus="<[slave10_4:10]"}
            }
          input_reg.tmp_float[11] = (float) RTC_Counter01;             //Number STM10count "count [%.1f ]"              (smt32modbus10RO)     {modbus="<[slave10_402:3]"}
          //Number STMfcountppRW "STM10countppRW [%.2f]"        (smt32modbus10RW)     {modbus=">[slave10_3:5]"}
          input_reg.tmp_float[12] = hold_reg.tmp_float[5];             //Number STM10countfPPro "countfPPro [%.1f]"     (smt32modbus10RO)     {modbus="<[slave10_402:4]"}
          input_reg.tmp_u16[11] = hold_reg.tmp_u16[27];                //Number STM10countPPRO  "ROcountPP [%d]"        (smt32modbus10RO)     {modbus="<[slave10_4:11]"}
          //Number STMucountPP   "STM10fcountRWPP [%d]"         (smt32modbus10RW)     {modbus="slave10_302:2"}

          //Number STMucountprov "STM10countRWprov [%d]"        (smt32modbus10RW)     {modbus="<[slave10_302:0], >[slave10_302:1]"}
          //Number STMucountprov1 "STM10countRWprov1 [%d]"       (smt32modbus10RW)     {modbus="slave10_302:1"}
          hold_reg.tmp_u16[26] = hold_reg.tmp_u16[27];                 //prov2
          //hold_reg.tmp_u16[26] = (u16) hold_reg.tmp_u16[24] + 1;
          oprosite();
          if (Coils_RW[9] != 0) {
              if (input_reg.tmp_i16[11] > 0) {
                  RTC_Counter02 = RTC_Counter02 + ((uint32_t)input_reg.tmp_i16[11]);
                } else {
                  input_reg.tmp_i16[11] = input_reg.tmp_i16[11] * (-1);
                  RTC_Counter02 = RTC_Counter02 + ((uint32_t)input_reg.tmp_i16[11]);
                }
              RTC_SetCounter(RTC_Counter02);
              //res_ftable[5] = 0;
              Coils_RW[9] = 0;
            }
        }
      /*if ( ((RTC_Counter02 = RTC_GetCounter()) - RTC_Counter01) >= 4) {
            RTC_Counter01 = RTC_Counter02;
            oprosite ();
            //res_ftable[1] = schitatfTemp("\x28\xee\xcd\xa9\x19\x16\x01\x0c");
            res_ftable[1] = schitatfTemp("\x28\xee\x6c\x08\x1a\x16\x01\x30");
            res_ftable[2] = schitatfTemp("\x28\xee\x09\x03\x1a\x16\x01\x67");
            //res_table[3] = schitatiTemp("\x28\xee\xcd\xa9\x19\x16\x01\x0c");
            res_table[3] = schitatiTemp("\x28\xee\x6c\x08\x1a\x16\x01\x30");
            res_table[4] = schitatiTemp("\x28\xee\x09\x03\x1a\x16\x01\x67");
            fResult = res_ftable[1];
            sprintf(buffer, "T01F %d.%d  ",(int) fResult, (int) (0.0625*1000)*(((int) (fResult * 16)) % 16));
            USARTSend(buffer);
            USARTSend(buffer);
            fResult = res_ftable[2];
            sprintf(buffer, "T02F %d.%d  ",(int) fResult, (int) (0.0625*1000)*(((int) (fResult * 16)) % 16));
            USARTSend(buffer);
            sprintf(buffer, "T01 :%d  ", res_table[3]);
            USARTSend(buffer);
            sprintf(buffer, "T02 :%d  ", res_table[4]);
            USARTSend(buffer);
            res003 = DHT11_read(&dev001);
            delay_ms(1);
            sprintf(buffer, "res :%d  ", res003);
            USARTSend(buffer);
            //USARTSend(buffer);
            sprintf(buffer, "temp :%d  ", dev001.temparature);
            USARTSend(buffer);
            sprintf(buffer, "hum :%d \n \n", dev001.humidity);
            USARTSend(buffer);
            RTC_GetDateTime(RTC_Counter01, &RTC_DateTime);
            sprintf(buffer, " %d:%d:%d\r\n",

                    (int)RTC_DateTime.RTC_Hours, (int)RTC_DateTime.RTC_Minutes, (int)RTC_DateTime.RTC_Seconds);
            USARTSend(buffer);
            sprintf(buffer, "rxtimer:%d  rxcnt:%d  txcnt:%d  txlen:%d  rxgap:%d  delay:%d\r\n",
                    uart1.rxtimer, uart1.rxcnt, uart1.txcnt,
                uart1.txlen, uart1.rxgap, uart1.delay);
            USARTSend(buffer);
            for(u8 i001 = 0; i001 < 40;i001++) {
                vvhex(uart1.buffer[i001]);
                if (i001 > 3 && (i001 +1) % 8 == 0) {USARTSend("\n"); } else { USARTSend(" ");}
                //USARTSend(" ");
                //buffer[i001] = uart1.buffer[i001];
              }
            //USARTSend(buffer);
            USARTSend(" end \n");
          }*/
      /* if (RX_FLAG_END_LINE == 1) {
          // Reset RX_Flag end line
          RX_FLAG_END_LINE = 0;

          USARTSend("\n\rI has received a line:\n\r");
          USARTSend(RX_BUF);
          USARTSend("\n\r");

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
              RTC_Counter01 = RTC_GetCounter();
              sprintf(buffer, "COUNTER: %d\r\n", (int)RTC_Counter01);
              USARTSend(buffer);
              RTC_GetDateTime(RTC_Counter01, &RTC_DateTime);
              sprintf(buffer, "%d.%d.%d  %d:%d:%d\r\n",
                      (int)RTC_DateTime.RTC_Date, (int)RTC_DateTime.RTC_Month, (int)RTC_DateTime.RTC_Year,
                      (int)RTC_DateTime.RTC_Hours, (int)RTC_DateTime.RTC_Minutes, (int)RTC_DateTime.RTC_Seconds);
              USARTSend(buffer);

              // Функция генерирует в буфере дату собственного формата
              RTC_GetMyFormat(&RTC_DateTime, buffer);
              USARTSend(buffer);
              USARTSend("\n\r");

            }
          if (strncmp(RX_BUF, "SCAN\r", 4) == 0 || strncmp(RX_BUF, "scan\r", 4) == 0 ) {
              for(int i = 0;i < RX_BUF_SIZE - 1; i++) RX_BUF08[i] = (u8) RX_BUF[i];
              OW_Scan(RX_BUF08, 1);
              for(int i = 0;i < RX_BUF_SIZE - 1; i++) RX_BUF[i] = (char) RX_BUF08[i];
              //char cifry[10];
              sendaddrow();
            }
          if (strncmp(RX_BUF, "0\r", 4) == 0) {
              schitatTemp("\x28\xee\x6c\x08\x1a\x16\x01\x30");
              //schitatTemp("\x28\xee\xcd\xa9\x19\x16\x01\x0c");
              //iResult = schitatiTemp("\x28\xee\x6c\x08\x1a\x16\x01\x30");
              sprintf(cifry, "int %d\r\n", schitatiTemp("\x28\xee\x6c\x08\x1a\x16\x01\x30"));
              USARTSend(cifry);
              fResult = schitatfTemp("\x28\xee\x6c\x08\x1a\x16\x01\x30");
              sprintf(cifry, "float %d.%d\r\n",(int) fResult, (int) (0.0625*1000)*(((int) (fResult * 16)) % 16));
              USARTSend(cifry);
            }
          if (strncmp(RX_BUF, "1\r", 4) == 0) {
              schitatTemp("\x28\xee\x09\x03\x1a\x16\x01\x67");
            }
          if (strncmp(RX_BUF, "5\r", 4) == 0) {
              oprosite ();
              USARTSend("oprosheno\n\r");
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
        }*/
    }
}

void atSTART(void) {
  coilFROMback(); //######################################## coilFROMback();coilFROMback();coilFROMback();
  Coils_RW[8] = 0;
  setCOILS(Coils_RW);
  for(u8 i = 0; i < OBJ_SZ; i++) {
      input_reg.tmp_u32[i] = 0;
      hold_reg.tmp_u32[i] = 0;
    }
}

