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
  dev001.pointtemparature = 0;
  DHT11_init(&dev001, dev001.port, dev001.pin);
  GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1 GDN set!
  uart1.delay=150; //modbus gap 9600
  uart1.rxtimer = 0;
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
  delay_ms(1000);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);   // C13 -- 0 VCC
  atSTART();
  //oprosite();



  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

  // Allow access to BKP Domain
  PWR_BackupAccessCmd(ENABLE);

  if (RTC_GetCounter() <= 10000) {
      RTCInit();
      // Если первая инициализация RTC устанавливаем начальную дату, например 22.09.2016 14:30:00
      RTC_DateTime.RTC_Date = 26;
      RTC_DateTime.RTC_Month = 1;
      RTC_DateTime.RTC_Year = 2019;

      RTC_DateTime.RTC_Hours = 12;
      RTC_DateTime.RTC_Minutes = 3;
      RTC_DateTime.RTC_Seconds = 51;
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
          //ds18b20Value = schitatU16Temp("\x28\xee\x09\x03\x1a\x16\x01\x67");
          //input_reg.tmp_u16[4] = ds18b20Value >> 4;                    //Number STM10DS002 "DS002Temperature [%d °C]"   (smt32modbus10RO)     {modbus="<[slave10_4:4]"}
          //input_reg.tmp_float[10] = (float) (ds18b20Value / 16.0);     //Number STM10DS02f "DS02 floatTemp [%.2f °C]"   (smt32modbus10RO)     {modbus="<[slave10_402:2]"}
          //hold_reg.tmp_float[2] = (float) (ds18b20Value / 16.0);           //Number STMfloatDS002 "DS002Temperature [%.2f °C]"   (smt32modbus10RW)     {modbus="slave10_3:2"}
          input_reg.tmp_u16[2] = DHT11_read(&dev001);                  //Number STM10DHTres "DHTstatus [%d]"            (smt32modbus10RO)     {modbus="<[slave10_4:2]"}
          hold_reg.tmp_float[3] = (float) RTC_Counter01;                   //Number STMfloatcount "STM10countRW [%.2f]"          (smt32modbus10RW)     {modbus="<[slave10_3:3], >[slave10_3:4]"}
          if (input_reg.tmp_u16[2] == DHT11_SUCCESS) {
              input_reg.tmp_u16[0] = dev001.humidity;                  //Number STM10DHThum "humidity [%d %%]"          (smt32modbus10RO)     {modbus="<[slave10_4:0]"}
              input_reg.tmp_u16[1] = dev001.temparature;               //Number STM10DHTtemp "DHTtemp [%d °C]"          (smt32modbus10RO)     {modbus="<[slave10_4:1]"}
              input_reg.tmp_u16[4] = dev001.temparature;
              hold_reg.tmp_float[2] = ((float)dev001.temparature + (0.1 * dev001.pointtemparature) );
              input_reg.tmp_float[10] = ((float)dev001.temparature + (0.1 * dev001.pointtemparature) );
            }
          if ( (RTC_Counter02 - RTC_Counter03) >= 60) {
              n++;
              if (n > 6) {
                  if ((hold_reg.tmp_u16[24] - hold_reg.tmp_u16[25]) > 10) {
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
          hold_reg.tmp_u16[26] = hold_reg.tmp_u16[25];                 //prov2
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

