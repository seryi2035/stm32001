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
void COILtimerMINUTES (uint8_t coilSETED, uint16_t inREGcount, uint16_t inREGbkp, uint16_t holdREGtimer, uint16_t holdREGbkp);


int main(void) {
  uint32_t RTC_Counter01 = 0;
  uint32_t RTC_Counter02 = 0;
  uint32_t RTC_Counter03 = 0;
  u8 n = 0;
  millisec2 = 0;
  // Включить тактирование модулей управления питанием и управлением резервной областью
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  // Разрешить доступ к области резервных данных
  PWR_BackupAccessCmd(ENABLE);

  //uint16_t res003;
  SET_PAR[0] = 20; //адрес этого устройства 20 (modbus) 1-247

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

  uart1.delay=80; //modbus gap 9600
  uart1.rxtimer = 0;

  atSTART();
  oprosite();
  GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1 GDN set!
  delay_ms(1000);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);   // C13 -- 0 VCC
  iwdg_init();

  while (1) {

      if (Coils_RW[8] == 0) {
          IWDG_ReloadCounter();
        }
      if(uart1.rxgap==1) {

          GPIO_SetBits(USART1PPport, USART1PPpin);
          MODBUS_SLAVE(&uart1);
          //net_tx1(&uart1);

          GPIO_ResetBits(USART1PPport, USART1PPpin);

        }
      if ( ((RTC_Counter02 = GETglobalsecs()) - RTC_Counter01) >= 4) {
          RTC_Counter01 = RTC_Counter02;
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
              input_reg.tmp_u16[2] = (RTC_Counter02 / 3600) % 24;   //Number STM20hour   "hour [%d]"                 (gmod20_INreg)     {modbus="<[slave20_4:2]"}
              input_reg.tmp_u16[1] = (RTC_Counter02 / 60) % 60;     //Number STM20minute   "minute [:%d]"            (gmod20_INreg)     {modbus="<[slave20_4:1]"}
              input_reg.tmp_u16[0] = RTC_Counter02 % 60;            //Number STM20second  "seconds [:%d]"            (gmod20_INreg)     {modbus="<[slave20_4:0]"}
              input_reg.tmp_u16[3] = (RTC_Counter02 / (3600 * 24)); //Number STM20date  "date [%d]"                  (gmod20_INreg)     {modbus="<[slave20_4:3]"}
              COILtimerMINUTES(Coils_RW[1], input_reg.tmp_u16[12], BKP_DR5, hold_reg.tmp_u16[28], BKP_DR9);   //B11     slave20_403:4       slave20_302:4
              COILtimerMINUTES(Coils_RW[2], input_reg.tmp_u16[13], BKP_DR6, hold_reg.tmp_u16[29], BKP_DR10);  //B10     slave20_403:5       slave20_302:5
              COILtimerMINUTES(Coils_RW[3], input_reg.tmp_u16[14], BKP_DR7, hold_reg.tmp_u16[30], BKP_DR11);  //B1      slave20_403:6       slave20_302:6
              COILtimerMINUTES(Coils_RW[4], input_reg.tmp_u16[15], BKP_DR8, hold_reg.tmp_u16[31], BKP_DR12);  //B0      slave20_403:7       slave20_302:7
            }
          ds18b20Value = schitatU16Temp("\x28\xee\xe8\x19\x17\x16\x02\xa1");
          input_reg.tmp_float[9] = (float) (ds18b20Value / 16.0);   //Number STM20DS03f "DS01 floatTemp [%.2f °C]"   (gmod20_INreg)     {modbus="<[slave20_402:1]"}
          input_reg.tmp_u16[4] = DHT11_read(&dev001);               //Number STM20DHTres "DHTstatus [%d]"            (gmod20_INreg)     {modbus="<[slave20_4:4]"}
          if (input_reg.tmp_u16[4] == DHT11_SUCCESS) {
              input_reg.tmp_u16[5] = dev001.humidity;               //Number STM20DHThum "humidity [%d %%]"          (gmod20_INreg)     {modbus="<[slave20_4:5]"}
              input_reg.tmp_float[8] = ((float)dev001.temparature + (0.1 * dev001.pointtemparature) );
              //Number STM20DHTtemp "DHTtemp [%.1f °C]"  (gmod20_INreg)     {modbus="<[slave20_402:0]"}
            }
          input_reg.tmp_u16[11] = hold_reg.tmp_u16[27];             //Number STM20countPPRO  "ROcountPP [%d]"        (gmod20_INreg)     {modbus="<[slave20_4:11]"}
          hold_reg.tmp_u16[26] = hold_reg.tmp_u16[25];              //prov2
          input_reg.tmp_float[11] = (float) RTC_Counter01;          //Number STM20count "count [%.1f ]"              (gmod20_INreg)     {modbus="<[slave20_402:3]"}

          oprosite();
          if (Coils_RW[9] != 0) {
              if (input_reg.tmp_i16[11] > 0) {
                  RTC_Counter02 = RTC_Counter02 + ((uint32_t)input_reg.tmp_i16[7]);
                } else {
                  input_reg.tmp_i16[11] = input_reg.tmp_i16[11] * (-1);
                  RTC_Counter02 = RTC_Counter02 + ((uint32_t)input_reg.tmp_i16[7]);
                }
              SETglobalsecs(RTC_Counter02);
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
  /*for(u8 i = 0; i < OBJ_SZ; i++) {
      input_reg.tmp_u32[i] = 0;
      hold_reg.tmp_u32[i] = 0;
    }*/
  hold_reg.tmp_u16[28] = BKP_ReadBackupRegister(BKP_DR9);
  hold_reg.tmp_u16[29] = BKP_ReadBackupRegister(BKP_DR10);
  hold_reg.tmp_u16[30] = BKP_ReadBackupRegister(BKP_DR11);
  hold_reg.tmp_u16[31] = BKP_ReadBackupRegister(BKP_DR12);
}

void COILtimerMINUTES (uint8_t coilSETED, uint16_t inREGcount,uint16_t inREGbkp, uint16_t holdREGtimer ,uint16_t holdREGbkp) {
  inREGcount = BKP_ReadBackupRegister(inREGbkp);

  if ( coilSETED != 0) {
      inREGcount--;
    } else {
      inREGcount = holdREGtimer;
    }
  if (inREGcount < 1) {
      coilSETED = 0;
    }
  BKP_WriteBackupRegister(inREGbkp, inREGcount);
  BKP_WriteBackupRegister(holdREGbkp, holdREGtimer);
}
