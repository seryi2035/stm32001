#include "001.h"
#include "onewire.h"
#include "tim2_delay.h"
#include "string.h"
#include "stdio.h"

void GETonGPIO() {
  GPIO_InitTypeDef GPIO_InitStructure;
  //LED C.13
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  // A7 PP
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //A6 PP
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);



  //KNOPKA B0
  /*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //KNOPKA B1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);*/
}

void usart1_init(void) {
  //USART 1 and GPIO A 9 10 ON A11pp
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  //NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  //
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //GPIO
  GPIO_InitTypeDef GPIO_InitStructure;
  //  A 9 TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //  A 10 RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // A11 PP
  GPIO_InitStructure.GPIO_Pin = USART1PPpin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART1PPport, &GPIO_InitStructure);
  //  USART 1
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init (USART1, &USART_InitStructure);

  USART_Cmd(USART1, ENABLE);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  //GDN on A11
  //GPIO_SetBits(USART1PPport, USART1PPpin);
  GPIO_ResetBits(USART1PPport, USART1PPpin);
}
void USART1_IRQHandler(void) {
  /*if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET) {
      RXc =(char) USART_ReceiveData(USART1);
      RX_BUF[RXi] = RXc;
      RXi++;
      RX_FLAG_END_LINE = 0;
      if (RXc != 13) {
          if (RXi > RX_BUF_SIZE - 1) {
              clear_RXBuffer();
            }
        } else {
          RX_FLAG_END_LINE = 1;
        }
      //Echo
      USART_SendData(USART1,(u16) RXc);
    }*/
  //Receive Data register not empty interrupt
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  {
      USART_ClearITPendingBit(USART1, USART_IT_RXNE); //очистка признака прерывания
      uart1.rxtimer = 0;
      if(uart1.rxcnt > (BUF_SZ-2)) {
          uart1.rxcnt=0;
        }
      uart1.buffer[uart1.rxcnt++]=USART_ReceiveData (USART1);
    }
  //Transmission complete interrupt
  if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)  {
      USART_ClearITPendingBit(USART1, USART_IT_TC);//очистка признака прерывания
      if(uart1.txcnt<uart1.txlen)  {
          USART_SendData(USART1,uart1.buffer[uart1.txcnt++]);//Передаем
        }
      else {
          //посылка закончилась и мы снимаем высокий уровень сRS485 TXE
          uart1.txlen=0;
          GPIO_WriteBit(USART1PPport, USART1PPpin,Bit_RESET);
          USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
          USART_ITConfig(USART1, USART_IT_TC, DISABLE);
          TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
        }
    }
}
void clear_RXBuffer(void) {
  for (RXi = 0; RXi < RX_BUF_SIZE; RXi++)
    RX_BUF[RXi] = '\0';
  RXi = 0;
}
void USART1Send(char *pucBuffer) {
  while (*pucBuffer) {
      USART_SendData(USART1,(uint16_t) *pucBuffer++);
      while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
void USART1Send485(char *pucBuffer) {
  GPIO_SetBits(USART1PPport, USART1PPpin);
  //GPIO_ResetBits(USART1PPport, USART1PPpin);
  delay_ms(2);
  USART1Send(pucBuffer);
  delay_ms(2);
  //GPIO_SetBits(USART1PPport, USART1PPpin);
  GPIO_ResetBits(USART1PPport, USART1PPpin);
}
unsigned char RTC_Init(void) {
  // Включить тактирование модулей управления питанием и управлением резервной областью
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  // Разрешить доступ к области резервных данных
  PWR_BackupAccessCmd(ENABLE);
  // Если RTC выключен - инициализировать
  if((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN) {
      // Сброс данных в резервной области
      RCC_BackupResetCmd(ENABLE);
      RCC_BackupResetCmd(DISABLE);

      // Установить источник тактирования кварц 32768
      RCC_LSEConfig(RCC_LSE_ON);
      while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY){}
      RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

      RTC_SetPrescaler(0x7FFF); // Устанавливаем делитель, чтобы часы считали секунды
      // Включаем RTC
      RCC_RTCCLKCmd(ENABLE);
      // Ждем синхронизацию
      RTC_WaitForSynchro();

      return 1;
    }
  return 0;
}

void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct) { //get cirrent date
  unsigned long time;
  unsigned long t1, a, b, c, d, e, m;
  uint16_t year = 0;
  uint8_t mon = 0;
  uint8_t wday = 0;
  uint8_t mday = 0;
  uint8_t hour = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  uint64_t jd = 0;
  uint64_t jdn = 0;

  jd = ((RTC_Counter+43200)/(86400>>1)) + (2440587<<1) + 1;
  jdn = jd>>1;

  time = RTC_Counter;
  t1 = time / 60;
  sec =(uint8_t) (time - t1 * 60);

  time = t1;
  t1 = time / 60;
  min =(uint8_t) (time - t1 * 60);

  time = t1;
  t1 = time / 24;
  hour =(uint8_t) (time - t1 * 24);

  wday =(uint8_t) ( jdn%7);

  a =(unsigned long) (jdn + 32044);
  b = (4 * a + 3) / 146097;
  c = a - (146097 * b) / 4;
  d = (4 * c + 3) / 1461;
  e = c - (1461 * d) / 4;
  m = (5 * e + 2) / 153;
  mday =(uint8_t) ( e - (153 * m + 2) / 5 + 1);
  mon =(uint8_t) ( m + 3 - 12 * (m / 10));
  year =(uint16_t) (100 * b + d - 4800 + (m / 10));

  RTC_DateTimeStruct->RTC_Year = year;
  RTC_DateTimeStruct->RTC_Month = mon;
  RTC_DateTimeStruct->RTC_Date = mday;
  RTC_DateTimeStruct->RTC_Hours = hour;
  RTC_DateTimeStruct->RTC_Minutes = min;
  RTC_DateTimeStruct->RTC_Seconds = sec;
  RTC_DateTimeStruct->RTC_Wday = wday;
}
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct) {  // Convert Date to Counter
  uint32_t a;
  uint32_t y;
  uint32_t m;
  uint32_t JDN;

  a =(uint32_t) ( (14 - RTC_DateTimeStruct->RTC_Month) / 12);
  y =(uint32_t) ( RTC_DateTimeStruct->RTC_Year + 4800 - (u8)a);
  m =(uint32_t) ( RTC_DateTimeStruct->RTC_Month + (12 * a) - 3);

  JDN = RTC_DateTimeStruct->RTC_Date;
  JDN += (153 * m + 2) / 5;
  JDN += 365 * y;
  JDN += y / 4;
  JDN += -y / 100;
  JDN += y / 400;
  JDN = JDN - 32045;
  JDN = JDN - JULIAN_DATE_BASE;
  JDN *= 86400;
  JDN +=(uint32_t) (RTC_DateTimeStruct->RTC_Hours * 3600);
  JDN +=(uint32_t) (RTC_DateTimeStruct->RTC_Minutes * 60);
  JDN +=(uint32_t) (RTC_DateTimeStruct->RTC_Seconds);

  return JDN;
}
void RTC_GetMyFormat(RTC_DateTimeTypeDef* RTC_DateTimeStruct, char *  buffer01) {
  const char WDAY0[] = "Monday";
  const char WDAY1[] = "Tuesday";
  const char WDAY2[] = "Wednesday";
  const char WDAY3[] = "Thursday";
  const char WDAY4[] = "Friday";
  const char WDAY5[] = "Saturday";
  const char WDAY6[] = "Sunday";
  const char * WDAY[7]={WDAY0, WDAY1, WDAY2, WDAY3, WDAY4, WDAY5, WDAY6};

  const char MONTH1[] = "January";
  const char MONTH2[] = "February";
  const char MONTH3[] = "March";
  const char MONTH4[] = "April";
  const char MONTH5[] = "May";
  const char MONTH6[] = "June";
  const char MONTH7[] = "July";
  const char MONTH8[] = "August";
  const char MONTH9[] = "September";
  const char MONTH10[] = "October";
  const char MONTH11[] = "November";
  const char MONTH12[] = "December";
  const char * MONTH[12]={MONTH1, MONTH2, MONTH3, MONTH4, MONTH5,
                          MONTH6, MONTH7, MONTH8, MONTH9, MONTH10, MONTH11, MONTH12};

  sprintf(buffer01, "%s %d %s %04d",
          WDAY[RTC_DateTimeStruct->RTC_Wday],
      RTC_DateTimeStruct->RTC_Date,
      MONTH[RTC_DateTimeStruct->RTC_Month -1],
      RTC_DateTimeStruct->RTC_Year);
}

char get_ab_xFF(int a){
  char ff;
  switch (a) {
    case 0:
      ff = '0';
      break;
    case 1:
      ff = '1';
      break;
    case 2:
      ff = '2';
      break;
    case 3:
      ff = '3';
      break;
    case 4:
      ff = '4';
      break;
    case 5:
      ff = '5';
      break;
    case 6:
      ff = '6';
      break;
    case 7:
      ff = '7';
      break;
    case 8:
      ff = '8';
      break;
    case 9:
      ff = '9';
      break;
    case 10:
      ff = 'a';
      break;
    case 11:
      ff = 'b';
      break;
    case 12:
      ff = 'c';
      break;
    case 13:
      ff = 'd';
      break;
    case 14:
      ff = 'e';
      break;
    case 15:
      ff = 'f';
      break;

    default:
      ff = 'X';
      break;
    }
  return ff;
}
u8 convT_DS18B20(u8 LSB, u8 MSB)
{
  LSB >>= 4; // убираем дробную часть
  MSB = (u8) (MSB * 16); // убираем лишние знаки
  return(MSB | LSB); // объединяем 2 байта -> возврат
}
void schitatTemp(char* imya) {
  //-----------------------------------------------------------------------------
  // процедура общения с шиной 1-wire
  // sendReset - посылать RESET в начале общения.
  // 		OW_SEND_RESET или OW_NO_RESET
  // command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOT
  // cLen - длина буфера команд, столько байт отошлется в шину
  // data - если требуется чтение, то ссылка на буфер для чтения
  // dLen - длина буфера для чтения. Прочитается не более этой длины
  // readStart - с какого символа передачи начинать чтение (нумеруются с 0)
  //		можно указать OW_NO_READ, тогда можно не задавать data и dLen
  //-----------------------------------------------------------------------------
  //OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart)
  //OW_Send(OW_SEND_RESET, '\x28\xEE\x09\x03\x1A\x16\x01\x67\x88\xbe\xff\xff", 12, RX_BUF, 2, 10);
  uint8_t buf[2];
  //OW_Send(OW_SEND_RESET, "\xcc\xbe\xff\xff", 4, buf,2, 2);
  u8 command01[12] = {(u8)'\x55',(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],(u8) imya[4],(u8) imya[5],
                      (u8) imya[6],(u8) imya[7],(u8)'\xbe',(u8) '\xff',(u8) '\xff'};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  //USARTSend("\n\rTHIS IS 000\n\r");
  //USARTSend(buf);
  //USARTSend("\n\r");
  //int temp = ((buf[1] * 256) + buf[0]) * 16;
  char cifry[20];
  vvhex((char)buf[1]);
  vvhex((char)buf[0]);
  USARTSend("\n\r");
  //USARTSend(imya);
  for(int i = 0; i <= 7; i++)
    vvhex(imya[i]);
  USARTSend("\n\r");
  int temp = convT_DS18B20(buf[0], buf[1]);
  sprintf(cifry, "termperature :%d.%d\r\n", temp, (int) ((125*(buf[0] % 16))/2 + buf[0] % 2));
  USARTSend(cifry);
  //sprintf(cifry, ".%d\r\n", (int) (0.0625*1000)*(buf[0] % 16));
  //USARTSend(cifry);
}
void vvhex(char vv) {
  int a, b;
  char ff[2];
  ff[1] =(char) '\0';
  ff[0] =(char) 'Z';
  a = vv / 16;
  b = vv % 16;
  ff[0] = get_ab_xFF(a);
  USARTSend(ff);
  ff[0] = get_ab_xFF(b);
  USARTSend(ff);
}

/*void usart3_init(void) { //ппц юсарт 3 к 1-ware те же GPIO B 10 11/ а не А2 А3
  //USART 3 and GPIO B 10 11 ON
  RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOB, ENABLE);

  //NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  //
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //GPIO
  GPIO_InitTypeDef GPIO_InitStructure;
  //B0 DE
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //B1 RE
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //  B 10 TX DI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //  B 11 RX RO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //  USART 3
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init (USART3, &USART_InitStructure);

  USART_Cmd(USART3, ENABLE);

  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  GPIO_SetBits(GPIOB, GPIO_Pin_0);
}*/
/*void USART3_IRQHandler(void) {
  if ((USART3->SR & USART_FLAG_RXNE) != (u16)RESET) {
      RXc =(char) USART_ReceiveData(USART3);
      RX_BUF[RXi] = RXc;
      RXi++;
      RX_FLAG_END_LINE = 0;
      if (RXc != 13) {
          if (RXi > RX_BUF_SIZE - 1) {
              clear_RXBuffer();
            }
        } else {
          RX_FLAG_END_LINE = 1;
        }
      //Echo
      USART_SendData(USART3, RXc);
    }
}*/
/*void USART3_IRQHandler(void)
{
  //Receive Data register not empty interrupt
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
      USART_ClearITPendingBit(USART3, USART_IT_RXNE);
      uart3.rxtimer=0;

      if(uart3.rxcnt>(BUF_SZ-2)) {
          uart3.rxcnt=0;
        }
      uart3.buffer[uart3.rxcnt++]=(unsigned char) USART_ReceiveData (USART3);
      //Transmission complete interrupt
      if(USART_GetITStatus(USART3, USART_IT_TC) != RESET)
        {

          USART_ClearITPendingBit(USART3, USART_IT_TC);
          if(uart3.txcnt<uart3.txlen)
            {
              USART_SendData(USART3,uart3.buffer[uart3.txcnt++]);
            }
          else
            {
              uart3.txlen=0;
              //rs485 tx disable
              //GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET);
              GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_RESET);
              GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_RESET);

              USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
              USART_ITConfig(USART3, USART_IT_TC, DISABLE);
            }
        }
    }
}*/
/*void USART3Send(char *pucBuffer) {
  //for RS 485;
  //GPIO_ResetBits(GPIOC, GPIO_Pin_13);
  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
  GPIO_SetBits(GPIOB, GPIO_Pin_1);

  delay_ms(2);
  //u16 a;
  while (*pucBuffer) {
      USART_SendData(USART3,(uint16_t) *pucBuffer++);
      while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
        {
        }
      //a = ((uint16_t) *pucBuffer++);
      //USART_SendData(USART3,a);
    }
  delay_ms(2);
  //GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1
  //GPIO_ResetBits(GPIOC, GPIO_Pin_13);   //C13 --0
  GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  GPIO_SetBits(GPIOB, GPIO_Pin_0);
  //GPIO_SetBits(GPIOC, GPIO_Pin_13);
  //delay_ms(500);
}*/
void sendaddrow (void) {
  for(int i=0; i < RX_BUF_SIZE && i < 20;i++) {
      if (RX_BUF[i] != 0) {
          int a, b;
          char ff[2];
          ff[1] =(char) '\0';
          ff[0] =(char) 'Z';
          a = RX_BUF[i] / 16;
          b = RX_BUF[i] % 16;
          ff[0] = get_ab_xFF(a);
          USARTSend(ff);
          ff[0] = get_ab_xFF(b);
          USARTSend(ff);
          /*sprintf(cifry, "%\xd", a);
          USARTSend(cifry);
          sprintf(cifry, "%d\r\n", b);
          USARTSend(cifry);*/
        }
      if ((i+1) % 8 == 0)
        USARTSend("\n\r");
      //USARTSend("\n\rTHIS IS 8\n\r");
    }
}
/*
u16 schitatTemp(char* imya) {
  //-----------------------------------------------------------------------------
  // процедура общения с шиной 1-wire
  // sendReset - посылать RESET в начале общения.
  // 		OW_SEND_RESET или OW_NO_RESET
  // command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOT
  // cLen - длина буфера команд, столько байт отошлется в шину
  // data - если требуется чтение, то ссылка на буфер для чтения
  // dLen - длина буфера для чтения. Прочитается не более этой длины
  // readStart - с какого символа передачи начинать чтение (нумеруются с 0)
  //		можно указать OW_NO_READ, тогда можно не задавать data и dLen
  //-----------------------------------------------------------------------------
  //OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart)
  //OW_Send(OW_SEND_RESET, "\x28\xEE\x09\x03\x1A\x16\x01\x67\x88\xbe\xff\xff", 12, RX_BUF, 2, 10);
  uint8_t buf[2];
  //OW_Send(OW_SEND_RESET, "\xcc\xbe\xff\xff", 4, buf,2, 2);
  //char command01[12] = {'\x55', imya[0], imya[1], imya[2], imya[3], imya[4],
    imya[5], imya[6], imya[7],'\xbe', '\xff', '\xff'};
  u8 command01[12] = {(u8)'\x55',(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8)
imya[3],(u8) imya[4],(u8) imya[5],(u8) imya[6],(u8) imya[7],(u8)'\xbe',(u8) '\xff',(u8) '\xff'};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  //USARTSend("\n\rTHIS IS 000\n\r");
  //USARTSend(buf);
  //USARTSend("\n\r");
  //int temp = ((buf[1] * 256) + buf[0]) * 16;
  char cifry[20];
  vvhex(buf[1]);
  vvhex(buf[0]);
  USARTSend("\n\r");
  //USARTSend(imya);
  for(int i = 0; i <= 7; i++)
    vvhex(imya[i]);
  USARTSend("\n\r");
  int temp = convT_DS18B20(buf[0], buf[1]);
  sprintf(cifry, "termperature :%d.%d\r\n", temp, (int) (0.0625*1000)*(buf[0] % 16));
  USARTSend(cifry);
  //sprintf(cifry, ".%d\r\n", (int) (0.0625*1000)*(buf[0] % 16));
  //USARTSend(cifry);
  retern  ;
}
*/

void MODBUS_SLAVE(UART_DATA *MODBUS) {
  unsigned int tmp;
  //recive and checking rx query
  if((MODBUS->buffer[0]!=0)&(MODBUS->rxcnt>5)& ((MODBUS->buffer[0]==SET_PAR[0])|(MODBUS->buffer[0]==255)))  {
      tmp=Crc16(MODBUS->buffer,MODBUS->rxcnt-2);
      if((MODBUS->buffer[MODBUS->rxcnt-2]==(tmp&0x00FF)) & (MODBUS->buffer[MODBUS->rxcnt-1]==(tmp>>8)))  {
          //если мы сюда попали значит пакет наш и crc совпало - надо проверить поддерживаем ли мы такой запрос
          //choosing function
          switch(MODBUS->buffer[1])
            {
            case 3:
              TX_03_04(MODBUS);
              break;
            case 4:
              TX_03_04(MODBUS);
              break;
            case 6:
              TX_06(MODBUS);
              break;
            case 66:
              TX_66(MODBUS);
              break;
            default://если нет то выдаем ошибку
              //illegal operation
              TX_EXCEPTION(MODBUS,0x01);
            }
          //добавляем CRC и готовим к отсылке
          //adding CRC16 to reply
          tmp=Crc16(MODBUS->buffer,MODBUS->txlen-2);
          MODBUS->buffer[MODBUS->txlen-2]=(unsigned char) tmp;
          MODBUS->buffer[MODBUS->txlen-1]=(unsigned char) (tmp>>8);
          MODBUS->txcnt=0;
        }
    }
  //сброс индикаторов окончания посылки
  MODBUS->rxgap=0;
  MODBUS->rxcnt=0;
  MODBUS->rxtimer=0xFFFF;
}
unsigned int Crc16(unsigned char *ptrByte, int byte_cnt) {
  unsigned int w=0;
  char shift_cnt;
  if(ptrByte)  {
      w=0xffffU;
      for(; byte_cnt>0; byte_cnt--) {
          w=(unsigned int)((w/256U)*256U+((w%256U)^(*ptrByte++)));
          for(shift_cnt=0; shift_cnt<8; shift_cnt++) {
              if((w&0x1)==1) {
                  w=(unsigned int)((w>>1)^0xa001U);
                }
              else {
                  w>>=1;
                }
            }
        }
    }
  return w;
}
void TX_03_04(UART_DATA *MODBUS)
{
  unsigned int tmp,tmp1;
  unsigned int m=0,n=0;
  int tmp_val,tmp_val_pos;

  //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
  //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
  //MODBUS->buffer[2] = data byte count

  //2-3  - starting address
  tmp=(unsigned int)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения

  //4-5 - number of registers
  tmp1=(unsigned int)((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);//количество регистров для чтения

  //default answer length if error
  n=3;

  //если нас устраивает длина запроса и и стартовый адрес
  if((((tmp+tmp1)<OBJ_SZ)&(tmp1<MODBUS_WRD_SZ+1)))
    {

      for(m=0;m<tmp1;m++)
        {
          tmp_val=res_table[m+tmp];//читаем текущее значение

          if(tmp_val<0)
            {
              //пакуем отрицательное
              tmp_val_pos=tmp_val;
              MODBUS->buffer[n]=(uint8_t) (tmp_val_pos>>8)|0b10000000;
              MODBUS->buffer[n+1]=(uint8_t) tmp_val_pos;
            }
          else
            {
              //пакуем положительное
              MODBUS->buffer[n]=(uint8_t) (tmp_val>>8);
              MODBUS->buffer[n+1]=(uint8_t) tmp_val;
            }
          n=n+2;
        }

      //запишем длину переменных пакета в байтах и вставим всообщение
      MODBUS->buffer[2]=(uint8_t) (m*2); //byte count
      //подготовим к отправке
      MODBUS->txlen=(uint8_t) (m*2+5); //responce length

    }
  else
    {
      //exception illegal data adress 0x02
      TX_EXCEPTION(MODBUS,0x02);
    }

}
void TX_06(UART_DATA *MODBUS)
{
  unsigned int tmp;

  //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
  //MODBUS[1] = 6; //query type - - stay a same as in recived

  //2-3  - adress   , 4-5 - value

  tmp=(unsigned int)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //adress

  //MODBUS->buffer[2]  - byte count a same as in rx query

  if(tmp<OBJ_SZ)
    {
      MODBUS->txlen=MODBUS->rxcnt; //responce length
      res_table[tmp]=(MODBUS->buffer[4]<<8)+MODBUS->buffer[5];
    }
  else
    {
      //illegal data
      TX_EXCEPTION(MODBUS,0x02) ;
    }

}
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type)
{
  //modbus exception - illegal data=01 ,adress=02 etc
  //illegal operation
  MODBUS->buffer[2]=error_type; //exception
  MODBUS->txlen=5; //responce length
}
float schitatfTemp(char* imya) {
  uint8_t buf[2];
  u8 command01[12] = { 0x55,(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],(u8) imya[4],
                       (u8) imya[5],(u8) imya[6],(u8) imya[7], 0xbe, 0xff, 0xff};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  float ftemp;
  ftemp = (float) ( (float) ((buf[1] << 8) | buf[0]) / 16.0);
  return ftemp;
}
int schitatiTemp(char* imya) {
  uint8_t buf[2];
  u8 command01[12] = { 0x55,(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],
                       (u8) imya[4],(u8) imya[5],(u8) imya[6],(u8) imya[7], 0xbe, 0xff, 0xff};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  //int itemp;
  //itemp = ((buf[1] << 8) | buf[0]) *1000 / 16;
  //delay_ms(10);
  return ((int) convT_DS18B20(buf[0], buf[1]));
}
void TX_66(UART_DATA *MODBUS)
{
  void oprosite (void);
  res_ftable[1] = schitatfTemp("\x28\xee\xcd\xa9\x19\x16\x01\x0c");
  res_ftable[2] = schitatfTemp("\x28\xee\x09\x03\x1a\x16\x01\x67");
  res_table[3] = schitatiTemp("\x28\xee\xcd\xa9\x19\x16\x01\x0c");
  res_table[4] = schitatiTemp("\x28\xee\x09\x03\x1a\x16\x01\x67");

  unsigned int tmp;

  //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
  //MODBUS[1] = 6; //query type - - stay a same as in recived

  //2-3  - adress   , 4-5 - value

  tmp=(unsigned int) ((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //adress

  //MODBUS->buffer[2]  - byte count a same as in rx query

  if(tmp<OBJ_SZ)
    {
      MODBUS->txlen=MODBUS->rxcnt; //responce length
      res_table[tmp]=(MODBUS->buffer[4]<<8)+MODBUS->buffer[5];
    }
  else
    {
      //illegal data
      TX_EXCEPTION(MODBUS,0x02) ;
    }

}
void oprosite(void) {
  u8 comm[2];
  comm[0] = 0xcc;
  comm[1] = 0x44;
  OW_Send(OW_SEND_RESET, comm, 2, NULL, 0, OW_NO_READ);
  delay_ms(100);
  comm[1] = 0x4e;
  OW_Send(OW_SEND_RESET, comm, 2, NULL, 0, OW_NO_READ);
  delay_ms(100);
  //USARTSend("oprosheno\n\r");
}
/*void net_tx3(UART_DATA *uart)
{
  if((uart->txlen>0)&(uart->txcnt==0))
    {
      USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
      USART_ITConfig(USART3, USART_IT_TC, ENABLE);

      GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET);
      GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET);

      USART_SendData(USART3, uart->buffer[uart->txcnt++]);
    }

}*/
void net_tx1(UART_DATA *uart)
{
  if((uart->txlen>0)&(uart->txcnt==0))
    {
      USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //выкл прерывание на прием
      USART_ITConfig(USART1, USART_IT_TC, ENABLE); //включаем на окочание передачи
      //включаем rs485 на передачу
      GPIO_WriteBit(USART1PPport,USART1PPpin,Bit_SET);
      USART_SendData(USART1, uart->buffer[uart->txcnt++]);
    }
}
// ////////////////////////////////////////////////////////DHT11
int DHT11_init(struct DHT11_Dev* dev, GPIO_TypeDef* port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStructure;

  dev->port = port;
  dev->pin = pin;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  //Initialise GPIO DHT11
  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(dev->port, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOA, dev->pin,Bit_SET);
  return 0;
}
int DHT11_read(struct DHT11_Dev* dev) {
  dev->temparature = 0;
  dev->humidity = 0;
  //Initialisation
  uint8_t i, j, temp;
  //uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t data[5] = {0, 0, 0, 0, 0};
  GPIO_InitTypeDef GPIO_InitStructure;

  //Generate START condition
  //o
  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(dev->port, &GPIO_InitStructure);

  //dev->port->MODER |= GPIO_MODER_MODER6_0;

  //Put LOW for at least 18ms
  GPIO_ResetBits(dev->port, dev->pin);

  delay_ms(18);
  //wait 18ms
  //TIM2->CNT = 0;
  //while((TIM2->CNT) <= 18000);

  //Put HIGH for 20-40us
  GPIO_SetBits(dev->port, dev->pin);

  delay_us(40);
  //wait 40us
  //TIM2->CNT = 0;
  //while((TIM2->CNT) <= 40);
  //End start condition

  //io();
  //Input mode to receive data
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(dev->port, &GPIO_InitStructure);
  //while(!GPIO_ReadInputDataBit(GPIOA, dev->pin));
  //DHT11 ACK
  //should be LOW for at least 80us
  //while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
  TIM4->CNT = 0;

  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)){
      if(TIM4->CNT > 100)
        return DHT11_ERROR_TIMEOUT;
    }
  //should be HIGH for at least 80us
  //while(GPIO_ReadInputDataBit(dev->port, dev->pin));
  TIM4->CNT = 0;
  while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      if(TIM4->CNT > 100)
        return DHT11_ERROR_TIMEOUT;
    }
  //Read 40 bits (8*5)
  for(j = 0; j < 5; ++j) {
      for(i = 0; i < 8; ++i) {
          TIM4->CNT = 0;
          //LOW for 50us
          while(!GPIO_ReadInputDataBit(dev->port, dev->pin));

          /*while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                          if(TIM2->CNT > 60)
                                                  return DHT11_ERROR_TIMEOUT;
                                  }*/

          //Start counter
          TIM4->CNT = 0;
          //HIGH for 26-28us = 0 / 70us = 1
          while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {      }
          /*while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                          if(TIM2->CNT > 100)
                                                  return DHT11_ERROR_TIMEOUT;
                                  }*/

          //Calc amount of time passed
          temp = TIM4->CNT;

          //shift 0
          data[j] = data[j] << 1;

          //if > 30us it's 1
          if(temp > 40) {
              data[j] = data[j]+1;
            }

        }
    }

  //verify the Checksum
  if(data[4] != (u8) (data[0] + data[2] + data[1] + data[3]))
    return DHT11_ERROR_CHECKSUM;
  //set data
  dev->temparature = data[2];
  dev->humidity = data[0];
  return DHT11_SUCCESS;
}

void wwdgenable(void){
  // Enable Watchdog
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE);
  WWDG_DeInit();
  WWDG_SetPrescaler(WWDG_Prescaler_8); //1, 2, 4, 8
  WWDG_SetWindowValue(127); // 64...127
  WWDG_Enable(100);
  WWDG_EnableIT();

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;    //WWDG interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   // NVIC initialization
}
void WWDG_IRQHandler(void) {
  //int i;
  WWDG_ClearFlag(); //This function reset flag WWDG->SR and cancel the resetting
  WWDG_SetCounter(100);

  // Toggle LED which connected to PC13
  GPIOC->ODR ^= GPIO_Pin_13;
}
void iwdg_init(void) {
  // включаем LSI
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
  // разрешается доступ к регистрам IWDG
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  // устанавливаем предделитель
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  // значение для перезагрузки
  IWDG_SetReload(0x300); //256/40000*0x300=4.9152
  // перезагрузим значение
  IWDG_ReloadCounter();
  // LSI должен быть включен
  IWDG_Enable();
}
