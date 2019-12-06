#include "001.h"
#include "onewire.h"
#include "tim2_delay.h"
#include "string.h"
#include "stdio.h"
#include "dht11.h"

void GETonGPIO() {
  GPIO_InitTypeDef GPIO_InitSructure;
  //LED C.13
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitSructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitSructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitSructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitSructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  // A7 PP
  GPIO_InitSructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitSructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitSructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitSructure);
  //A6 PP
  GPIO_InitSructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitSructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitSructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitSructure);



  //KNOPKA B0
  /*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitSructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitSructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitSructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitSructure);
  //KNOPKA B1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitSructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitSructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitSructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitSructure);*/
}

void usart1_init(void) {
  //USART 1 and GPIO A 9 10 ON
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

}

void USART1_IRQHandler(void) {
  if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET) {
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
    }
}

void clear_RXBuffer(void) {
  for (RXi = 0; RXi < RX_BUF_SIZE; RXi++)
    RX_BUF[RXi] = '\0';
  RXi = 0;
}

void USART1Send(char *pucBuffer)
//;
{
  while (*pucBuffer) {
      USART_SendData(USART1,(uint16_t) *pucBuffer++);
      while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}

void ADC1init(void) {
  // clock for ADC (max 14MHz --> 72/6=12MHz)
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_TempSensorVrefintCmd(ENABLE);
  ADC_Cmd(ADC1, ENABLE);

  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
  ADC_TempSensorVrefintCmd(ENABLE);


  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  //while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
}

void usart_dma_init(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  //DMA
  DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
  DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&buffer[0];
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStruct.DMA_BufferSize = sizeof(buffer);
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
  DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStruct);

  //NVIC
  NVIC_InitTypeDef NVIC_InitStructure;
  //Enable the USARTx Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //GPIO
  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure USART1 Tx (PA.09) as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART1 Rx (PA.10) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure the USART1
  USART_InitTypeDef USART_InitStructure;

  // USART1 configuration ------------------------------------------------------
  /* USART1 configured as follow:
          - BaudRate = 115200 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Hardware flow control disabled (RTS and CTS signals)
          - Receive and transmit enabled
          - USART Clock disabled
          - USART CPOL: Clock is active low
          - USART CPHA: Data is captured on the middle
          - USART LastBit: The clock pulse of the last data bit is not output to
              the SCLK pin
       */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  //ENABLE USART1
  USART_Cmd(USART1, ENABLE);

  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  //DMA_Cmd(DMA1_Channel4, ENABLE);

  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);

  // Enable the USART1 Receive interrupt: this interrupt is generated when the
  //  USART1 receive data register is not empty
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USARTSendDMA(char *pucBuffer) {
  //strcpy(buffer, pucBuffer);
  //restart DMA Channel
  DMA_Cmd(DMA1_Channel4, DISABLE);
  DMA1_Channel4->CNDTR = strlen(pucBuffer);
  DMA_Cmd(DMA1_Channel4, ENABLE);
}

void DMA1_Channel4_IRQHandler(void) {
  DMA_ClearITPendingBit(DMA1_IT_TC4);
  DMA_Cmd(DMA1_Channel4, DISABLE);
}

void TIM4_IRQHandler(void) {
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
      sonar_start();
      FLAG_ECHO = 1;
      TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void TIMER4init(void) {
  //timer4
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 7200;
  TIMER_InitStructure.TIM_Period = 10000;
  TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM4, ENABLE);

  // NVIC Configuration
  // Enable the TIM4_IRQn Interrupt
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void gpioTIMpwm_init (void) {
  GPIO_InitTypeDef port;
  TIM_TimeBaseInitTypeDef timer;
  TIM_OCInitTypeDef timerPWM;

  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /*GPIO_StructInit(&port);
      port.GPIO_Mode = GPIO_Mode_IPU;
      port.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
      port.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_Init(GPIOA, &port);*/
  GPIO_StructInit(&port);
  port.GPIO_Mode = GPIO_Mode_AF_PP;
  port.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  port.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &port);

  TIM_TimeBaseStructInit(&timer);
  timer.TIM_Prescaler = 720;
  timer.TIM_Period = PERIOD;
  timer.TIM_ClockDivision = 0;
  timer.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &timer);

  TIM_OCStructInit(&timerPWM);
  timerPWM.TIM_Pulse = 0;
  timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
  timerPWM.TIM_OutputState = TIM_OutputState_Enable;
  timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &timerPWM);
  TIM_OC2Init(TIM4, &timerPWM);
  TIM_OC3Init(TIM4, &timerPWM);

  TIM_Cmd(TIM4, ENABLE);

}

/*void TIM3_delay(int i) {
  while (i > 0) {

      if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
          //Обязательно сбрасываем флаг
          TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
          //GPIOC->ODR ^= GPIO_Pin_13;
          TIM_SetCounter(TIM3, 0);
          while (TIM_GetCounter(TIM3) < 10);
          i--;

        }
    }
}
*/
/*void TIMER3init(void) {
  //timer3
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 7200;
  TIMER_InitStructure.TIM_Period = 10000;
  TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  /// NVIC Configuration
  // Enable the TIM3_IRQn Interrupt
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
*/
void servo_init(void) {
  GPIO_InitTypeDef port;
  TIM_TimeBaseInitTypeDef timer;
  TIM_OCInitTypeDef timerPWM;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  GPIO_StructInit(&port);
  port.GPIO_Mode = GPIO_Mode_AF_PP;
  port.GPIO_Pin = GPIO_Pin_6;
  port.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &port);

  TIM_TimeBaseStructInit(&timer);
  timer.TIM_Prescaler = PRESCALER;
  timer.TIM_Period = SYSCLK / PRESCALER / 50;
  timer.TIM_ClockDivision = 0;
  timer.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &timer);

  TIM_OCStructInit(&timerPWM);
  timerPWM.TIM_Pulse = 1000;
  timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
  timerPWM.TIM_OutputState = TIM_OutputState_Enable;
  timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &timerPWM);

  TIM_Cmd(TIM4, ENABLE);
}

void sonar_init(void) {
  GPIO_InitTypeDef gpio_cfg;
  GPIO_StructInit(&gpio_cfg);

  //Timer 3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseInitTypeDef timer_base;
  TIM_TimeBaseStructInit (&timer_base);
  timer_base.TIM_CounterMode = TIM_CounterMode_Up;
  timer_base.TIM_Prescaler = 72;
  TIM_TimeBaseInit(TIM3, &timer_base);
  TIM_Cmd(TIM3, ENABLE);

  //Trigger Pin
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  gpio_cfg.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_cfg.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOB, &gpio_cfg);

  //Set variables used
  GPIO_InitTypeDef GPIO_InitStruct;
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;

  //Enable clock for AFIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  //Set pin as input
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Add IRQ vector to NVIC
  //PB0 is connected to EXTI_Line0, which has EXTI0_IRQn vector
  NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  //Tell system that you will use PB0 for EXTI_Line0
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

  //PD0 is connected to EXTI_Line0
  EXTI_InitStruct.EXTI_Line = EXTI_Line0;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_Init(&EXTI_InitStruct);
}

void sonar_start(void) {
  GPIO_SetBits(GPIOB, GPIO_Pin_15);
  delay_ms(1);
  GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}

unsigned int sonar_get() {
  unsigned long Sonar;
  //unsigned long Sonar;
  // 354000 - Sound speed (mm/sec)
  // 72000000 - F_CPU
  // 16 - Timer Prescaler
  // Result = mm
  Sonar = (354/2) * (unsigned long)SonarValue / (72000 / 72);
  if (Sonar > 4000) Sonar = 4000;
  if (Sonar < 20) Sonar = 20;

  return (unsigned int)Sonar;
}

#define _TBIAS_DAYS		((70 * (u32)365) + 17)
#define _TBIAS_SECS		(_TBIAS_DAYS * (uint32_t)86400)
#define	_TBIAS_YEAR		1900
#define MONTAB(year)		((((year) & 03) || ((year) == 0)) ? mos : lmos)

const u16	lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const u16	mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define	Daysto32(year, mon)	(((year - 1) / 4) + MONTAB(year)[mon])


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

//get cirrent date
void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct) {
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
// Convert Date to Counter
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct) {
        /* convert time structure to scalar time */
  s32		days;
  uint32_t		secs;
  s32		mon, year;

          /* Calculate number of days. */
          mon = RTC_DateTimeStruct->RTC_Month - 1;
          year = RTC_DateTimeStruct->RTC_Year - _TBIAS_YEAR;
          days  = Daysto32(year, mon) - 1;
          days += 365 * year;
          days += RTC_DateTimeStruct->RTC_Date;
          days -= _TBIAS_DAYS;

          /* Calculate number of seconds. */
          secs  = 3600 * RTC_DateTimeStruct->RTC_Hours;
          secs += 60 * RTC_DateTimeStruct->RTC_Minutes;
          secs += RTC_DateTimeStruct->RTC_Seconds;

          secs += (days * (uint32_t)86400);

          return (secs);

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
  const char * MONTH[12]={MONTH1, MONTH2, MONTH3, MONTH4, MONTH5, MONTH6, MONTH7, MONTH8, MONTH9, MONTH10, MONTH11, MONTH12};

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
  u8 command01[12] = {(u8)'\x55',(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],(u8) imya[4],(u8) imya[5],(u8) imya[6],(u8) imya[7],(u8)'\xbe',(u8) '\xff',(u8) '\xff'};
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
  sprintf(cifry, "termperature :%d.%d\r\n", temp, (int) (0.0625*1000)*(buf[0] % 16));
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
//ппц юсарт 3 к 1-ware те же GPIO B 10 11/ а не А2 А3
void usart3_init(void) {
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

  //GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  // GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

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
      //USART_SendData(USART1, RXc);
    }
}
*/
void USART3_IRQHandler(void)
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
}
void USART3Send(char *pucBuffer)
//for RS 485;
{
  delay_ms(2);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
  GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  GPIO_SetBits(GPIOA, GPIO_Pin_6);
  GPIO_SetBits(GPIOA, GPIO_Pin_7);

  delay_ms(2);
  u16 a;
  while (*pucBuffer) {
      /*USART_SendData(USART3,(uint16_t) *pucBuffer++);
      while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
        {
        }*/
      a = ((uint16_t) *pucBuffer++);
      USART_SendData(USART3,a);
      /*while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
        {
        }*/
      delay_ms(1);
    }
  delay_ms(2);
  GPIO_SetBits(GPIOC, GPIO_Pin_13);     // C13 -- 1
  //GPIO_ResetBits(GPIOC, GPIO_Pin_13);   //C13 --0
  GPIO_SetBits(GPIOB, GPIO_Pin_1);
  GPIO_SetBits(GPIOB, GPIO_Pin_0);
  //GPIO_SetBits(GPIOC, GPIO_Pin_13);
  GPIO_ResetBits(GPIOA, GPIO_Pin_6);
  GPIO_ResetBits(GPIOA, GPIO_Pin_7);
  delay_ms(2);
}

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
  //char command01[12] = {'\x55', imya[0], imya[1], imya[2], imya[3], imya[4], imya[5], imya[6], imya[7],'\xbe', '\xff', '\xff'};
  u8 command01[12] = {(u8)'\x55',(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],(u8) imya[4],(u8) imya[5],(u8) imya[6],(u8) imya[7],(u8)'\xbe',(u8) '\xff',(u8) '\xff'};
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

void MODBUS_SLAVE(UART_DATA *MODBUS)
{
  unsigned int tmp;


  //recive and checking rx query
  if((MODBUS->buffer[0]!=0)&(MODBUS->rxcnt>5)& ((MODBUS->buffer[0]==SET_PAR[0])|(MODBUS->buffer[0]==255)))
    {
      tmp=Crc16(MODBUS->buffer,MODBUS->rxcnt-2);

      if((MODBUS->buffer[MODBUS->rxcnt-2]==(tmp&0x00FF)) & (MODBUS->buffer[MODBUS->rxcnt-1]==(tmp>>8)))
        {
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

unsigned int Crc16(unsigned char *ptrByte, int byte_cnt)
{
  unsigned int w=0;
  char shift_cnt;

  if(ptrByte)
    {
      w=0xffffU;
      for(; byte_cnt>0; byte_cnt--)
        {
          w=(unsigned int)((w/256U)*256U+((w%256U)^(*ptrByte++)));
          for(shift_cnt=0; shift_cnt<8; shift_cnt++)
            {
              if((w&0x1)==1)
                w=(unsigned int)((w>>1)^0xa001U);
              else
                w>>=1;
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
              MODBUS->buffer[n]=(unsigned char) (tmp_val_pos>>8)|0b10000000;
              MODBUS->buffer[n+1]=(unsigned char) tmp_val_pos;
            }
          else
            {
              //пакуем положительное
              MODBUS->buffer[n]=(unsigned char) (tmp_val>>8);
              MODBUS->buffer[n+1]=(unsigned char) tmp_val;
            }
          n=n+2;
        }

      //запишем длину переменных пакета в байтах и вставим всообщение
      MODBUS->buffer[2]=(unsigned char) (m*2); //byte count
      //подготовим к отправке
      MODBUS->txlen=(unsigned char) (m*2+5); //responce length

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
  u8 command01[12] = { 0x55,(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],(u8) imya[4],(u8) imya[5],(u8) imya[6],(u8) imya[7], 0xbe, 0xff, 0xff};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  float ftemp;
  ftemp = (float) ((buf[1] << 8) | buf[0]) /(float) 16.0;
  return ftemp;
}
int schitatiTemp(char* imya) {
  uint8_t buf[2];
  u8 command01[12] = { 0x55,(u8) imya[0],(u8) imya[1],(u8) imya[2],(u8) imya[3],(u8) imya[4],(u8) imya[5],(u8) imya[6],(u8) imya[7], 0xbe, 0xff, 0xff};
  OW_Send(OW_SEND_RESET, command01, 12, buf, 2, 10);
  int itemp;
  itemp = ((buf[1] << 8) | buf[0]) *1000 / 16;
  delay_ms(10);
  return itemp;
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
  /*u8 comm[2];
  comm[0] = (u8) '\xcc';
  comm[1] = (u8) '\x44';
  OW_Send(OW_SEND_RESET, comm, 2, NULL, 0, OW_NO_READ);
  delay_ms(8000);
  comm[1] = (u8) '\x4E';
  OW_Send(OW_SEND_RESET, comm, 2, NULL, 0, OW_NO_READ);
  delay_ms(1000);
  //USARTSend("oprosheno\n\r");
*/
  //ПОПРОБОВАТЬ ПОЗЖЕ
  u8 comm[2];
  comm[0] = 0xcc;
  comm[1] = 0x44;
  OW_Send(OW_SEND_RESET, comm, 2, NULL, 0, OW_NO_READ);
  delay_ms(1000);
  comm[1] = 0x4e;
  OW_Send(OW_SEND_RESET, comm, 2, NULL, 0, OW_NO_READ);
  delay_ms(1000);
  USARTSend("oprosheno\n\r");

}
void net_tx3(UART_DATA *uart)
{
  if((uart->txlen>0)&(uart->txcnt==0))
    {
      USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
      USART_ITConfig(USART3, USART_IT_TC, ENABLE);

      GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET);
      GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET);

      USART_SendData(USART3, uart->buffer[uart->txcnt++]);
    }

}
void net_tx1(UART_DATA *uart)
{
  if((uart->txlen>0)&(uart->txcnt==0))
    {
      USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
      USART_ITConfig(USART1, USART_IT_TC, ENABLE);
      USART_SendData(USART1, uart->buffer[uart->txcnt++]);
      GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET);
    }

}
// ////////////////////////////////////////////////////////DHT11
#include "dht11.h"

int DHT11_init(struct DHT11_Dev* dev, GPIO_TypeDef* port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStructure;

  dev->port = port;
  dev->pin = pin;

  //Initialise TIMER2
  /*TIM_TimBaseStructure.TIM_Period = 84000000 - 1;
  TIM_TimBaseStructure.TIM_Prescaler = 84;
  TIM_TimBaseStructure.TIM_ClockDivision = 0;
  TIM_TimBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimBaseStructure);
  TIM_Cmd(TIM2, ENABLE);*/
  //и так есть основной void TIM2_init(void);

  //Initialise GPIO DHT11
  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(dev->port, &GPIO_InitStructure);

  return 0;
}

int DHT11_read(struct DHT11_Dev* dev) {

  //Initialisation
  uint8_t i, j, temp;
  uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
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

  //DHT11 ACK
  //should be LOW for at least 80us
  //while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
  //TIM2->CNT = 0;
  u16 tiks001 = 0;
  while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      delay_us(1);
      tiks001 += 1;
    }
  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      if(tiks001 > 100)
        return DHT11_ERROR_TIMEOUT;
    }

  //should be HIGH for at least 80us
  //while(GPIO_ReadInputDataBit(dev->port, dev->pin));
  //TIM2->CNT = 0;
  tiks001 = 0;
  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      delay_us(1);
      tiks001 += 1;
    }
  while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      if(tiks001 > 100)
        return DHT11_ERROR_TIMEOUT;
    }

  //Read 40 bits (8*5)
  for(j = 0; j < 5; ++j) {
      for(i = 0; i < 8; ++i) {

          //LOW for 50us
          while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
          /*TIM2->CNT = 0;
                                  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                          if(TIM2->CNT > 60)
                                                  return DHT11_ERROR_TIMEOUT;
                                  }*/

          //Start counter
          tiks001 = 0;

          //HIGH for 26-28us = 0 / 70us = 1
          while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
              delay_us(1);
              tiks001 += 1;
            }
          /*while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                          if(TIM2->CNT > 100)
                                                  return DHT11_ERROR_TIMEOUT;
                                  }*/

          //Calc amount of time passed
          temp = tiks001;

          //shift 0
          data[j] = data[j] << 1;

          //if > 30us it's 1
          if(temp > 40)
            data[j] = data[j]+1;
        }
    }

  //verify the Checksum
  if(data[4] != (data[0] + data[2]))
    return DHT11_ERROR_CHECKSUM;


  //set data
  dev->temparature = data[2];
  dev->humidity = data[0];

  return DHT11_SUCCESS;
}

int DHT11_read002(struct DHT11_Dev* dev) {

  //Initialisation
  uint8_t i, j, temp;
  uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
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

  //DHT11 ACK
  //should be LOW for at least 80us
  //while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
  //TIM2->CNT = 0;
  u16 tiks001 = 0;
  /*while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      delay_us(1);
      tiks001 += 1;
    }*/
  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      delay_us(1);
      tiks001 += 1;
      if(tiks001 > 100)
        return DHT11_ERROR_TIMEOUT;
    }

  //should be HIGH for at least 80us
  //while(GPIO_ReadInputDataBit(dev->port, dev->pin));
  //TIM2->CNT = 0;
  tiks001 = 0;
  /*while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      delay_us(1);
      tiks001 += 1;
    }*/
  while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      delay_us(1);
      tiks001 += 1;
      if(tiks001 > 100)
        return DHT11_ERROR_TIMEOUT;
    }

  //Read 40 bits (8*5)
  for(j = 0; j < 5; ++j) {
      for(i = 0; i < 8; ++i) {

          //LOW for 50us
          while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
          /*TIM2->CNT = 0;
                                  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                          if(TIM2->CNT > 60)
                                                  return DHT11_ERROR_TIMEOUT;
                                  }*/

          //Start counter
          tiks001 = 0;

          //HIGH for 26-28us = 0 / 70us = 1
          while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
              delay_us(1);
              tiks001 += 1;
            }
          /*while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                          if(TIM2->CNT > 100)
                                                  return DHT11_ERROR_TIMEOUT;
                                  }*/

          //Calc amount of time passed
          temp = tiks001;

          //shift 0
          data[j] = data[j] << 1;

          //if > 30us it's 1
          if(temp > 40)
            data[j] = data[j]+1;
        }
    }

  //verify the Checksum
  if(data[4] != (data[0] + data[2]))
    return DHT11_ERROR_CHECKSUM;


  //set data
  dev->temparature = data[2];
  dev->humidity = data[0];

  return DHT11_SUCCESS;
}

int DHT11_read003(struct DHT11_Dev* dev) {

  //Initialisation
  uint8_t i, j, temp;
  uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  GPIO_InitTypeDef GPIO_InitStructure;

  //Generate START condition
  //o
  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(dev->port, &GPIO_InitStructure);
  /*
  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(dev->port, &GPIO_InitStructure);
*/
  //dev->port->MODER |= GPIO_MODER_MODER6_0;

  //Put LOW for at least 18ms
  GPIO_ResetBits(dev->port, dev->pin);

  //wait 18ms
  delay_ms(18);

  //Put HIGH for 20-40us
  GPIO_SetBits(dev->port, dev->pin);

  //wait 40us
  delay_us(40);
  //End start condition

  //io();
  //Input mode to receive data
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(dev->port, &GPIO_InitStructure);

  //DHT11 ACK
  //should be LOW for at least 80us
  //while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
  temp = 0;
  while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      delay_us(1);
      temp += 1;
      if(temp > 100)
        return DHT11_ERROR_TIMEOUT;
    }

  //should be HIGH for at least 80us
  //while(GPIO_ReadInputDataBit(dev->port, dev->pin));
  char cifry[10];
  sprintf(cifry, "%d\r\n", temp);
  USARTSend(cifry);
  temp = 0;
  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      temp += 1;
      if(temp > 100)
        return DHT11_ERROR_TIMEOUT;
    }
  sprintf(cifry, "%d\r\n", temp);
  USARTSend(cifry);
  //Read 40 bits (8*5)
  for(j = 0; j < 5; ++j) {
      for(i = 0; i < 8; ++i) {

          //LOW for 50us
          while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
          /*TIM2->CNT = 0;
                        while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                if(TIM2->CNT > 60)
                                        return DHT11_ERROR_TIMEOUT;
                        }*/

          //Start counter

          temp = 0;

          //HIGH for 26-28us = 0 / 70us = 1
          while(GPIO_ReadInputDataBit(dev->port, dev->pin))
            temp += 1;
          /*while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                if(TIM2->CNT > 100)
                                        return DHT11_ERROR_TIMEOUT;
                        }*/

          //Calc amount of time passed
          //temp = tiks001;
          sprintf(cifry, "%d\r\n", temp);
          USARTSend(cifry);
          //shift 0
          data[j] = data[j] << 1;

          //if > 30us it's 1
          if(temp > 40)
            data[j] = data[j]+1;
        }
    }

  //verify the Checksum
  if(data[4] != (data[0] + data[2]))
    return DHT11_ERROR_CHECKSUM;

  //set data
  dev->temparature = data[2];
  dev->humidity = data[0];

  return DHT11_SUCCESS;
}

int DHT11_read004(struct DHT11_Dev* dev) {

  //Initialisation
  uint8_t i, j, temp;
  uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  GPIO_InitTypeDef GPIO_InitStructure;

  //Generate START condition
  //o
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(dev->port, &GPIO_InitStructure);
  /*
  GPIO_InitStructure.GPIO_Pin = dev->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(dev->port, &GPIO_InitStructure);
*/
  //dev->port->MODER |= GPIO_MODER_MODER6_0;

  //Put LOW for at least 18ms
  GPIO_ResetBits(GPIOA, GPIO_Pin_8);

  //wait 18ms
  delay_ms(18);

  //Put HIGH for 20-40us
  GPIO_SetBits(GPIOA, GPIO_Pin_8);

  //wait 40us
  delay_us(35);
  //End start condition

  //io();
  //Input mode to receive data
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(dev->port, &GPIO_InitStructure);

  //DHT11 ACK
  //should be LOW for at least 80us
  //while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
  temp = 0;
  while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      delay_us(1);
      temp += 1;
      if(temp > 100)
        break;
        //return DHT11_ERROR_TIMEOUT;
    }

  //should be HIGH for at least 80us
  //while(GPIO_ReadInputDataBit(dev->port, dev->pin));
  char cifry[10];
  sprintf(cifry, "%d\r\n", temp);
  USARTSend(cifry);
  temp = 0;
  while(GPIO_ReadInputDataBit(dev->port, dev->pin)) {
      temp += 1;
      if(temp > 100)
        break;
        //return DHT11_ERROR_TIMEOUT;
    }
  sprintf(cifry, "%d\r\n", temp);
  USARTSend(cifry);
  //Read 40 bits (8*5)
  for(j = 0; j < 5; ++j) {
      for(i = 0; i < 8; ++i) {

          //LOW for 50us
          while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
          /*TIM2->CNT = 0;
                        while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                if(TIM2->CNT > 60)
                                        return DHT11_ERROR_TIMEOUT;
                        }*/

          //Start counter

          temp = 0;

          //HIGH for 26-28us = 0 / 70us = 1
          while(GPIO_ReadInputDataBit(dev->port, dev->pin))
            temp += 1;
          /*while(!GPIO_ReadInputDataBit(dev->port, dev->pin)) {
                                if(TIM2->CNT > 100)
                                        return DHT11_ERROR_TIMEOUT;
                        }*/

          //Calc amount of time passed
          //temp = tiks001;
          sprintf(cifry, "%d\r\n", temp);
          USARTSend(cifry);
          //shift 0
          data[j] = data[j] << 1;

          //if > 30us it's 1
          if(temp > 40)
            data[j] = data[j]+1;
        }
    }

  //verify the Checksum
  if(data[4] != (data[0] + data[2]))
    return DHT11_ERROR_CHECKSUM;

  //set data
  dev->temparature = data[2];
  dev->humidity = data[0];

  return DHT11_SUCCESS;
}
