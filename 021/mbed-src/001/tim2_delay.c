#include "tim2_delay.h"
#include "001.h"
//#include "libmodbus.h"
#include "modbus.h"

//extern UART_DATA uart1;


void TIM2_init(void) {
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseStructInit(&TIMER_InitStructure);

  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 36000;
  TIMER_InitStructure.TIM_Period = 49999;
  TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);
  TIM_Cmd(TIM2, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // ñ÷èòàåì îäèí ðàç
  TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);
}
void TIM2_IRQHandler(void) {

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)   {
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
void delay_us(uint32_t n_usec) {
  TIM4->CNT = 0;
  while (TIM4->CNT < n_usec);

}
void delay_ms(uint32_t n_msec) {
  TIM2->CNT = 0;
  while (TIM2->CNT < (2 * n_msec)){}

}

void TIM4_init(void) {
  TIM_TimeBaseInitTypeDef TIM_TimBaseStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  //Initialise TIMER4
  TIM_TimBaseStructure.TIM_Period = 50000;
  TIM_TimBaseStructure.TIM_Prescaler = 72;
  TIM_TimBaseStructure.TIM_ClockDivision = 0;
  TIM_TimBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimBaseStructure);
  TIM_Cmd(TIM4, ENABLE);

  // NVIC Configuration
  // Enable the TIM4_IRQn Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //и так есть основной void TIM2_init(void);
}
void TIM4_IRQHandler(void) {
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
      TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
      //TimeSec++;
    }
}

void TIM3_init(void) {
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  //TIM_TimeBaseStructInit(&TIMER_InitStructure);

  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 72;
  TIMER_InitStructure.TIM_Period = 1000;
  TIMER_InitStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // ñ÷èòàåì îäèí ðàç
  //TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Single);
}
void TIM3_IRQHandler(void) {
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)   {
      TIM_ClearITPendingBit(TIM3, TIM_IT_Update);//очищаем прерывания
      //если наш таймер больше уставки задержки и есть символы то есть gap -перерыв в посылке
      //и можно ее обрабатывать

      uart1.rxtimer++;
      if((uart1.rxtimer > uart1.delay)&(uart1.rxcnt > 1)) {
          uart1.rxgap=1;
        }
      else {
          uart1.rxgap=0;
        }
      millisec2++;
      if (millisec2 >= 1000) {
          globalsecs = GETglobalsecs();
          globalsecs++;
          SETglobalsecs(globalsecs);
          millisec2 = 0;
        }
    }
}
