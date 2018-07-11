#include "tim2_delay.h"

volatile uint8_t f_timer_2_end;

void TIM2_init(void)
{
  TIM_TimeBaseInitTypeDef TIMER_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseStructInit(&TIMER_InitStructure);

  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIMER_InitStructure.TIM_Prescaler = 72;
  TIMER_InitStructure.TIM_Period = 1;
  TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // ñ÷èòàåì îäèí ðàç
  TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);
}
void TIM2_IRQHandler(void)
{
  extern volatile uint8_t f_timer_2_end;

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  TIM2->SR &= ~TIM_SR_UIF;
  f_timer_2_end = 1;

  TIM_Cmd(TIM2, DISABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
}
void delay_us(uint32_t n_usec)
{
  f_timer_2_end = 0;

  TIM2->PSC = 72 - 1;
  TIM2->ARR = (uint16_t)( n_usec);
  TIM_Cmd(TIM2, ENABLE);

  // äëÿ òîãî ÷òîáû óñòàíîâèëñÿ PSC
  TIM2->EGR |= TIM_EGR_UG;
  TIM2->SR &= ~TIM_SR_UIF;

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);

  while(f_timer_2_end == 0);
}
void delay_ms(uint32_t n_msec)
{
  f_timer_2_end = 0;

  TIM2->PSC = 36000 - 1;
  TIM2->ARR = (uint16_t)(2 * n_msec);
  // äëÿ òîãî ÷òîáû óñòàíîâèëñÿ PSC
  TIM2->EGR |= TIM_EGR_UG;
  TIM2->SR &= ~TIM_SR_UIF;

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);

  while(f_timer_2_end == 0);
}

void TIM4_init(void)
{
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //и так есть основной void TIM2_init(void);
}
void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
      TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
      //TimeSec++;
    }
}
