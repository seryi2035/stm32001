/*
#include "modbus.h"
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

unsigned int Crc16(unsigned char *ptrByte, int byte_cnt);
void TX_03_04(UART_DATA *MODBUS);
void TX_06(UART_DATA *MODBUS);
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type);

// *********************************************************************
//Modbus slave function
// *********************************************************************
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

            default://если нет то выдаем ошибку
              //illegal operation
              TX_EXCEPTION(MODBUS,0x01);
            }
          //добавляем CRC и готовим к отсылке
          //adding CRC16 to reply
          tmp=Crc16(MODBUS->buffer,MODBUS->txlen-2);
          MODBUS->buffer[MODBUS->txlen-2]=tmp;
          MODBUS->buffer[MODBUS->txlen-1]=tmp>>8;
          MODBUS->txcnt=0;

        }

    }
  //сброс индикаторов окончания посылки
  MODBUS->rxgap=0;
  MODBUS->rxcnt=0;
  MODBUS->rxtimer=0xFFFF;

}

// ******************************************************************
//READING input & holding registers
// *******************************************************************
void TX_03_04(UART_DATA *MODBUS)
{
  unsigned int tmp,tmp1;
  unsigned int m=0,n=0;
  int tmp_val,tmp_val_pos;

  //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
  //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
  //MODBUS->buffer[2] = data byte count

  //2-3  - starting address
  tmp=((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения

  //4-5 - number of registers
  tmp1=((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);//количество регистров для чтения

  //default answer length if error
  n=3;

  //если нас устраивает длина запроса и и стартовый адрес
  if((((tmp+tmp1)<OBJ_SZ)&(tmp1<MODBUS_WRD_SZ+1)))
    {

      for(m=0;m<tmp1;m++)
        {
          tmp_val=res_table.regs[m+tmp];//читаем текущее значение

          if(tmp_val<0)
            {
              //пакуем отрицательное
              tmp_val_pos=tmp_val;
              MODBUS->buffer[n]=(tmp_val_pos>>8)|0b10000000;
              MODBUS->buffer[n+1]=tmp_val_pos;
            }
          else
            {
              //пакуем положительное
              MODBUS->buffer[n]=tmp_val>>8;
              MODBUS->buffer[n+1]=tmp_val;
            }
          n=n+2;
        }

      //запишем длину переменных пакета в байтах и вставим всообщение
      MODBUS->buffer[2]=m*2; //byte count
      //подготовим к отправке
      MODBUS->txlen=m*2+5; //responce length

    }
  else
    {
      //exception illegal data adress 0x02
      TX_EXCEPTION(MODBUS,0x02);
    }

}
// *******************************************************
//Writing
// *******************************************************
void TX_06(UART_DATA *MODBUS)
{
  unsigned int tmp;

  //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
  //MODBUS[1] = 6; //query type - - stay a same as in recived

  //2-3  - adress   , 4-5 - value

  tmp=((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //adress

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

// ********************************************************************
//Exception if wrong query
// *********************************************************************

//modbus exception - illegal data=01 ,adress=02 etc
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type)
{
  //illegal operation
  MODBUS->buffer[2]=error_type; //exception
  MODBUS->txlen=5; //responce length
}

// *********************************************************************
//CRC16 for Modbus Calculation
// *********************************************************************
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
void SetupTIM6()
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
  TIM_DeInit(TIM6);

  //0.0001 sec setup APB=36Mhz/(36*100)
  TIM_TimeBaseStructure.TIM_Prescaler= 36;//на эту величину поделим частоту шины
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//частота без деления 36Мгц
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//считаем вверх
  TIM_TimeBaseStructure.TIM_Period=100;//до этого значения будет считать таймер
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM6, TIM_FLAG_Update);
  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM6, ENABLE);


  // Настройка прерывания
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
}

void TIM6_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);//очищаем прерывания

  //моргаем светодиодом дабы показать активность таймера
  if(GPIO_ReadOutputDataBit  ( GPIOD,GPIO_Pin_11))
    GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_RESET);
  else
    GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_SET);

  //если наш таймер больше уставки задержки и есть символы то есть gap -перерыв в посылке
  //и можно ее обрабатывать
  if((uart3.rxtimer++>uart3.delay)&(uart3.rxcnt>1))
    uart3.rxgap=1;
  else
    uart3.rxgap=0;

  //тоже самое для usart1
  if((uart1.rxtimer++>uart1.delay)&(uart1.rxcnt>1))
    uart1.rxgap=1;
  else
    uart1.rxgap=0;

}

void net_tx3(UART_DATA *uart)
{
  if((uart->txlen>0)&(uart->txcnt==0))
    {
      USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//выкл прерывание на прием
      USART_ITConfig(USART3, USART_IT_TC, ENABLE);//включаем на окочание передачи

      //моргаем светодиодом и включаем rs485 на передачу
      GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET);
      GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_SET);

      USART_SendData(USART3, uart->buffer[uart->txcnt++]);
    }

}

*/

/*
  // *****************************************************************************
// SIMPLE MODBUS SLAVE EXAMPLE YURI GUSEV V1
// *****************************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_tim.h"

#include "misc.h"
#include "controller.h"

// **************************************************************************
 * Declare function prototypes
 ****************************************************************************
void SetupClock(void);
void SetupUSART1(USART_InitTypeDef USART_InitStructure);
void SetupUSART2(USART_InitTypeDef USART_InitStructure);
void SetupUSART3(USART_InitTypeDef USART_InitStructure);
void SetupTIM6(void);
void SetupGPIO(void);

USART_InitTypeDef USART_InitStructure;

// ***************************************************************************
//send data from uart1 if data is ready
// ***************************************************************************
void net_tx1(UART_DATA *uart)
{
  if((uart->txlen>0)&(uart->txcnt==0))
  {
            USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
            USART_ITConfig(USART1, USART_IT_TC, ENABLE);
                USART_SendData(USART1, uart->buffer[uart->txcnt++]);
                GPIO_WriteBit(GPIOD,GPIO_Pin_8,Bit_SET);
  }

}

// ***************************************************************************
//send data from uart1 if data is ready
// ***************************************************************************
void net_tx3(UART_DATA *uart)
{
  if((uart->txlen>0)&(uart->txcnt==0))
  {
            USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
            USART_ITConfig(USART3, USART_IT_TC, ENABLE);

		GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_SET);

                USART_SendData(USART3, uart->buffer[uart->txcnt++]);
  }

}


// ***************************************************************************
// *  USART1 interrupt
// **************************************************************************
void USART1_IRQHandler(void)
{
        //Receive Data register not empty interrupt
        if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
   {
                 USART_ClearITPendingBit(USART1, USART_IT_RXNE);
                uart1.rxtimer=0;

                        if(uart1.rxcnt>(BUF_SZ-2))
                        uart1.rxcnt=0;

                        uart1.buffer[uart1.rxcnt++]=USART_ReceiveData (USART1);


   }

        //Transmission complete interrupt
    if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
        {

                USART_ClearITPendingBit(USART1, USART_IT_TC);
                  if(uart1.txcnt<uart1.txlen)
                {
                        USART_SendData(USART1,uart1.buffer[uart1.txcnt++]);
                }
                 else
                {
                 uart1.txlen=0;
                 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
                 USART_ITConfig(USART1, USART_IT_TC, DISABLE);
                 GPIO_WriteBit(GPIOD,GPIO_Pin_8,Bit_RESET);
                }
        }

}



// ***************************************************************************
// *  USART3 interrupt
// **************************************************************************
void USART3_IRQHandler(void)
{
        //Receive Data register not empty interrupt
        if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
   {
                 USART_ClearITPendingBit(USART3, USART_IT_RXNE);
                uart3.rxtimer=0;

                        if(uart3.rxcnt>(BUF_SZ-2))
                        uart3.rxcnt=0;

                        uart3.buffer[uart3.rxcnt++]=USART_ReceiveData (USART3);


   }

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
                 GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET);
                 GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_RESET);
                 USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
                 USART_ITConfig(USART3, USART_IT_TC, DISABLE);
                 TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
                }
        }

}

// ***************************************************************************
// Timer interrupt
// ***************************************************************************
void TIM6_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

	if(GPIO_ReadOutputDataBit  ( GPIOD,GPIO_Pin_11))
	     GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_RESET);
	else
		GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_SET);

	   if((uart3.rxtimer++>uart3.delay)&(uart3.rxcnt>1))
	   uart3.rxgap=1;
	   else
	   uart3.rxgap=0;

	   if((uart1.rxtimer++>uart1.delay)&(uart1.rxcnt>1))
	   uart1.rxgap=1;
	   else
	   uart1.rxgap=0;

}

// ***************************************************************************
//main()
// ***************************************************************************
int main(void)
{
	   SetupClock();

	  //setting parametrs common for all uarts
	      USART_InitStructure.USART_BaudRate            = 9600;
	  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	  USART_InitStructure.USART_Parity              = USART_Parity_No ;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

       //uarts inints+interrupts
          SetupUSART1(USART_InitStructure); //RS232
          SetupUSART3(USART_InitStructure); //RS485

       //tim6 +interrupts
       SetupTIM6();

       //RS485 TXE PIN AND LED PINS
       SetupGPIO();

       SET_PAR[0]=1;//modbus address

       //timer 0.0001sec one symbol on 9600 ~1ms
       uart1.delay=30; //modbus gap

       //timer 0.0001sec one symbol on 9600 ~1ms
       uart3.delay=30; //modbus gap


     //Main loop
     while(1)
     {

        if(uart3.rxgap==1)
         {
         MODBUS_SLAVE(&uart3);
         net_tx3(&uart3);
         }

        if(uart1.rxgap==1)
          {
            MODBUS_SLAVE(&uart1);
           net_tx1(&uart1);
          }

     }
}

// **************************************************************************
 * @brief Setup clocks
 *****************************************************************************
void SetupClock()
{
      RCC_DeInit ();                    // RCC system reset(for debug purpose)
      RCC_HSEConfig (RCC_HSE_ON);       // Enable HSE

      // Wait till HSE is ready
      while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

      RCC_HCLKConfig   (RCC_SYSCLK_Div1);   // HCLK   = SYSCLK
      RCC_PCLK2Config  (RCC_HCLK_Div1);     // PCLK2  = HCLK
      RCC_PCLK1Config  (RCC_HCLK_Div2);     // PCLK1  = HCLK/2
      RCC_ADCCLKConfig (RCC_PCLK2_Div4);    // ADCCLK = PCLK2/4

      // PLLCLK = 8MHz * 9 = 72 MHz
      RCC_PLLConfig (RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

      RCC_PLLCmd (ENABLE);                  // Enable PLL

      // Wait till PLL is ready
      while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

      // Select PLL as system clock source
      RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);

      // Wait till PLL is used as system clock source
      while (RCC_GetSYSCLKSource() != 0x08);

      // Enable USART1 and GPIOA clock
   //   RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

}
// *************************************************************************
 // * @brief Init USART1,2,3
 // *****************************************************************************
void SetupUSART1(USART_InitTypeDef USART_InitStructure)
{
	 NVIC_InitTypeDef  NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
      //USART_InitTypeDef USART_InitStructure;

      // Enable GPIOA clock
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1, ENABLE);

      // Configure USART1 Rx (PA10) as input floating
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      // Configure USART1 Tx (PA9) as alternate function push-pull
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

          USART_Init(USART1, &USART_InitStructure);
          USART_Cmd(USART1, ENABLE);

        //Setting interrupts
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // Enable Receive interrupt


}
// *****************************************************************************************8

void SetupUSART2(USART_InitTypeDef USART_InitStructure)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		NVIC_InitTypeDef  NVIC_InitStructure;

            // Enable GPIOA clock
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

            // Configure USART2 Rx (PA3) as input floating
            GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
            GPIO_Init(GPIOA, &GPIO_InitStructure);

            // Configure USART2 Tx (PA2) as alternate function push-pull
            GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            USART_Init(USART2, &USART_InitStructure);
                USART_Cmd(USART2, ENABLE);

                //Setting interrupts
                NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);

                USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  // Enable Receive interrupt

}

// ********************************************************************************************

void SetupUSART3(USART_InitTypeDef USART_InitStructure)
{
		     NVIC_InitTypeDef  NVIC_InitStructure;
		     GPIO_InitTypeDef  GPIO_InitStructure;

                     // Enable GPIOB clock
                 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

                 // Configure USART3 Rx (PA11) as input floating
                 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
                 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
                 GPIO_Init(GPIOB, &GPIO_InitStructure);

                 // Configure USART3 Tx (PA10) as alternate function push-pull
                 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
                 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                 GPIO_Init(GPIOB, &GPIO_InitStructure);

                  USART_Init(USART3, &USART_InitStructure);
                  USART_Cmd(USART3, ENABLE);

                  //Setting interrupts

                  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
                  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
                  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                  NVIC_Init(&NVIC_InitStructure);

                  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  // Enable Receive interrupt

}

// *******************************************************

void SetupTIM6()
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
    TIM_DeInit(TIM6);

    //0.0001 sec setup APB=36Mhz/(36*100)
    TIM_TimeBaseStructure.TIM_Prescaler= 36;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=100;//till what value timer will count

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM6, ENABLE);


   // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
      NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);

      TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
}

// ***********************************************************************

void SetupGPIO(void)
{
        GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure PB2 as rs485 tx select
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // Configure PD10 leds
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}
// ***************************************************************************

*/
