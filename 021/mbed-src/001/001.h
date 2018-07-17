#include "stm32f10x.h"
#include "stm32f10x_exti.h"

#define PERIOD 1000
#define SYSCLK 72000000
#define PRESCALER 72

#define RX_BUF_SIZE 80
volatile char RX_FLAG_END_LINE;
volatile unsigned int RXi;
volatile char RXc;
volatile uint8_t RXu;
char RX_BUF[RX_BUF_SIZE]; //= {"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
u8 RX_BUF08[RX_BUF_SIZE];
char buffer[80];// = {"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
volatile int TimeResult;
volatile int iResult;
volatile float fResult;
volatile int TimeSec;
volatile uint8_t TimeState;
volatile uint8_t FLAG_ECHO;
volatile uint16_t SonarValue;
// (UnixTime = 00:00:00 01.01.1970 = JD0 = 2440588)
#define JULIAN_DATE_BASE    2440588
typedef struct {
  uint8_t RTC_Hours;
  uint8_t RTC_Minutes;
  uint8_t RTC_Seconds;
  uint8_t RTC_Date;
  uint8_t RTC_Wday;
  uint8_t RTC_Month;
  uint16_t RTC_Year;
} RTC_DateTimeTypeDef;

void GETonGPIO(void);
#define USART1PPport GPIOA
#define USART1PPpin GPIO_Pin_11
void usart1_init(void);
void USART1_IRQHandler(void);
#define USARTSend USART1Send485
void clear_RXBuffer(void);
void USART01Send(u8 *pucBuffer);
void USART1Send(char *pucBuffer);
void USART1Send485(char *pucBuffer);
unsigned char RTC_Init(void);
void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct);
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct);
void RTC_GetMyFormat(RTC_DateTimeTypeDef* RTC_DateTimeStruct, char * buffer);
char get_ab_xFF(int a);
u8 convT_DS18B20(u8 LSB, u8 MSB);
void schitatTemp(char* imya);
void vvhex(char vv);
void sendaddrow (void);

float schitatfTemp(char* imya);
int schitatiTemp(char* imya);
void oprosite (void);

#define DHT11_SUCCESS         1
#define DHT11_ERROR_CHECKSUM  2
#define DHT11_ERROR_TIMEOUT   3
typedef struct DHT11_Dev {
  uint8_t temparature;
  uint8_t humidity;
  GPIO_TypeDef* port;
  uint16_t pin;
} DHT11_Dev;
int DHT11_init(struct DHT11_Dev* dev, GPIO_TypeDef* port, uint16_t pin);
int DHT11_read(struct DHT11_Dev* dev);

void wwdgenable(void);
void WWDG_IRQHandler(void);
void iwdg_init(void);


