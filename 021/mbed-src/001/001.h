#include "stm32f10x.h"
#include "stm32f10x_exti.h"

#define PERIOD 1000
#define SYSCLK 72000000
#define PRESCALER 72

#define RX_BUF_SIZE 80
volatile char RX_FLAG_END_LINE;
volatile unsigned int RXi;
volatile char RXc;
char RX_BUF[RX_BUF_SIZE]; //= {"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
u8 RX_BUF08[RX_BUF_SIZE];
char buffer[80];// = {"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"};
volatile int TimeResult;
volatile int TimeSec;
volatile uint8_t TimeState;
volatile uint8_t FLAG_ECHO;
volatile uint16_t SonarValue;
// (UnixTime = 00:00:00 01.01.1970 = JD0 = 2440588)
#define JULIAN_DATE_BASE    2440588
typedef struct
{
  uint8_t RTC_Hours;
  uint8_t RTC_Minutes;
  uint8_t RTC_Seconds;
  uint8_t RTC_Date;
  uint8_t RTC_Wday;
  uint8_t RTC_Month;
  uint16_t RTC_Year;
} RTC_DateTimeTypeDef;

void GETonGPIO(void);
void usart1_init(void);
void USART1_IRQHandler(void);
void clear_RXBuffer(void);
void USART1Send(char *pucBuffer);
unsigned char RTC_Init(void);
void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct);
uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct);
void RTC_GetMyFormat(RTC_DateTimeTypeDef* RTC_DateTimeStruct, char * buffer);
char get_ab_xFF(int a);
u8 convT_DS18B20(u8 LSB, u8 MSB);
void schitatTemp(char* imya);
void vvhex(char vv);
void usart3_init(void);
void USART3_IRQHandler(void);
void USART3Send(char *pucBuffer);
void sendaddrow (void);

#define USARTSend USART1Send
//MODBUS
//#include "modbus.h"
#define OBJ_SZ 123 //это количество объектов
#define SETUP 4 //это просто количество данных в массиве 0-элемент которого означает адрес
//PARAMETERRS ARRAY 0 PARAMETER = MODBUS ADDRESS
unsigned char SET_PAR[SETUP];//0-элемент это адрес
//OBJECT ARRAY WHERE READING AND WRITING OCCURS
int res_table[OBJ_SZ];//массив с объектами то откуда мы читаем и куда пишем
float res_ftable[OBJ_SZ];
//buffer uart
#define BUF_SZ 256 //размер буфера
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //максимальное количество регистров в ответе
//uart structure
typedef struct
{
  unsigned char buffer[BUF_SZ];//буфер
  unsigned int rxtimer;//этим мы считаем таймоут
  unsigned char rxcnt; //количество принятых символов
  unsigned char txcnt;//количество переданных символов
  unsigned char txlen;//длина посылки на отправку
  unsigned char rxgap;//окончание приема
  unsigned char protocol;//тип протокола - здесь не используется
  unsigned char delay;//задержка
  char ddddddDOBAVKA[2];
} UART_DATA;
UART_DATA uart3,uart1;//структуры для соответсвующих усартов
void MODBUS_SLAVE(UART_DATA *MODBUS);//функция обработки модбас и формирования ответа
// /////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int Crc16(unsigned char *ptrByte, int byte_cnt);
void TX_03_04(UART_DATA *MODBUS);
void TX_06(UART_DATA *MODBUS);
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type);

float schitatfTemp(char* imya);
int schitatiTemp(char* imya);
void TX_66(UART_DATA *MODBUS);
void oprosite (void);
void net_tx3(UART_DATA *uart);
void net_tx1(UART_DATA *uart);

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
int DHT11_read000(struct DHT11_Dev* dev);
void wwdgenable(void);
void WWDG_IRQHandler(void);
void iwdg_init(void);
void TIM4_IRQHandler(void);
int DHT11_read002(struct DHT11_Dev* dev);
