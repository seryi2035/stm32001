/*//MODBUS
//#include "modbus.h"
#define OBJ_SZ 123 //это количество объектов
#define SETUP 4 //это просто количество данных в массиве 0-элемент которого означает адрес
//PARAMETERRS ARRAY 0 PARAMETER = MODBUS ADDRESS
uint8_t SET_PAR[SETUP];//0-элемент это адрес
//OBJECT ARRAY WHERE READING AND WRITING OCCURS
int res_table[OBJ_SZ];//массив с объектами то откуда мы читаем и куда пишем
float res_ftable[OBJ_SZ];
//buffer uart
#define BUF_SZ 256 //размер буфера
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //максимальное количество регистров в ответе
//uart structure
typedef struct {
  uint8_t buffer[BUF_SZ];//буфер
  uint16_t rxtimer;//этим мы считаем таймоут
  uint8_t rxcnt; //количество принятых символов
  uint8_t txcnt;//количество переданных символов
  uint8_t txlen;//длина посылки на отправку
  uint8_t volatile rxgap;//окончание приема
  uint8_t protocol;//тип протокола - здесь не используется
  uint16_t delay;//задержка
  uint8_t ddddddDOBAVKA[1];
} UART_DATA;
UART_DATA uart1;//структуры для соответсвующих усартов
void MODBUS_SLAVE(UART_DATA *MODBUS);//функция обработки модбас и формирования ответа
//timer 0.0001sec one symbol on 9600 ~1ms
//uart3.delay=30; //modbus gap 9600
//uart3.delay=10; //modbus gap 38400
// /////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t Crc16(unsigned char *ptrByte, int byte_cnt);
void TX_03_04(UART_DATA *MODBUS);
void TX_06(UART_DATA *MODBUS);
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type);
void TX_66(UART_DATA *MODBUS);
//void net_tx3(UART_DATA *uart);
void net_tx1(UART_DATA *uart);
void TX_01(UART_DATA *MODBUS);
void TX_02(UART_DATA *MODBUS);
*/
