//MODBUS
//#include "modbus.h"
#define OBJ_SZ 36 //это количество объектов
#define SETUP 4 //это просто количество данных в массиве 0-элемент которого означает адрес
//PARAMETERRS ARRAY 0 PARAMETER = MODBUS ADDRESS
uint8_t SET_PAR[SETUP];//0-элемент это адрес
//OBJECT ARRAY WHERE READING AND WRITING OCCURS
//uint16_t res_table[OBJ_SZ];//массив с объектами то откуда мы читаем и куда пишем
//float res_ftable[OBJ_SZ];
//buffer uart
#define BUF_SZ 128 //размер буфера
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //максимальное количество регистров в ответе
//uart structure
typedef struct UART_DATA {
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
struct UART_DATA uart1;//структуры для соответсвующих усартов
//timer 0.0001sec one symbol on 9600 ~1ms
//uart3.delay=30; //modbus gap 9600
//uart3.delay=10; //modbus gap 38400
// /////////////////////////////////////////////////////////////////////////////////////////////////////
void net_tx1(UART_DATA *uart);
void MODBUS_SLAVE(UART_DATA *MODBUS);//функция обработки модбас и формирования ответа
void TX_06(UART_DATA *MODBUS);
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type);

void TX_01(UART_DATA *MODBUS);
void TX_02(UART_DATA *MODBUS);

uint8_t Coils_RW[OBJ_SZ];
uint8_t Discrete_Inputs_RO[OBJ_SZ];
void setCOILS(uint8_t *Coils_RW);
void read_Discrete_Inputs_RO(void);
//void startCOILS(uint8_t *Coils_RW);
void read_Coils_RW(void);
void TX_05(UART_DATA *MODBUS);
void TX_04(UART_DATA *MODBUS);
void TX_03(UART_DATA *MODBUS);
union FloatU8  {
  float tmp_val_float;
  uint8_t tmp_val_u8[4];
  uint32_t tmp_val_u32;
  uint16_t tmp_val_u16[2];
  int16_t tmp_val_i16[2];
  int32_t tmp_val_i32;
} f001;
//typedef union FloatU8 ;
void coilTOback(void);
void coilFROMback(void);
void TX_16(UART_DATA *MODBUS);
typedef union   {
  float tmp_float[OBJ_SZ];
  uint8_t tmp_u8[OBJ_SZ*4];
  uint32_t tmp_u32[OBJ_SZ];
  uint16_t tmp_u16[OBJ_SZ*2];
  int16_t tmp_i16[OBJ_SZ*2];
  int32_t tmp_i32[OBJ_SZ];
} REGISTRS001;
REGISTRS001 hold_reg;
REGISTRS001 input_reg;
//hold_reg, input_reg;
