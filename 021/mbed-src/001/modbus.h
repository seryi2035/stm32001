#define OBJ_SZ 123 //это количество объектов
#define SETUP 4 //это просто количество данных в массиве 0-элемент которого означает адрес

//PARAMETERRS ARRAY 0 PARAMETER = MODBUS ADDRESS
unsigned char SET_PAR[SETUP];//0-элемент это адрес

//OBJECT ARRAY WHERE READING AND WRITING OCCURS
int res_table[OBJ_SZ];//массив с объектами то откуда мы читаем и куда пишем

//buffer uart
#define BUF_SZ 256 //размер буфера
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //максимальное количество регистров в ответе

//uart structure
typedef struct {
unsigned char buffer[BUF_SZ];//буфер
unsigned int rxtimer;//этим мы считаем таймоут
unsigned char rxcnt; //количество принятых символов
unsigned char txcnt;//количество переданных символов
unsigned char txlen;//длина посылки на отправку
unsigned char rxgap;//окончание приема
//unsigned char protocol;//тип протокола - здесь не используется
unsigned char delay;//задержка
} UART_DATA;

UART_DATA uart3,uart1;//структуры для соответсвующих усартов

void MODBUS_SLAVE(UART_DATA *MODBUS);//функция обработки модбас и формирования ответа
