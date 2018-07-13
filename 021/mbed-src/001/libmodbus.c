#include "001.h"
#include "onewire.h"
#include "tim2_delay.h"
#include "libmodbus.h"

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
