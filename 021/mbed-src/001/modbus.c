#include "001.h"
#include "onewire.h"
#include "tim2_delay.h"
//#include "libmodbus.h"
#include "modbus.h"

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);
// Table of CRC values for high-order byte
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
// Table of CRC values for low-order byte
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

void net_tx1(UART_DATA *uart) {

    //GPIO_WriteBit(USART1PPport,USART1PPpin,Bit_SET);
    if((uart->txlen>0)&(uart->txcnt==0)) {
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //выкл прерывание на прием
        USART_ITConfig(USART1, USART_IT_TC, ENABLE); //включаем на окочание передачи
        //включаем rs485 на передачу
        GPIO_WriteBit(USART1PPport,USART1PPpin,Bit_SET);
        /*for (uart->txcnt=0; uart->txcnt < uart->txlen; uart->txcnt++) {
          USART_SendData(USART1,(u16) uart->buffer[uart->txcnt]);
          while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
            {
            }
        }*/
        USART01Send(uart1.buffer);
    }
}

void MODBUS_SLAVE(UART_DATA *MODBUS) {
    uint16_t tmp, tmp001;
    //recive and checking rx query
    if((MODBUS->buffer[0] != 0) && (MODBUS->rxcnt > 5) && ((MODBUS->buffer[0]==SET_PAR[0]) ||(MODBUS->buffer[0]==255)))  {
        tmp=crc16(MODBUS->buffer,MODBUS->rxcnt-2);
        tmp001 = (uint16_t) ( ( (uint16_t)MODBUS->buffer[MODBUS->rxcnt - 2])<<8)+MODBUS->buffer[MODBUS->rxcnt - 1];
        if(tmp == tmp001)  {
            //если мы сюда попали значит пакет наш и crc совпало - надо проверить поддерживаем ли мы такой запрос
            //choosing function
            switch(MODBUS->buffer[1])
            {
            case 1:
                TX_01(MODBUS);
                break;
            case 2:
                TX_02(MODBUS);
                break;
            case 3:
                TX_03_04(MODBUS);
                break;
            case 4:
                TX_03_04(MODBUS);
                break;
            case 5:
                TX_05(MODBUS);
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
            tmp=crc16(MODBUS->buffer,MODBUS->txlen-2);
            MODBUS->buffer[MODBUS->txlen-1]=(uint8_t) tmp;
            MODBUS->buffer[MODBUS->txlen-2]=(uint8_t) (tmp>>8);
            MODBUS->txcnt=0;
        } /*else {
          TX_EXCEPTION(MODBUS,0x03);
          tmp=crc16(MODBUS->buffer,MODBUS->txlen-2);
          MODBUS->buffer[MODBUS->txlen-1]=(uint8_t) tmp;
          MODBUS->buffer[MODBUS->txlen-2]=(uint8_t) (tmp>>8);
          MODBUS->txcnt=0;
        }*/
    }
    //сброс индикаторов окончания посылки
    MODBUS->rxgap=0;
    MODBUS->rxcnt=0;
    MODBUS->rxtimer=0xFFFF;
}
void TX_03_04(UART_DATA *MODBUS)
{
    uint32_t tmp,tmp1;
    uint32_t m=0,n=0;
    int tmp_val,tmp_val_pos;

    //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
    //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
    //MODBUS->buffer[2] = data byte count

    //2-3  - starting address
    tmp=(uint32_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения

    //4-5 - number of registers
    tmp1=(uint32_t)((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);//количество регистров для чтения

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
    uint32_t tmp;

    //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
    //MODBUS[1] = 6; //query type - - stay a same as in recived

    //2-3  - adress   , 4-5 - value

    tmp=(uint32_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //adress

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

    uint32_t tmp;

    //MODBUS[0] =SET_PAR[0]; // adress - stays a same as in recived
    //MODBUS[1] = 6; //query type - - stay a same as in recived

    //2-3  - adress   , 4-5 - value

    tmp=(uint32_t) ((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //adress

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

void TX_01(UART_DATA *MODBUS) {
    read_Coils_RW();
    uint32_t tmp,tmp1;
    uint32_t m=0,n=0;
    int tmp_val;
    u16 tmp_val_pos = 0, tmp001 = 0, tmp002;

    //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
    //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
    //MODBUS->buffer[2] = data byte count

    //2-3  - starting address
    tmp=(uint32_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения

    //4-5 - number of registers
    tmp1=(uint32_t)((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);//количество регистров для чтения

    //default answer length if error
    n=3;

    //если нас устраивает длина запроса и и стартовый адрес
    if((((tmp+tmp1)<OBJ_SZ)&(tmp1<MODBUS_WRD_SZ+1))) {
        for(m=0;m<tmp1;m++)  {
            //m == 0 ? tmp001 = 1 : tmp001 *= 2;
            if (tmp001 == 0) {
                tmp001 = 1;
            } else {
                tmp001 *= 2;
            }
            tmp_val= Coils_RW[m+tmp];//читаем текущее значение
            if (tmp_val) {
                tmp_val_pos = tmp_val_pos + tmp001;
            }
            if (m > 3 && (m+1)% 16 == 0) {
                MODBUS->buffer[n]=(uint8_t) (tmp_val_pos>>8);
                MODBUS->buffer[n+1]=(uint8_t) tmp_val_pos;
                n=n+2;
                tmp_val_pos = 0;
                tmp001 = 0;
            }
        }
        if (tmp1 <= 16) {
            MODBUS->buffer[n]=(uint8_t) (tmp_val_pos>>8);
            MODBUS->buffer[n+1]=(uint8_t) tmp_val_pos;
            n=n+2;
        }
        //m%8 == 0 ? tmp002 = 0 : tmp002 = 1;
        if (tmp1%8 == 0) {
            tmp002 = 0;
        } else {
            tmp002 = 1;
        }
        tmp001 = tmp1/8;
        m = tmp001 + tmp002;
        //запишем длину переменных пакета в байтах и вставим всообщение
        MODBUS->buffer[2]=(uint8_t) (m); //byte count
        //подготовим к отправке
        MODBUS->txlen=(uint8_t) (m+5); //responce length
    }
    else  {
        //exception illegal data adress 0x02
        TX_EXCEPTION(MODBUS,0x02);
    }
}
void TX_02(UART_DATA *MODBUS) {
    read_Discrete_Inputs_RO();
    uint32_t tmp,tmp1;
    uint32_t m=0,n=0;
    int tmp_val;
    u16 tmp_val_pos = 0, tmp001 = 0, tmp002;

    //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
    //MODBUS->buffer[1] = 3; //query type - - stay a same as in recived
    //MODBUS->buffer[2] = data byte count

    //2-3  - starting address
    tmp=(uint32_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); //стратовый адрес для чтения

    //4-5 - number of registers
    tmp1=(uint32_t)((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);//количество регистров для чтения

    //default answer length if error
    n=3;

    //если нас устраивает длина запроса и и стартовый адрес
    if((((tmp+tmp1)<OBJ_SZ)&(tmp1<MODBUS_WRD_SZ+1))) {
        for(m=0;m<tmp1;m++)  {
            //m == 0 ? tmp001 = 1 : tmp001 *= 2;
            if (tmp001 == 0) {
                tmp001 = 1;
            } else {
                tmp001 *= 2;
            }
            tmp_val= Discrete_Inputs_RO[m+tmp];//читаем текущее значение
            if (tmp_val) {
                tmp_val_pos = tmp_val_pos + tmp001;
            }
            if (m > 3 && (m+1)% 16 == 0) {
                MODBUS->buffer[n]=(uint8_t) (tmp_val_pos>>8);
                MODBUS->buffer[n+1]=(uint8_t) tmp_val_pos;
                n=n+2;
                tmp_val_pos = 0;
                tmp001 = 0;
            }
        }
        if (tmp1 <= 16) {
            MODBUS->buffer[n]=(uint8_t) (tmp_val_pos>>8);
            MODBUS->buffer[n+1]=(uint8_t) tmp_val_pos;
            n=n+2;
        }
        //m%8 == 0 ? tmp002 = 0 : tmp002 = 1;
        if (tmp1%8 == 0) {
            tmp002 = 0;
        } else {
            tmp002 = 1;
        }
        tmp001 = tmp1/8;
        m = tmp001 + tmp002;
        //запишем длину переменных пакета в байтах и вставим всообщение
        MODBUS->buffer[2]=(uint8_t) (m); //byte count
        //подготовим к отправке
        MODBUS->txlen=(uint8_t) (m+5); //responce length
    }
    else  {
        //exception illegal data adress 0x02
        TX_EXCEPTION(MODBUS,0x02);
    }
}

void setCOILS(uint8_t *Coils_RW) {
    if (Coils_RW[0]) { GPIO_ResetBits(GPIOC, GPIO_Pin_13);  } else { GPIO_SetBits(GPIOC, GPIO_Pin_13); }
    if (Coils_RW[1]) { GPIO_SetBits(GPIOB, GPIO_Pin_11);    } else { GPIO_ResetBits(GPIOB, GPIO_Pin_11);  }
    if (Coils_RW[2]) { GPIO_SetBits(GPIOB, GPIO_Pin_10);    } else { GPIO_ResetBits(GPIOB, GPIO_Pin_10);  }
    if (Coils_RW[3]) { GPIO_SetBits(GPIOB, GPIO_Pin_1);     } else { GPIO_ResetBits(GPIOB, GPIO_Pin_1);   }
    if (Coils_RW[4]) { GPIO_SetBits(GPIOB, GPIO_Pin_0);     } else { GPIO_ResetBits(GPIOB, GPIO_Pin_0);   }
}
void read_Discrete_Inputs_RO(void) {

    if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == (uint8_t)Bit_SET) { Discrete_Inputs_RO[0] = 1; }else{ Discrete_Inputs_RO[0] = 0;}
    //GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11) == (uint8_t)Bit_SET ? Discrete_Inputs_RO[1] = 1 : Discrete_Inputs_RO[1] = 0;
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11) == (uint8_t)Bit_SET) { Discrete_Inputs_RO[1] = 1; }else{ Discrete_Inputs_RO[1] = 0;}
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_10) == (uint8_t)Bit_SET) { Discrete_Inputs_RO[2] = 1; }else{ Discrete_Inputs_RO[2] = 0;}
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1)  == (uint8_t)Bit_SET) { Discrete_Inputs_RO[3] = 1; }else{ Discrete_Inputs_RO[3] = 0;}
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)  == (uint8_t)Bit_SET) { Discrete_Inputs_RO[4] = 1; }else{ Discrete_Inputs_RO[4] = 0;}
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[5] = 1; }else{ Discrete_Inputs_RO[5] = 0;}
    //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == (uint8_t)Bit_SET ? Discrete_Inputs_RO[6] = 1; : Discrete_Inputs_RO[6] = 0;
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[6] = 1; }else{ Discrete_Inputs_RO[6] = 0;}
    //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == (uint8_t)Bit_SET ? Discrete_Inputs_RO[7] = 1; : Discrete_Inputs_RO[7] = 0;
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)   == (uint8_t)Bit_SET) { Discrete_Inputs_RO[7] = 1; }else{ Discrete_Inputs_RO[7] = 0;}
    /*Discrete_Inputs_RO[0] = 1;
  Discrete_Inputs_RO[1] = 1;
  Discrete_Inputs_RO[2] = 0;
  Discrete_Inputs_RO[3] = 1;
  Discrete_Inputs_RO[4] = 1;
  Discrete_Inputs_RO[5] = 1;
  Discrete_Inputs_RO[6] = 1;
  Discrete_Inputs_RO[7] = 1;*/
    for(u8 i = 9; i < 16; i++) {
        Discrete_Inputs_RO[i] = Discrete_Inputs_RO[i-9];
    }
    Discrete_Inputs_RO[8] = Discrete_Inputs_RO[7];
    for(u8 i = 16; i < 32; i++) {
        Discrete_Inputs_RO[i] = Discrete_Inputs_RO[i-16];
    }
}
void startCOILS(uint8_t *Coils_RW) {
    for(u8 i = 0; i < 32; i++) {
        Coils_RW[i] = 0;
    }
}
void read_Coils_RW(void) {

    if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == (uint8_t)Bit_SET) { Coils_RW[0] = 1; }else{ Coils_RW[0] = 0;}
    //GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11) == (uint8_t)Bit_SET ? Coils_RW[1] = 1 : Coils_RW[1] = 0;
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11) == (uint8_t)Bit_SET) { Coils_RW[1] = 1; }else{ Coils_RW[1] = 0;}
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_10) == (uint8_t)Bit_SET) { Coils_RW[2] = 1; }else{ Coils_RW[2] = 0;}
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1)  == (uint8_t)Bit_SET) { Coils_RW[3] = 1; }else{ Coils_RW[3] = 0;}
    if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0)  == (uint8_t)Bit_SET) { Coils_RW[4] = 1; }else{ Coils_RW[4] = 0;}
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)   == (uint8_t)Bit_SET) { Coils_RW[5] = 1; }else{ Coils_RW[5] = 0;}
    //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == (uint8_t)Bit_SET ? Coils_RW[6] = 1; : Coils_RW[6] = 0;
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)   == (uint8_t)Bit_SET) { Coils_RW[6] = 1; }else{ Coils_RW[6] = 0;}
    //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == (uint8_t)Bit_SET ? Coils_RW[7] = 1; : Coils_RW[7] = 0;
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)   == (uint8_t)Bit_SET) { Coils_RW[7] = 1; }else{ Coils_RW[7] = 0;}
    Coils_RW[8]  = 1;
    Coils_RW[9]  = 1;
    Coils_RW[10] = 1;
    Coils_RW[11] = 1;
    Coils_RW[12] = 1;
    Coils_RW[13] = 1;
    Coils_RW[14] = 1;
    Coils_RW[15] = 1;
    for(u8 i = 16; i < 32; i++) {
        Coils_RW[i] = Coils_RW[i-16];
    }
}
void TX_05(UART_DATA *MODBUS) {
    uint16_t tmp,tmp1;
    //MODBUS->buffer[0] =SET_PAR[0]; // adress - stays a same as in received
    //MODBUS->buffer[1] = 5; //query type - - stay a same as in recived
    //2-3  -  address
    tmp=(uint16_t)((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]); // адрес
    //4-5 - значение 0xff00 ON / 0x0000 OFF
    tmp1=(uint16_t)((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);
    //если нас устраивает
    if(tmp <= 32) {
        if (tmp1 == (uint16_t) 0xff00) {
            Coils_RW[tmp-1] = 1;
        } else if (tmp1 == 0x0000) {
            Coils_RW[tmp-1] = 0;
        } else {
            TX_EXCEPTION(MODBUS,0x03);
        }
        //подготовим к отправке
        MODBUS->txlen=(uint8_t) (8); //responce length
    }
    else  {
        //exception illegal data adress 0x02
        TX_EXCEPTION(MODBUS,0x02);
    }
    setCOILS(Coils_RW);
}

