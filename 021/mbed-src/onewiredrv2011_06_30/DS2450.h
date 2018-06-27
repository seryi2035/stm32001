/* DS2450 specific values (see datasheet www.maxim-ic.com/datasheet/index.mvp/id/2921) */
#ifndef _DS2450_
#define _DS2450_
#define DS2450_ID           0x20
#define DS2450_READ_MEMORY  0xAA
#define DS2450_WRITE_MEMORY 0x55
#define DS2450_CONVERT      0x3C

#define DS2450_SP_SIZE 13
#define DS2450_PAGE0 0x00
#define DS2450_PAGE1 0x08
#define DS2450_PAGE2 0x10
#define DS2450_PAGE3 0x18

#define DS2450_ADCA 0x00 //channel A
#define DS2450_ADCB 0x02 //channel B
#define DS2450_ADCC 0x04 //channel C
#define DS2450_ADCD 0x06 //channel D

#define DS2450_IR_2V5          0x00 //input voltage range 2.55V
#define DS2450_IR_5V1          0x01 //input voltage range 5.1V
#define DS2450_AFH              0x20 //flag alarm value higher
#define DS2450_AFL              0x10 //flag alarm value lower
#define DS2450_AEH_ENABLE       0x08 //alarm value higher enable  
#define DS2450_AEL_ENABLE       0x04 //alarm value lower enable

#define DS2450_DISABLE_OUT  0x00 //disable ouput
#define DS2450_ENABLE_OUT   0x80//enable ouput

#define DS2450_16_BIT       0x00 //ADC 16bits enable ouput
#define DS2450_15_BIT       0x0F //ADC 15bits enable ouput
#define DS2450_12_BIT       0x0C //ADC 12bits enable ouput
#define DS2450_8_BIT        0x08   //ADC 8bits enable ouput
#define DS2450_1_BIT        0x01   //ADC 1bits enable ouput
uint8_t DS2450_read_page(uint8_t id[], uint8_t adresse, uint8_t *val);
uint8_t DS2450_convert(uint8_t id[], uint8_t input_select_mask,uint8_t read_out_control);
uint8_t DS2450_read_ADC(uint8_t id[], uint16_t adc[]);
uint8_t DS2450_start_and_read_ADC(uint8_t id[], uint16_t adc[]);
uint8_t DS2450_configure_channel_ADC(uint8_t id[],uint8_t channel,uint8_t conflsb,uint8_t confmsb);
uint8_t DS2450_configure_page(uint8_t id[], uint8_t adresse,uint8_t configpage[]);
#endif