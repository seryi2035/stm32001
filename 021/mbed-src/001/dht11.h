// ////////////////////////////////////////////////////////DHT11
#ifndef DHT11_H
#define DHT11_H

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
int DHT11_read002(struct DHT11_Dev* dev);
int DHT11_read003(struct DHT11_Dev* dev);
int DHT11_read004(struct DHT11_Dev* dev);

#endif /* DHT11_H */
