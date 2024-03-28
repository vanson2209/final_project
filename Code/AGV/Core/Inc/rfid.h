#ifndef __rfid_h
#define __rfid_h
#include "stm32f1xx_hal.h"
extern SPI_HandleTypeDef hspi2;
#define MAX_LEN 16
#define	uint8_t	unsigned char
#define	uint	unsigned int
void MFRC522_Init(void);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *TagType);
uint8_t MFRC522_Anticoll(uint8_t *serNum);
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t *writeData);
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t *recvData);
void AntennaOn(void);
void AntennaOff(void);
#endif
