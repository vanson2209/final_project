#include "rfid.h"
uint8_t RC522_SPI_Transfer(uint8_t data){
	uint8_t rx_data;
	HAL_SPI_TransmitReceive(&hspi2,&data,&rx_data,1,100);	
	return rx_data;
}
void Write_MFRC522(uint8_t addr, uint8_t val){
	GPIOB -> ODR &= ~GPIO_PIN_12; 	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	RC522_SPI_Transfer((addr<<1)&0x7E);	
	RC522_SPI_Transfer(val);
	GPIOB -> ODR |= GPIO_PIN_12;		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
}
uint8_t Read_MFRC522(uint8_t addr){
	uint8_t val;
	GPIOB -> ODR &= ~GPIO_PIN_12;		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	RC522_SPI_Transfer(((addr<<1)&0x7E) | 0x80);	
	val = RC522_SPI_Transfer(0x00);
	GPIOB -> ODR |= GPIO_PIN_12;		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	return val;		
}
void SetBitMask(uint8_t reg, uint8_t mask){
  uint8_t tmp;
  tmp = Read_MFRC522(reg);
  Write_MFRC522(reg, tmp | mask);  // set bit mask
}
void ClearBitMask(uint8_t reg, uint8_t mask)  {
  uint8_t tmp;
  tmp = Read_MFRC522(reg);
  Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}
void AntennaOn(void){
	Read_MFRC522(0x14);
	SetBitMask(0x14, 0x03);
}
void AntennaOff(void){
	ClearBitMask(0x14, 0x03);
}
void MFRC522_Reset(void){
  Write_MFRC522(0x01, 0x0F);
}
void MFRC522_Init(void)
{
	GPIOB -> ODR |= GPIO_PIN_12;		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	GPIOA -> ODR |= GPIO_PIN_8;			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
	MFRC522_Reset(); 	
	Write_MFRC522(0x2A, 0x8D);		//auto=1; f(Timer) = 6.78MHz/TPreScaler
	Write_MFRC522(0x2B, 0x3E);	//TModeReg[3..0] + TPrescalerReg
	Write_MFRC522(0x2D, 30);           
	Write_MFRC522(0x2C, 0);
	Write_MFRC522(0x15, 0x40);		//100%ASK
	Write_MFRC522(0x11, 0x3D);		//CRC Original value 0x6363	???	
}
uint8_t MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint *backLen)
{
    uint8_t status = 2;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint i;

    switch (command)
    {
        case 0x0E:		//Acknowledging the liver
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case 0x0C:	// FIFO data collection
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
    }
   
    Write_MFRC522(0x02, irqEn|0x80);	//Yeu cau ngat
    ClearBitMask(0x04, 0x80);			//Clear all the bits
    SetBitMask(0x0A, 0x80);			//FlushBuffer=1, FIFO
    
	Write_MFRC522(0x01, 0x00);	//NO action; Huy bo lenh hien hanh	???

	// Record in FIFO
    for (i=0; i<sendLen; i++)
    {   
		Write_MFRC522(0x09, sendData[i]);    
	}

	//chay
	Write_MFRC522(0x01, command);
    if (command == 0x0C)
    {    
		SetBitMask(0x0D, 0x80);		//StartSend=1,transmission of data starts  
	}   
    
	//The team is allowed to be stored
	i = 2000;	//i tuy thuoc tan so thach anh, thoi gian toi da cho the M1 la 25ms
    do 
    {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_MFRC522(0x04);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(0x0D, 0x80);			//StartSend=0
	
    if (i != 0)
    {    
        if(!(Read_MFRC522(0x06) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = 0;
            if (n & irqEn & 0x01)
            {   
				status = 1;			//??   
			}

            if (command == 0x0C)
            {
               	n = Read_MFRC522(0x0A);
              	lastBits = Read_MFRC522(0x0C) & 0x07;
                if (lastBits)
                {   
					*backLen = (n-1)*8 + lastBits;   
				}
                else
                {   
					*backLen = n*8;   
				}

                if (n == 0)
                {   
					n = 1;    
				}
                if (n > 16)
                {   
					n = 16;   
				}			
				//FIFO doc in the received data
                for (i=0; i<n; i++)
                {   
					backData[i] = Read_MFRC522(0x09);    
				}
            }
        }
        else
        {   
			status = 2;  
		}     
    }
    return status;
}
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *TagType)
{
	uint8_t status;  
	uint backBits;			//The bits are manipulated

	Write_MFRC522(0x0D, 0x07);		//TxLastBists = BitFramingReg[2..0]	???
	
	TagType[0] = reqMode;
	status = MFRC522_ToCard(0x0C, TagType, 1, TagType, &backBits);

	if ((status != 0) || (backBits != 0x10))
	{    
		status = 2;
	}
   
	return status;
}
uint8_t MFRC522_Anticoll(uint8_t *serNum)
{
    uint8_t status;
    uint8_t i;
	uint8_t serNumCheck=0;
    uint unLen;
    

    //ClearBitMask(Status2Reg, 0x08);		//TempSensclear
    //ClearBitMask(CollReg,0x80);			//ValuesAfterColl
	Write_MFRC522(0x0D, 0x00);		//TxLastBists = BitFramingReg[2..0]
 
    serNum[0] = 0x93;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(0x0C, serNum, 2, serNum, &unLen);

    if (status == 0)
	{
		//Check the serial number
		for (i=0; i<4; i++)
		{   
		 	serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i])
		{   
			status = 2;    
		}
    }
	    //SetBitMask(CollReg, 0x80);		//ValuesAfterColl=1
    return status;
} 
