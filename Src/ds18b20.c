#include "ds18b20.h"
//--------------------------------------------------
void DelayMicro(__IO uint32_t micros)
{
	//	micros *= ((SystemCoreClock / 1000000)/9);
/* Wait till done */
   micros *= 8;	
	
while (micros--) ;
}
//--------------------------------------------------
void port_init(void)
{
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
	
 __HAL_RCC_GPIOA_CLK_ENABLE();
	
		/*Configure GPIO pin : PA0 */
	GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
	GPIO_InitStructure.Speed= GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	
}
////--------------------------------------------------
//uint8_t ds18b20_Reset(void)
//{
//  uint16_t status;
//	GPIOA->ODR &= ~GPIO_PIN_0;//низкий уровень
//  DelayMicro(485);//задержка как минимум на 480 микросекунд
//  GPIOA->ODR |= GPIO_PIN_0;//высокий уровень
//  DelayMicro(65);//задержка как минимум на 60 микросекунд
//  status = GPIOA->IDR & GPIO_PIN_0;//провер€ем уровень
//  DelayMicro(500);//задержка как минимум на 480 микросекунд
//  //(на вс€кий случай подождЄм побольше, так как могут быть неточности в задержке)
//  return (status ? 1 : 0);//вернЄм результат
//}
//----------------------------------------------------------
//uint8_t ds18b20_ReadBit(void)
//{
//  uint8_t bit = 0;
//  GPIOA->ODR &= ~GPIO_PIN_0;//низкий уровень
//  DelayMicro(2);
//	GPIOA->ODR |= GPIO_PIN_0;//высокий уровень
//	DelayMicro(13);
//	bit = (GPIOA->IDR & GPIO_PIN_0 ? 1 : 0);//провер€ем уровень	
//	DelayMicro(45);
//  return bit;
//}
////-----------------------------------------------
//uint8_t ds18b20_ReadByte(void)
//{
//  uint8_t data = 0;
//  for (uint8_t i = 0; i <= 7; i++)
//  data += ds18b20_ReadBit() << i;
//  return data;
//}
//-----------------------------------------------
//void ds18b20_WriteBit(uint8_t bit)
//{
//  GPIOA->ODR &= ~GPIO_PIN_0;
//  DelayMicro(bit ? 3 : 65);
//  GPIOA->ODR |= GPIO_PIN_0;
//  DelayMicro(bit ? 65 : 3);
//}
////-----------------------------------------------
//void ds18b20_WriteByte(uint8_t dt)
//{
//  for (uint8_t i = 0; i < 8; i++)
//  {
//    ds18b20_WriteBit(dt >> i & 1);
//    //Delay Protection
//    DelayMicro(5);
//  }
//}
//-----------------------------------------------
//uint8_t ds18b20_init(uint8_t mode)
//{
//	if(ds18b20_Reset()) return 1;
//  if(mode==SKIP_ROM)
//  {
//		//SKIP ROM
//		ds18b20_WriteByte(0xCC);
//		//WRITE SCRATCHPAD
//		ds18b20_WriteByte(0x4E);
//		//TH REGISTER 100 градусов
//		//ds18b20_WriteByte(0x64);
//		//TL REGISTER - 30 градусо
//	//	ds18b20_WriteByte(0x9E);
//		//Resolution 12 bit
//		ds18b20_WriteByte(RESOLUTION_12BIT);
//  }
//  return 0;
//}
//----------------------------------------------------------
//void ds18b20_MeasureTemperCmd(uint8_t mode, uint8_t DevNum)
//{
//  ds18b20_Reset();
//	DelayMicro(270);
//  if(mode==SKIP_ROM)
// // {
//    //SKIP ROM
//    ds18b20_WriteByte(0xCC);
// // }
//  //CONVERT T
//  ds18b20_WriteByte(0x44);
//}
//----------------------------------------------------------
//void ds18b20_ReadStratcpad(uint8_t mode, uint8_t *Data, uint8_t DevNum)
//{
//  uint8_t i;
//  ds18b20_Reset();
//	DelayMicro(270);
// // if(mode==SKIP_ROM)
// // {
//    //SKIP ROM
//    ds18b20_WriteByte(0xCC);
// // }
//  //READ SCRATCHPAD
//  ds18b20_WriteByte(0xBE);
//  for(i=0;i < 8;i++)
//  {
//    Data[i] = ds18b20_ReadByte();
//  }
//}
//----------------------------------------------------------
//uint8_t ds18b20_GetSign(uint16_t dt)
//{
//  //ѕроверим 11-й бит
//  if (dt&(1<<11)) return 1;
//  else return 0;
//}
//----------------------------------------------------------
//float ds18b20_Convert(uint16_t dt)
//{
//  float t;
//  t = (float) ((dt&0x07FF)>>4); //отборосим знаковые и дробные биты
//  //ѕрибавим дробную часть
//  t += (float)(dt&0x000F) / 16.0f;
//  return t;
//}
//----------------------------------------------------------

//__STATIC_INLINE void Micro(__IO uint32_t micros)
//{
//		micros *= ((SystemCoreClock / 1000000))/7;
//	
///* Wait till done */
//while (micros--) ;
//}
