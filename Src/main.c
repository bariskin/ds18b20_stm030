/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ds18b20.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t ID_buffer[8];
uint8_t *pointer;
uint32_t delaymicrosec;
uint16_t data;

 uint16_t  temp1;
 uint16_t  temp2;			
 uint8_t k;
 uint8_t p;
 float temper;
 
 uint8_t flag_off_on = 0;
 
 
 float T_MIN = 5.0;
 float T_MAX = 30.0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
//------------------------------------------------------------------
//------------------------------------------------------------------	
void one_wire_write_bit(uint8_t bit){
	GPIOA->ODR &= ~GPIO_PIN_0;
  DelayMicro(bit ? 3 : 32); //  1:0
	GPIOA->ODR |= GPIO_PIN_0;
	DelayMicro(bit ? 40 : 12); //  1:0
	DelayMicro(3); //  min 1 microsecunda mejdu bitami 
}
//------------------------------------------------------------------
//------------------------------------------------------------------

uint8_t one_wire_read_bit(){
	uint8_t bit;
	GPIOA->ODR &= ~GPIO_PIN_0;
	DelayMicro(5);            //слот считывания 
	GPIOA->ODR |= GPIO_PIN_0;
	DelayMicro(5);	
	bit = (GPIOA->IDR&GPIO_PIN_0?1:0);
	DelayMicro(20);// до окончания слота 
 
  DelayMicro(2); // между битами
	
	return bit;	 
}
uint8_t one_wire_read_byte(){
	
  uint8_t data1 = 0 ;
	uint8_t i;
  for (i = 0; i < 8; i++) 
	    { 
       data1 += (uint8_t)one_wire_read_bit()<<i;
			 DelayMicro(2);
	    }
  return data1;
}
	
uint8_t send_presence(){ 	
   
	  uint8_t c;

    GPIOA->ODR = GPIO_PIN_0;
  	DelayMicro(10);
	  GPIOA->ODR = 0;
	  DelayMicro(270); // минимум 480us	
	  GPIOA->ODR = GPIO_PIN_0;
		DelayMicro(16); /// для резистора  + пауза 32 микросекунды 
		
		for ( c = 0; c<60; c++) {// 120 микросекунд 
     if (!(GPIOA->IDR&GPIO_PIN_0)) {
      // Если обнаружен импульс присутствия, ждём его окончания
      while (!(GPIOA->IDR&GPIO_PIN_0)) { } // Ждём конца сигнала PRESENCE
       return 1;
      }
   	DelayMicro(1);
  }
 return 0;	
		// сделать  паузу 32 микросекунды 
		// длительность презенс  115 микросекунды 
}	
		
void one_wire_write_byte(uint8_t data){  
  int i;
	for( i = 0; i<8; i++) 		
	one_wire_write_bit(data>>i & 1);
}

	
	
int main(void)
{
  /* USER CODE BEGIN 1 */
	

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
   void one_wire_write_byte(uint8_t data);
	 uint8_t send_presence(void);	
	 uint8_t one_wire_read_byte(void);
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
  /* USER CODE BEGIN 2 */
 
 // Тестированиt выхода, мигнет два раз 
 //**************************************************
 for(int i = 0;i  < 2;i++){
	 
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(500); 
 }	
//*************************************************** 
	HAL_GPIO_WritePin( GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		
	port_init();
	
  /* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
if(send_presence())
	   {
     	DelayMicro(100);
	    one_wire_write_byte(0x33);
       DelayMicro(50); 
       for (p = 0; p < 8; p++) {
      ID_buffer[p] = one_wire_read_byte();	
      }
	   }
	    pointer = ID_buffer;
						
	send_presence();
	DelayMicro(270);										
	one_wire_write_byte(0xCC);  // SKIP ROM [CCh]  - пропуск ROM 
	one_wire_write_byte(0x4E);  // Write Scratchpad [4Eh]; "запись в память"
	one_wire_write_byte(0x4B);  // байт для регистра TH
	one_wire_write_byte(0x46);  // байт для регистра TL
	one_wire_write_byte(0x7F);  // байт конфигурации, разрешающая способность 12 бит, времея конвертации минимум 750 мсек

  while (1)
  {		
	/* USER CODE END WHILE */
	
  /* USER CODE END 3 */
 
  HAL_Delay(800);
	
	send_presence();
	DelayMicro(270);
										
	one_wire_write_byte(0xCC);   // SKIP ROM [CCh]
	one_wire_write_byte(0x44);   // Convert T
	
  HAL_Delay(800);              // Время для конвертирования температуры минимум 750 мсек
	
	send_presence();
	DelayMicro(270);
	
	one_wire_write_byte(0xCC);   // SKIP ROM [CCh]
	one_wire_write_byte(0xBE);   // Read Scratchpad [BEh]
	
	DelayMicro(30);
  data = 0;
	for( k = 0; k < 16; k++){
         data += (uint16_t)one_wire_read_bit()<<k;
	     }      	
	
             temp2 = data;	
			 
             temper = (float)data/16.0;   // для положительных температур
			 
			       temp1 = (temp2>>11)&0x0001;  // " + "  or " - "
			     	 
	  if(temp1==1){    
                temper = (float)(data - 4096)/16.0;// для отрицательных температур 
	            }
	
	
//Для управления температурой включения и выключения	нагрузки 
//********************************************************	
	if((temper < T_MIN)){
		
  	//flag_off_on = 1;
	  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
	
 else if((temper > T_MAX)) {
		//flag_off_on = 0;
	  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	}
 
  //temper = 0;
  //temp1 = 0; 
  //temp2 = 0; 
  }
//********************************************************	
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1  */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


