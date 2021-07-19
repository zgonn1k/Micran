
#include "main.h"

void SystemClock_Config(void);
void TIM4_CH2_PWM_Init(void);
void TIM2_CH1_PWM_Init(void);
void TIM7_Init(void);
void usDelay(uint32_t time);
void Writing_To_STPMC1(WritingMode mode);
void Reading_From_STPMC1(Data_t *data);
void Send_Byte(STPMC1_cfg_bits_address, uint8_t bitState);
void Set_Pins_For_Writing_Mode(void);
void Reset_Pins_For_Writing_Mode(void);
static void MX_USART2_UART_Init(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);


#define  PERIOD_VALUE		(uint32_t)(20 - 1)  /* Period Value  */
#define  PAUSE_VALUE		(uint32_t)(PERIOD_VALUE / 2)        /* Capture Compare  Value  */
#define  PRESCALER_VALUE	(uint16_t)83

uint32_t currTick;
uint8_t  TxBuffer[BUFFER_SIZE];
uint8_t  byteSent;
char	 UART_TxBuffer[256];

uint8_t SPI_RxBuffer;

Data_t data;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	TIM7_Init();

	TIM2_CH1_PWM_Init();
	TIM4_CH2_PWM_Init();

	Writing_To_STPMC1(temporary);



	while (1)
	{
/*
		while(!(SPI1->SR & SPI_SR_RXNE));

		SPI_RxBuffer = LL_SPI_ReceiveData8(SPI1);

		sprintf(UART_TxBuffer, "%d\r\n", SPI_RxBuffer);
		SendString(UART_TxBuffer);
		*/
/*
		LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_15);
		LL_mDelay(500);
		LL_GPIO_TogglePin(GPIOD, LL_GPIO_PIN_15);
		LL_mDelay(500);
*/
	}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
	{
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1)
	{

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168,
			LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1)
	{

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{

	}
	LL_Init1msTick(168000000);
	LL_SetSystemCoreClock(168000000);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct =	{ 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct =	{ 0 };

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**SPI1 GPIO Configuration
	 PA5   ------> SPI1_SCK
	 PA6   ------> SPI1_MISO
	 */
	GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* SPI1 parameter configuration*/
	SPI_InitStruct.TransferDirection = LL_SPI_SIMPLEX_RX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;    /* 21MBits/s */
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(SPI1, &SPI_InitStruct);
	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =	{ 0 };

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	/**/
	LL_GPIO_ResetOutputPin(SYN_GPIO_Port, SYN_Pin);


	/**/
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);

	/**/
	GPIO_InitStruct.Pin = SYN_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SYN_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = SPI1_SS_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void Writing_To_STPMC1(WritingMode mode)
{

	LL_SPI_Disable(SPI1);	/*1. disable SPI */
	Set_Pins_For_Writing_Mode(); 	/*2. MISO, SCLK and SS as output */
	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin);	/*3. pin SYN to '1' */
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin); /*4. activate SCS, then SYN */
	usDelay(1);	/*then Delay function > 30ns*/
	LL_GPIO_ResetOutputPin(SYN_GPIO_Port, SYN_Pin);	/*activate SYN*/

	if(mode == temporary)
	{
		Send_Byte(RD, 1);
	}
	else
	{
		Send_Byte(RD, 0);
	}

	Send_Byte(TSTD, 0);		/*enable test modes and system signals*/
	SendString("1st byte sent\r\n");

	Send_Byte(MDIV, 1);	/*fMCLK = fXTAL1*/
	SendString("2nd byte sent\r\n");

	Send_Byte(HSA, 0);	/*fCLK = fXTAL1/4,*/
	SendString("3rd byte sent\r\n");

	Send_Byte(APL_0, 0);
	SendString("4th byte sent\r\n");
	Send_Byte(APL_1, 0); /*APL = 0: peripheral MOP, MON=ZCR, WatchDOG, LED=pulses (X)*/
	SendString("5th byte sent\r\n");

	Send_Byte(TCS, 1);	/*Current transformer (CT) or shunt*/
	SendString("6th byte sent\r\n");

	Send_Byte(FRS, 0);	/*Nominal base frequency 50Hz*/
	SendString("7th byte sent\r\n");

	Send_Byte(FUND, 0);	/*full bandwidth active energy controls the stepper;
						  full bandwidth reactive energy computation.*/
	SendString("8th byte sent\r\n");

	Send_Byte(ART, 0);	/*natural computation*/
	SendString("9th byte sent\r\n");

	Send_Byte(MSBF, 0);	/*msb first*/
	SendString("10th byte sent\r\n");

	Send_Byte(ABS_0, 0);
	SendString("11th byte sent\r\n");
	Send_Byte(ABS_1, 0);	/*LTCH=0: 0,00125 * FS,*/
	SendString("12th byte sent\r\n");




	Send_Byte(SYS_0, 1);
	SendString("17th byte sent\r\n");
	Send_Byte(SYS_1, 1);
	SendString("18th byte sent\r\n");
	Send_Byte(SYS_2, 1);	/*SYS=7: 1-phase, 2-wire __TN, 1-system __T_*/
	SendString("19th byte sent\r\n");

	Send_Byte(PM, 1);	/*Class 0.1*/
	SendString("21th byte sent\r\n");

	Send_Byte(FR1, 0);	/*fMCLK =8.192 MHz*/
	SendString("21th byte sent\r\n");



	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin); /* deactivate SYN, then SCS*/
	usDelay(1);	/*then Delay function > 30ns*/
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin); /*deactivate SCS*/
	Reset_Pins_For_Writing_Mode();
	LL_SPI_Enable(SPI1);
}

void Reading_From_STPMC1(Data_t *data)
{
	uint8_t i;
	uint8_t j;

	LL_SPI_Disable(SPI1);

	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	LL_GPIO_ResetOutputPin(SYN_GPIO_Port, SYN_Pin);
	usDelay(5);
	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin);
	usDelay(1);
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	usDelay(1);
	LL_GPIO_ResetOutputPin(SYN_GPIO_Port, SYN_Pin);
	usDelay(1);
	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin);
	usDelay(1);
	LL_SPI_Enable(SPI1);

	for (i = 0; i < DATA_SIZE; i++)
	{
		for (j = 0; j < 4; j++)
		{
			while (LL_SPI_IsActiveFlag_RXNE(SPI1));
			data->dataRegister[i] = (LL_SPI_ReceiveData8(SPI1) << (3-j) * 8);
		}
	}

	LL_SPI_Disable(SPI1);

	for(uint8_t i = 0; i < 28; i++)
	{
		sprintf(UART_TxBuffer, "Value from first register %lu", data->dataRegister[i]);
		SendString(UART_TxBuffer);
	}
}

static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void Send_Byte(STPMC1_cfg_bits_address address, uint8_t bitState)
{
	uint8_t i = 0;
	//uint8_t binary = 0;
	uint8_t decimal = address;

	for (i = 0; i < BUFFER_SIZE - 1; i++)
	{
		TxBuffer[i] = decimal % 2;
		decimal = decimal / 2;
		//binary = binary + TxBuffer[i] * pow(10, i); //variable for binary number as integer
	}

	TxBuffer[BUFFER_SIZE - 1] = bitState;

	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->EGR |= TIM_EGR_UG;

	while (byteSent != 1 )
	{

	}
	byteSent = 0;

}


void Set_Pins_For_Writing_Mode(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =	{ 0 };


	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER6);

	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL5);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL6);

	/*MISO pins as output*/
	GPIO_InitStruct.Pin =  SPI1_MISO_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);



	/*SCK pin as PWM*/
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_0; 	/*TIM2 CH1 for PA5*/
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5_1 | GPIO_AFRL_AFRL5_2 | GPIO_AFRL_AFRL5_3);
	GPIOA->MODER |= GPIO_MODER_MODER5_1; 	/*Alternate function mode*/
}

void Reset_Pins_For_Writing_Mode(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =	{ 0 };

	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER6);

	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL5);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL6);

	/**/
	GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


void TIM7_Init(void)
{
	//TIM7 тактируется от шины APB1 (42MHz), х2 на таймеры => f = 84MHz
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	TIM7->PSC = 84 - 1; // 84 000 000/(83 + 1) => 1мкс
	TIM7->ARR = 10 - 1;
	TIM7->CR1 |= TIM_CR1_URS;
	TIM7->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM7_IRQn);

	NVIC_SetPriority(TIM7_IRQn, 1);
}




void usDelay(uint32_t Delay)
{
	TIM7->CR1 |= TIM_CR1_CEN;
	currTick = 0;
	while (currTick < Delay)		;
	TIM7->CR1 &= ~TIM_CR1_CEN;
}


void TIM4_CH2_PWM_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->CR1 &= ~TIM_CR1_CKD; /*Clock division tDTS = tCK_INT */
	TIM4->CR1 &= ~TIM_CR1_ARPE; /*ARPE: Auto-reload preload disable*/
	TIM4->CR1 &= ~TIM_CR1_CMS; /*Edge-aligned mode*/
	TIM4->CR1 &= ~TIM_CR1_DIR; /*upcounter mode*/
	TIM4->CR1 &= ~TIM_CR1_URS; /*Any events generate an update interrupt*/
	TIM4->CR1 &= ~TIM_CR1_UDIS;

	TIM4->SMCR &= ~TIM_SMCR_MSM; /*Master/Slave mode disable*/

	TIM4->DIER |= TIM_DIER_UIE; /*Update interrupt enable*/
	TIM4->DIER &= ~TIM_DIER_CC2IE; /*interrupt enabled for channel 2*/

	TIM4->CCMR1 &= ~TIM_CCMR1_CC2S; /*channel 2 is configured as output*/
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; /*PWM1 mode for channel 2*/
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE; /*preload enable*/
	TIM4->CCMR1 |= TIM_CCMR1_OC2FE; /*fast enable*/

	TIM4->CCER |= TIM_CCER_CC2P; /*active low*/
	TIM4->CCER |= TIM_CCER_CC2E; /*output enable*/

	TIM4->PSC = PRESCALER_VALUE; /*prescaler value*/

	TIM4->ARR = PERIOD_VALUE; /*auto-reload value*/

	TIM4->CCR2 = PAUSE_VALUE; /*Capture/Compare value*/

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= GPIO_MODER_MODER13_1; /*Alternate function mode*/
	GPIOD->OTYPER &= ~GPIO_OTYPER_OT13; /*Output push-pull*/
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED13; /*Very high speed*/
	GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD13; /*No pull-up, pull-down*/
	GPIOD->AFR[1] |= GPIO_AFRH_AFSEL13_1; /*Alternate function for 13 pin*/

	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 0);
}


void TIM2_CH1_PWM_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 &= ~TIM_CR1_CKD; /*Clock division tDTS = tCK_INT */
	TIM2->CR1 &= ~TIM_CR1_ARPE; /*ARPE: Auto-reload preload disable*/
	TIM2->CR1 &= ~TIM_CR1_CMS; /*Edge-aligned mode*/
	TIM2->CR1 &= ~TIM_CR1_DIR; /*upcounter mode*/
	TIM2->CR1 &= ~TIM_CR1_URS; /*Any events generate an update interrupt*/
	TIM2->CR1 &= ~TIM_CR1_UDIS;

	TIM2->SMCR &= ~TIM_SMCR_MSM; /*Master/Slave mode disable*/

	TIM2->DIER |= TIM_DIER_UIE; /*Update interrupt enable*/
	TIM2->DIER &= ~TIM_DIER_CC1IE; /*interrupt enabled for channel 1*/

	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S; /*channel 1 is configured as output*/
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; /*PWM1 mode for channel 1*/
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; /*preload enable*/
	TIM2->CCMR1 |= TIM_CCMR1_OC1FE; /*fast enable*/

	TIM2->CCER |= TIM_CCER_CC1P; /*active low*/
	TIM2->CCER |= TIM_CCER_CC1E; /*output enable*/

	TIM2->PSC = PRESCALER_VALUE; /*prescaler value*/

	TIM2->ARR = PERIOD_VALUE; /*auto-reload value*/

	TIM2->CCR1 = PAUSE_VALUE; /*Capture/Compare value*/


	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0);

	//TIM2->CR1 |= TIM_CR1_CEN;
}

void SendChar(uint8_t data)
{
    while(!(USART2->SR & USART_SR_TC));

    USART2->DR = data;
}

void SendString(char array[])
{
    for(uint8_t i = 0; i < strlen(array); i++)
        SendChar(array[i]);
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
