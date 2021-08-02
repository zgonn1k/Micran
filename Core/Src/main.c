
#include "main.h"

void SystemClock_Config(void);
void TIM4_CH2_PWM_Init(void);
void TIM2_CH1_PWM_Init(void);
void TIM7_Init(void);
void usDelay(uint32_t time);
void STPMC1_Writing(WritingMode mode);
void STPMC1_Reading(Data_t *data);
void Send_Byte(STPMC1_cfg_bits_address, uint8_t bitState);
void Writing_Mode_Enable(void);
void Writing_Mode_Disable(void);
void STPMC1_Data_Init(Data_t *Data);
float STPMC1_GetMomVoltage(uint8_t phase);
float STPMC1_GetMomCurrent(uint8_t phase);
float STPMC1_GetRMSVoltage(uint8_t phase);
float STPMC1_GetRMSCurrent(uint8_t phase);
float ConversionToActualValue(uint8_t elecParam, uint8_t type, uint8_t phase, Parameters_t *const Parameters, Data_t *const Data);
static void MX_USART2_UART_Init(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);

uint32_t currTick;
uint8_t  TxBuffer[BUFFER_SIZE];
uint8_t  byteSent;
char	 UART_TxBuffer[256];

Data_t Data;
Parameters_t Parameters;

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
	MX_DMA_Init();
	TIM2_CH1_PWM_Init();
	TIM4_CH2_PWM_Init();

	STPMC1_Writing(temporary);

	LL_mDelay(100);

	while (1)
	{
		STPMC1_Reading(&Data);
		STPMC1_Data_Init(&Data);


		sprintf(UART_TxBuffer, "Mom Voltage = %lf\r\n", STPMC1_GetMomVoltage(PHASE_T)); // @suppress("Float formatting support")
				SendString(UART_TxBuffer);

		sprintf(UART_TxBuffer, "Mom Current = %lf\r\n", STPMC1_GetMomCurrent(PHASE_T));	// @suppress("Float formatting support")
						SendString(UART_TxBuffer);

		sprintf(UART_TxBuffer, "RMS Voltage = %lf\r\n", STPMC1_GetRMSVoltage(PHASE_T));	// @suppress("Float formatting support")
						SendString(UART_TxBuffer);

		sprintf(UART_TxBuffer, "RMS Current = %lf\r\n", STPMC1_GetRMSCurrent(PHASE_T));	// @suppress("Float formatting support")
						SendString(UART_TxBuffer);

		LL_mDelay(100);

		/*
		while(!(SPI1->SR & SPI_SR_RXNE));

		SPI_RxBuffer = LL_SPI_ReceiveData8(SPI1);


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


	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_3);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);
}


static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SYN_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = SPI1_SS_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_12;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void STPMC1_Writing(WritingMode mode)
{

	LL_SPI_Disable(SPI1);	/*1. disable SPI */
	Writing_Mode_Enable(); 	/*2. MISO, SCLK and SS as output */
	usDelay(1000);
	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin);	/*3. pin SYN to '1' */
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin); /*4. activate SCS, then SYN */
	usDelay(5);	/*then Delay function > 30ns*/
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
	Writing_Mode_Disable();
	LL_SPI_Enable(SPI1);
}

void STPMC1_Reading(Data_t *Data)
{

	uint8_t i;
	uint8_t j;
	uint8_t SPI1_RxBuffer[DATA_SIZE * 4];
	LL_SPI_Disable(SPI1);

	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	LL_GPIO_ResetOutputPin(SYN_GPIO_Port, SYN_Pin);
	usDelay(50);
	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin);
	usDelay(1);
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	usDelay(1);
	LL_GPIO_ResetOutputPin(SYN_GPIO_Port, SYN_Pin);
	usDelay(1);
	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin);
	usDelay(1);
	LL_SPI_Enable(SPI1);


	LL_SPI_ReceiveData8(SPI1);

	for (i = 0; i < DATA_SIZE; i++)
	{
		for (j = 0; j < 4; j++)
		{
			while (LL_SPI_IsActiveFlag_RXNE(SPI1));
			dataRegister[i] = (LL_SPI_ReceiveData8(SPI1) << (3-j) * 8);
		}
	}

	LL_SPI_Disable(SPI1);

	for(uint8_t i = 0; i < DATA_SIZE; i++)
	{
		sprintf(UART_TxBuffer, "Value from first register %lu", dataRegister[i]);
		SendString(UART_TxBuffer);
	}


}


void STPMC1_Data_Init(Data_t *Data)
{
	Data->momVoltage[PHASE_R] = (int32_t) (dataRegister[DMR]&VOLTAGE_VALUES_MASK) >> 16;
	Data->momVoltage[PHASE_S] = (int32_t) (dataRegister[DMS]&VOLTAGE_VALUES_MASK) >> 16;
	Data->momVoltage[PHASE_T] = (int32_t) (dataRegister[DMT]&VOLTAGE_VALUES_MASK) >> 16;

	Data->momCurrent[PHASE_R] = (int32_t) (dataRegister[DMR]&CURRENT_VALUES_MASK);
	Data->momCurrent[PHASE_S] = (int32_t) (dataRegister[DMS]&CURRENT_VALUES_MASK);
	Data->momCurrent[PHASE_T] = (int32_t) (dataRegister[DMT]&CURRENT_VALUES_MASK);

	Data->period = (uint32_t) (dataRegister[PRD]&VOLTAGE_VALUES_MASK) >> 16;

	Data->configBits[0] = (uint32_t) (dataRegister[CF0]&CFG_MASK);
	Data->configBits[1] = (uint32_t) (dataRegister[CF1]&CFG_MASK);
	Data->configBits[2] = (uint32_t) (dataRegister[CF2]&CFG_MASK);
	Data->configBits[3] = (uint32_t) (dataRegister[CF3]&CFG_MASK);

	Data->powerActive = (uint32_t) (dataRegister[DAP]);
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


void Writing_Mode_Enable(void)
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

	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	LL_GPIO_SetOutputPin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin);
}

void Writing_Mode_Disable(void)
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

	LL_GPIO_SetOutputPin(SYN_GPIO_Port, SYN_Pin);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);


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

void STPMC1_Init(void)
{
	Parameters.R1 = 100000;
	Parameters.R2 = 470;
	Parameters.Ku = 0.9375;
	Parameters.Ki = 0.9375;
	Parameters.Kisum = 0.875;
	Parameters.Au = 4;
	Parameters.len_i = 65536;
	Parameters.len_u = 4096;
	Parameters.len_isum = 4096;
	Parameters.Kint_comp = 1.004;
	Parameters.Fm = 4000000;
	Parameters.Kut = 2;
	Parameters.Vref = 1.23;
}

float STPMC1_GetMomVoltage(uint8_t phase)
{
	float momVoltage;

	switch (phase)
	{
		case PHASE_R:
		{
			momVoltage = ConversionToActualValue(VOLTAGE, MOMENTARY, PHASE_R, &Parameters, &Data);
		}
		break;

		case PHASE_S:
		{
			momVoltage = ConversionToActualValue(VOLTAGE, MOMENTARY, PHASE_S, &Parameters, &Data);
		}
		break;

		case PHASE_T:
		{
			momVoltage = ConversionToActualValue(VOLTAGE, MOMENTARY, PHASE_T, &Parameters, &Data);
		}
		break;

		case PHASE_N:
		{
			momVoltage = ConversionToActualValue(VOLTAGE, MOMENTARY, PHASE_N, &Parameters, &Data);
		}
		break;
	}
	return momVoltage;
}

float STPMC1_GetMomCurrent(uint8_t phase)
{
	float momCurrent;

	switch (phase)
	{
		case PHASE_R:
		{
			momCurrent = ConversionToActualValue(CURRENT, MOMENTARY, PHASE_R, &Parameters, &Data);
		}
		break;

		case PHASE_S:
		{
			momCurrent = ConversionToActualValue(CURRENT, MOMENTARY, PHASE_S, &Parameters, &Data);
		}
		break;

		case PHASE_T:
		{
			momCurrent = ConversionToActualValue(CURRENT, MOMENTARY, PHASE_T, &Parameters, &Data);
		}
		break;

		case PHASE_N:
		{
			momCurrent = ConversionToActualValue(CURRENT, MOMENTARY, PHASE_N, &Parameters, &Data);
		}
		break;
	}
	return momCurrent;
}

float STPMC1_GetRMSVoltage(uint8_t phase)
{
	float RMSVoltage;

	switch (phase)
	{
		case PHASE_R:
		{
			RMSVoltage = ConversionToActualValue(VOLTAGE, RMS, PHASE_R, &Parameters, &Data);
		}
		break;

		case PHASE_S:
		{
			RMSVoltage = ConversionToActualValue(VOLTAGE, RMS, PHASE_S, &Parameters, &Data);
		}
		break;

		case PHASE_T:
		{
			RMSVoltage = ConversionToActualValue(VOLTAGE, RMS, PHASE_T, &Parameters, &Data);
		}
		break;

		case PHASE_N:
		{
			RMSVoltage = ConversionToActualValue(VOLTAGE, RMS, PHASE_N, &Parameters, &Data);
		}
		break;
	}
	return RMSVoltage;
}

float STPMC1_GetRMSCurrent(uint8_t phase)
{
	float RMSCurrent;

	switch (phase)
	{
		case PHASE_R:
		{
			RMSCurrent = ConversionToActualValue(CURRENT, RMS, PHASE_R, &Parameters, &Data);
		}
		break;

		case PHASE_S:
		{
			RMSCurrent = ConversionToActualValue(CURRENT, RMS, PHASE_S, &Parameters, &Data);
		}
		break;

		case PHASE_T:
		{
			RMSCurrent = ConversionToActualValue(CURRENT, RMS, PHASE_T, &Parameters, &Data);
		}
		break;

		case PHASE_N:
		{
			RMSCurrent = ConversionToActualValue(CURRENT, RMS, PHASE_N, &Parameters, &Data);
		}
		break;
	}
	return RMSCurrent;
}

float ConversionToActualValue(uint8_t elecParam, uint8_t type, uint8_t phase, Parameters_t *const Parameters, Data_t *const Data)
{
	float actualValue;
	float Kint = 2 * Data->period * 32 / (3.14159 * 65536);
	float Kdif = 1 / (2 * Kint);
	int32_t x_u;
	int32_t x_i;

	switch (elecParam)
	{
	case VOLTAGE:
	{
		float Kdspu = Kdif * Kint;

		if(type == MOMENTARY)
		{
			x_u = Data->momVoltage[phase];
		}
		else
		{
			x_u = Data->rmsVoltage[phase];
		}
		actualValue = (1 + Parameters->R1 / Parameters->R2)
				* x_u * Parameters->Vref
				/ (Parameters->Kut * Parameters->Ku * Parameters->Au
						* Parameters->len_u * Parameters->Kint_comp * Kdspu);
	}
		break;
	case CURRENT:
	{
		float Kdspi = Kdif;

		if(type == MOMENTARY)
				{
					x_i = Data->momCurrent[phase];
				}
				else
				{
					x_i = Data->rmsCurrent[phase];
				}

		actualValue = x_i * Parameters->Vref
				/ (Parameters->Ks * Parameters->Ki * Parameters->Ai
						* Parameters->len_i * Kint * Parameters->Kint_comp
						* Kdspi);
	}
		break;
	}
	return actualValue;
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
