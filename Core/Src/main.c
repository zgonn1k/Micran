
#include "main.h"


static void MX_USART2_UART_Init(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);

int __io_putchar(int ch);

uint8_t  TxBuffer[BUFFER_SIZE];
uint8_t  byteSent;
char	 UART_TxBuffer[256];
uint8_t  dataRegister_8bit[112];
uint32_t dataRegister[DATA_SIZE];

Data_t STPMC1_Data;
Parameters_t Parameters;
ActualValue_t ActualValue;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_DMA_Init();
	TIM7_Init();
	TIM2_CH1_PWM_Init();
	TIM4_CH2_PWM_Init();

	STPMC1_Init();
	STPMC1_Writing(temporary);

	LL_mDelay(100);

	while (1)
	{
		STPMC1_Reading(&Data, dataRegister);
		STPMC1_DataUnpacking(&Data);

		ActualValue.momVoltage[PHASE_R] = STPMC1_GetMomVoltage(PHASE_R);
		ActualValue.rmsVoltage[PHASE_R] = STPMC1_GetRMSVoltage(PHASE_R);
		LL_mDelay(100);
	}
}


int __io_putchar(int ch)
{
    ITM_SendChar(ch);
    return ch;
}

void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
	{
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSE_Enable();

	while (LL_RCC_HSE_IsReady() != 1)
	{

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();

	while (LL_RCC_PLL_IsReady() != 1)
	{

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);


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
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;    /* 2,5 MBits/s */
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	LL_SPI_Init(SPI1, &SPI_InitStruct);
	LL_SPI_EnableDMAReq_RX(SPI1);



}


static void MX_DMA_Init(void)
{

	LL_DMA_InitTypeDef DMA_InitStruct =	{ 0 };

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	DMA_InitStruct.Channel = LL_DMA_CHANNEL_3;
	DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_InitStruct.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
	DMA_InitStruct.MemBurst = LL_DMA_MBURST_SINGLE;
	DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)dataRegister_8bit;
	DMA_InitStruct.MemoryOrM2MDstDataSize = sizeof(dataRegister_8bit);
	DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
	DMA_InitStruct.NbData = sizeof(dataRegister_8bit);
	DMA_InitStruct.PeriphBurst = LL_DMA_PBURST_SINGLE;
	DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&SPI1->DR);
	DMA_InitStruct.PeriphOrM2MSrcDataSize = sizeof(uint8_t);
	DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;
	LL_DMA_Init(DMA2, LL_DMA_STREAM_0, &DMA_InitStruct);

	//NVIC_SetPriority(DMA2_Stream0_IRQn,
	//NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	//NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =	{ 0 };

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	LL_GPIO_ResetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);

	GPIO_InitStruct.Pin = SPI1_SYN_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SPI1_SYN_GPIO_Port, &GPIO_InitStruct);

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





static void MX_USART2_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

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

  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

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

}




void TIM7_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	TIM7->PSC =  84 - 1; // 84 000 000/(84) => 1мкс
	TIM7->ARR =  0xFFFF - 1;
	TIM7->CR1 |= TIM_CR1_URS;
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;

}




void usDelay(uint32_t time)
{
	TIM7->CNT = 0;
	while (TIM7->CNT < time);
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
	Parameters.Au = 16;
	Parameters.len_i = 65536;
	Parameters.len_u = 4096;
	Parameters.len_isum = 4096;
	Parameters.Kint_comp = 1.004;
	Parameters.Fm = 4000000;
	Parameters.Kut = 2;
	Parameters.Vref = 1.23;
	Parameters.Kdiv = (1 + Parameters.R1 / Parameters.R2);
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
		float Kint = 0.815;
		float Kdif = 0.6135;
		uint32_t x_u;
		uint32_t x_i;



		switch (elecParam)
		{
		case VOLTAGE :
		{
			float Kdspu = Kdif * Kint;

			if (type == MOMENTARY)
			{
				x_u = Data->momVoltage[phase];
			}
			else
			{
				x_u = Data->rmsVoltage[phase];
			}
			actualValue = Parameters->Kdiv * x_u
					* Parameters->Vref
					/ (Parameters->Kut * Parameters->Ku * Parameters->Au
							* Parameters->len_u * Parameters->Kint_comp * Kdspu);
		}
			break;
		case CURRENT :
		{
			float Kdspi = Kdif;

			if (type == MOMENTARY)
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
