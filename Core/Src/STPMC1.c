#include "STPMC1.h"

static void writing_mode_enable(void);
static void writing_mode_disable(void);
static void reading_mode_enable(void);
static void stpmc1_write_byte(enum stpmc1_config_bit address, uint8_t bit_state);
static ErrorStatus is_parity_bad(uint8_t *byte, uint8_t grp);

void stpmc1_write(enum writing_mode mode)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =	{ 0 };

	LL_SPI_Disable(SPI1);

//	writing_mode_enable();

	if(mode == temporary)
	{
		stpmc1_write_byte(RD, 1);
		usDelay(TIME_BETWEEN_TX	);
	}
	else
	{
		stpmc1_write_byte(RD, 0);
		usDelay(TIME_BETWEEN_TX	);
	}

	stpmc1_write_byte(TSTD, 0);		/*enable test modes and system signals*/
	usDelay(TIME_BETWEEN_TX);

	stpmc1_write_byte(MDIV, 1);	/*fMCLK = fXTAL1 = 8 MHz*/
	usDelay(TIME_BETWEEN_TX);

	stpmc1_write_byte(HSA, 0);	/*fCLK = fXTAL1/4 = 2 MHz */
	usDelay(TIME_BETWEEN_TX);

	stpmc1_write_byte(APL_0, 0);
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(APL_1, 0); /*APL = 0: peripheral MOP, MON=ZCR, WatchDOG, LED=pulses (X)*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(TCS, 1);	/*Current transformer (CT) or shunt*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(FRS, 0);	/*Nominal base frequency 50Hz*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(FUND, 0);	/*full bandwidth active energy controls the stepper;
						  full bandwidth reactive energy computation.*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(ART, 0);	/*natural computation*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(MSBF, 0);	/*msb first*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(ABS_0, 0);
	usDelay(TIME_BETWEEN_TX	);
	stpmc1_write_byte(ABS_1, 0);	/*LTCH=0: 0,00125 * FS,*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(LTCH_0, 0);
	usDelay(TIME_BETWEEN_TX	);
	stpmc1_write_byte(LTCH_1, 0);	/*LTCH=0: 0,00125 * FS,*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(KMOT_0, 1);
	usDelay(TIME_BETWEEN_TX	);
	stpmc1_write_byte(KMOT_1, 0);
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(SYS_0, 1);
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(SYS_1, 1);
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(SYS_2, 0);	/*SYS=3: 1-phase, 2-wire __TN, 1-system __T_*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(SCLP, 0);
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(PM, 1);	/*Class 0.1*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(FR1, 0);	/*fMCLK =8.192 MHz*/
	usDelay(TIME_BETWEEN_TX	);

	stpmc1_write_byte(CHK, 1);	/*fMCLK =8.192 MHz*/
	usDelay(TIME_BETWEEN_TX	);

	writing_mode_disable();


}

/*
ErrorStatus start_spi_dma()
{
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, sizeof(dataRegister_8bit));
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, (uint32_t) (&SPI1->DR));
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t) dataRegister_8bit);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);

	return SUCCESS;
}
*/

void reading_mode_enable()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	writing_mode_disable();

	LL_SPI_Disable(SPI1);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	LL_GPIO_ResetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	usDelay(2);
	LL_GPIO_SetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	usDelay(1);
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	LL_GPIO_SetOutputPin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin);
	usDelay(1);
	LL_GPIO_ResetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	usDelay(1);
	LL_GPIO_SetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);

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



ErrorStatus stpmc1_read(uint32_t *data_buffer)
{
	uint8_t spi_rx_buffer[STPMC1_RX_BUF_SIZE];

	reading_mode_enable();

	spi_read_rx(SPI1, spi_rx_buffer, STPMC1_RX_BUF_SIZE);

	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);

	uint8_t i = 0;
	for (i = 0; i < STPMC1_RX_BUF_SIZE * 4; i++)
	{
		printf("dataReg[%d] = %x\r\n", i, spi_rx_buffer[i]);
	}

	for (i = 0; i < DATA_SIZE; i++)
	{
		uint8_t tmp;
		tmp = spi_rx_buffer[i * 4 + 0];
		spi_rx_buffer[i * 4 + 0] = spi_rx_buffer[i * 4 + 3];
		spi_rx_buffer[i * 4 + 3] = tmp;

		tmp = spi_rx_buffer[i * 4 + 1];
		spi_rx_buffer[i * 4 + 1] = spi_rx_buffer[i * 4 + 2];
		spi_rx_buffer[i * 4 + 2] = tmp;

		data_buffer[i] = spi_rx_buffer[i * 4 + 0];
		data_buffer[i] = ((data_buffer[i] << 8) | spi_rx_buffer[i * 4 + 1]);
		data_buffer[i] = ((data_buffer[i] << 8) | spi_rx_buffer[i * 4 + 2]);
		data_buffer[i] = ((data_buffer[i] << 8) | spi_rx_buffer[i * 4 + 3]);

		if (is_parity_bad(&spi_rx_buffer[i * 4], (i / 4)) != SUCCESS)
		{
			printf("param[%d]: 0x%08X parity bad\r\n", i, data_buffer[i]);
		}
		else
		{
			printf("param[%d]: 0x%08X parity ok\r\n", i, data_buffer[i]);
		}
	}

	return SUCCESS;
}

ErrorStatus is_parity_bad(uint8_t *byte, uint8_t grp)
{
	uint8_t prty = grp;

	prty ^= *byte;
	prty ^= *(byte + 1);
	prty ^= *(byte + 2);
	prty ^= *(byte + 3);
	prty ^= prty << 4;

	if ((prty & 0xF0) == 0xF0)
	{
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}

}

void stpmc1_data_unpacking(struct stpmc1_data *data, uint32_t *data_buffer)
{
	data->momVoltage[PHASE_R] =  ((data_buffer[DMR]&VOLTAGE_MASK) >> 16);
	data->momVoltage[PHASE_S] =  ((data_buffer[DMS]&VOLTAGE_MASK) >> 16);
	data->momVoltage[PHASE_T] =  ((data_buffer[DMT]&VOLTAGE_MASK) >> 16);

	data->momCurrent[PHASE_R] = (data_buffer[DMR]&CURRENT_MASK);
	data->momCurrent[PHASE_S] = (data_buffer[DMS]&CURRENT_MASK);
	data->momCurrent[PHASE_T] = (data_buffer[DMT]&CURRENT_MASK);

	data->rmsVoltage[PHASE_R] = ((data_buffer[DER]&VOLTAGE_MASK) >> 16);
	data->rmsVoltage[PHASE_S] = ((data_buffer[DES]&VOLTAGE_MASK) >> 16);
	data->rmsVoltage[PHASE_T] = ((data_buffer[DET]&VOLTAGE_MASK) >> 16);

	data->DC = (data_buffer[PRD])&VOLTAGE_MASK >> 16;
	data->configBits[0] = (data_buffer[CF0]&CFG_MASK);
	data->configBits[1] = (data_buffer[CF1]&CFG_MASK);
	data->configBits[2] = (data_buffer[CF2]&CFG_MASK);
	data->configBits[3] = (data_buffer[CF3]&CFG_MASK);

	data->powerActive = (data_buffer[DAP])&ENERGY_MASK >> 8;
}

void stpmc1_write_byte(enum stpmc1_config_bit address, uint8_t bit_state)
{
	uint8_t i = 0;
	uint8_t decimal = address;

	for (i = 0; i < BUFFER_SIZE - 1; i++)
	{
		spi_tx_buffer[i] = decimal % 2;
		decimal = decimal / 2;
	}

	spi_tx_buffer[BUFFER_SIZE - 1] = bit_state;

	writing_mode_enable();

	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->EGR |= TIM_EGR_UG;

	while (byte_sent_flag != 1 );
	byte_sent_flag = 0;

	writing_mode_disable();
}


void writing_mode_enable(void)
{
	CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODER5 | GPIO_MODER_MODER6));
	CLEAR_BIT(GPIOA->AFR[0], (GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6));
	LL_GPIO_SetAFPin_0_7(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, LL_GPIO_AF_1);
	LL_GPIO_SetPinMode(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(SPI1_MISO_GPIO_Port, SPI1_MISO_Pin, LL_GPIO_MODE_OUTPUT);
/*
	LL_GPIO_SetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	usDelay(100);
*/
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	usDelay(5);
	LL_GPIO_ResetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
}

void writing_mode_disable(void)
{
	LL_GPIO_SetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	usDelay(5);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);

	CLEAR_BIT(GPIOA->AFR[0], (GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6));
	LL_GPIO_SetAFPin_0_7(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, LL_GPIO_AF_5);
	LL_GPIO_SetAFPin_0_7(SPI1_MISO_GPIO_Port, SPI1_MISO_Pin, LL_GPIO_AF_5);
}

float stpmc1_get_voltage(struct stpmc1_data const *const data,
						 struct stpmc1_param const *const param, uint8_t type, uint8_t phase)
{
	float actualValue;
	uint32_t x_u;

	float Kdspu = param->Kdif * param->Kint;

	if (type == MOMENTARY)
	{
		x_u = data->momVoltage[phase];
	}
	else
	{
		x_u = data->rmsVoltage[phase];
	}
	actualValue = param->Kdiv * x_u * param->Vref
			/ (param->Kut * param->Ku * param->Au * param->len_u
					* param->Kint_comp * Kdspu);

	return actualValue;
}


float stpmc1_get_current(struct stpmc1_data const *const data,
						 struct stpmc1_param const *const param, uint8_t type, uint8_t phase)
{
	float actualValue;
	uint32_t x_i;

	float Kdspi = param->Kdif;

	if (type == MOMENTARY)
	{
		x_i = data->momCurrent[phase];
	}
	else
	{
		x_i = data->rmsCurrent[phase];
	}

	actualValue = x_i * param->Vref
			/ (param->Ks * param->Ki * param->Ai * param->len_i * param->Kint
					* param->Kint_comp * Kdspi);

	return actualValue;
}

void stpmc1_init(struct stpmc1_param *const param)
{
	param->R1 = 100000;
	param->R2 = 470;
	param->Ku = 0.9375;
	param->Ki = 0.9375;
	param->Kisum = 0.875;
	param->Au = 16;
	param->len_i = 65536;
	param->len_u = 4096;
	param->len_isum = 4096;
	param->Kint_comp = 1.004;
	param->Fm = 4000000;
	param->Kut = 2;
	param->Vref = 1.23;
	param->Kdiv = (1 + param->R1 / param->R2);
	param->Kint = 0.815;
	param->Kdif = 0.6135;
}
