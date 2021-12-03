#include "STPMC1.h"

#define STPMC1_SET_REG(REG, VAL)	(VAL == 1) ? (REG | (1 << 7)) : (REG &~(1 << 7))

static void writing_mode_enable(void);
static void writing_mode_disable(void);
static void reading_mode_enable(void);
static void stpmc1_write_reg(uint8_t *reg);
static ErrorStatus is_parity_bad(uint8_t *byte, uint8_t grp);
static void stpmc1_spi_init(void);
static ErrorStatus spi_receive(SPI_TypeDef *spip, uint8_t *rx_buf, size_t buf_size);

uint8_t stpmc1_data_buf[28];

static struct stpmc1_data stpmc1_data;

const struct stpmc1_cfg stpmc1_cfg =
{
		.R1 = 100000,
		.R2 = 470,
		.Ku = 0.9375,
		.Ki = 0.9375,
		.Kisum = 0.875,
		.Au = 16,
		.len_i = 65536,
		.len_u = 4096,
		.len_isum = 4096,
		.Kint_comp = 1.004,
		.Fm = 4000000,
		.Kut = 2,
		.Vref = 1.23,
		.Kint = 0.815,
		.Kdif = 0.6135,
};



void stpmc1_init()
{
	uint8_t stpmc1_cfg_reg[] =
	{

		STPMC1_SET_REG(RD, 1),
		STPMC1_SET_REG(TSTD, 0), /*enable test modes and system signals*/
		STPMC1_SET_REG(MDIV, 1), /*fMCLK = fXTAL1 = 8 MHz*/
		STPMC1_SET_REG(HSA, 0), /*fCLK = fXTAL1/4 = 2 MHz */
		STPMC1_SET_REG(APL_0, 0), /*APL = 0: peripheral MOP, MON=ZCR, WatchDOG, LED=pulses (X)*/
		STPMC1_SET_REG(APL_1, 0),
		STPMC1_SET_REG(TCS, 1), /*Current transformer (CT) or shunt*/
		STPMC1_SET_REG(FRS, 0), /*Nominal base frequency 50Hz*/
		STPMC1_SET_REG(FUND, 0),
		STPMC1_SET_REG(ART, 0), /*natural computation*/
		STPMC1_SET_REG(MSBF, 0), /*msb first*/
		STPMC1_SET_REG(ABS_0, 0),
		STPMC1_SET_REG(ABS_1, 0),
		STPMC1_SET_REG(LTCH_0, 0), /*LTCH=0: 0,00125 * FS,*/
		STPMC1_SET_REG(LTCH_1, 0),
		STPMC1_SET_REG(KMOT_0, 1),
		STPMC1_SET_REG(KMOT_1, 1),
		STPMC1_SET_REG(KMOT_0, 1),
		STPMC1_SET_REG(SYS_0, 1), /*SYS=3: 1-phase, 2-wire __TN, 1-system __T_*/
		STPMC1_SET_REG(SYS_1, 1),
		STPMC1_SET_REG(SYS_2, 0),
		STPMC1_SET_REG(SCLP, 0),
		STPMC1_SET_REG(PM, 1), /*Class 0.1*/
		STPMC1_SET_REG(FR1, 0), /*fMCLK =8.192 MHz*/
		STPMC1_SET_REG(CHK, 1) /*fMCLK =8.192 MHz*/
	};

	stpmc1_spi_init();


	writing_mode_enable();

	for(uint8_t i = 0; i < sizeof(stpmc1_cfg_reg); i++)
	{
		stpmc1_write_reg(&stpmc1_cfg_reg[i]);
		usDelay(TIME_BETWEEN_TX);
	}
	writing_mode_disable();
}

void reading_mode_enable()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	writing_mode_disable();

	LL_SPI_Disable(STPMC1_SPI);
	LL_GPIO_SetOutputPin(SPI_SS_GPIO_Port, SPI_SS_Pin);
	LL_GPIO_ResetOutputPin(SPI_SYN_GPIO_Port, SPI_SYN_Pin);
	usDelay(2);
	LL_GPIO_SetOutputPin(SPI_SYN_GPIO_Port, SPI_SYN_Pin);
	usDelay(1);
	LL_GPIO_ResetOutputPin(SPI_SS_GPIO_Port, SPI_SS_Pin);
	LL_GPIO_SetOutputPin(SPI_SCK_GPIO_Port, SPI_SCK_Pin);
	usDelay(1);
	LL_GPIO_ResetOutputPin(SPI_SYN_GPIO_Port, SPI_SYN_Pin);
	usDelay(1);
	LL_GPIO_SetOutputPin(SPI_SYN_GPIO_Port, SPI_SYN_Pin);

	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER6);

	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL5);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL6);

	/**/
	GPIO_InitStruct.Pin = SPI_SCK_Pin | SPI_MISO_Pin;
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

	spi_receive(STPMC1_SPI, spi_rx_buffer, STPMC1_RX_BUF_SIZE);

	LL_GPIO_SetOutputPin(SPI_SS_GPIO_Port, SPI_SS_Pin);

	uint8_t i = 0;
	for (i = 0; i < STPMC1_RX_BUF_SIZE; i++)
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
			printf("param[%d]: 0x%08X parity is bad\r\n", i, data_buffer[i]);
		}
		else
		{
			printf("param[%d]: 0x%08X parity is ok\r\n", i, data_buffer[i]);
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

void stpmc1_write_reg(uint8_t *reg)
{
	uint8_t i = 0;
	uint8_t decimal = *reg;

	for (i = 0; i < BUFFER_SIZE; i++)
	{
		spi_tx_buffer[i] = decimal % 2;
		decimal = decimal / 2;
	}

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
	LL_GPIO_SetAFPin_0_7(SPI_SCK_GPIO_Port, SPI_SCK_Pin, LL_GPIO_AF_1);
	LL_GPIO_SetPinMode(SPI_SCK_GPIO_Port, SPI_SCK_Pin, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(SPI_MISO_GPIO_Port, SPI_MISO_Pin, LL_GPIO_MODE_OUTPUT);
/*
	LL_GPIO_SetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	usDelay(100);
*/
	LL_GPIO_ResetOutputPin(SPI_SS_GPIO_Port, SPI_SS_Pin);
	usDelay(5);
	LL_GPIO_ResetOutputPin(SPI_SYN_GPIO_Port, SPI_SYN_Pin);
}

void writing_mode_disable(void)
{
	LL_GPIO_SetOutputPin(SPI_SYN_GPIO_Port, SPI_SYN_Pin);
	usDelay(5);
	LL_GPIO_SetOutputPin(SPI_SS_GPIO_Port, SPI_SS_Pin);

	CLEAR_BIT(GPIOA->AFR[0], (GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6));
	LL_GPIO_SetAFPin_0_7(SPI_SCK_GPIO_Port, SPI_SCK_Pin, LL_GPIO_AF_5);
	LL_GPIO_SetAFPin_0_7(SPI_MISO_GPIO_Port, SPI_MISO_Pin, LL_GPIO_AF_5);
}

float stpmc1_get_voltage(uint8_t type, uint8_t phase)
{
	float actual_value;
	uint32_t x_u;

	float Kdspu = stpmc1_cfg.Kdif * stpmc1_cfg.Kint;

	if (type == MOMENTARY)
	{
		stpmc1_data.mom_vol[phase] =  ((stpmc1_data_buf[phase + 4]&VOLTAGE_MASK) >> 16);
		x_u = stpmc1_data.mom_vol[phase];

	}
	else
	{
		stpmc1_data.rms_vol[phase] = ((stpmc1_data_buf[phase + 8]&VOLTAGE_MASK) >> 16);
		x_u = stpmc1_data.rms_vol[phase];
	}
	actual_value = stpmc1_cfg.Kdiv * x_u * stpmc1_cfg.Vref
			/ (stpmc1_cfg.Kut * stpmc1_cfg.Ku * stpmc1_cfg.Au * stpmc1_cfg.len_u
					* stpmc1_cfg.Kint_comp * Kdspu);

	return actual_value;
}


float stpmc1_get_current(uint8_t type, uint8_t phase)
{
	float actual_value;
	uint32_t x_i;

	float Kdspi = stpmc1_cfg.Kdif;

	if (type == MOMENTARY)
	{
		stpmc1_data.mom_cur[phase] = (stpmc1_data_buf[phase + 4]&CURRENT_MASK);
		x_i = stpmc1_data.mom_cur[phase];
	}
	else
	{
		stpmc1_data.rms_cur[phase] = (stpmc1_data_buf[phase + 8]&CURRENT_MASK);
		x_i = stpmc1_data.rms_cur[phase];
	}

	actual_value = x_i * stpmc1_cfg.Vref
			/ (stpmc1_cfg.Ks * stpmc1_cfg.Ki * stpmc1_cfg.Ai * stpmc1_cfg.len_i * stpmc1_cfg.Kint
					* stpmc1_cfg.Kint_comp * Kdspi);

	return actual_value;
}

static void stpmc1_spi_init(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct =	{ 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	LL_APB2_GRP1_EnableClock(SPI_BUS);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/**SPI1 GPIO Configuration
	 PA5   ------> SPI1_SCK
	 PA6   ------> SPI1_MISO
	 */
	GPIO_InitStruct.Pin = SPI_SCK_Pin | SPI_MISO_Pin;
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
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64; /* 2,5 MBits/s */
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	LL_SPI_Init(STPMC1_SPI, &SPI_InitStruct);
}

static ErrorStatus spi_receive(SPI_TypeDef *spip, uint8_t *rx_buf, size_t buf_size)
{
	ErrorStatus status;


	if ((rx_buf == NULL) || (buf_size == 0U))
	{
		status = ERROR;
	}

	LL_SPI_Enable(spip);

	for (uint8_t i = 0; i < buf_size; i++)
	{
		while (!(LL_SPI_IsActiveFlag_RXNE(spip)));
		rx_buf[i] = LL_SPI_ReceiveData8(spip);
	}
	LL_SPI_Disable(spip);

	return status;
}
