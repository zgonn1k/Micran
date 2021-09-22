/*
 * STPMC1.c
 *
 *  Created on: Jul 25, 2021
 *      Author: vladz
 */
#include "STPMC1.h"

uint32_t dataRegister[DATA_SIZE];

void STPMC1_Writing(WritingMode mode)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =	{ 0 };

	LL_SPI_Disable(SPI1);

	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL5);

	GPIO_InitStruct.Pin = SPI1_SCK_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	LL_GPIO_SetOutputPin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin);

	if(mode == temporary)
	{
		STPMC1_SendByte(RD, 1);
		usDelay(TIME_BETWEEN_TX	);
	}
	else
	{
		STPMC1_SendByte(RD, 0);
		usDelay(TIME_BETWEEN_TX	);
	}

	STPMC1_SendByte(TSTD, 0);		/*enable test modes and system signals*/
	usDelay(TIME_BETWEEN_TX);

	STPMC1_SendByte(MDIV, 1);	/*fMCLK = fXTAL1 = 8 MHz*/
	usDelay(TIME_BETWEEN_TX);

	STPMC1_SendByte(HSA, 0);	/*fCLK = fXTAL1/4 = 2 MHz */
	usDelay(TIME_BETWEEN_TX);

	STPMC1_SendByte(APL_0, 0);
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(APL_1, 0); /*APL = 0: peripheral MOP, MON=ZCR, WatchDOG, LED=pulses (X)*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(TCS, 1);	/*Current transformer (CT) or shunt*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(FRS, 0);	/*Nominal base frequency 50Hz*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(FUND, 0);	/*full bandwidth active energy controls the stepper;
						  full bandwidth reactive energy computation.*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(ART, 0);	/*natural computation*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(MSBF, 0);	/*msb first*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(ABS_0, 0);
	usDelay(TIME_BETWEEN_TX	);
	STPMC1_SendByte(ABS_1, 0);	/*LTCH=0: 0,00125 * FS,*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(LTCH_0, 0);
	usDelay(TIME_BETWEEN_TX	);
	STPMC1_SendByte(LTCH_1, 0);	/*LTCH=0: 0,00125 * FS,*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(KMOT_0, 1);
	usDelay(TIME_BETWEEN_TX	);
	STPMC1_SendByte(KMOT_1, 0);
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(SYS_0, 1);
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(SYS_1, 1);
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(SYS_2, 0);	/*SYS=3: 1-phase, 2-wire __TN, 1-system __T_*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(SCLP, 0);
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(PM, 1);	/*Class 0.1*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(FR1, 0);	/*fMCLK =8.192 MHz*/
	usDelay(TIME_BETWEEN_TX	);

	STPMC1_SendByte(CHK, 1);	/*fMCLK =8.192 MHz*/
	usDelay(TIME_BETWEEN_TX	);

	Writing_Mode_Disable();


}

void STPMC1_Reading(Data_t *Data, uint32_t *pRxBuffer)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	uint8_t dataRegister_8bit[DATA_SIZE * 4];



	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, sizeof(dataRegister_8bit));
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, (uint32_t) (&SPI1->DR));
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t) dataRegister_8bit);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);

	Writing_Mode_Enable();

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


	LL_SPI_Enable(SPI1);
	uint8_t currentByte;
	for (currentByte = 0; currentByte < DATA_SIZE * 4; currentByte++)
	{
		while (!(LL_SPI_IsActiveFlag_RXNE(SPI1)));
		dataRegister_8bit[currentByte] = LL_SPI_ReceiveData8(SPI1);
	}
	LL_SPI_Disable(SPI1);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);


	for (currentByte = 0; currentByte < DATA_SIZE * 4; currentByte++)
	{
		printf("dataReg[%d] = %x\r\n", currentByte, dataRegister_8bit[currentByte]);
	}

	uint8_t currentAdressOfPackage;
	for (currentAdressOfPackage = 0; currentAdressOfPackage < DATA_SIZE; currentAdressOfPackage++)
	{
		uint8_t tmp;
		tmp = dataRegister_8bit[currentAdressOfPackage * 4 + 0];
		dataRegister_8bit[currentAdressOfPackage * 4 + 0] = dataRegister_8bit[currentAdressOfPackage * 4 + 3];
		dataRegister_8bit[currentAdressOfPackage * 4 + 3] = tmp;

		tmp = dataRegister_8bit[currentAdressOfPackage * 4 + 1];
		dataRegister_8bit[currentAdressOfPackage * 4 + 1] = dataRegister_8bit[currentAdressOfPackage * 4 + 2];
		dataRegister_8bit[currentAdressOfPackage * 4 + 2] = tmp;

		pRxBuffer[currentAdressOfPackage] = dataRegister_8bit[currentAdressOfPackage * 4 + 0];
		pRxBuffer[currentAdressOfPackage] = (pRxBuffer[currentAdressOfPackage] << 8) | dataRegister_8bit[currentAdressOfPackage * 4 + 1];
		pRxBuffer[currentAdressOfPackage] = (pRxBuffer[currentAdressOfPackage] << 8) | dataRegister_8bit[currentAdressOfPackage * 4 + 2];
		pRxBuffer[currentAdressOfPackage] = (pRxBuffer[currentAdressOfPackage] << 8) | dataRegister_8bit[currentAdressOfPackage * 4 + 3];

		if (IsParityBad(&dataRegister_8bit[currentAdressOfPackage * 4], (currentAdressOfPackage / 4)) == SUCCESS)
		{
			printf("param[%d]: 0x%08X parity ok\r\n", currentAdressOfPackage, pRxBuffer[currentAdressOfPackage]);
		}
		else
		{
			printf("param[%d]: 0x%08X parity bad\r\n", currentAdressOfPackage, pRxBuffer[currentAdressOfPackage]);
		}
	}
}

ErrorStatus IsParityBad(uint8_t *bp, uint8_t grp)
{
	uint8_t prty = grp;

	prty ^= *bp;
	prty ^= *(bp + 1);
	prty ^= *(bp + 2);
	prty ^= *(bp + 3);
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

void STPMC1_DataUnpacking(Data_t *Data)
{
	Data->momVoltage[PHASE_R] =  ((dataRegister[DMR]&VOLTAGE_VALUES_MASK) >> 16);
	Data->momVoltage[PHASE_S] =  ((dataRegister[DMS]&VOLTAGE_VALUES_MASK) >> 16);
	Data->momVoltage[PHASE_T] =  ((dataRegister[DMT]&VOLTAGE_VALUES_MASK) >> 16);

	Data->momCurrent[PHASE_R] = (dataRegister[DMR]&CURRENT_VALUES_MASK);
	Data->momCurrent[PHASE_S] = (dataRegister[DMS]&CURRENT_VALUES_MASK);
	Data->momCurrent[PHASE_T] = (dataRegister[DMT]&CURRENT_VALUES_MASK);

	Data->rmsVoltage[PHASE_R] = ((dataRegister[DER]&VOLTAGE_VALUES_MASK) >> 16);
	Data->rmsVoltage[PHASE_S] = ((dataRegister[DES]&VOLTAGE_VALUES_MASK) >> 16);
	Data->rmsVoltage[PHASE_T] = ((dataRegister[DET]&VOLTAGE_VALUES_MASK) >> 16);

	Data->DC = (dataRegister[PRD])&VOLTAGE_VALUES_MASK >> 16;
	Data->configBits[0] = (dataRegister[CF0]&CFG_MASK);
	Data->configBits[1] = (dataRegister[CF1]&CFG_MASK);
	Data->configBits[2] = (dataRegister[CF2]&CFG_MASK);
	Data->configBits[3] = (dataRegister[CF3]&CFG_MASK);

	Data->powerActive = (dataRegister[DAP])&ENERGY_MASK >> 8;
}

void STPMC1_SendByte(STPMC1_Config_Bit address, uint8_t bitState)
{
	uint8_t i = 0;
	uint8_t decimal = address;

	for (i = 0; i < BUFFER_SIZE - 1; i++)
		{
			TxBuffer[i] = decimal % 2;
			decimal = decimal / 2;
		}

	TxBuffer[BUFFER_SIZE - 1] = bitState;

	LL_GPIO_SetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	usDelay(1000);
	LL_GPIO_ResetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	usDelay(5);
	LL_GPIO_ResetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);

	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER6);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL5);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL6);

	GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_0; /*TIM2 CH1 for PA5*/
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5_1 | GPIO_AFRL_AFRL5_2 | GPIO_AFRL_AFRL5_3);
	GPIOA->MODER |= GPIO_MODER_MODER5_1; /*Alternate function mode*/

	LL_GPIO_SetPinMode(SPI1_MISO_GPIO_Port, SPI1_MISO_Pin, LL_GPIO_MODE_OUTPUT);

	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->EGR |= TIM_EGR_UG;

	while (byteSent != 1 );
	byteSent = 0;

	usDelay(10);

	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL5);

	LL_GPIO_SetPinMode(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetOutputPin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin);
	usDelay(10);
	LL_GPIO_SetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	usDelay(10);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
	usDelay(100);
}


void Writing_Mode_Enable(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL5);

	GPIO_InitStruct.Pin = SPI1_SCK_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void Writing_Mode_Disable(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct =	{ 0 };

	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER5);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODER6);

	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL5);
	CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL6);

	GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	LL_GPIO_SetOutputPin(SPI1_SYN_GPIO_Port, SPI1_SYN_Pin);
	LL_GPIO_SetOutputPin(SPI1_SS_GPIO_Port, SPI1_SS_Pin);
}
