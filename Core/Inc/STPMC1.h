/*
 * STPMC1.h
 *
 *  Created on: Jul 25, 2021
 *      Author: vladz
 */

#ifndef INC_STPMC1_H_
#define INC_STPMC1_H_

#include "main.h"

#define VOLTAGE_VALUES_MASK		(uint32_t)0x0FFF0000
#define CURRENT_VALUES_MASK		(uint32_t)0x0000FFFF
#define ENERGY_MASK				(uint32_t)0x0FFFFF00
#define CFG_MASK				(uint32_t)0x0FFFFFFF
#define PERIOD_VALUE			(uint32_t)(20 - 1) 				    /* Period Value  */
#define PAUSE_VALUE				(uint32_t)(PERIOD_VALUE / 2)        /* Capture Compare  Value  */
#define PRESCALER_VALUE			(uint16_t)83

#define PHASE_R					0
#define PHASE_S					1
#define PHASE_T					2
#define PHASE_N					3
#define MAX_PHASES				3

#define VOLTAGE					(uint8_t)0
#define CURRENT					(uint8_t)1

#define MOMENTARY				(uint8_t)0
#define RMS					 	(uint8_t)1

#define TIME_BETWEEN_TX			10000

typedef enum
{
	TSTD = 0,
	MDIV,
	HSA,
	APL_0,  APL_1,
	TCS,
	FRS,
	FUND,
	ART,
	MSBF,
	ABS_0,	ABS_1,
	LTCH_0,	LTCH_1,
	KMOT_0,	KMOT_1,
	LVS,
	SYS_0,	SYS_1,	SYS_2,
	SCLP,
	PM,
	FR1,
	CCA_0,  CCA_1,  CCA_2,  CCA_3,  CCA_4,  CCA_5,  CCA_6,  CCA_7,  CCA_8,
	CIN_0,	CIN_1,	CIN_2,	CIN_3,	CIN_4,	CIN_5,	CIN_6,	CIN_7,
	CIR_0,	CIR_1,	CIR_2,	CIR_3,	CIR_4,	CIR_5,	CIR_6,	CIR_7,
	CIS_0,	CIS_1,	CIS_2,	CIS_3,	CIS_4,	CIS_5,	CIS_6,	CIS_7,
	CIT_0,	CIT_1,	CIT_2,	CIT_3,	CIT_4,	CIT_5,	CIT_6,	CIT_7,
	CVR_0,	CVR_1,	CVR_2,	CVR_3,	CVR_4,	CVR_5,	CVR_6,	CVR_7,
	CVS_0,	CVS_1,	CVS_2,	CVS_3,	CVS_4,	CVS_5,	CVS_6,	CVS_7,
	CVT_0,	CVT_1,	CVT_2,	CVT_3,	CVT_4,	CVT_5,	CVT_6,	CVT_7,
	CPR_0,	CPR_1,	CPR_2,	CPR_3,
	CPS_0,	CPS_1,	CPS_2,	CPS_3,
	CPT_0,	CPT_1,	CPT_2,	CPT_3,
	CCB_0,	CCB_1,	CCB_2,	CCB_3,	CCB_4,	CCB_5,	CCB_6,	CCB_7,
	CPC_0,	CPC_1,
	ENH,
	CHK,
	TSG_0,  TSG_1,	TSG_2,	TSG_3,
	BANK = 120,
	PUMP,
	TST_0,
	TST1,
	TST2,
	RD,
	WE,
	Precharge
}STPMC1_Config_Bit;

typedef enum
{
	temporary = 0,
	permanent

}WritingMode;

typedef enum
{
	DAP = 0,
	DRP,
	DFP,
	PRD,
	DMR,
	DMS,
	DMT,
	DMN,
	DER,
	DES,
	DET,
	DEN,
	DAR,
	DAS,
	DAT,
	CF0,
	DRR,
	DRS,
	DRT,
	CF1,
	DFR,
	DFS,
	DFT,
	CF2,
	ACR,
	ACS,
	ACT,
	CF3
}Number_Of_Register_t;

typedef struct
{
	uint32_t       powerActive;
	uint32_t       powerActiveFund;
	uint32_t       powerReactive;
	uint32_t       momVoltage[MAX_PHASES];
	uint32_t	   momCurrent[MAX_PHASES];
	uint32_t       rmsVoltage[MAX_PHASES];
	uint32_t       rmsCurrent[MAX_PHASES];
	uint32_t	   period;
	uint32_t 	   DC;
	uint32_t 	   configBits[4];
}Data_t;

typedef struct
{
	float       powerActive;
	float       powerActiveFund;
	float       powerReactive;
	float		momVoltage[MAX_PHASES];
	float		momCurrent[MAX_PHASES];
	float       rmsVoltage[MAX_PHASES];
	float       rmsCurrent[MAX_PHASES];
	float		DC;
	float		period;

}ActualValue_t;

typedef struct
{
	uint32_t R1;
	uint32_t R2;
	float 	 Ks;
	float 	 Ai;
	float 	 Ku;
	float 	 Ki;
	float 	 Kisum;
	float 	 Au;
	uint32_t len_i;
	uint32_t len_u;
	uint32_t len_isum;
	float	 Kint_comp;
	uint32_t Fm;
	uint32_t Kut;
	float Vref;
	float Kdiv;
} Parameters_t;


void STPMC1_DataUnpacking(Data_t *const Data);
void STPMC1_Init(void);
float STPMC1_GetMomVoltage(uint8_t phase);
float STPMC1_GetMomCurrent(uint8_t phase);
float STPMC1_GetRMSVoltage(uint8_t phase);
float STPMC1_GetRMSCurrent(uint8_t phase);
float ConversionToActualValue(uint8_t elecParam, uint8_t type, uint8_t phase, Parameters_t *const Parameters, Data_t *const Data);
void STPMC1_Writing(WritingMode mode);
void STPMC1_Reading(Data_t *const Data, uint32_t *pRxBuffer);
void STPMC1_SendByte(STPMC1_Config_Bit, uint8_t bitState);
ErrorStatus IsParityBad(uint8_t *bp, uint8_t);

#endif /* INC_STPMC1_H_ */
