#ifndef INC_STPMC1_H_
#define INC_STPMC1_H_

#include "main.h"

#define STPMC1_SPI				SPI1
#define SPI_BUS					LL_APB2_GRP1_PERIPH_SPI1

#define SPI_SCK_Pin 			LL_GPIO_PIN_5
#define SPI_SCK_GPIO_Port 		GPIOA
#define SPI_MISO_Pin 			LL_GPIO_PIN_6
#define SPI_MISO_GPIO_Port 		GPIOA
#define SPI_SYN_Pin 			LL_GPIO_PIN_4
#define SPI_SYN_GPIO_Port 		GPIOC
#define SPI_SS_Pin 				LL_GPIO_PIN_4
#define SPI_SS_GPIO_Port 		GPIOA
#define SS_GPIO_BUS				LL_AHB1_GRP1_PERIPH_GPIOA



#define VOLTAGE_MASK		(uint32_t)0x0FFF0000
#define CURRENT_MASK		(uint32_t)0x0000FFFF
#define ENERGY_MASK			(uint32_t)0x0FFFFF00
#define CFG_MASK			(uint32_t)0x0FFFFFFF
#define PERIOD_VALUE		(uint32_t)(20 - 1) 				    /* Period Value  */
#define PAUSE_VALUE			(uint32_t)(PERIOD_VALUE / 2)        /* Capture Compare  Value  */
#define PRESCALER_VALUE		(uint16_t)83

#define PHASE_R				0
#define PHASE_S				1
#define PHASE_T				2
#define PHASE_N				3
#define MAX_PHASES			3

#define MOMENTARY			(uint8_t)0
#define RMS					(uint8_t)1

#define TIME_BETWEEN_TX		10000
#define STPMC1_RX_BUF_SIZE	112


enum stpmc1_config_bit
{
	TSTD = 0,
	MDIV,
	HSA,
	APL_0,
	APL_1,
	TCS,
	FRS,
	FUND,
	ART,
	MSBF,
	ABS_0,
	ABS_1,
	LTCH_0,
	LTCH_1,
	KMOT_0,
	KMOT_1,
	LVS,
	SYS_0,
	SYS_1,
	SYS_2,
	SCLP,
	PM,
	FR1,
	CCA_0,
	CCA_1,
	CCA_2,
	CCA_3,
	CCA_4,
	CCA_5,
	CCA_6,
	CCA_7,
	CCA_8,
	CIN_0,
	CIN_1,
	CIN_2,
	CIN_3,
	CIN_4,
	CIN_5,
	CIN_6,
	CIN_7,
	CIR_0,
	CIR_1,
	CIR_2,
	CIR_3,
	CIR_4,
	CIR_5,
	CIR_6,
	CIR_7,
	CIS_0,
	CIS_1,
	CIS_2,
	CIS_3,
	CIS_4,
	CIS_5,
	CIS_6,
	CIS_7,
	CIT_0,
	CIT_1,
	CIT_2,
	CIT_3,
	CIT_4,
	CIT_5,
	CIT_6,
	CIT_7,
	CVR_0,
	CVR_1,
	CVR_2,
	CVR_3,
	CVR_4,
	CVR_5,
	CVR_6,
	CVR_7,
	CVS_0,
	CVS_1,
	CVS_2,
	CVS_3,
	CVS_4,
	CVS_5,
	CVS_6,
	CVS_7,
	CVT_0,
	CVT_1,
	CVT_2,
	CVT_3,
	CVT_4,
	CVT_5,
	CVT_6,
	CVT_7,
	CPR_0,
	CPR_1,
	CPR_2,
	CPR_3,
	CPS_0,
	CPS_1,
	CPS_2,
	CPS_3,
	CPT_0,
	CPT_1,
	CPT_2,
	CPT_3,
	CCB_0,
	CCB_1,
	CCB_2,
	CCB_3,
	CCB_4,
	CCB_5,
	CCB_6,
	CCB_7,
	CPC_0,
	CPC_1,
	ENH,
	CHK,
	TSG_0,
	TSG_1,
	TSG_2,
	TSG_3,
	BANK = 120,
	PUMP,
	TST_0,
	TST1,
	TST2,
	RD,
	WE,
	Precharge
};

enum writing_mode
{
	TEMP = 0,
	PERM
};

enum num_of_reg
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
};

struct stpmc1_data
{
	uint32_t       power_active;
	uint32_t       power_active_fund;
	uint32_t       power_reactive;
	uint32_t       mom_vol[MAX_PHASES];
	uint32_t	   mom_cur[MAX_PHASES];
	uint32_t       rms_vol[MAX_PHASES];
	uint32_t       rms_cur[MAX_PHASES];
	uint32_t	   period;
	uint32_t 	   DC;
	uint32_t 	   cfg_reg[4];
};

struct stpmc1_cfg
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
	float Kint;
	float Kdif;
};


void stpmc1_init();

ErrorStatus stpmc1_read(uint32_t *data_buffer);

float stpmc1_get_voltage(uint8_t type, uint8_t phase);
float stpmc1_get_current(uint8_t type, uint8_t phase);

#endif /* INC_STPMC1_H_ */
