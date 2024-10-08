#ifndef VL6180_H_
#define VL6180_H_
//-------------------------------------------------
#include "stm32l1xx_hal.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stmpe1600.h"
//-------------------------------------------------
#define VL6180_CHIPEN (1<<12)
//-------------------------------------------------
#define VL6180_ADDRESS 0x52 //what we use as API device
//-------------------------------------------------
#define IDENTIFICATION__MODEL_ID 0x0000
//-------------------------------------------------
#define SYSTEM_INTERRUPT_CONFIG_GPIO 0x014
#define CONFIG_GPIO_ALS_SHIFT 3 /** ALS bits mask in #SYSTEM_INTERRUPT_CONFIG_GPIO (unshifted)*/
#define CONFIG_GPIO_ALS_MASK (0x7<<CONFIG_GPIO_ALS_SHIFT)
#define SYSTEM_INTERRUPT_CLEAR 0x015
#define SYSTEM_FRESH_OUT_OF_RESET 0x016
#define SYSALS_START 0x038
#define SYSALS_THRESH_HIGH 0x03A
#define SYSALS_THRESH_LOW 0x03C
#define SYSALS_ANALOGUE_GAIN 0x03F
#define SYSALS_INTERMEASUREMENT_PERIOD 0x03E
#define SYSALS_INTEGRATION_PERIOD 0x040
#define RESULT_ALS_STATUS 0x4E
#define RESULT_INTERRUPT_STATUS_GPIO 0x4F
#define RESULT_ALS_VAL 0x50
#define FW_ALS_RESULT_SCALER 0x120
//-------------------------------------------------
#define INTERRUPT_CLEAR_RANGING 0x01
#define INTERRUPT_CLEAR_ALS 0x02
#define INTERRUPT_CLEAR_ERROR 0x04
//-------------------------------------------------
#define CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY 0x04
//-------------------------------------------------
#define INVALID_PARAMS -2 /*!< parameter passed is invalid or out of range */
//-------------------------------------------------
#define MODE_START_STOP 0x01
/** bit 1 write 1 in #SYSRANGE_START set continuous operation mode */
#define MODE_CONTINUOUS 0x02
/** bit 1 write 0 in #SYSRANGE_START set single shot mode */
#define MODE_SINGLESHOT 0x00
//-------------------------------------------------
#define RES_INT_STAT_GPIO_NEW_SAMPLE_READY 0x04
//-------------------------------------------------
#define DEF_INT_PEFRIOD 100
#define LUXRES_FIX_PREC 8
#define GAIN_FIX_PREC 8 /* ! if not sme as LUX_PREC then :( adjust GetLux */
#define AN_GAIN_MULT (1 << GAIN_FIX_PREC)
//-------------------------------------------------
#define vl6180_ClearAllInterrupt() vl6180_ClearInterrupt(INTERRUPT_CLEAR_ERROR|INTERRUPT_CLEAR_RANGING|INTERRUPT_CLEAR_ALS)
#define vl6180_AlsClearInterrupt() vl6180_ClearInterrupt(INTERRUPT_CLEAR_ALS)
//-------------------------------------------------
typedef struct state_t {
  uint8_t mode;
}State_ptr;
//-------------------------------------------------
typedef struct VL6180xDevData_t {
  uint16_t IntegrationPeriod; /*!< cached als Integration period avoid slow read from device at each measure */
  uint16_t AlsGainCode; /*!< cached Als gain avoid slow read from device at each measure */
  uint16_t AlsScaler; /*!< cached Als scaler avoid slow read from device at each measure */
}VL6180xDevData_ptr;
//-------------------------------------------------
typedef struct VL6180x_AlsData_st {
  uint32_t lux; /**< Light measurement (Lux) */
	uint8_t valueStatus; /* Value status */
  uint32_t errorStatus; /**< Error status of the current measurement. \n
  * No Error := 0. \n
  * Refer to product sheets for other error codes. */
} VL6180x_AlsData_t;
//-------------------------------------------------
#define ValueNormal 0
#define ValueLow 1
#define ValueHigh 2
//-------------------------------------------------
#define RunRangePoll 0
#define RunAlsPoll 1
#define InitErr 2
#define ScaleSwap 3
#define WaitForReset 4
#define Start 5
#define Run 6
#define FromSwitch 7
//-------------------------------------------------
int vl6180_ini(void);
void vl6180_ReadData(void);
int stmpe1600_GetSwitch(void);
//-------------------------------------------------
#endif /* VL6180_H_ */