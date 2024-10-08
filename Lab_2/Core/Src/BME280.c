#include "BME280.h"

//------------------------------------------------

extern I2C_HandleTypeDef hi2c2;

//extern UART_HandleTypeDef huart2;

//extern char str1[100];
BME280_CalibData CalibData;
int32_t temp_int;

//------------------------------------------------
void Error(void)

{

  //LED_OFF;

}

//------------------------------------------------
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)

{

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);

}

//------------------------------------------------

static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)

{

  HAL_StatusTypeDef status = HAL_OK;

  uint8_t value = 0;

  status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);

  return value;

}

//------------------------------------------------
//------------------------------------------------

static void I2Cx_ReadData16(uint16_t Addr, uint8_t Reg, uint16_t *Value)

{

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 2, 0x10000);


}

//------------------------------------------------

static void I2Cx_ReadData24(uint16_t Addr, uint8_t Reg, uint32_t *Value)

{

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 3, 0x10000);

}

//------------------------------------------------
void BME280_WriteReg(uint8_t Reg, uint8_t Value)

{

  I2Cx_WriteData(BME280_ADDRESS, Reg, Value);

}

//------------------------------------------------
uint8_t BME280_ReadReg(uint8_t Reg)

{

  uint8_t res = I2Cx_ReadData(BME280_ADDRESS,Reg);

  return res;

}

//------------------------------------------------
void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value)

{

  I2Cx_ReadData16(BME280_ADDRESS,Reg,Value);

}

//------------------------------------------------

void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value)

{

  I2Cx_ReadData16(BME280_ADDRESS,Reg, (uint16_t*) Value);

}
//------------------------------------------------

void BME280_ReadReg_U24(uint8_t Reg, uint32_t *Value)

{

  I2Cx_ReadData24(BME280_ADDRESS,Reg,Value);

  *(uint32_t *) Value &= 0x00FFFFFF;

}
//------------------------------------------------

void BME280_ReadReg_BE_U24(uint8_t Reg, uint32_t *Value)

{

  I2Cx_ReadData24(BME280_ADDRESS,Reg,Value);

  *(uint32_t *) Value = be24toword(*(uint32_t *) Value) & 0x00FFFFFF;

}
//------------------------------------------------

uint8_t BME280_ReadStatus(void)

{

  //clear unuset bits

  uint8_t res = BME280_ReadReg(BME280_REGISTER_STATUS)&0x09;

  return res;

}

//------------------------------------------------
void BME280_ReadCoefficients(void)

{

  BME280_ReadReg_U16(BME280_REGISTER_DIG_T1,&CalibData.dig_T1);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_T2,&CalibData.dig_T2);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_T3,&CalibData.dig_T3);
  BME280_ReadReg_U16(BME280_REGISTER_DIG_P1,&CalibData.dig_P1);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_P2,&CalibData.dig_P2);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_P3,&CalibData.dig_P3);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_P4,&CalibData.dig_P4);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_P5,&CalibData.dig_P5);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_P6,&CalibData.dig_P6);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_P7,&CalibData.dig_P7);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_P8,&CalibData.dig_P8);
  BME280_ReadReg_S16(BME280_REGISTER_DIG_P9,&CalibData.dig_P9);

}

//------------------------------------------------
void BME280_SetStandby(uint8_t tsb) {

  uint8_t reg;

  reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_STBY_MSK;

  reg |= tsb & BME280_STBY_MSK;

  BME280_WriteReg(BME280_REG_CONFIG,reg);

}

//------------------------------------------------
void BME280_SetFilter(uint8_t filter) {

 uint8_t reg;

 reg = BME280_ReadReg(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;

 reg |= filter & BME280_FILTER_MSK;

 BME280_WriteReg(BME280_REG_CONFIG,reg);

}

//------------------------------------------------
void BME280_SetOversamplingTemper(uint8_t osrs)

{

  uint8_t reg;

  reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;

  reg |= osrs & BME280_OSRS_T_MSK;

  BME280_WriteReg(BME280_REG_CTRL_MEAS,reg);

}

//------------------------------------------------

void BME280_SetOversamplingPressure(uint8_t osrs)

{

  uint8_t reg;

  reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;

  reg |= osrs & BME280_OSRS_P_MSK;

  BME280_WriteReg(BME280_REG_CTRL_MEAS,reg);

}

//------------------------------------------------
void BME280_SetMode(uint8_t mode) {

  uint8_t reg;

  reg = BME280_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;

  reg |= mode & BME280_MODE_MSK;

  BME280_WriteReg(BME280_REG_CTRL_MEAS,reg);

}

//------------------------------------------------
//------------------------------------------------

float BME280_ReadTemperature(void)

{

  float temper_float = 0.0f;
  uint32_t temper_raw_unsigned;
  int32_t temper_raw;
  int32_t val1, val2;


  BME280_ReadReg_BE_U24(BME280_REGISTER_TEMPDATA,&temper_raw_unsigned);
  temper_raw = (int32_t)temper_raw_unsigned;
  temper_raw >>= 4;

  val1 = ((((temper_raw>>3) - ((int32_t)CalibData.dig_T1 <<1))) *

  ((int32_t)CalibData.dig_T2)) >> 11;

  val2 = (((((temper_raw>>4) - ((int32_t)CalibData.dig_T1)) *

  ((temper_raw>>4) - ((int32_t)CalibData.dig_T1))) >> 12) *

  ((int32_t)CalibData.dig_T3)) >> 14;

  temp_int = val1 + val2;
  temper_float = ((temp_int * 5 + 128) >> 8);
  temper_float /= 100.0f;

  return temper_float;

}

//------------------------------------------------

uint32_t BME280_ReadPressure(void)

{

  //float press_float = 0.0f;

  uint32_t press_raw, pres_int;

  int64_t val1, val2, p;

  BME280_ReadTemperature(); // must be done first to get t_fine

  BME280_ReadReg_BE_U24(BME280_REGISTER_PRESSUREDATA,&press_raw);

  press_raw >>= 4;

  val1 = ((int64_t) temp_int) - 128000;

  val2 = val1 * val1 * (int64_t)CalibData.dig_P6;

  val2 = val2 + ((val1 * (int64_t)CalibData.dig_P5) << 17);

  val2 = val2 + ((int64_t)CalibData.dig_P4 << 35);

  val1 = ((val1 * val1 * (int64_t)CalibData.dig_P3) >> 8) + ((val1 * (int64_t)CalibData.dig_P2) << 12);

  val1 = (((((int64_t)1) << 47) + val1)) * ((int64_t)CalibData.dig_P1) >> 33;

  if (val1 == 0) {

    return 0; // avoid exception caused by division by zero

  }

  p = 1048576 - press_raw;

  p = (((p << 31) - val2) * 3125) / val1;

  val1 = (((int64_t)CalibData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;

  val2 = (((int64_t)CalibData.dig_P8) * p) >> 19;

  p = ((p + val1 + val2) >> 8) + ((int64_t)CalibData.dig_P7 << 4);

  pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
  pres_int /= 1000;

  //press_float = pres_int / 1000.0f;

  //return press_float;
  return pres_int;

}

//------------------------------------------------

float BME280_ReadAltitude(float seaLevel)

{

  float att = 0.0f;

  return att;

}

//------------------------------------------------

uint32_t Get_Pressure(void)
{
  BME280_ReadPressure();
}

//------------------------------------------------

void BME280_Init(void)

{

  uint8_t value=0;

  //uint32_t value32=0;

  //LED_ON;
  value = BME280_ReadReg(BME280_REG_ID);
  //sprintf(str1, "rnrnID: 0x%02Xrn", value);

  //HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);

  BME280_WriteReg(BME280_REG_SOFTRESET,BME280_SOFTRESET_VALUE);
  while (BME280_ReadStatus() & BME280_STATUS_IM_UPDATE) ;
  BME280_ReadCoefficients();
  BME280_SetStandby(BME280_STBY_1000);
  BME280_SetFilter(BME280_FILTER_4);
  BME280_SetOversamplingTemper(BME280_OSRS_T_x4);

  BME280_SetOversamplingPressure(BME280_OSRS_P_x2);

  //BME280_SetOversamplingHum(BME280_OSRS_H_x1);
  //value32 = BME280_ReadReg(BME280_REG_CTRL_MEAS);

  //value32 |= BME280_ReadReg(BME280_REG_CTRL_HUM) << 8;
  BME280_SetMode(BME280_MODE_NORMAL);

  return;
}

//------------------------------------------------