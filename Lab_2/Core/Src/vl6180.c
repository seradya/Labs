#include "vl6180.h"
//-------------------------------------------------
//extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
//-------------------------------------------------
extern uint16_t gpio_data;
extern volatile uint32_t TickCnt;
extern char str1[60];
State_ptr VL6180_State;
extern volatile int8_t switch_state;
extern volatile int8_t new_switch_state;
VL6180xDevData_ptr VL6180_DevData;
VL6180x_AlsData_t Als; /* ALS measurement */
//-------------------------------------------------
static const uint16_t AlsGainLookUp[8] = {
  (uint16_t)(20.0f * AN_GAIN_MULT),
  (uint16_t)(10.0f * AN_GAIN_MULT),
  (uint16_t)(5.0f * AN_GAIN_MULT),
  (uint16_t)(2.5f * AN_GAIN_MULT),
  (uint16_t)(1.67f * AN_GAIN_MULT),
  (uint16_t)(1.25f * AN_GAIN_MULT),
  (uint16_t)(1.0f * AN_GAIN_MULT),
  (uint16_t)(40.0f * AN_GAIN_MULT),
};
//-------------------------------------------------
int vl6180_I2C_Write(uint8_t *buff, uint8_t len)
{
  int status;
  status = HAL_I2C_Master_Transmit(&hi2c1, VL6180_ADDRESS, buff, len , HAL_MAX_DELAY);
  return status;
}
//-------------------------------------------------
int vl6180_I2C_Read(uint8_t *buff, uint8_t len)
{
  int status;
  status = HAL_I2C_Master_Receive(&hi2c1, VL6180_ADDRESS, buff, len , HAL_MAX_DELAY);
  return status;
}
//-------------------------------------------------
int vl6180_ReadByte(uint16_t index, uint8_t *data)
{
  int status;
  uint8_t buffer[2];
  buffer[0] = index>>8;
  buffer[1] = index&0xFF;
  status = vl6180_I2C_Write(buffer, (uint8_t)2);
  if( !status ){
    status=vl6180_I2C_Read(buffer,1);
    if( !status ){
      *data=buffer[0];
    }
  }
  return status;
}
//-------------------------------------------------
int vl6180_ReadWord(uint16_t index, uint16_t *data)
{
  int status;
  uint8_t buffer[2];
  buffer[0] = index>>8;
  buffer[1] = index&0xFF;
  status = vl6180_I2C_Write(buffer, (uint8_t)2);
  if( !status ){
    status=vl6180_I2C_Read(buffer,2);
    if( !status ){
      *data=((uint16_t)buffer[0]<<8)|(uint16_t)buffer[1];
    }
  }
  return status;
}
//-------------------------------------------------
int vl6180_WriteByte(uint16_t index, uint8_t data)
{
  int status;
  uint8_t buffer[3];
  buffer[0] = index>>8;
  buffer[1] = index&0xFF;
  buffer[2] = data;
  status = vl6180_I2C_Write(buffer, (uint8_t)3);
  return status;
}
//-------------------------------------------------
int vl6180_WriteWord(uint16_t index, uint16_t data)
{
  int status;
  uint8_t buffer[4];
  buffer[0]=index>>8;
  buffer[1]=index&0xFF;
  buffer[2]=data>>8;
  buffer[3]=data&0xFF;
  status = vl6180_I2C_Write(buffer, (uint8_t)4);
  return status;
}
//-------------------------------------------------
int vl6180_UpdateByte(uint16_t index, uint8_t AndData, uint8_t OrData){
  int status;
  uint8_t buffer[3];
  buffer[0]=index>>8;
  buffer[1]=index&0xFF;
  status=vl6180_I2C_Write((uint8_t *)buffer,(uint8_t)2);
  if( !status ){
    status=vl6180_I2C_Read(&buffer[2],1);
    if( !status ){
      buffer[2]=(buffer[2]&AndData)|OrData;
      status=vl6180_I2C_Write(buffer, (uint8_t)3);
    }
  }
  return status;
}
//-------------------------------------------------
void Delay_MS_Tim(int ms)
{
  uint32_t start, now;
  int dif;
  start=TickCnt;
  do{
    now=TickCnt;
    dif= now -start;
  }
  while(dif<ms);
}
//-------------------------------------------------
static void vl6180_SetChipEn(int state)
{
  if(state)
    gpio_data|=VL6180_CHIPEN;
  else
    gpio_data&=~VL6180_CHIPEN;
  stmpe1600_WriteReg(GPSR, (uint8_t*)&gpio_data,2);
}
//-------------------------------------------------
int vl6180_WaitDeviceBooted(void)
{
  int status;
  uint8_t FreshOutReset;
  do {
    status = vl6180_ReadByte(SYSTEM_FRESH_OUT_OF_RESET, &FreshOutReset);
  } while (FreshOutReset != 1 && status == 0);
  return status;
}
//-------------------------------------------------
int stmpe1600_GetSwitch(void)
{
  int status;
  GPIO_PinState state;
  uint16_t Value;
  status=stmpe1600_ReadReg(GPMR, (uint8_t*)&Value,2);
  if(status ==0) Value&=V2_DISP_SEL;
  else Value=0;
  state = Value ? GPIO_PIN_SET : GPIO_PIN_RESET;
  return state;
}
//-------------------------------------------------
int vl6180_AlsSetAnalogueGain(uint8_t gain)
{
  int status;
  uint8_t GainTotal;
  gain &= ~0x40;
  if (gain > 7) {
    gain = 7;
  }
  GainTotal = gain | 0x40;
  status = vl6180_WriteByte(SYSALS_ANALOGUE_GAIN, GainTotal);
  if (!status) {
    VL6180_DevData.AlsGainCode = gain;
  }
  return status;
}
//-------------------------------------------------
int vl6180_StaticInit(void)
{
  int status;
  uint8_t data;
  do {
    status = vl6180_ReadByte(FW_ALS_RESULT_SCALER, &data);
    if (status)
      break;
    VL6180_DevData.AlsScaler = data;
	  status = vl6180_ReadByte(SYSALS_ANALOGUE_GAIN, &data);
		if (status)
			break;
		vl6180_AlsSetAnalogueGain(data);
  } while (0);
  return status;
}
//-------------------------------------------------
int vl6180_AlsSetIntegrationPeriod(uint16_t period_ms)
{
  int status;
  uint16_t SetIntegrationPeriod;
  if (period_ms >= 1)
    SetIntegrationPeriod = period_ms - 1;
  else
    SetIntegrationPeriod = period_ms;
  if (SetIntegrationPeriod > 464) {
    SetIntegrationPeriod = 464;
  } else if (SetIntegrationPeriod == 255) {
    SetIntegrationPeriod++; /* can't write 255 since this causes the device to lock out.*/
  }
  status = vl6180_WriteWord(SYSALS_INTEGRATION_PERIOD, SetIntegrationPeriod);
  if (!status) {
    VL6180_DevData.IntegrationPeriod = SetIntegrationPeriod;
  }
  return status;
}
//-------------------------------------------------
int vl6180_AlsSetInterMeasurementPeriod(uint16_t intermeasurement_period_ms)
{
  int status;
  if (intermeasurement_period_ms >= 255 * 10)
  intermeasurement_period_ms = 255 * 10;
  status = vl6180_WriteByte(SYSALS_INTERMEASUREMENT_PERIOD, (uint8_t)(intermeasurement_period_ms / 10));
  return status;
}
//-------------------------------------------------
int vl6180_AlsSetThresholds(uint8_t low, uint8_t high)
{
  int status;
  status = vl6180_WriteByte(SYSALS_THRESH_LOW, low);
  if (!status) {
    status = vl6180_WriteByte(SYSALS_THRESH_HIGH, high);
  }
  return status;
}
//-------------------------------------------------
int vl6180_AlsConfigInterrupt(uint8_t ConfigGpioInt)
{
  int status;
  if (ConfigGpioInt <= CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY) {
  status = vl6180_UpdateByte(SYSTEM_INTERRUPT_CONFIG_GPIO, (uint8_t)(~CONFIG_GPIO_ALS_MASK), (ConfigGpioInt << 3));
  } else {
    status = INVALID_PARAMS;
  }
  return status;
}
//-------------------------------------------------
int vl6180_ClearInterrupt(uint8_t IntClear)
{
  int status;
  if (IntClear <= 7) {
    status = vl6180_WriteByte(SYSTEM_INTERRUPT_CLEAR, IntClear);
  } else {
    status = INVALID_PARAMS;
  }
  return status;
}
//-------------------------------------------------
int vl6180_Prepare(void)
{
  int status;
	do {
		status = vl6180_StaticInit();
		if (status)
			break;
		status = vl6180_AlsSetIntegrationPeriod(100);
		if (status)
			break;
		status = vl6180_AlsSetInterMeasurementPeriod(200);
		if (status)
			break;
		//âûñòàâèì óñèëåíèå äëÿ îáåñïå÷åíèÿ òî÷íîñòè â çàäàííîì äèàïàçîíå
		if(new_switch_state) status=vl6180_AlsSetAnalogueGain(0);
		else status=vl6180_AlsSetAnalogueGain(3);
		if (status)
			break;
		status = vl6180_AlsSetThresholds(0, 0xFF);
		if (status)
			break;
		status = vl6180_AlsConfigInterrupt(CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
		if (status)
			break;
		status = vl6180_ClearAllInterrupt();
	} while (0);	
  return status;
}
//-------------------------------------------------
void InitAlsMode(void)
{
  //anything after prepare and prior to go into AlsState
  int time = 100;
  vl6180_AlsSetIntegrationPeriod(time);
}
//-------------------------------------------------
int vl6180_AlsSetSystemMode(uint8_t mode)
{
  int status;
  if (mode <= 3) {
    status = vl6180_WriteByte(SYSALS_START, mode);
  } else {
    status = INVALID_PARAMS;
  }
  return status;
}
//-------------------------------------------------
int vl6180_AlsGetInterruptStatus(uint8_t *pIntStatus)
{
  int status;
  uint8_t IntStatus;
  status = vl6180_ReadByte(RESULT_INTERRUPT_STATUS_GPIO, &IntStatus);
  *pIntStatus = (IntStatus >> 3) & 0x07;
  return status;
}
//-------------------------------------------------
void vl6180_PollDelay(void)
{
  //Delay_MS_Tim(5);
}
//-------------------------------------------------
int vl6180_AlsGetLux(uint32_t *pLux)
{
  int status;
  uint16_t RawAls;
  uint32_t luxValue = 0;
  uint32_t IntPeriod;
  uint32_t AlsAnGain;
  uint32_t GainFix;
  uint32_t AlsScaler;
	const uint32_t LuxResxIntIme = (uint32_t)(0.56f * DEF_INT_PEFRIOD * (1 << LUXRES_FIX_PREC));
	status = vl6180_ReadWord(RESULT_ALS_VAL, &RawAls);
	if (!status) {
		IntPeriod = VL6180_DevData.IntegrationPeriod;
		AlsScaler = VL6180_DevData.AlsScaler;
		IntPeriod++; // what stored is real time ms -1 and it can be 0 for or 0 or 1ms
		luxValue = (uint32_t)RawAls * LuxResxIntIme; // max # 16+8bits + 6bit (0.56*100)
		luxValue /= IntPeriod; // max # 16+8bits + 6bit 16+8+1 to 9 bit
		if (new_switch_state)
		{
			luxValue *=10;
		}
		//between 29 - 21 bit
		AlsAnGain = VL6180_DevData.AlsGainCode;
		GainFix = AlsGainLookUp[AlsAnGain];
		luxValue = luxValue / (AlsScaler * GainFix);
		//Ñíà÷àëà óñòàíîâèì íîðìàëüíîå ñîñòîÿíèå ïîêàçàíèé
		Als.valueStatus = ValueNormal;
		//Îãðàíè÷èì ìèíèìàëüíûå çíà÷åíèÿ
		if (new_switch_state)
		{
			if(luxValue<28) Als.valueStatus=ValueLow;
		}
		else
		{
			if(luxValue<2) Als.valueStatus=ValueLow;
		}
		*pLux = luxValue;
	}
  return status;
}
//-------------------------------------------------
int vl6180_AlsGetMeasurement(VL6180x_AlsData_t *pAlsData)
{
  int status;
  uint8_t ErrStatus;
	status = vl6180_AlsGetLux(&pAlsData->lux);
	if (!status) {
			status = vl6180_ReadByte(RESULT_ALS_STATUS, &ErrStatus);
		pAlsData->errorStatus = ErrStatus >> 4;
	}	
  return status;
}
//-------------------------------------------------
int vl6180_AlsPollMeasurement(VL6180x_AlsData_t *pAlsData)
{
  int status;
  int ClrStatus;
  uint8_t IntStatus;
	status = vl6180_AlsSetSystemMode(MODE_START_STOP | MODE_SINGLESHOT);
  if (status) {
    goto over;
  }
	while(1)
  {
    status = vl6180_AlsGetInterruptStatus(&IntStatus);
    if (status) {
      break;
    }
    if (IntStatus == RES_INT_STAT_GPIO_NEW_SAMPLE_READY) {
      break; /* break on new data (status is 0) */
    }
		vl6180_PollDelay();
  };
	if (!status) {
    status = vl6180_AlsGetMeasurement(pAlsData);
  }
  ClrStatus = vl6180_AlsClearInterrupt();
  if (ClrStatus) {
    if (!status) {
      status = ClrStatus; /* leave previous if already on error */
    }
  }
over:
  return status;
}
//-------------------------------------------------
void vl6180_AlsState(void)
{
  int status;
	status = vl6180_AlsPollMeasurement(&Als);
  if (status) {
    //HAL_UART_Transmit(&huart2, (uint8_t*)"Er 4",6,0x1000);
  }
}
//-------------------------------------------------
void vl6180_ReadData(void)
{
  int status;
	if (new_switch_state != switch_state)
	{
		//sprintf(str1,"Switch state: %d\r\n",new_switch_state);
		//HAL_UART_Transmit(&huart2, (uint8_t*)str1,strlen(str1),0x1000);
		switch_state=new_switch_state;
		status = vl6180_Prepare();
		//if(status) Error_Handler();
		VL6180_State.mode = RunAlsPoll;
    InitAlsMode();
	}
	switch (VL6180_State.mode)
  {
    case RunAlsPoll:
      vl6180_AlsState();
      break;
  }
}
//-------------------------------------------------
int vl6180_ini(void)
{
  int status;
	uint8_t dt;
	vl6180_SetChipEn(0);
	//Delay_MS_Tim(10);
	vl6180_SetChipEn(1);
	//Delay_MS_Tim(1);
	status = vl6180_WaitDeviceBooted();
	if(status) return 1;
	vl6180_ReadByte(IDENTIFICATION__MODEL_ID, &dt);
	//sprintf(str1,"vl6180 ID: 0x%02X\r\n",dt);
	//HAL_UART_Transmit(&huart2, (uint8_t*)str1,strlen(str1),0x1000);
  return status;
}
//-------------------------------------------------