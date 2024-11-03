#include "stmpe1600.h"
//-------------------------------------------------
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
//-------------------------------------------------
uint16_t gpio_data;
//-------------------------------------------------
int stmpe1600_WriteReg( int index, uint8_t *data, int n_data)
{
  int status;
  uint8_t RegAddr[16];
  RegAddr[0]=index;
  memcpy(RegAddr+1, data, n_data);
  status=HAL_I2C_Master_Transmit(&hi2c1, EXPANDER_I2C_ADDRESS, RegAddr, n_data+1, 100);
  return status;
}
//-------------------------------------------------
int stmpe1600_ReadReg(int index, uint8_t *data, int n_data)
{
  int status;
  uint8_t RegAddr;
  RegAddr=index;
  do{
    status=HAL_I2C_Master_Transmit(&hi2c1, EXPANDER_I2C_ADDRESS, &RegAddr, 1, 1000);
    if( status )
      break;
    status =HAL_I2C_Master_Receive(&hi2c1, EXPANDER_I2C_ADDRESS, data, n_data, 1000);
  }while(0);
  return status;
}
//-------------------------------------------------
int stmpe1600_ini(void)
{
  int status;
	uint16_t gpio_direction;
	uint8_t RegAddr=0;
	uint8_t data[3];
	do{
		status=HAL_I2C_Master_Transmit(&hi2c1, EXPANDER_I2C_ADDRESS, &RegAddr, 1, 100);
		if( status )
			break;
		status =HAL_I2C_Master_Receive(&hi2c1, EXPANDER_I2C_ADDRESS, data, 2, 200);
	} while(0);
	if( status == 0 && data[0]==0 && data[1]==0x16)
	{
		data[0] = GPDR; 
		gpio_direction = ~V2_DISP_SEL; 
		memcpy(data+1, &gpio_direction, 2);
		status=HAL_I2C_Master_Transmit(&hi2c1, EXPANDER_I2C_ADDRESS, data, 3, 100);
		gpio_data = (V2_D1|V2_D2|V2_D3| V2_D4);
		gpio_data |= 0x7F; // clear 7 seg bits
		gpio_data |= V2_D1|V2_D2|V2_D3|V2_D4; // all segment off
		gpio_data &= ~(V2_D1<<1); // digit on
		gpio_data &= ~(0x0F);
		stmpe1600_WriteReg(GPSR, (uint8_t*)&gpio_data, 2);
	}



	
  return status;
}
//-------------------------------------------------