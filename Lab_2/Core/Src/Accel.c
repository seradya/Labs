#include "Accel.h"
#include "stm32l1xx_hal.h"
#include "i2c.h"

void LIS331_Init(LIS331_t* lis, comm_mode mode) {
    lis->mode = mode;
    
    // Устанавливаем стандартный режим
    LIS331_SetPowerMode(lis, NORMAL);
    for (uint32_t i = 100; i > 0; i--){};
    // Включаем оси
    uint8_t data = 0;
    LIS331_ReadReg(lis, CTRL_REG1, &data, 1);
    data |= 0x07;
    LIS331_WriteReg(lis, CTRL_REG1, &data, 1);
}

void LIS331_SetPowerMode(LIS331_t* lis, power_mode pmode) {
    uint8_t data;
    LIS331_ReadReg(lis, CTRL_REG1, &data, 1);
    for (uint32_t i = 100; i > 0; i--){};
    data &= ~0xE0;  
    data |= pmode << 5;  
    LIS331_WriteReg(lis, CTRL_REG1, &data, 1);
}

void LIS331_SetODR(LIS331_t* lis, data_rate drate) {
    uint8_t data;
    LIS331_ReadReg(lis, CTRL_REG1, &data, 1);

    for (uint32_t i = 100; i > 0; i--){};
    
    data &= ~0x18; 
    data |= drate << 3;  
    LIS331_WriteReg(lis, CTRL_REG1, &data, 1);
}

void LIS331_ReadAxes(LIS331_t* lis, int16_t* x, int16_t* y, int16_t* z) {
    uint8_t x_l, x_h, y_l, y_h, z_l, z_h;
    
    LIS331_ReadReg(lis, OUT_X_L, &x_l, 1);
    LIS331_ReadReg(lis, OUT_X_H, &x_h, 1);
    LIS331_ReadReg(lis, OUT_Y_L, &y_l, 1);
    LIS331_ReadReg(lis, OUT_Y_H, &y_h, 1);
    LIS331_ReadReg(lis, OUT_Z_L, &z_l, 1);
    LIS331_ReadReg(lis, OUT_Z_H, &z_h, 1);

    *x = ((int16_t)x_h << 8) | x_l;
    *y = ((int16_t)y_h << 8) | y_l;
    *z = ((int16_t)z_h << 8) | z_l;
}

void LIS331_WriteReg(LIS331_t* lis, uint8_t reg_address, uint8_t* data, uint8_t len) {
    if (lis->mode == USE_I2C) {
        // Используем HAL для I2C
        //uint8_t buff[2] = {reg_address, *data};
        //HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(lis->address << 1), buff, len + 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(lis->address << 1), reg_address, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
    } else if (lis->mode == USE_SPI) {
        // Для SPI
        //HAL_GPIO_WritePin(lis->CSPinPort, lis->CSPin, GPIO_PIN_RESET);  
        //HAL_SPI_Transmit(&hspi1, &reg_address, 1, HAL_MAX_DELAY);  
        //HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);  
        //HAL_GPIO_WritePin(lis->CSPinPort, lis->CSPin, GPIO_PIN_SET);  
    }
}

void LIS331_ReadReg(LIS331_t* lis, uint8_t reg_address, uint8_t* data, uint8_t len) {
    if (lis->mode == USE_I2C) {
        HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(lis->address << 1), reg_address, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
    } else if (lis->mode == USE_SPI) {
        reg_address |= 0x80;  
        //HAL_GPIO_WritePin(lis->CSPinPort, lis->CSPin, GPIO_PIN_RESET);  
        //HAL_SPI_Transmit(&hspi1, &reg_address, 1, HAL_MAX_DELAY);  
        //HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);  
        //HAL_GPIO_WritePin(lis->CSPinPort, lis->CSPin, GPIO_PIN_SET); 
    }
}