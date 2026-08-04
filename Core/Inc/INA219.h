/*
 * INA219.h
 *
 *  Created on: May 15, 2026
 *      Author: simph
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stm32l4xx_hal.h"

// Default I2C Address (A0/A1 tied to GND)
#define INA219_ADDRESS      (0x40 << 1)

// Register Map
#define INA219_REG_CONFIG   0x00
#define INA219_REG_SHUNT    0x01
#define INA219_REG_BUS      0x02
#define INA219_REG_POWER    0x03
#define INA219_REG_CURRENT  0x04
#define INA219_REG_CALIB    0x05

typedef struct {
    I2C_HandleTypeDef 	*hi2c;
    float				voltage_lsb;
    float 				current_lsb;
    float 				power_lsb;
} INA219_t;

// Function Prototypes
HAL_StatusTypeDef INA219_Init(INA219_t *dev, I2C_HandleTypeDef *hi2c);
float INA219_ReadBusVoltage(INA219_t *dev);
float INA219_ReadCurrent(INA219_t *dev);
float INA219_ReadPower(INA219_t *dev);

#endif /* INC_INA219_H_ */
