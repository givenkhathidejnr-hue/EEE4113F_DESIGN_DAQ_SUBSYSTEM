/*
 * INA219.c
 *
 *  Created on: May 15, 2026
 *      Author: simph
 */
#include "INA219.h"

/* Initializes communication with the INA219 currents sensor */
HAL_StatusTypeDef INA219_Init(INA219_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;

    // 1. Calibration for 0.1 Ohm Shunt, Max 3.2A
    uint16_t calValue = 4096;
    uint8_t calData[3];
    calData[0] = INA219_REG_CALIB;
    calData[1] = (calValue >> 8) & 0xFF;
    calData[2] = calValue & 0xFF;

    dev->current_lsb = 0.0001f; // 100uA per bit
    dev->power_lsb = 0.002f;    // 2mW per bit (20 * Current_LSB)

    // 2. Write Configuration (32V Range, 12-bit ADC, Shunt+Bus continuous)
    uint16_t config = 0x399F;
    uint8_t configData[3] = {INA219_REG_CONFIG, (config >> 8), (config & 0xFF)};

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, INA219_ADDRESS, calData, 3, 100);
    if(status != HAL_OK) return status;

    return HAL_I2C_Master_Transmit(dev->hi2c, INA219_ADDRESS, configData, 3, 100);
}

float INA219_ReadCurrent(INA219_t *dev) {
    uint8_t reg = INA219_REG_CURRENT;
    uint8_t data[2];
    HAL_I2C_Master_Transmit(dev->hi2c, INA219_ADDRESS, &reg, 1, 100);
    HAL_I2C_Master_Receive(dev->hi2c, INA219_ADDRESS, data, 2, 100);

    int16_t rawCurrent = (data[0] << 8) | data[1];
    return (float)rawCurrent * dev->current_lsb * 1000.0f; // Return in mA
}

float INA219_ReadBusVoltage(INA219_t *dev) {
    uint8_t reg = INA219_REG_BUS;
    uint8_t data[2];
    HAL_I2C_Master_Transmit(dev->hi2c, INA219_ADDRESS, &reg, 1, 100);
    HAL_I2C_Master_Receive(dev->hi2c, INA219_ADDRESS, data, 2, 100);

    uint16_t rawBus = (data[0] << 8) | data[1];
    return (float)((rawBus >> 3) * 4) * 0.001f; // 4mV per LSB
}

float INA219_ReadPower(INA219_t *dev) {
    uint8_t reg = INA219_REG_POWER;
    uint8_t data[2];

    // Point to Power Register
    HAL_I2C_Master_Transmit(dev->hi2c, INA219_ADDRESS, &reg, 1, 100);
    // Read 2 bytes
    HAL_I2C_Master_Receive(dev->hi2c, INA219_ADDRESS, data, 2, 100);

    uint16_t rawPower = (data[0] << 8) | data[1];

    // Based on the 100uA Current LSB calibration:
    // Power LSB = 20 * Current LSB = 2mW per bit
    return (float)rawPower * dev->power_lsb * 1000.0f; // Returns value in mW
}

