/*
 * Shift_Register.h
 *
 *  Created on: 05-Sep-2021
 *      Author: SSE
 */

#ifndef INC_SHIFT_REGISTER_H_
#define INC_SHIFT_REGISTER_H_



#include "stm32h7xx_hal.h" //need for SPI and GPIO pins

// Functions to use shift registers of different measurement modules

HAL_StatusTypeDef CAL_WriteShiftRegister_CurrentModule(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t data);

HAL_StatusTypeDef CAL_WriteShiftRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t* data);

HAL_StatusTypeDef CAL_WriteShiftRegister_VoltageModule(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t data);

#endif /* INC_SHIFT_REGISTER_H_ */
