/*
 * Shift_Register.c
 *
 *  Created on: 05-Sep-2021
 *      Author: SSE
 */

#include "Shift_Register.h"
#include "stm32h7xx_hal.h"

// Current module contain 5 different ranges of current.
// Function to select one of the current range and proper gain of the amplifier by sending 16 bit serial data to shift register
// First 8 bits in 16 bits for gain selection and last 8 bits for relay control to select correct current range

float shunt_resistance;

HAL_StatusTypeDef CAL_WriteShiftRegister_CurrentModule(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t data)

{

	uint8_t tx[2]={0x03,0x01};
	uint8_t tx1[2]={0x03,0x02};
	uint8_t tx2[2]={0x03,0x04};
	uint8_t tx3[2]={0x03,0x08};
	uint8_t tx4[2]={0x03,0x10};

	switch(data)
	{

	case 0x01:



		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // reset state

			  // Writing shift register

			  HAL_SPI_Transmit(hspi, tx, 2, 100); // To select input current range

			  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
			  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
			  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
			  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

		shunt_resistance = 0.0111;
         break;
	case 0x02:

		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

					  HAL_SPI_Transmit(hspi, tx1, 2, 100);

					  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
					  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
					  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
					  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

				shunt_resistance = 0.111;
				break;
	case 0x03:

		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

					  HAL_SPI_Transmit(hspi, tx2, 2, 100);

					  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
					  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
					  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
					  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

				shunt_resistance = 1.111;
				break;

	case 0x04:

		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

					  HAL_SPI_Transmit(hspi, tx3, 2, 100);

					  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
					  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
					  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
					  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

				shunt_resistance = 10.11;
				break;

	case 0x05:

			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

						  HAL_SPI_Transmit(hspi, tx4, 2, 100);

						  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
						  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
						  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
						  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

					shunt_resistance = 111.111;
					break;

	case 0x06:

				HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

							  HAL_SPI_Transmit(hspi, (uint8_t *)0x40, 1, 100);

							  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
							  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
							  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
							  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

							  break;

	case 0x07:

				HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);


							  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
							  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
							  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
							  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

							  break;

		default:

		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

					  HAL_SPI_Transmit(hspi, tx3, 2, 100);

					  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
					  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
					  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
					  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

				shunt_resistance = 10.11;
				break;
	}



	return shunt_resistance;

}

// Function to enable ADC chips on Thermocouple module and to select chip select for ADC SPI
// Thermocouple module contain 4 channels for ADC chip. User can enable each channel of ADC separately
// To select channel user have to write correct 8 bit serial data into shift register.

HAL_StatusTypeDef CAL_WriteShiftRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t* data)

{



	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // reset state

				  // Writing shift register

				  HAL_SPI_Transmit(hspi, data, 1, 100); // To select input current range

				  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
				  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
				  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
				  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);


}

//HAL_StatusTypeDef CAL_WriteShiftRegister_VoltageModule(uint8_t data);
