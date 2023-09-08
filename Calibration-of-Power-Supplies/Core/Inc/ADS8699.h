/*
 * ADS8699.h
 *
 *  Created on: 04-Sep-2021
 *      Author: SSE
 */

#ifndef INC_ADS8699_H_
#define INC_ADS8699_H_

#include "stm32h7xx_hal.h" //need for SPI and GPIO pins

// Defining configuration registers

//Write operation selection to configure shift registers in ADC ADS8699
//ALL configuration registers are 32bit registers

/*#define DEVICE_ID_REG_WRITE 0xD0
#define RST_PWRCTL_REG_WRITE 0xD0
#define SDI_CTL_REG_WRITE 0xD0
#define SDO_CTL_REG_WRITE 0xD0
#define DATAOUT_CTL_REG_WRITE 0xD0
#define RANGE_SEL_REG_WRITE 0xD0
#define ALARAM_REG_WRITE 0xD0
#define ALARAM_H_TH_REG_WRITE 0xD0
#define ALARAM_L_TH_REG_WRITE 0xD0

// Read operation to read data from ADC

#define SDO_CTL_REG_READ 0x48 // only data from SDO_CTL_REG register is read

// Address of the registers

#define DEVICE_ID_REG_ADDRESS 0x00
#define RST_PWRCTL_REG_ADDRESS 0x04
#define SDI_CTL_REG_ADDRESS 0x08
#define SDO_CTL_REG_ADDRESS 0x0C
#define DATAOUT_CTL_REG_ADDRESS 0x10
#define RANGE_SEL_REG_ADDRESS 0x14
#define ALARAM_REG_ADDRESS 0x20
#define ALARAM_H_TH_REG_ADDRESS 0x24
#define ALARAM_L_TH_REG_ADDRESS 0x28

// First 8 bits of data to write into the registers

#define DEVICE_ID_REG_DATA1 0x00
#define RST_PWRCTL_REG_DATA1 0x00
#define SDI_CTL_REG_DATA1 0x00
#define SDO_CTL_REG_DATA1 0x00
#define DATAOUT_CTL_REG_DATA1 0x00
#define RANGE_SEL_REG_DATA1 0x00
#define ALARAM_REG_DATA1 0x00
#define ALARAM_H_TH_REG_DATA1 0x00
#define ALARAM_L_TH_REG_DATA1 0x00

// Last 8 bits of data to write into the registers

#define DEVICE_ID_REG_DATA2 0x00
#define RST_PWRCTL_REG_DATA2 0x00
//#define SDI_CTL_REG_DATA2
#define SDO_CTL_REG_DATA2 0x00
//#define DATAOUT_CTL_REG_DATA2
//#define RANGE_SEL_REG_DATA2
#define ALARAM_REG_DATA2 0x00
#define ALARAM_H_TH_REG_DATA2 0x00
#define ALARAM_L_TH_REG_DATA2 0x00 */

#define channe11 0xE1   // Data to select the first channel1 of thermocouple
#define channel1_1 0xF1

#define channe12 0xD2   // Data to select the first channel2 of thermocouple
#define channel2_1 0xF2

#define channe13 0xB4   // Data to select the first channel3 of thermocouple
#define channel3_1 0xF4

#define channe14 0x78   // Data to select the first channel4 of thermocouple
#define channel4_1 0xF8



// Functions to configure ADC and read data from ADC

HAL_StatusTypeDef CAL_ADS8699_Configuration(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t SDI_CTL_REG_DATA2,uint8_t DATAOUT_CTL_REG_DATA2,uint8_t RANGE_SEL_REG_DATA2);

HAL_StatusTypeDef CAL_ADS8699_Read_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t *data);

HAL_StatusTypeDef CAL_ADC_Averaging_Buffer(uint8_t data);

HAL_StatusTypeDef CAL_ADC_Real_Measured_Data(uint8_t RANGE_SEL_REG_DATA2, uint8_t *data,int data1);

HAL_StatusTypeDef CAL_ADC_ThermoCoupleModule_Configuration(SPI_HandleTypeDef *spi,SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t *tx,uint8_t *tx1, uint8_t SDI_CTL_REG_DATA2,uint8_t DATAOUT_CTL_REG_DATA2,uint8_t RANGE_SEL_REG_DATA2);

HAL_StatusTypeDef CAL_ADC_ThermoCoupleModule_Read_Data(SPI_HandleTypeDef *spi,SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t *tx,uint8_t *tx1,uint8_t *data);

HAL_StatusTypeDef Thermo_Couple_Channel_Select_Data(int data);

HAL_StatusTypeDef Thermo_Couple_Channel_Select_Data1(int data);

#endif /* INC_ADS8699_H_ */

