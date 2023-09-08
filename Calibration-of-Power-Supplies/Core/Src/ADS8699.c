/*
 * ADS8699.c
 *
 *  Created on: 04-Sep-2021
 *      Author: SSE
 */
#include "ADS8699.h"
#include "stm32h7xx_hal.h"
#include "Shift_Register.h"

//Write operation selection to configure ADC

uint8_t DEVICE_ID_REG_WRITE =0xD0;
uint8_t RST_PWRCTL_REG_WRITE =0xD0;
uint8_t SDI_CTL_REG_WRITE =0xD0;
uint8_t SDO_CTL_REG_WRITE =0xD0;
uint8_t DATAOUT_CTL_REG_WRITE =0xD0;
uint8_t RANGE_SEL_REG_WRITE =0xD0;
uint8_t ALARAM_REG_WRITE =0xD0;
uint8_t ALARAM_H_TH_REG_WRITE =0xD0;
uint8_t ALARAM_L_TH_REG_WRITE =0xD0;

// Read operation to read data from ADC

uint8_t SDO_CTL_REG_READ =0x48; // only data from SDO_CTL_REG register is read

// Address of the registers

uint8_t DEVICE_ID_REG_ADDRESS =0x00;
uint8_t RST_PWRCTL_REG_ADDRESS =0x04;
uint8_t SDI_CTL_REG_ADDRESS =0x08;
uint8_t SDO_CTL_REG_ADDRESS =0x0C;
uint8_t DATAOUT_CTL_REG_ADDRESS= 0x10;
uint8_t RANGE_SEL_REG_ADDRESS =0x14;
uint8_t ALARAM_REG_ADDRESS =0x20;
uint8_t ALARAM_H_TH_REG_ADDRESS= 0x24;
uint8_t ALARAM_L_TH_REG_ADDRESS =0x28;

// First 8 bits of data to write into the registers

uint8_t DEVICE_ID_REG_DATA1 =0x00;
uint8_t RST_PWRCTL_REG_DATA1 =0x00;
uint8_t SDI_CTL_REG_DATA1 =0x00;
uint8_t SDO_CTL_REG_DATA1 =0x00;
uint8_t DATAOUT_CTL_REG_DATA1= 0x00;
uint8_t RANGE_SEL_REG_DATA1 =0x00;
uint8_t ALARAM_REG_DATA1 =0x00;
uint8_t ALARAM_H_TH_REG_DATA1 =0x00;
uint8_t ALARAM_L_TH_REG_DATA1 =0x00;

// Last 8 bits of data to write into the registers

uint8_t DEVICE_ID_REG_DATA2 =0x00;
uint8_t RST_PWRCTL_REG_DATA2 =0x00;
//#define SDI_CTL_REG_DATA2
uint8_t SDO_CTL_REG_DATA2 =0x00;
//#define DATAOUT_CTL_REG_DATA2
//#define RANGE_SEL_REG_DATA2
uint8_t ALARAM_REG_DATA2 =0x00;
uint8_t ALARAM_H_TH_REG_DATA2 =0x00;
uint8_t ALARAM_L_TH_REG_DATA2 =0x00;

// Function to configure ADC ADS8699
// Each configuration register is 32 bit size and ADS8699 contain 9 configuration registers
// The structure of each 32 bit register is <7-bit OPcode for read and write operation>_<9-bit address>_<16-bit Data>
// <9-bit address> is realized by adding a 0 at the MSB location followed by an 8-bit register address as defined
// Configuring registers which are necessary for project here but one can configure all 9 configuration registers according to project

HAL_StatusTypeDef CAL_ADS8699_Configuration(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t SDI_CTL_REG_DATA2,uint8_t DATAOUT_CTL_REG_DATA2,uint8_t RANGE_SEL_REG_DATA2)

{

	HAL_StatusTypeDef errorcode = HAL_OK;

// Configuring SDI_CTL_REG register to select the SPI mode of ADC
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

		HAL_SPI_Transmit(hspi, &SDI_CTL_REG_WRITE, 1, 100);
		HAL_SPI_Transmit(hspi, &SDI_CTL_REG_ADDRESS, 1, 100);
		HAL_SPI_Transmit(hspi, &SDI_CTL_REG_DATA1, 1, 100);
		HAL_SPI_Transmit(hspi, &SDI_CTL_REG_DATA2, 1, 100); // User can send this data to select the one of 4 SPI modes of ADC as described in data sheet of ADS8699

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

		       	   {

		       	   }

    HAL_Delay(100);

// Configuring SDO_CTL_REG register

    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

    	HAL_SPI_Transmit(hspi, &SDO_CTL_REG_WRITE, 1, 100);
    	HAL_SPI_Transmit(hspi, &SDO_CTL_REG_ADDRESS, 1, 100);
    	HAL_SPI_Transmit(hspi, &SDO_CTL_REG_DATA1, 1, 100);
    	HAL_SPI_Transmit(hspi, &SDO_CTL_REG_DATA2, 1, 100);

    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

    	       	   {

    	       	   }

    HAL_Delay(100);

// Configuring RANGE_SEL_REG register to select multiple input ranges


 	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

 		HAL_SPI_Transmit(hspi, &RANGE_SEL_REG_WRITE, 1, 100);
 		HAL_SPI_Transmit(hspi, &RANGE_SEL_REG_ADDRESS, 1, 100);
 		HAL_SPI_Transmit(hspi, &RANGE_SEL_REG_DATA1, 1, 100);
 		HAL_SPI_Transmit(hspi, &RANGE_SEL_REG_DATA2, 1, 100); // User can send this data to select the bipolar and unipolar input ranges of ADC as described in data sheet of ADS8699


    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

   	   	   	   	   {

   	   	   	   	   }

    HAL_Delay(100);

// Configuring DATAOUT_CTL_REG register to control the conversion data

    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

   	   HAL_SPI_Transmit(hspi, &DATAOUT_CTL_REG_WRITE, 1, 100);
   	   HAL_SPI_Transmit(hspi, &DATAOUT_CTL_REG_ADDRESS, 1, 100);
   	   HAL_SPI_Transmit(hspi, &DATAOUT_CTL_REG_DATA1, 1, 100);
   	   HAL_SPI_Transmit(hspi, &DATAOUT_CTL_REG_DATA2, 1, 100); // User can send this data to select test mode and conversion mode of ADC as described in data sheet of ADS8699

   	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

   	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

   	       	     {

   	       	     }

   	HAL_Delay(100);

// Configuring SDO_CTL_REG register to read data from the ADC

   	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

   	   HAL_SPI_Transmit(hspi, &SDO_CTL_REG_READ, 1, 100);
   	   HAL_SPI_Transmit(hspi, &SDO_CTL_REG_ADDRESS, 1, 100);
   	   HAL_SPI_Transmit(hspi, &SDO_CTL_REG_DATA1, 1, 100);
   	   HAL_SPI_Transmit(hspi, &SDO_CTL_REG_DATA2, 1, 100);

   	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

   	HAL_Delay(100);

   	return errorcode;
}

// Function to read 32bits of data from ADC ADS8699

HAL_StatusTypeDef CAL_ADS8699_Read_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t *data)
{

HAL_StatusTypeDef errorcode=HAL_OK;
//float measured_data;

 //int i=CAL_ADC_Averaging_Buffer(x);

	//	uint8_t rx[i];

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);


			 HAL_SPI_Receive(hspi,data,sizeof(data),100);


	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);


	//measured_data = CAL_ADC_Real_Measured_Data(RANGE_SEL_REG_DATA2, rx,i);


return errorcode;
}

// Function to take averaging buffer size for ADC readings

HAL_StatusTypeDef CAL_ADC_Averaging_Buffer(uint8_t data)

{
	int i;

		switch(data)
		{


			case 0x00:

				 i=4;
				 break;

			case 0x01:

				 i=12;
				 break;

			case 0x02:

				 i=56;
				 break;

			case 0x03:

				i=100;
				break;

			case 0x04:

				i=200;
				break;

			case 0x05:

				i=500;
				break;

			case 0x06:

				i=1000;
				break;

			case 0x07:

				i=2000;
				break;

			default:

				i=4;
				break;

		}

		return i;
}

// ADC will provide the measured data in raw format like number of steps for each level
// Function to convert the ADC raw data to real time data

HAL_StatusTypeDef CAL_ADC_Real_Measured_Data(uint8_t RANGE_SEL_REG_DATA2, uint8_t *data,int data1)
{
    int i;
    int j=0;
    int k = data1/4;
	uint32_t raw[k];
	uint32_t raw1[k];

	uint32_t raw2[k];
	float result[k];
	float result1=0;
	float result2;
	float measured_data;
	float input_range;
	float LSB;

	switch(RANGE_SEL_REG_DATA2)
	{

	case 0x00:

		input_range=12.288;
		LSB=0.00009375;

         for(i=0; i<(sizeof(raw))/4; i++)
         {

        	 if(j!=sizeof(data))
        	 {

     		raw[i] = data[j+0]<<24|data[j+1]<<16|data[j+2]<<8|data[j+3]; // Storing all 4 8bits of data into 32 bit

     		raw1[i]=raw[i]>>14; // Taking only 18 bits of data from 32 bits. Because ADS8699 is a 18 bit ADC

     		raw2[i]=(262143-raw1[i]); // Subtracting measured ADC value from Final ADC value

     		result[i]=raw2[i]*LSB; // Multiplying resulting number of steps with step size to measure the voltage

     		result1=(result1+result[i]); // Averaging the resulted value with selected buffer size

     		j=j+4;

             }


         }

         result2=result1/(sizeof(raw))/4;
         measured_data=input_range-result2;
         break;

	case 0x01:

		input_range=10.24;
		LSB=0.000078125;

         for(i=0; i<(sizeof(raw))/4; i++)
         {

        	 if(j!=sizeof(data))
        	 {

     		raw[i] = data[j+0]<<24|data[j+1]<<16|data[j+2]<<8|data[j+3]; // Storing all 4 8bits of data into 32 bit

     		raw1[i]=raw[i]>>14; // Taking only 18 bits of data from 32 bits. Because ADS8699 is a 18 bit ADC

     		raw2[i]=(262143-raw1[i]); // Subtracting measured ADC value from Final ADC value

     		result[i]=raw2[i]*LSB; // Multiplying resulting number of steps with step size to measure the voltage

     		result1=(result1+result[i]); // Averaging the resulted value with selected buffer size

     		j=j+4;

             }


         }

         result2=result1/(sizeof(raw))/4;
         measured_data=input_range-result2;
         break;

	case 0x02:

		input_range=6.144;
		LSB=0.000046875;

         for(i=0; i<(sizeof(raw))/4; i++)
         {

        	 if(j!=sizeof(data))
        	 {

     		raw[i] = data[j+0]<<24|data[j+1]<<16|data[j+2]<<8|data[j+3]; // Storing all 4 8bits of data into 32 bit

     		raw1[i]=raw[i]>>14; // Taking only 18 bits of data from 32 bits. Because ADS8699 is a 18 bit ADC

     		raw2[i]=(262143-raw1[i]); // Subtracting measured ADC value from Final ADC value

     		result[i]=raw2[i]*LSB; // Multiplying resulting number of steps with step size to measure the voltage

     		result1=(result1+result[i]); // Averaging the resulted value with selected buffer size

     		j=j+4;

             }


         }

         result2=result1/(sizeof(raw))/4;
         measured_data=input_range-result2;
         break;
	case 0x03:

		input_range=5.12;
		LSB=0.00003906;

         for(i=0; i<(sizeof(raw))/4; i++)
         {

        	 if(j!=sizeof(data))
        	 {

     		raw[i] = data[j+0]<<24|data[j+1]<<16|data[j+2]<<8|data[j+3]; // Storing all 4 8bits of data into 32 bit

     		raw1[i]=raw[i]>>14; // Taking only 18 bits of data from 32 bits. Because ADS8699 is a 18 bit ADC

     		raw2[i]=(262143-raw1[i]); // Subtracting measured ADC value from Final ADC value

     		result[i]=raw2[i]*LSB; // Multiplying resulting number of steps with step size to measure the voltage

     		result1=(result1+result[i]); // Averaging the resulted value with selected buffer size

     		j=j+4;

             }


         }

         result2=result1/(sizeof(raw))/4;
         measured_data=input_range-result2;
         break;

	case 0x04:

		input_range=2.56;
		LSB=0.00001953;

         for(i=0; i<(sizeof(raw))/4; i++)
         {

        	 if(j!=data1)
        	 {

     		raw[i] = data[j+0]<<24|data[j+1]<<16|data[j+2]<<8|data[j+3]; // Storing all 4 8bits of data into 32 bit

     		raw1[i]=raw[i]>>14; // Taking only 18 bits of data from 32 bits. Because ADS8699 is a 18 bit ADC

     		raw2[i]=(262143-raw1[i]); // Subtracting measured ADC value from Final ADC value

     		result[i]=raw2[i]*LSB; // Multiplying resulting number of steps with step size to measure the voltage

     		result1=(result1+result[i]); // Averaging the resulted value with selected buffer size

     		j=j+4;

             }


         }

         result2=result1/(sizeof(raw))/4;
         measured_data=input_range-result2;
         break;
}
	return measured_data;
}

//Function to configure ADC chips on thermocouple module
//Thermocouple module configuration different when compared to voltage and current module ADC configuration
//First 8 bit serial data should be send to shift register to select the SPI chip select and ADC enable pin for each channel
//After selecting channel ADC should configure exactly same as shown in  CAL_ADS8699_Configuration

HAL_StatusTypeDef CAL_ADC_ThermoCoupleModule_Configuration(SPI_HandleTypeDef *spi,SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t *tx,uint8_t *tx1, uint8_t SDI_CTL_REG_DATA2,uint8_t DATAOUT_CTL_REG_DATA2,uint8_t RANGE_SEL_REG_DATA2)

{
	HAL_StatusTypeDef errorcode = HAL_OK;



	CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx);

	// Configuring SDI_CTL_REG register to select the SPI mode of ADC

			HAL_SPI_Transmit(hspi, &SDI_CTL_REG_WRITE, 1, 100);
			HAL_SPI_Transmit(hspi, &SDI_CTL_REG_ADDRESS, 1, 100);
			HAL_SPI_Transmit(hspi, &SDI_CTL_REG_DATA1, 1, 100);
			HAL_SPI_Transmit(hspi, &SDI_CTL_REG_DATA2, 1, 100); // User can send this data to select the one of 4 SPI modes of ADC as described in data sheet of ADS8699

	CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx1);

		while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

			       	   {

			       	   }

	    HAL_Delay(100);

	// Configuring SDO_CTL_REG register

	  CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx);

	    	HAL_SPI_Transmit(hspi, &SDO_CTL_REG_WRITE, 1, 100);
	    	HAL_SPI_Transmit(hspi, &SDO_CTL_REG_ADDRESS, 1, 100);
	    	HAL_SPI_Transmit(hspi, &SDO_CTL_REG_DATA1, 1, 100);
	    	HAL_SPI_Transmit(hspi, &SDO_CTL_REG_DATA2, 1, 100);

	 CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx1);

	    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

	    	       	   {

	    	       	   }

	    HAL_Delay(100);

	// Configuring RANGE_SEL_REG register to select multiple input ranges


	    CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx);

	 		HAL_SPI_Transmit(hspi, &RANGE_SEL_REG_WRITE, 1, 100);
	 		HAL_SPI_Transmit(hspi, &RANGE_SEL_REG_ADDRESS, 1, 100);
	 		HAL_SPI_Transmit(hspi, &RANGE_SEL_REG_DATA1, 1, 100);
	 		HAL_SPI_Transmit(hspi, &RANGE_SEL_REG_DATA2, 1, 100); // User can send this data to select the bipolar and unipolar input ranges of ADC as described in data sheet of ADS8699


	 	CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx1);

	    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

	   	   	   	   	   {

	   	   	   	   	   }

	    HAL_Delay(100);

	// Configuring DATAOUT_CTL_REG register to control the conversion data

	    CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx);

	   	   HAL_SPI_Transmit(hspi, &DATAOUT_CTL_REG_WRITE, 1, 100);
	   	   HAL_SPI_Transmit(hspi, &DATAOUT_CTL_REG_ADDRESS, 1, 100);
	   	   HAL_SPI_Transmit(hspi, &DATAOUT_CTL_REG_DATA1, 1, 100);
	   	   HAL_SPI_Transmit(hspi, &DATAOUT_CTL_REG_DATA2, 1, 100); // User can send this data to select test mode and conversion mode of ADC as described in data sheet of ADS8699

	   	CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx1);

	   	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

	   	       	     {

	   	       	     }

	   	HAL_Delay(100);

	// Configuring SDO_CTL_REG register to read data from the ADC

	   	CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx);

	   	   HAL_SPI_Transmit(hspi, &SDO_CTL_REG_READ, 1, 100);
	   	   HAL_SPI_Transmit(hspi, &SDO_CTL_REG_ADDRESS, 1, 100);
	   	   HAL_SPI_Transmit(hspi, &SDO_CTL_REG_DATA1, 1, 100);
	   	   HAL_SPI_Transmit(hspi, &SDO_CTL_REG_DATA2, 1, 100);

	   	CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx1);

	   	HAL_Delay(100);

	   	return errorcode;
}

HAL_StatusTypeDef CAL_ADC_ThermoCoupleModule_Read_Data(SPI_HandleTypeDef *spi,SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t *tx,uint8_t *tx1,uint8_t *data)

{
	// int j=0;
		//HAL_StatusTypeDef errorcode = HAL_OK;

		CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx);



			 HAL_SPI_Receive(hspi,data,sizeof(data),100);



		CAL_WriteShiftRegister(spi,GPIOx, GPIO_Pin,tx1);

		//return errorcode;
}

// Function to switch between channels of thermocouple
//This function is send data to shift register to make the chip select of ADC SPI low and ADC enable high

HAL_StatusTypeDef Thermo_Couple_Channel_Select_Data(int data)
{
	uint8_t tx;
	switch(data)
		{

		case 0:

		tx=channe11;

		break;

			case 1:

				 tx=channe12;


				 break;

			case 2:

				 tx=channe13;


				 break;

			case 3:

				tx=channe14;


				break;


			default:

			tx=channe11;

				break;

		}

return tx;
}

// Function to switch between channels of thermocouple
//This function is send data to shift register to make the chip select of ADC SPI high and ADC enable high

HAL_StatusTypeDef Thermo_Couple_Channel_Select_Data1(int data)
{
	uint8_t tx;
	switch(data)
		{

		case 0:
			tx=channel1_1;
		break;

			case 1:

				 tx=channel2_1;


				 break;

			case 2:

				 tx=channel3_1;


				 break;

			case 3:

				tx=channel4_1;


				break;


			default:

			tx=channel1_1;

				break;

		}

return tx;
}
