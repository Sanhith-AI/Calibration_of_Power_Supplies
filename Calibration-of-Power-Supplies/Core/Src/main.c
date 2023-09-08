/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "ADS8699.h"
#include "Shift_Register.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t shiftregister_thermocouple=0xE1; // Default data to configure the ADC chip on thermocouple module when power up the calibration station
uint8_t shiftregister_thermocouple1=0xF1; // Configure Channel1 of the ADC in thermocouple module in default
uint8_t tx3;
uint8_t tx4;
int i=1;
int j=0;

//Function to select the current range and gain of the programmable gain instrumentation amplifier

HAL_StatusTypeDef CAL_writeShiftRegister1(SPI_HandleTypeDef* spi1, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t* tx)

{

  // Make sure we start with strobe pin
  HAL_GPIO_WritePin(GPIOx,GPIO_Pin, GPIO_PIN_RESET); // reset state

  // Write and read
  HAL_SPI_Transmit(spi1, tx,2, 100);

  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
  HAL_GPIO_WritePin(GPIOx,GPIO_Pin, GPIO_PIN_SET);
  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
  HAL_GPIO_WritePin(GPIOx,GPIO_Pin, GPIO_PIN_RESET);
}

struct send_data {

	  uint8_t sync ;
	  uint8_t current_range;
	  uint8_t voltage_range;
	  float current_value;              //Structure to transmit and receive data to and from the LCD display
	  float voltage_value;
	  float temperature_chanel1;
	  float temperature_chanel2;
	  float temperature_chanel3;
	  float temperature_chanel4;
	  uint8_t ack;

}send_data;


struct send_data test_tx_data;

char transmit_buffer1[sizeof(send_data)+1];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	HAL_UART_Receive_IT(&huart4,transmit_buffer1,sizeof(transmit_buffer1));

	memcpy(&test_tx_data,transmit_buffer1,sizeof(send_data));
}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOF, uC_OK_Pin, GPIO_PIN_SET); // OK LED turn on -> Init complete

    uint8_t voltage_buff[4];
 	uint8_t current_buff[4];  //Receive buffers to receive 32 bit data from ADC ADS8699 SDO_CTL_REG register using SPI
 	uint8_t temperature_buff[4];

    uint32_t ADC_data[3];
    uint32_t ADC_raw_voltage;
    uint32_t ADC_raw_current;
    uint32_t ADC_raw_temperature;
    float intermediate_result[7];
    float measured_voltage;
    float measured_current;
    float measured_temperature;
    uint8_t shiftregister_current[2]={0x03,0x01}; //Data to select gain and current range

	test_tx_data.sync=0x00;
	test_tx_data.current_range=0x01;
	test_tx_data.voltage_range=0x00;
	test_tx_data.current_value=0x00;
	test_tx_data.voltage_value=0x00;
	test_tx_data.temperature_chanel1=0x00;   // Storing all default values in structure to send to LCD display
	test_tx_data.temperature_chanel2=0x00;
	test_tx_data.temperature_chanel3=0x00;
	test_tx_data.temperature_chanel4=0x00;
	test_tx_data.ack=0x01;

  memcpy(transmit_buffer1,&test_tx_data,sizeof(send_data)); // storing all data from structure to transmit buffer

  while(test_tx_data.ack !=0x1F)

  {

  HAL_UART_Transmit(&huart4,transmit_buffer1,sizeof(transmit_buffer1),100);
  HAL_UART_Receive_IT(&huart4,transmit_buffer1,sizeof(transmit_buffer1));

  }

  HAL_GPIO_WritePin(MOD1_DEN_GPIO_Port, MOD1_DEN_Pin, GPIO_PIN_SET);   // To enable ADC
  HAL_GPIO_WritePin(ADC_nRST_GPIO_Port, ADC_nRST_Pin, GPIO_PIN_SET);   //To reset ADC
  CAL_ADS8699_Configuration(&hspi1,GPIOD, MOD1_nCS_Pin,0x00,0x00,0x00); // function call to configure ADC in slot1

  CAL_writeShiftRegister1(&hspi4,GPIOH,MOD2_STRB_Pin,shiftregister_current);
  HAL_GPIO_WritePin(ADC_nRST_GPIO_Port, ADC_nRST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOD2_DEN_GPIO_Port, MOD2_DEN_Pin, GPIO_PIN_SET);
  CAL_ADS8699_Configuration(&hspi2,MOD2_nCS_GPIO_Port, MOD2_nCS_Pin,0x00,0x00,0x00);  //function call to configure ADC in slot2

  HAL_GPIO_WritePin(ADC_nRST_GPIO_Port, ADC_nRST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOD5_DEN_GPIO_Port, MOD5_DEN_Pin, GPIO_PIN_SET);                //function call to configure ADC inslot 5
  HAL_GPIO_WritePin(MOD5_nCS_GPIO_Port, MOD5_nCS_Pin, GPIO_PIN_RESET);
  CAL_ADC_ThermoCoupleModule_Configuration(&hspi3,&hspi5,MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &shiftregister_thermocouple,&shiftregister_thermocouple1,0x00,0x00,0x00);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

  CAL_ADS8699_Read_Data(&hspi1,GPIOD, MOD1_nCS_Pin, voltage_buff); // To read data from first slot

  CAL_ADS8699_Read_Data(&hspi2,MOD2_nCS_GPIO_Port, MOD2_nCS_Pin, current_buff); //To read data from second slot

  CAL_ADC_ThermoCoupleModule_Read_Data(&hspi3,&hspi5,MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &shiftregister_thermocouple,&shiftregister_thermocouple1, temperature_buff); //To read data from third slot

  //Converting measured ADC data into real time values using proper formulas

      ADC_data[0] = voltage_buff[0]<<24|voltage_buff[1]<<16|voltage_buff[2]<<8|voltage_buff[3];
	       ADC_raw_voltage = ADC_data[0]>>14;                         // Considering olny 18 bits data from 32 bit data. Because ADS8699 is a 18 bit ADC
	intermediate_result[0] = (262143-ADC_raw_voltage)*0.00009375;     // Converting ADC raw value into real time voltage using formula 𝑀𝑒𝑎𝑠𝑢𝑟𝑒𝑑_𝑉𝑜𝑙𝑡𝑎𝑔𝑒=𝐼𝑛𝑝𝑢𝑡 𝑟𝑎𝑛𝑔𝑒 𝑜𝑓 𝐴𝐷𝐶−((2^(no.of bits)−𝐴𝐷𝐶 𝑜𝑢𝑡𝑝𝑢𝑡)×𝑅𝑒𝑠𝑜𝑙𝑢𝑡𝑖𝑜𝑛 𝑜𝑓 𝐴𝐷𝐶)
		  measured_voltage = 12.288-intermediate_result[0];

  		       ADC_data[1] = current_buff[0]<<24|current_buff[1]<<16|current_buff[2]<<8|current_buff[3];
  		   ADC_raw_current = ADC_data[1]>>14;
  	intermediate_result[1] = (262143-ADC_raw_current)*0.00009375;
  	intermediate_result[2] = 12.288-intermediate_result[1];
  	intermediate_result[3] = (intermediate_result[2]-2.5)*10.7;    //Converting ADC raw value to current using formula 𝑀𝑒𝑎𝑠𝑢𝑟𝑒𝑑_𝐶𝑢𝑟𝑟𝑒𝑛𝑡=(𝑀𝑒𝑎𝑠𝑢𝑟𝑒𝑑_𝑉𝑜𝑙𝑡𝑎𝑔𝑒−2.5)/(gain of the PGIA×𝑆ℎ𝑢𝑛𝑡 𝑟𝑒𝑠𝑖𝑠𝑡𝑎𝑛𝑐𝑒)
  	intermediate_result[4] = intermediate_result[3]/1000;
  	      measured_current = intermediate_result[4]/10.11;

               ADC_data[2] = temperature_buff[0]<<24|temperature_buff[1]<<16|temperature_buff[2]<<8|temperature_buff[3];
       ADC_raw_temperature = ADC_data[2]>>14;
    intermediate_result[5] = (262143-ADC_raw_temperature)*0.00009375;
    intermediate_result[6] = 12.288-intermediate_result[5];
	  measured_temperature = (intermediate_result[6]/0.005)-3;      //Converting ADC raw value to temperature using formula 𝑀𝑒𝑎𝑠𝑢𝑟𝑒𝑑_𝑇𝑒𝑚𝑝𝑒𝑟𝑎𝑡𝑢𝑟𝑒=(𝑀𝑒𝑎𝑠𝑢𝑟𝑒𝑑_𝑉𝑜𝑙𝑡𝑎𝑔𝑒)/(5𝑚𝑉/°𝐶) + 𝑅𝑒𝑓𝑒𝑟𝑒𝑛𝑐𝑒 𝑉𝑜𝑙𝑡𝑎𝑔𝑒 ± 3°C offset

  HAL_UART_Receive_IT(&huart4,transmit_buffer1,sizeof(transmit_buffer1)); //Receiving data from LCD display and storing in structure
  memcpy(&test_tx_data,transmit_buffer1,sizeof(send_data));

  switch(j)

   {
  			case 0:

  		test_tx_data.temperature_chanel1 = measured_temperature;   //storing the measured temperature in structure depending on the configured channel

  			break;

  		    case 1:

  		test_tx_data.temperature_chanel2 = measured_temperature;

  			break;

  			case 2:

  		test_tx_data.temperature_chanel3 = measured_temperature;

  			break;

  			case 3:

  		test_tx_data.temperature_chanel4 = measured_temperature;

  			break;

  			default:

  		test_tx_data.temperature_chanel1 = measured_temperature;

  			break;
  		}

  if(test_tx_data.sync==0xFF)
  	{

  	    test_tx_data.current_value=measured_current;
  		test_tx_data.voltage_value=measured_voltage; //Transmitting measured data to LCD display
  	    memcpy(transmit_buffer1,&test_tx_data,sizeof(send_data));
        HAL_UART_Transmit(&huart4,(uint8_t *)transmit_buffer1,sizeof(transmit_buffer1),100);
  	}

  if(i!=4)
   {
	  shiftregister_thermocouple=Thermo_Couple_Channel_Select_Data(i);
	  shiftregister_thermocouple1=Thermo_Couple_Channel_Select_Data1(i); //Selecting the channel of ADC in thermocouple module

  	  CAL_ADC_ThermoCoupleModule_Configuration(&hspi3,&hspi5,GPIOH, MOD5_STRB_Pin,&shiftregister_thermocouple,&shiftregister_thermocouple1,0x00,0x00,0x00);	//Reconfiguring the ADC with new data

  	  i=i+1;
  	  j=j+1;

  		}

  	else
  	{

  		CAL_ADC_ThermoCoupleModule_Configuration(&hspi3,&hspi5,GPIOH, MOD5_STRB_Pin,(uint8_t *)0xE1, (uint8_t *)0xF1,0x00,0x00,0x00); //Configuring default channel1 of thermocouple module
  		i=0;
  	    j=0;

  	}

  HAL_Delay(1);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4|RCC_PERIPHCLK_SPI5
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.PLL2.PLL2M = 32;
  PeriphClkInitStruct.PLL2.PLL2N = 128;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, uC_OK_Pin|uC_BUSY_Pin|uC_ERR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, MOD1_STRB_Pin|MOD2_STRB_Pin|MOD3_STRB_Pin|MOD4_STRB_Pin
                          |MOD5_STRB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MOD1_DEN_Pin|MOD2_DEN_Pin|MOD3_DEN_Pin|MOD4_DEN_Pin
                          |MOD5_DEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_nRST_GPIO_Port, ADC_nRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MOD1_nCS_Pin|MOD2_nCS_Pin|MOD3_nCS_Pin|MOD4_nCS_Pin
                          |MOD5_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : uC_OK_Pin uC_BUSY_Pin uC_ERR_Pin */
  GPIO_InitStruct.Pin = uC_OK_Pin|uC_BUSY_Pin|uC_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : MOD1_STRB_Pin MOD2_STRB_Pin MOD3_STRB_Pin MOD4_STRB_Pin
                           MOD5_STRB_Pin */
  GPIO_InitStruct.Pin = MOD1_STRB_Pin|MOD2_STRB_Pin|MOD3_STRB_Pin|MOD4_STRB_Pin
                          |MOD5_STRB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : MOD1_DEN_Pin MOD2_DEN_Pin MOD3_DEN_Pin MOD4_DEN_Pin
                           MOD5_DEN_Pin */
  GPIO_InitStruct.Pin = MOD1_DEN_Pin|MOD2_DEN_Pin|MOD3_DEN_Pin|MOD4_DEN_Pin
                          |MOD5_DEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_nRST_Pin */
  GPIO_InitStruct.Pin = ADC_nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_nRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOD1_nCS_Pin MOD2_nCS_Pin MOD3_nCS_Pin MOD4_nCS_Pin
                           MOD5_nCS_Pin */
  GPIO_InitStruct.Pin = MOD1_nCS_Pin|MOD2_nCS_Pin|MOD3_nCS_Pin|MOD4_nCS_Pin
                          |MOD5_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
