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
#include "shift.h"

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
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */

uint8_t tx3;
uint8_t tx4;
	uint8_t tx=0xE1;
uint8_t tx1=0xF1;


int i=1;
int j=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	HAL_StatusTypeDef CAL_writeShiftRegister1(SPI_HandleTypeDef* spi1, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t* tx) {


	  // Make sure we start with strobe pin
	  HAL_GPIO_WritePin(GPIOx,GPIO_Pin, GPIO_PIN_RESET); // reset state

	  // Write and read
	  HAL_SPI_Transmit(spi1, tx,2, 100);

	  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
	 HAL_GPIO_WritePin(GPIOx,GPIO_Pin, GPIO_PIN_SET);
	  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
	  HAL_GPIO_WritePin(GPIOx,GPIO_Pin, GPIO_PIN_RESET);
	}

/*struct receive_data {
	
    uint8_t sync ;	
	  uint8_t current_range;
	  uint8_t voltage_range;
	  uint8_t ack;
	
}receive_data;

struct receive_data tx_data;
struct receive_data rx_data;*/

struct send_data {
	
		 uint8_t sync ;	
	  uint8_t current_range;
	  uint8_t voltage_range;
	  
	float current_value;
	float voltage_value;
	float temperature_chanel1;
	float temperature_chanel2;
	float temperature_chanel3;
	float temperature_chanel4;
		uint8_t ack;
}send_data;
	

struct send_data test_tx_data;
struct send_data test_rx_data;

//float recieve_buffer[2];
char transmit_buffer1[sizeof(send_data)+1];
char receive_range_buffer[4];
char transmit_range_buffer[4];


/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
memcpy(transmit_range_buffer,&tx_data,sizeof(receive_data));

HAL_UART_Transmit_IT(&huart4,transmit_range_buffer,sizeof(transmit_range_buffer));
}*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	HAL_UART_Receive_IT(&huart4,transmit_buffer1,sizeof(transmit_buffer1));
	
	//HAL_UART_Receive_IT(&huart4,(uint8_t *)&ack,1);

	// HAL_UART_Receive_IT(&huart2,(uint8_t*)test_recieve_data,sizeof(test_send_data));
	memcpy(&test_tx_data,transmit_buffer1,sizeof(send_data));
}

//char transmit_buffer[40];

HAL_StatusTypeDef CAL_writeShiftRegister(SPI_HandleTypeDef* spi, GPIO_TypeDef* STRB_Port, uint16_t STRB_Pin, uint8_t* tx) {

HAL_StatusTypeDef errorcode=HAL_OK;
	  // Make sure we start with strobe pin
	  HAL_GPIO_WritePin(MOD1_STRB_GPIO_Port, MOD1_STRB_Pin, GPIO_PIN_RESET); // reset state

	  // Write and read
	  HAL_SPI_Transmit(spi, tx,1, 100);

	  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
	  HAL_GPIO_WritePin(STRB_Port, STRB_Pin, GPIO_PIN_SET);
	  for(int i = 0; i < 500; i++) { __NOP(); } // Wait N cycles
	  HAL_GPIO_WritePin(STRB_Port, STRB_Pin, GPIO_PIN_RESET);

	  return errorcode;
}

/*void Test_ADC_modules(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t data)
	{



		uint8_t SDI_wr=0xD0, SDI_add=0x08, SDI_da1=0x00, SDI_da2=0x00; //32 bits includes input write command, address of the register followed by data

  //Configuring data protocol used to transmit data out from the SDO-x pins of the ADC SDO_CTL_REG Register

	uint8_t SDO_wr=0xD0, SDO_add=0x0C, SDO_da1=0x00, SDO_da2=0x00;

  //Configuring the register to controls the data output by the device using DATAOUT_CTL_REG Register

	uint8_t DATA_wr=0xD0, DATA_add=0x10, DATA_da1=0x00, DATA_da2=data;

 uint8_t RANGE_sel=0xD0,RANGE_add=0x14, RANGE_da1=0x00,RANGE_da2=0x00;

  //Data read command to read data from the ADC using host controller

	uint8_t SDO1_rd=0x48, SDO1_add=0x0C, SDO1_da1=0x00, SDO1_da2=0x00;

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx);
	HAL_SPI_Transmit(hspi, &SDI_wr, 1, 100);
	HAL_SPI_Transmit(hspi, &SDI_add, 1, 100);
	HAL_SPI_Transmit(hspi, &SDI_da1, 1, 100);
	HAL_SPI_Transmit(hspi, &SDI_da2, 1, 100);

	CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx1);
	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

	       	   {

	       	   }
	HAL_Delay(100);

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx);

	HAL_SPI_Transmit(hspi, &SDO_wr, 1, 100);
	HAL_SPI_Transmit(hspi, &SDO_add, 1, 100);
	HAL_SPI_Transmit(hspi, &SDO_da1, 1, 100);
	HAL_SPI_Transmit(hspi, &SDO_da2, 1, 100);

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx1);

						 while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

	       	   {

	       	   }

	HAL_Delay(100);

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx);

	HAL_SPI_Transmit(hspi, &RANGE_sel, 1, 100);
	HAL_SPI_Transmit(hspi, &RANGE_add, 1, 100);
	HAL_SPI_Transmit(hspi, &RANGE_da1, 1, 100);
	HAL_SPI_Transmit(hspi, &RANGE_da2, 1, 100);

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx1);

	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

	       	   {

	       	   }
HAL_Delay(100);

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx);

	HAL_SPI_Transmit(hspi, &DATA_wr, 1, 100);
	HAL_SPI_Transmit(hspi, &DATA_add, 1, 100);
	HAL_SPI_Transmit(hspi, &DATA_da1, 1, 100);
	HAL_SPI_Transmit(hspi, &DATA_da2, 1, 100);

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx1);

					while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)

	       	   {

	       	   }

	HAL_Delay(100);

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx);

	HAL_SPI_Transmit(hspi, &SDO1_rd, 1, 100);
	HAL_SPI_Transmit(hspi, &SDO1_add, 1, 100);
	HAL_SPI_Transmit(hspi, &SDO1_da1, 1, 100);
	HAL_SPI_Transmit(hspi, &SDO1_da2, 1, 100);

CAL_writeShiftRegister(&hspi3, MOD5_STRB_GPIO_Port, MOD5_STRB_Pin, &tx1);

	HAL_Delay(100);
	}*/


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
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET); // OK LED turn on -> Init complete
 
	 uint8_t rx[4];
	 uint8_t rx1[4];
	 uint8_t rx2[4];
//char buff[38];
//int buff_len;
uint32_t raw;
uint32_t raw1;
uint32_t raw5;
uint32_t raw6;
uint32_t raw7;
uint32_t raw8;
float raw2;
float raw3;
float raw4;
float raw9;
float raw10;
float raw11;
float raw12;
float raw13;
float result;
float result1;
uint8_t tx2[2]={0x03,0x01};


/*test_tx_data.current_range=0x01;
test_tx_data.voltage_range=0x01;
test_tx_data.sync=0xFF;
test_tx_data.ack=0x00;*/

//HAL_UART_Transmit_IT(&huart4,(uint8_t *)transmit_buffer1,sizeof(transmit_buffer1));

//memcpy(transmit_buffer1,&test_tx_data,sizeof(send_data));
	  test_tx_data.sync=0x00;
	test_tx_data.current_range=0x01;
	test_tx_data.voltage_range=0x00;
	test_tx_data.current_value=0x00;
	test_tx_data.voltage_value=0x00;
	test_tx_data.temperature_chanel1=0x00;
	test_tx_data.temperature_chanel2=0x00;
	test_tx_data.temperature_chanel3=0x00;
	test_tx_data.temperature_chanel4=0x00;
	test_tx_data.ack=0x01;

memcpy(transmit_buffer1,&test_tx_data,sizeof(send_data));


//HAL_UART_Receive_IT(&huart4,(uint8_t *)&ack,1);
		 while(test_tx_data.ack !=0x1F)
	
{


HAL_UART_Transmit(&huart4,transmit_buffer1,sizeof(transmit_buffer1),100);
	HAL_UART_Receive_IT(&huart4,transmit_buffer1,sizeof(transmit_buffer1));

//HAL_UART_Receive(&huart4,(uint8_t *)&ack,sizeof(ack),100);
}

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);	 
//Test1_ADC_modules(&hspi1,GPIOD,GPIO_PIN_0,0x00);
CAL_ADS8699_Configuration(&hspi1,GPIOD,GPIO_PIN_0,0x00,0x00,0x00);

CAL_writeShiftRegister1(&hspi4,GPIOH,MOD2_STRB_Pin,tx2);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
//Test2_ADC_modules(&hspi2,GPIOD,GPIO_PIN_1,0x06);
CAL_ADS8699_Configuration(&hspi2,GPIOD,GPIO_PIN_1,0x00,0x00,0x00);

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	

	CAL_ADC_ThermoCoupleModule_Configuration(&hspi3,&hspi5,GPIOH, MOD5_STRB_Pin,&tx,&tx1,0x00,0x00,0x00);	
		
		
//CAL_writeShiftRegister(&hspi3,MOD1_STRB_GPIO_Port, MOD1_STRB_Pin,&tx);

     
	
 //CAL_ADC_ThermoCoupleModule_Configuration(&hspi5,&hspi3,MOD3_STRB_GPIO_Port, MOD3_STRB_Pin,&tx,&tx1,0x00,0x07,0x00);

		
		
	/*float p=0;
	float pp=0;
	float k=0;
	float l=0;
	float m=0;
	float n=0;*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	

		

//HAL_UART_Transmit(&huart4,transmit_range_buffer,sizeof(transmit_range_buffer),100);

CAL_ADS8699_Read_Data(&hspi1,GPIOD,GPIO_PIN_0,rx);
CAL_ADS8699_Read_Data(&hspi2,GPIOD,GPIO_PIN_1,rx1);
 if(tx==0xE1 & tx1==0xF1)
 {
CAL_ADC_ThermoCoupleModule_Read_Data(&hspi3,&hspi5, GPIOH, MOD5_STRB_Pin,&tx,&tx1,rx2);
 }
 else
 {
	 

   CAL_ADC_ThermoCoupleModule_Read_Data(&hspi3,&hspi5, GPIOH, MOD5_STRB_Pin,&tx3,&tx4,rx2);	 
 }
//CAL_writeShiftRegister(&hspi3, GPIOH, MOD5_STRB_Pin, &tx);	 

	//HAL_SPI_Receive(&hspi5,rx2,4,100);
  
//CAL_writeShiftRegister(&hspi3, GPIOH, MOD5_STRB_Pin, &tx1);

		
	raw = rx2[0]<<24|rx2[1]<<16|rx2[2]<<8|rx2[3];
		raw1=raw>>14;
		raw2 = (262143-raw1)*0.00009375;
		raw3 = 12.288-raw2;
		raw4=(raw3/0.005)-3;
		raw5 = rx[0]<<24|rx[1]<<16|rx[2]<<8|rx[3];
		raw6=raw5>>14;
		raw9 = (262143-raw6)*0.00009375;
		raw10 = 12.288-raw9;
		raw7 = rx1[0]<<24|rx1[1]<<16|rx1[2]<<8|rx1[3];
		raw8=raw7>>14;
		raw11 = (262143-raw8)*0.00009375;
		raw12 = 12.288-raw11;
		result = (raw12-2.5)*10.7;
		result1 = result/1000;
		raw13=result1/10.11;
		
		//buff_len=sprintf(buff,"%f,%f,%f,%f\n",raw10,raw13,raw4,dummy1);
		
		//HAL_UART_Transmit(&huart4,(uint8_t *)buff,buff_len,100);
			HAL_UART_Receive_IT(&huart4,transmit_buffer1,sizeof(transmit_buffer1));
	memcpy(&test_tx_data,transmit_buffer1,sizeof(send_data));
	
	switch(j)
		{
			case 0:
		test_tx_data.temperature_chanel1=raw4;

			break;
			
		  case 1:

			
		test_tx_data.temperature_chanel2=0;
			
			
	
			break;
			
			case 2:
		
		test_tx_data.temperature_chanel3=0;
		
			break;
			
			case 3:
	
		test_tx_data.temperature_chanel4=0;
			
			break;
			
			default:
		test_tx_data.temperature_chanel1=raw4;

			break;
		} 
		
		
	if(test_tx_data.sync==0xFF)
	{
			
				test_tx_data.current_value=raw13;
		test_tx_data.voltage_value=raw10;
	memcpy(transmit_buffer1,&test_tx_data,sizeof(send_data));	
HAL_UART_Transmit(&huart4,(uint8_t *)transmit_buffer1,sizeof(transmit_buffer1),100);
				
	}
	

	
	//HAL_UART_Receive_IT(&huart4,(uint8_t *)&ack,1);
	
				if(i!=4)
		        {
			tx3=Thermo_Couple_Channel_Select_Data(i);
			tx4=Thermo_Couple_Channel_Select_Data1(i);
		
	CAL_ADC_ThermoCoupleModule_Configuration(&hspi3,&hspi5,GPIOH, MOD5_STRB_Pin,&tx3,&tx4,0x00,0x00,0x00);		
			
			i=i+1;
		  j=j+1;
			
		}
	else
	{
		
//CAL_ADC_ThermoCoupleModule_Configuration(&hspi3,&hspi5,GPIOH, MOD5_STRB_Pin,&tx,&tx1,0x00,0x00,0x00);	
		
		i=0;
	  j=0;
		
	}
	
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	HAL_Delay(1);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  HAL_GPIO_WritePin(GPIOF, LED_BUSY_Pin|LED_ERROR_Pin|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, MOD1_STRB_Pin|MOD2_STRB_Pin|MOD3_STRB_Pin|MOD4_STRB_Pin
                          |MOD5_STRB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_BUSY_Pin LED_ERROR_Pin PF13 */
  GPIO_InitStruct.Pin = LED_BUSY_Pin|LED_ERROR_Pin|GPIO_PIN_13;
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

  /*Configure GPIO pins : PG2 PG3 PG4 PG5
                           PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
