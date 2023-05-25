/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2cregdefs.h"
#include "confmem.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_VER	0xB1000000 //Bootloader version 1.0.0.0

#define EXTERNAL 	0xF0
#define INTERNAL	0x0F

#define FLASH_APP_BASEADDR 0x08020000 	//128KB application offset
#define FLASH_APP_SZ 0x00020000 		//256KB application firmware size

#define I2CSLV_RXBUF_SZ 	256

#define I2CSLV_FLASH		(0x21 << 1)
#define I2CSLV_CMD 			(0x20 << 1)

//Power states
#define PWR_OFF 		0
#define PWR_ON			1
#define PWR_PWRDOWN		0x10
#define PWR_PWRUP		0x11

//I2C3 Master defines

#define I2C3_WRREQ 0x05
#define I2C3_RDREQ 0x06
#define I2C3_WRRDREQ 0x07

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;
I2C_HandleTypeDef hi2c5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Regs regs;

uint16_t rxBufferPtr = 0;
uint16_t txBufferPtr = 0;
uint8_t rxBuffer[I2CSLV_RXBUF_SZ];

uint32_t rxFlashMemPtr = 0;
uint32_t txFlashMemPtr = 0;
uint8_t flashBuf[FLASH_APP_SZ]; //RAM flash buffer 256KB

uint8_t i2cSlvDest = 0;
uint16_t i2cSlvTxSize = 0;

uint8_t pwrState = PWR_OFF;

uint32_t tempAmbient = 0;

TIM_OC_InitTypeDef fan0PWMConf;
TIM_OC_InitTypeDef fan1PWMConf;

uint8_t i2cCmdPending = 0;

uint8_t i2c3_rxBuf[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_I2C5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void getBuildDate(uint8_t* year8, uint8_t* month8, uint8_t* day8);
void getBuildTime(uint8_t* hour8, uint8_t* min8, uint8_t* sec8);

void triggerSelect(uint8_t trigSel);
void sffSelect(uint8_t sffSel);
void refClkSelect(uint8_t refSel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_I2C5_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  printf("DBARLitePcie Bootloader v1.0.0.0\n");
  printf("Build ");
  printf(__TIMESTAMP__);
  printf("\n");

  ConfMem conf;
  ConfigMemory_Download(&conf);

  memcpy((void*)(&regs), (void*)(&conf), 0x20); //copy serial number and hw info from config memory

  //set info regs
  regs.info.status = 0x80;
  regs.info.hwRevision = 0x01;
  regs.info.serial[0] = 23;
  regs.info.serial[1] = 20;
  regs.info.serial[2] = 03;
  regs.info.hwVersion - 0x00;
  regs.info.fwVersion = FW_VER; //set firmware version
  sprintf(regs.info.snString, "DBLP%02d%02d%02d%02d%02d", regs.info.hwRevision,
		  regs.info.serial[0], regs.info.serial[1], regs.info.serial[2], regs.info.hwVersion);
  //set firmware build time
  getBuildDate(&(regs.info.buildYear), &(regs.info.buildMonth), &(regs.info.buildDay));
  getBuildTime(&(regs.info.buildHour), &(regs.info.buildMin), &(regs.info.buildSec));

  uint8_t confMemPtr = 0;
  regs.config.AddrOffset = confMemPtr;
  regs.config.Control = 0;
  uint8_t* srcAddress = (uint8_t*)&conf + confMemPtr;
  regs.config.Value = *srcAddress;

  cdcun1208_init(&hi2c3);

  //clear flash RAM buffer
  for(uint32_t n = 0; n < FLASH_APP_SZ; n++) {
	  flashBuf[n] = 0xFF;
  }

  //Start listening on I2C2 (ARRIUS_0 i2c)
  HAL_I2C_EnableListen_IT(&hi2c2);

  setFan0PWM(50);
  setFan1PWM(50);

  uint8_t ds160_rx;

  HAL_GPIO_WritePin(ARRIUS_0_PCIE_PERST_N_OD_GPIO_Port, ARRIUS_0_PCIE_PERST_N_OD_Pin,
  							HAL_GPIO_ReadPin(SFF_0_PERST_GPIO_Port, SFF_0_PERST_Pin));
  HAL_GPIO_WritePin(ARRIUS_1_PCIE_PERST_N_OD_GPIO_Port, ARRIUS_1_PCIE_PERST_N_OD_Pin,
  							HAL_GPIO_ReadPin(SFF_1_PERST_GPIO_Port, SFF_1_PERST_Pin));

  pwrState = PWR_OFF;

  i2cCmdPending = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(pwrState == PWR_PWRUP) {
		//start power up sequence
		setFan0PWM(100);
		setFan1PWM(100);

		HAL_GPIO_WritePin(ARRIUS_0_PMBUS_CNTRL_N_GPIO_Port, ARRIUS_0_PMBUS_CNTRL_N_Pin, GPIO_PIN_SET);
		HAL_Delay(100);

		lmk03328_enable();
		HAL_Delay(10);
		lmk03328_init(&hi2c3);

		ds160_enable();
		ds160_init(&hi2c3, EXTERNAL);
		HAL_Delay(10);

		pwrState = PWR_ON;
		setPowerLed(GPIO_PIN_SET);
		printf("POWER ON \n");
	}
	else if(pwrState == PWR_PWRDOWN) {
		//start power down sequence
		HAL_GPIO_WritePin(ARRIUS_0_PMBUS_CNTRL_N_GPIO_Port, ARRIUS_0_PMBUS_CNTRL_N_Pin, GPIO_PIN_RESET);
		lmk03328_disable();
		ds160_disable();

		setFan0PWM(50);
		setFan1PWM(50);

		pwrState = PWR_OFF;
		setPowerLed(GPIO_PIN_RESET);
		printf("POWER OFF \n");
	}
	else if(pwrState == PWR_ON) {
		//TO DO
		//* FANS PWM control
		//* OEM temp monitoring
		//* I2C error recovery
	}

	/*
	 * #define I2C3_WRREQ 0x05
#define I2C3_RDREQ 0x06
#define I2C3_WRRDREQ 0x07
	 */

	if(i2cCmdPending) {

		//I2C3 MASTER PASSTHROUGH COMMANDS
		if(regs.i2c.xferRequest == I2C3_WRREQ) {
			printf("I2C3 write request \n");
			HAL_StatusTypeDef i2c3_st;
			uint8_t devAddr = regs.i2c.devAddr;
			uint8_t txBuf;
			uint8_t txSz = regs.i2c.length & 0x0F;
			if(txSz  >4) { txSz = 4; }
			memcpy(txBuf, regs.i2c.wrBuf, txSz);
			regs.i2c.status |= 0x02;
			i2c3_st = HAL_I2C_Master_Transmit_IT(&hi2c3, devAddr, txBuf, txSz);
			if(i2c3_st = HAL_ERROR) {
				uint32_t err = HAL_I2C_GetError(&hi2c3);
				printf("I2C3 write request error %08X\n", err);
				regs.i2c.status |= 0x04;
			}
			regs.i2c.xferRequest &= ~0x04;
		}
		else if(regs.i2c.xferRequest == I2C3_RDREQ) {
			printf("I2C3 read request \n");
			HAL_StatusTypeDef i2c3_st;
			uint8_t devAddr = regs.i2c.devAddr;
			uint8_t rxSz = (regs.i2c.length & 0xF0) >> 4;
			if(rxSz  >4) { rxSz = 4; }
			regs.i2c.status |= 0x02;
			memset(i2c3_rxBuf, 0x00, 4);
			i2c3_st = HAL_I2C_Master_Receive_IT(&hi2c3, devAddr, i2c3_rxBuf, rxSz);
			if(i2c3_st = HAL_ERROR) {
				uint32_t err = HAL_I2C_GetError(&hi2c3);
				printf("I2C3 write request error %08X\n", err);
				regs.i2c.status |= 0x04;
			}
			regs.i2c.xferRequest &= ~0x04;
		}
		else if(regs.i2c.xferRequest == I2C3_WRRDREQ) {
			printf("I2C3 write-read request \n");
			HAL_StatusTypeDef i2c3_st;
			uint8_t devAddr = regs.i2c.devAddr;
			uint8_t txBuf[2];
			uint8_t txSz = regs.i2c.length & 0x0F;

			if(txSz > 2) {
				regs.i2c.status |= 0x04;
			}
			else {
				memcpy(txBuf, regs.i2c.wrBuf, txSz);
				uint16_t memAddr = 0;
				if(txSz == 1) {
					memAddr = (txBuf[0] << 1) + txBuf[1];
				}
				else if(txSz) {
					memAddr = txBuf[0];
				}

				uint8_t rxSz = (regs.i2c.length & 0xF0) >> 4;
				if(rxSz  >4) { rxSz = 4; }
				regs.i2c.status |= 0x02;
				memset(i2c3_rxBuf, 0x00, 4);
				i2c3_st = HAL_I2C_Mem_Read_IT(&hi2c3, devAddr, memAddr, txSz, i2c3_rxBuf, rxSz);
				if(i2c3_st = HAL_ERROR) {
					uint32_t err = HAL_I2C_GetError(&hi2c3);
					printf("I2C3 write request error %08X\n", err);
					regs.i2c.status |= 0x04;
				}
			}
			regs.i2c.xferRequest &= ~0x04;
		}
		//END I2C3 MASTER PASSTHROUGH COMMANDS

		i2cCmdPending = 0;
	}



    /* USER CODE END WHILE */

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
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 80;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C4|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_I2C5
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 200;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 8;
  PeriphClkInitStruct.PLL3.PLL3R = 4;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C1235CLKSOURCE_PLL3;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00401959;
  hi2c1.Init.OwnAddress1 = I2CSLV_CMD;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c1.Init.OwnAddress2 = I2CSLV_FLASH;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00401959;
  hi2c2.Init.OwnAddress1 = I2CSLV_CMD;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c2.Init.OwnAddress2 = I2CSLV_FLASH;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00401959;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00C0EAFF;
  hi2c4.Init.OwnAddress1 = 32;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief I2C5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C5_Init(void)
{

  /* USER CODE BEGIN I2C5_Init 0 */

  /* USER CODE END I2C5_Init 0 */

  /* USER CODE BEGIN I2C5_Init 1 */

  /* USER CODE END I2C5_Init 1 */
  hi2c5.Instance = I2C5;
  hi2c5.Init.Timing = 0x00C0EAFF;
  hi2c5.Init.OwnAddress1 = 32;
  hi2c5.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c5.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c5.Init.OwnAddress2 = 0;
  hi2c5.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c5.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c5.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c5, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C5_Init 2 */

  /* USER CODE END I2C5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 39;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 74;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  fan0PWMConf = sConfigOC;
  fan1PWMConf = sConfigOC;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 49999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ARRIUS_1_PCIE_PERST_N_OD_Pin|ARRIUS_1_PMBUS_CNTRL_N_Pin|ARRIUS_HV_CTRL_Pin|POWER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, CLK_PDN_Pin|SFF_MGTPWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SFF_0_CBLPRSNT__Pin|SFF_0_CADDR_Pin|TRIGGER_ERC_ADDR_CS_Pin|SFF_1_CINT__Pin
                          |SFF_1_CBLPRSNT__Pin|SFF_1_CADDR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SFF_0_CINT__Pin|ARRIUS_0_PCIE_PERST_N_OD_Pin|HV_CTRL_Pin|PC_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, TRIGGER_SEL_Pin|TRIGGER_OTTP_MISO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ARRIUS_0_PMBUS_CNTRL_N_GPIO_Port, ARRIUS_0_PMBUS_CNTRL_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGGER_MODE_GPIO_Port, TRIGGER_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS_PD_GPIO_Port, DS_PD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ARRIUS_1_PCIE_PERST_N_OD_Pin ARRIUS_1_PMBUS_CNTRL_N_Pin ARRIUS_HV_CTRL_Pin POWER_LED_Pin */
  GPIO_InitStruct.Pin = ARRIUS_1_PCIE_PERST_N_OD_Pin|ARRIUS_1_PMBUS_CNTRL_N_Pin|ARRIUS_HV_CTRL_Pin|POWER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ARRIUS_1_PMBUS_ALERT_N_Pin ARRIUS_0_PMBUS_ALERT_N_Pin */
  GPIO_InitStruct.Pin = ARRIUS_1_PMBUS_ALERT_N_Pin|ARRIUS_0_PMBUS_ALERT_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_PDN_Pin SFF_MGTPWR_EN_Pin */
  GPIO_InitStruct.Pin = CLK_PDN_Pin|SFF_MGTPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SFF_0_CBLPRSNT__Pin SFF_0_CADDR_Pin TRIGGER_ERC_ADDR_CS_Pin SFF_1_CINT__Pin
                           SFF_1_CBLPRSNT__Pin SFF_1_CADDR_Pin */
  GPIO_InitStruct.Pin = SFF_0_CBLPRSNT__Pin|SFF_0_CADDR_Pin|TRIGGER_ERC_ADDR_CS_Pin|SFF_1_CINT__Pin
                          |SFF_1_CBLPRSNT__Pin|SFF_1_CADDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SFF_0_CINT__Pin ARRIUS_0_PCIE_PERST_N_OD_Pin HV_CTRL_Pin DS_PD_Pin
                           PC_LED_Pin */
  GPIO_InitStruct.Pin = SFF_0_CINT__Pin|ARRIUS_0_PCIE_PERST_N_OD_Pin|HV_CTRL_Pin|DS_PD_Pin
                          |PC_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIGGER_SEL_Pin TRIGGER_OTTP_MISO_Pin */
  GPIO_InitStruct.Pin = TRIGGER_SEL_Pin|TRIGGER_OTTP_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARRIUS_0_PMBUS_CNTRL_N_Pin */
  GPIO_InitStruct.Pin = ARRIUS_0_PMBUS_CNTRL_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ARRIUS_0_PMBUS_CNTRL_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SFF_0_PERST_Pin */
  GPIO_InitStruct.Pin = SFF_0_PERST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SFF_0_PERST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIGGER_MODE_Pin */
  GPIO_InitStruct.Pin = TRIGGER_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIGGER_MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SFF_1_PERST_Pin */
  GPIO_InitStruct.Pin = SFF_1_PERST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SFF_1_PERST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_BTN_Pin */
  GPIO_InitStruct.Pin = POWER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(POWER_BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	char dbg[6];
	sprintf(dbg, "0x%04x", AddrMatchCode);
	printf(dbg);
	printf(" I2C Address detected\n");

	i2cSlvDest = AddrMatchCode; //CMD or FLASH

	rxBufferPtr = 0;

	uint8_t* txPtr = flashBuf;
	txPtr+=txFlashMemPtr;

	if(TransferDirection == I2C_DIRECTION_TRANSMIT) {
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, 1, I2C_NEXT_FRAME);
	}
	else if(TransferDirection == I2C_DIRECTION_RECEIVE) {
		if(i2cSlvDest == I2CSLV_CMD) {
			uint8_t* srcAddress;
			srcAddress = (uint8_t*)&regs + txBufferPtr;
			i2cSlvTxSize = (I2CSLV_REGS_SZ-txBufferPtr);
			//printf(" I2C response offset = %x, size = %d\n", srcAddress, i2cSlvTxSize);
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, srcAddress, i2cSlvTxSize , I2C_NEXT_FRAME);
		}
		else if(i2cSlvDest == I2CSLV_FLASH) {
			i2cSlvTxSize = 256;
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txPtr, i2cSlvTxSize, I2C_NEXT_FRAME);
		}

	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(i2cSlvDest == I2CSLV_CMD) {
		printf(" I2C cmd response sent\n");
		//Only gets called if regs memory space rolls over, transmit regs from 0x00 offset
		uint8_t* srcAddress = (uint8_t*)&regs;
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, srcAddress, I2CSLV_REGS_SZ, I2C_NEXT_FRAME);
	}
	else if(i2cSlvDest == I2CSLV_FLASH) {
		printf(" I2C flash buf response sent\n");
		i2cSlvTxSize = 256;
		txFlashMemPtr += i2cSlvTxSize;
		uint8_t* txPtr = flashBuf;
		txPtr += txFlashMemPtr;
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txPtr, i2cSlvTxSize, I2C_NEXT_FRAME);
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	if(rxBufferPtr == 0 && i2cSlvDest == I2CSLV_CMD)
		txBufferPtr = rxBuffer[0];

	printf(" I2C received byte 0x%02X\n", rxBuffer[rxBufferPtr]);
	if(	rxBufferPtr < 255) {
		rxBufferPtr++;
	}
	HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rxBuffer[rxBufferPtr], 1, I2C_LAST_FRAME);
}

void ProcessI2cCmd(uint8_t* buf, uint8_t len) {
	uint8_t offset = buf[0];
	uint8_t* destAddress;
	uint8_t* maskAddress;

	for(uint8_t n = 0; n < (len-1); n++){
		destAddress = (uint8_t*)&regs + offset + n;
		maskAddress = (uint8_t*)&regsMask + offset + n;
		*destAddress = (buf[n+1]&(*maskAddress));
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(i2cSlvDest == I2CSLV_FLASH) {
		memcpy(flashBuf + txFlashMemPtr, rxBuffer, (rxBufferPtr + 1));
		txFlashMemPtr += (rxBufferPtr + 1);
	}
	else if(i2cSlvDest == I2CSLV_CMD) {
		printf(" I2C listen cplt\n");
		//Process I2C command
		if(rxBufferPtr>1) {
			ProcessI2cCmd(rxBuffer, rxBufferPtr);
			i2cCmdPending = 1;
		}
	}
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	uint32_t err = HAL_I2C_GetError(hi2c);
	if( err != HAL_I2C_ERROR_AF ) { //Transaction terminated by master - don't care
		printf("I2C Error %X\n", err);
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	printf("I2C3 Master Tx Done\n");
	regs.i2c.status = 0x00;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	printf("I2C3 Master Rx Done\n");
	memcpy(regs.i2c.rdBuf, i2c3_rxBuf, 4);
	regs.i2c.status = 0x00;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	printf("I2C3 Master TxRx Done\n");
	memcpy(regs.i2c.rdBuf, i2c3_rxBuf, 4);
	regs.i2c.status = 0x00;
}

void triggerSelect(uint8_t trigSel) {
	if(trigSel == INTERNAL) {
		//config CDCUN to use ARRIUS0 trigger
		//TO DO
	}
	else if(trigSel == EXTERNAL) {
		//config CDCUN to use external trigger
		//TO DO
	}
}

void sffSelect(uint8_t sffSel) {
	if(sffSel == INTERNAL) {
		//config DS160s to use SFF8643 path
		//TO DO
	}
	else if(sffSel == EXTERNAL) {
		//config DS160s to use SFF8644 path
		//TO DO
	}
}

void refClkSelect(uint8_t refSel) {
	if(refSel == INTERNAL) {
		//config LMK for XTAL clock reference
		//TO DO
	}
	else if(refSel == EXTERNAL) {
		//config LMK for external clock reference
		//TO DO
	}
}

uint8_t getMonthNumber(const char* monthStr) {
    if (strcmp(monthStr, "Jan") == 0) return 1;
    else if (strcmp(monthStr, "Feb") == 0) return 2;
    else if (strcmp(monthStr, "Mar") == 0) return 3;
    else if (strcmp(monthStr, "Apr") == 0) return 4;
    else if (strcmp(monthStr, "May") == 0) return 5;
    else if (strcmp(monthStr, "Jun") == 0) return 6;
    else if (strcmp(monthStr, "Jul") == 0) return 7;
    else if (strcmp(monthStr, "Aug") == 0) return 8;
    else if (strcmp(monthStr, "Sep") == 0) return 9;
    else if (strcmp(monthStr, "Oct") == 0) return 10;
    else if (strcmp(monthStr, "Nov") == 0) return 11;
    else if (strcmp(monthStr, "Dec") == 0) return 12;
    else return 0;  // Invalid month
}

void getBuildDate(uint8_t* year8, uint8_t* month8, uint8_t* day8) {
	const char* dateStr = __DATE__;
	char monthStr[4];
	int year, month, day;

	sscanf(dateStr, "%s %d %d", monthStr, &day, &year);
	month = getMonthNumber(monthStr);

	year %= 100;
	*year8 = (uint8_t)year;
	*month8 = (uint8_t)month;
	*day8 = (uint8_t)day;
}

void getBuildTime(uint8_t* hour8, uint8_t* min8, uint8_t* sec8) {
	const char* timeStr = __TIME__;
	int hour, minute, second;

	sscanf(timeStr, "%d:%d:%d", &hour, &minute, &second);

	*hour8 = (uint8_t)hour;
	*min8 = (uint8_t)minute;
	*sec8 = (uint8_t)second;
}

void setFan0PWM(uint8_t dutyCycle) {
	if(dutyCycle > 100)
		dutyCycle = 100;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyCycle);
}

void setFan1PWM(uint8_t dutyCycle) {
	if(dutyCycle > 100)
		dutyCycle = 100;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyCycle);
}

void setPCLed(GPIO_PinState state) {
	HAL_GPIO_WritePin(PC_LED_GPIO_Port, PC_LED_Pin, state);
}

void setPowerLed(GPIO_PinState state) {
	HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, state);
}

void togglePower() {
	if(pwrState == PWR_ON) {
		pwrState = PWR_PWRDOWN;
		//start power down cycle
	}
	else if(pwrState == PWR_OFF) {
		pwrState = PWR_PWRUP;
		//start power on cycle
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_11) {
		//printf("SFF0 RST %d\n", HAL_GPIO_ReadPin(SFF_0_PERST_GPIO_Port, SFF_0_PERST_Pin));
		HAL_GPIO_WritePin(ARRIUS_0_PCIE_PERST_N_OD_GPIO_Port, ARRIUS_0_PCIE_PERST_N_OD_Pin,
							HAL_GPIO_ReadPin(SFF_0_PERST_GPIO_Port, SFF_0_PERST_Pin));
	}
	if(GPIO_Pin == GPIO_PIN_10) {
		//printf("SFF1 RST %d\n", HAL_GPIO_ReadPin(SFF_1_PERST_GPIO_Port, SFF_1_PERST_Pin));
		HAL_GPIO_WritePin(ARRIUS_1_PCIE_PERST_N_OD_GPIO_Port, ARRIUS_1_PCIE_PERST_N_OD_Pin,
							HAL_GPIO_ReadPin(SFF_1_PERST_GPIO_Port, SFF_1_PERST_Pin));
	}
}

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0x00FF);
	return ch;
}

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
