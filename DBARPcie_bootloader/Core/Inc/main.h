/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "lmk03328.h"
#include "cdcun1208.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ARRIUS_1_PCIE_PERST_N_OD_Pin GPIO_PIN_4
#define ARRIUS_1_PCIE_PERST_N_OD_GPIO_Port GPIOE
#define ARRIUS_1_PMBUS_ALERT_N_Pin GPIO_PIN_2
#define ARRIUS_1_PMBUS_ALERT_N_GPIO_Port GPIOE
#define ARRIUS_1_PMBUS_ALERT_N_EXTI_IRQn EXTI2_IRQn
#define CLK_PDN_Pin GPIO_PIN_10
#define CLK_PDN_GPIO_Port GPIOG
#define SFF_0_CBLPRSNT__Pin GPIO_PIN_5
#define SFF_0_CBLPRSNT__GPIO_Port GPIOD
#define SFF_0_CINT__Pin GPIO_PIN_12
#define SFF_0_CINT__GPIO_Port GPIOC
#define TRIGGER_SEL_Pin GPIO_PIN_14
#define TRIGGER_SEL_GPIO_Port GPIOH
#define ARRIUS_0_PCIE_PERST_N_OD_Pin GPIO_PIN_15
#define ARRIUS_0_PCIE_PERST_N_OD_GPIO_Port GPIOC
#define ARRIUS_0_PMBUS_CNTRL_N_Pin GPIO_PIN_3
#define ARRIUS_0_PMBUS_CNTRL_N_GPIO_Port GPIOE
#define HV_SYNC_Pin GPIO_PIN_4
#define HV_SYNC_GPIO_Port GPIOB
#define SFF_0_PERST_Pin GPIO_PIN_11
#define SFF_0_PERST_GPIO_Port GPIOG
#define SFF_0_CADDR_Pin GPIO_PIN_6
#define SFF_0_CADDR_GPIO_Port GPIOD
#define TRIGGER_OTTP_MISO_Pin GPIO_PIN_13
#define TRIGGER_OTTP_MISO_GPIO_Port GPIOH
#define HV_CTRL_Pin GPIO_PIN_14
#define HV_CTRL_GPIO_Port GPIOC
#define ARRIUS_HV_CTRL_Pin GPIO_PIN_6
#define ARRIUS_HV_CTRL_GPIO_Port GPIOE
#define TRIGGER_MODE_Pin GPIO_PIN_15
#define TRIGGER_MODE_GPIO_Port GPIOA
#define ARRIUS_0_PMBUS_ALERT_N_Pin GPIO_PIN_0
#define ARRIUS_0_PMBUS_ALERT_N_GPIO_Port GPIOE
#define ARRIUS_0_PMBUS_ALERT_N_EXTI_IRQn EXTI0_IRQn
#define SFF_MGTPWR_EN_Pin GPIO_PIN_13
#define SFF_MGTPWR_EN_GPIO_Port GPIOG
#define TRIGGER_ERC_ADDR_CS_Pin GPIO_PIN_0
#define TRIGGER_ERC_ADDR_CS_GPIO_Port GPIOD
#define GCLK_TIM3_Pin GPIO_PIN_2
#define GCLK_TIM3_GPIO_Port GPIOD
#define DS_PD_Pin GPIO_PIN_7
#define DS_PD_GPIO_Port GPIOC
#define GCLK_TIM5_Pin GPIO_PIN_4
#define GCLK_TIM5_GPIO_Port GPIOA
#define SFF_1_CINT__Pin GPIO_PIN_15
#define SFF_1_CINT__GPIO_Port GPIOD
#define SFF_1_CBLPRSNT__Pin GPIO_PIN_14
#define SFF_1_CBLPRSNT__GPIO_Port GPIOD
#define DC3V3_SYNC_Pin GPIO_PIN_1
#define DC3V3_SYNC_GPIO_Port GPIOA
#define FAN0_PWM_Pin GPIO_PIN_13
#define FAN0_PWM_GPIO_Port GPIOE
#define DC12V_SYNC_Pin GPIO_PIN_10
#define DC12V_SYNC_GPIO_Port GPIOH
#define SFF_1_PERST_Pin GPIO_PIN_10
#define SFF_1_PERST_GPIO_Port GPIOD
#define SFF_1_CADDR_Pin GPIO_PIN_12
#define SFF_1_CADDR_GPIO_Port GPIOD
#define POWER_BTN_Pin GPIO_PIN_2
#define POWER_BTN_GPIO_Port GPIOH
#define PC_LED_Pin GPIO_PIN_5
#define PC_LED_GPIO_Port GPIOC
#define FAN1_PWM_Pin GPIO_PIN_11
#define FAN1_PWM_GPIO_Port GPIOE
#define POWER_LED_Pin GPIO_PIN_15
#define POWER_LED_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
