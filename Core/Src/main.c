/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com
///
/// Edited and ported to STM32G474 by Joey Byrnes

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "structs.h"
#include <stdio.h>
#include <string.h>

#include "stm32g4xx.h"
#include "flash_writer.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "math_ops.h"
#include "calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* !!! Be careful the user area should be in another bank than the code !!! */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08040000)   /* Start @ of user Flash area */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define VERSION_NUM 2.0f


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[MAX_FLOATS_IN_FLASH];
int __int_reg[MAX_INTS_IN_FLASH];
PreferenceWriter prefs;

int count = 0;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;
CalStruct comm_encoder_cal;
CANTxMessage can_tx;
CANRxMessage can_rx;

/* init but don't allocate calibration arrays */
int *error_array = NULL;
int *lut_array = NULL;

uint8_t Serial2RxBuffer[1];//JB

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC5_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_ADC4_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  /* Load settings from flash */
  preference_writer_init(&prefs, 0);
  preference_writer_load(prefs);

  /* Sanitize configs in case flash is empty*/
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}
  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 5.0f;}
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 14.0f;}
  if(isnan(GR) || GR==-1){GR = 1.0f;}
  if(isnan(KT) || KT==-1){KT = 1.0f;}
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
  if(isnan(P_MAX)){P_MAX = 12.5f;}
  if(isnan(P_MIN)){P_MIN = -12.5f;}
  if(isnan(V_MAX)){V_MAX = 65.0f;}
  if(isnan(V_MIN)){V_MIN = -65.0f;}

  printf("\r\nFirmware Version Number: %.2f\n", VERSION_NUM);
  printf("\r\nPorted to STM32G4 by RoboDesign Lab UIUC\r\n\n");

  /* Controller Setup */
  if(PHASE_ORDER){							// Timer channel to phase mapping

  }
  else{

  }

  init_controller_params(&controller);

  /* calibration "encoder" zeroing */
  memset(&comm_encoder_cal.cal_position, 0, sizeof(EncoderStruct));

  /* commutation encoder setup */
  comm_encoder.m_zero = M_ZERO;
  comm_encoder.e_zero = E_ZERO;
  PPAIRS = 14.0f; //JB
  comm_encoder.ppairs = PPAIRS;
  ps_warmup(&comm_encoder, 100);			// clear the noisy data when the encoder first turns on

  if(EN_ENC_LINEARIZATION){memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));}	// Copy the linearization lookup table
  else{memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));}
  //for(int i = 0; i<128; i++){printf("%d\r\n", comm_encoder.offset_lut[i]);}

  /* Turn on ADCs */
//  HAL_ADC_Start(&hadc1);
//  HAL_ADC_Start(&hadc2);
//  HAL_ADC_Start(&hadc3);
//  HAL_ADC_Start(&hadc4);//JB

  /* DRV8323 setup */
  HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_SET ); 	// CS high
  HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_SET );
//  HAL_Delay(1);
//  drv_calibrate(drv);
//  HAL_Delay(1);
//  drv_write_DCR(drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
//  HAL_Delay(1);
//  int CSA_GAIN;
//  if(I_MAX <= 40.0f){CSA_GAIN = CSA_GAIN_40;}	// Up to 40A use 40X amplifier gain
//  else{CSA_GAIN = CSA_GAIN_20;}					// From 40-60A use 20X amplifier gain.  (Make this generic in the future)
//  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_0_25);
//  HAL_Delay(1);
//  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN, 0x1, 0x0, 0x0, 0x0, SEN_LVL_0_25);
//  HAL_Delay(1);
//  zero_current(&controller);
//  HAL_Delay(1);
//  drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_RETRY, OCP_DEG_4US, VDS_LVL_0_45);
//
//  HAL_Delay(1);
//  drv_disable_gd(drv);
//  HAL_Delay(1);

 //drv_enable_gd(drv);


  //=======================================================================================================================================
  //drv_calibrate(drv);
  HAL_Delay(1);
  drv_write_DCR(drv, 0x0, DIS_GDF_DIS, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
  HAL_Delay(1);
  drv_write_DCR(drv, 0x0, DIS_GDF_DIS, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x1, 0x0, 0x1);
  HAL_Delay(1);

  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_1_0);   // calibrate shunt amplifiers
  HAL_Delay(1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x1, 0x0, 0x0, 0x0, SEN_LVL_1_0);
  HAL_Delay(1);
  zero_current(&controller);
  HAL_Delay(1);
  drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_NONE, OCP_DEG_8US, VDS_LVL_1_88);
  HAL_Delay(1);

  HAL_Delay(1);
  drv_disable_gd(drv);
  HAL_Delay(1);


  ADC1->CR &= ~ADC_CR_DEEPPWD;
  ADC2->CR &= ~ADC_CR_DEEPPWD;
  ADC3->CR &= ~ADC_CR_DEEPPWD;
  ADC4->CR &= ~ADC_CR_DEEPPWD;

  ADC1->CR |= ADC_CR_ADVREGEN;
  ADC2->CR |= ADC_CR_ADVREGEN;
  ADC3->CR |= ADC_CR_ADVREGEN;
  ADC4->CR |= ADC_CR_ADVREGEN;

  HAL_Delay(1);

  ADC1->CR |= ADC_CR_ADCAL;
  ADC2->CR |= ADC_CR_ADCAL;
  ADC3->CR |= ADC_CR_ADCAL;
  ADC4->CR |= ADC_CR_ADCAL;

  while ((ADC1->CR & ADC_CR_ADCAL) ||
		  (ADC2->CR & ADC_CR_ADCAL) ||
		  (ADC3->CR & ADC_CR_ADCAL) ||
		  (ADC4->CR & ADC_CR_ADCAL));

  HAL_Delay(1);

  ADC1->ISR |= ADC_ISR_ADRDY;
  ADC2->ISR |= ADC_ISR_ADRDY;
  ADC3->ISR |= ADC_ISR_ADRDY;
  ADC4->ISR |= ADC_ISR_ADRDY;

  ADC1->CR |= ADC_CR_ADEN;
  ADC2->CR |= ADC_CR_ADEN;
  ADC3->CR |= ADC_CR_ADEN;
  ADC4->CR |= ADC_CR_ADEN;

  while (!(ADC1->ISR & ADC_ISR_ADRDY) ||
		  !(ADC2->ISR & ADC_ISR_ADRDY) ||
		  !(ADC3->ISR & ADC_ISR_ADRDY) ||
		  !(ADC4->ISR & ADC_ISR_ADRDY));

  ADC1->ISR |= ADC_ISR_ADRDY;
  ADC2->ISR |= ADC_ISR_ADRDY;
  ADC3->ISR |= ADC_ISR_ADRDY;
  ADC4->ISR |= ADC_ISR_ADRDY;

  ADC1->CFGR &= ~(ADC_CFGR_CONT);
  ADC2->CFGR &= ~(ADC_CFGR_CONT);
  ADC3->CFGR &= ~(ADC_CFGR_CONT);
  ADC4->CFGR &= ~(ADC_CFGR_CONT);

  // START ANALOG CALIBRATION:

  ADC1->CR |= 1; // Enable ADC1
  ADC2->CR |= 1; // Enable ADC2
  ADC3->CR |= 1; // Enable ADC3
  ADC4->CR |= 1; // Enable ADC4

  HAL_Delay(1);

  printf("Calculating ADC Offsets...\n");
  uint32_t offset1=0;
  uint32_t offset2=0;
  uint32_t offset3=0;

  for(int c=0;c<100;c++){
	  ADC1->CR |= 0x0004;
	  ADC2->CR |= 0x0004;
	  ADC3->CR |= 0x0004;
	  HAL_Delay(1);
	  offset3 += ADC3->DR;
	  offset2 += ADC2->DR;
	  offset1 += ADC1->DR;
  }

  controller.adc_c_offset = (int)((double)offset3/100.0f);
  controller.adc_b_offset = (int)((double)offset2/100.0f);
  controller.adc_a_offset = (int)((double)offset1/100.0f);

  //=======================================================================================================================================

  printf("ADC A OFFSET: %d, \tADC B OFFSET: %d \tADC C OFFSET: %d\r\n", controller.adc_a_offset, controller.adc_b_offset, controller.adc_c_offset);

  /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
//  {
//	  Error_Handler();
//  }
//  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
//  {
//	  Error_Handler();
//  }
//  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
//  {
//	  Error_Handler();
//  }

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);

  HAL_FDCAN_Start(&CAN_H);
  HAL_FDCAN_ActivateNotification(&CAN_H, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);


  /* Set Interrupt Priorities */
  HAL_NVIC_SetPriority(PWM_ISR, 0x0,0x0); // commutation > communication
  HAL_NVIC_SetPriority(CAN_ISR, 0x01, 0x01);

  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;


  /* Turn on interrupts */
  HAL_UART_Receive_IT(&huart3, (uint8_t *)Serial2RxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  HAL_Delay(100);
	  drv_print_faults(drv);
	 // if(state.state==MOTOR_MODE){
	  	  //printf("%.2f %.2f %.2f %.2f %.2f\r\n", controller.p_des, controller.v_des, controller.kp, controller.kd, controller.t_ff);
	  //}
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		/* Retrieve Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(&CAN_H, FDCAN_RX_FIFO0, &can_rx.rx_header, can_rx.data) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_FDCAN_GetRxMessage(&CAN_H, FDCAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);

		pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR);

		HAL_FDCAN_AddMessageToTxFifoQ(&CAN_H, &can_tx.tx_header, can_tx.data);

		if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) & (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFC))){
			update_fsm(&state, MOTOR_CMD);
		}
		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFD))){
			update_fsm(&state, MENU_CMD);
		}
		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFE))){
			update_fsm(&state, ZERO_CMD);
		}
		else{
			unpack_cmd(can_rx, controller.commands);
			controller.timeout = 0;
		}

	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
