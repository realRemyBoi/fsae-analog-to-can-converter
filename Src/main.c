
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  * 06/2021
  * Firmware written by:
  * Michael Park
  * Isaac Remy
  * James Edwards
  *
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "filter.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
static uint16_t CONFIG_COMMAND = 0x15; // will need to be array (for multi-channel reading)
static uint32_t FILTER_COMMAND = 0x784;
static uint16_t VBIAS_COMMAND = 0x00; // will need to be array (for multi-channel reading)
static uint16_t ADC_CONTROL_COMMAND = 0x100;
static const float AN_TIMER_PERIOD = AN_PRESCALER * 1.0F /AN_CLK_FREQUENCY;
static const uint32_t AN_ZERO_SPEED = 0x4C4B40;
volatile uint32_t rearCnt = 0;
static int count=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
 * The following code initializes the variables used in wheel speed calculations.
 */
#if AN_DEBUG_MODE
static uint32_t AN_counterArrayL[100] = {0};
static uint32_t AN_counterArrayR[100] = {0};
volatile static float AN_speedArrayL[100] = {0};
volatile static float AN_speedArrayR[100] = {0};
static int i=0;
static int j=0;

#endif
volatile static float AN_speedL=0.0F;
volatile static float AN_speedR=0.0F;
volatile static uint32_t AN_counterL=0;
volatile static uint32_t AN_counterR=0;
volatile static uint16_t AN_speedLCoded=0;
volatile static uint16_t AN_speedRCoded=0;

LPFilter AN_rearFilter = {.samplingTime = AN_SPEED_PERIOD, .freq = 200 * 2 * AN_PI,
		.a = 0.9, .prevInput = 0
};

LPFilter AN_frontFilter = {.samplingTime = AN_SPEED_PERIOD, .freq = 200 * 2 * AN_PI,
		.a = 0.9, .prevInput = 0
};
static bool AN_shouldExecuteSM = false;
int AN_counter;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /*
   * Sets the initial state of AnaCAN to NOMINAL (the default if nothing is malfunctioning).
   */
  AN.state = NOMINAL;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM14_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /**
   * Activates peripheral timers and CAN for the strain gauge.
   */
  AN_peripheralStartup();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_CAN_Start(&hcan1);
  SPI_commRegisterTrue(); // verifies SPI lines are functioning
  SPI_write16(0x01, ADC_CONTROL_COMMAND); // ADC_Control_Register
  SPI_write16(0x09, 0x00); // Writes to differential channel 0
  SPI_write16(0x19, CONFIG_COMMAND); // Configures ADC gain
  SPI_write24(0x21, FILTER_COMMAND); // Sets filter and output data rate
  SPI_write16(0x04, VBIAS_COMMAND); // Sets voltage bias
  SPI_write16(0x09, 0x8001); // Enables differential channel 0

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /**
   * The main loop is an infinite loop that is constantly running.
   * Main Loop:
   * Timer4 sets AN_shouldExecuteSM to true when its period elapses (see HAL_TIM_PeriodElapsedCallback).
   * When set to true, AN_executeState() is called which executes the desired function depending on the
   * state of AnaCAN. Afterwards, AN_shouldExecuteSM is set to false again.
   */
    if (AN_shouldExecuteSM){
      AN_executeState();
      AN_shouldExecuteSM = false;
	  }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void AN_peripheralStartup(){
/**
 * This function configures all CAN busses and timers.
 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_FULL);

  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_FULL);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_FULL);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);
}
/**
 * This function takes two 8-bit integer arrays and calculates wheel speed
 * from counter ticks and wheel slits. The calculated wheel speeds for the left
 * and the right side are then added to the arrays.
 *
 * Wheel speed is calculated using a hall effect (HE) sensor. The HE sensor emits a signal when
 * a ferrous object (object containing iron) is passed in front of it. On the FSAE car, the HE sensor is slotted
 * near the wheel and detects ferrous 'slits' that are evenly spaced around the wheel. When one slit passes the HE sensor,
 * a timer starts and stops the next time a slit passes the HE sensor. The time elapsed between slits combined with the
 * circumference of the wheel can be used to calculate wheel speed. The formula for wheel speed can be found by shift clicking
 * the AN_SPEED_CONVERSION_MS() function.
 */
void static AN_wheelConversion(uint8_t* dataTx1, uint8_t* dataTx2){
#if AN_DEBUG_MODE
  if(AN_counterArrayL[j]==AN_ZERO_SPEED){
    AN_speedArrayL[j]=0;
  }else{
#if FRONT
    AN_speedArrayL[j-1] = AN_SPEED_CONVERSION_MS(AN_counterArrayL[j], FRONT_WHEEL_SLITS);
#else
    AN_speedArrayL[j-1] = AN_SPEED_CONVERSION_MS(AN_counterArrayL[j], REAR_WHEEL_SLITS);
#endif
  }
  if(AN_counterArrayR[i]==AN_ZERO_SPEED){
    AN_speedArrayR[i]=0;
  }else{
#if FRONT
    AN_speedArrayR[i-1] = AN_SPEED_CONVERSION_MS(AN_counterArrayR[i], FRONT_WHEEL_SLITS);
#else
    AN_speedArrayR[i-1] = AN_SPEED_CONVERSION_MS(AN_counterArrayR[i], REAR_WHEEL_SLITS);
#endif
  }
  AN_speedL = AN_speedArrayL[j];
  AN_speedR = AN_speedArrayR[i];
#else
  if(AN_counterL==AN_ZERO_SPEED){
    AN_speedL=0;
  }else{
#if FRONT
    AN_speedL = AN_SPEED_CONVERSION_MS((AN_counterL+9), FRONT_WHEEL_SLITS);
#else
    AN_speedL = AN_SPEED_CONVERSION_MS((AN_counterL+9), REAR_WHEEL_SLITS);
#endif
  }
  if(AN_counterR==AN_ZERO_SPEED){
    AN_speedR=0;
  }else{
#if FRONT
    AN_speedR = AN_SPEED_CONVERSION_MS((AN_counterR+9), FRONT_WHEEL_SLITS);
#else
    AN_speedR = AN_SPEED_CONVERSION_MS((AN_counterR+9), REAR_WHEEL_SLITS);
#endif
  }
#endif

/*
 * If else check in the case that any of the wheels rotate >40m/s
 */
if(AN_speedL > 40.0){
  AN_speedL=40;
}else if(AN_speedR>40.0){
  AN_speedR=40;
}
  AN_speedLCoded=(AN_speedL*0xFFFF)/40;
  AN_speedRCoded=(AN_speedR*0xFFFF)/40;
  *dataTx1=AN_speedRCoded>>8;
  *(dataTx1+1)=AN_speedRCoded&0xFF;
  *dataTx2=AN_speedLCoded>>8;
  *(dataTx2+1)=AN_speedLCoded&0xFF;
}

/**
 * This function performs the default operation of AnaCAN: Perform SPI for left and and right sides to
 * get sensor data. Calls AN_wheelConversion to calculate wheel speed. Formats all data for CAN transmission.
 * If no errors in transmission, sets the state of AnaCAN to NOMINAL and SOMETINGWONG otherwise.
 */
static anacan_state AN_nominalState(){
  uint8_t dataTxRFast[8] ={0};
  uint8_t dataTxLFast[8] ={0};
  uint8_t dataTxRSlow[8] ={0};
  uint8_t dataTxLSlow[8] ={0};
  uint8_t sentSGData[3] = {0};
  SPI_rFastPrepare(dataTxRFast);
  SPI_lFastPrepare(dataTxLFast);
  AN_wheelConversion(dataTxRFast, dataTxLFast);
  uint32_t strainGaugeData = SPI_dataRead(0x42, 32);
  sentSGData[0] = strainGaugeData & 0xFF0000;
  sentSGData[1] = strainGaugeData & 0x00FF00;
  sentSGData[2] = strainGaugeData & 0x00FF00;

  /**
   * This if statement ensures that the 'Slow Prepare' SPI functions are called once every 100 calls of the
   * AN_nominalState() function.
   */
  if(count==99){
    count = -1;
    SPI_rSlowPrepare(dataTxRSlow);
    SPI_lSlowPrepare(dataTxLSlow);
  }
  //uint32_t tireTempLeft =
  //uint32_t tireTempRight =
  CAN_transmit(dataTxRFast, dataTxLFast, dataTxRSlow, dataTxLSlow, sentSGData);
  count++;
  AN_counter++;
  if(AN_counter%1000==0){
    CAN_updateStatus();
  }
  if(AN.errorBits.can1Fail||AN.errorBits.can2Fail||AN.errorBits.spiFail){
    return SOMETINGWONG;
  }
  else{
    return NOMINAL;
  }
}

/**
 * This function is called when AnaCAN is in the SOMETINGWONG state. This only occurs
 * when one of the critical peripherals malfunctions. Proceeds to try and re-initialize
 * the peripheral that malfunctioned and sets AnaCAN state afterwards. If successful in
 * re-initialization, AnaCAN state is set to NOMINAL and SOMETINGWONG otherwise.
 */
static anacan_state AN_sometingWongState(){
  if(AN.errorBits.can1Fail){
    MX_CAN1_Init();
    HAL_CAN_Start(&hcan1);
    AN.errorBits.can1Fail=false;
  } else if(AN.errorBits.can2Fail){
    MX_CAN2_Init();
    HAL_CAN_Start(&hcan2);
    AN.errorBits.can2Fail=false;
  } else if(AN.errorBits.spiFail){
    MX_SPI1_Init();
    AN.errorBits.spiFail=false;
  }
  if(AN.errorBits.can1Fail||AN.errorBits.can2Fail||AN.errorBits.spiFail){
    return SOMETINGWONG;
  } else{
    return NOMINAL;
  }
}

/**
 * This function calls the corresponding function depending on the state of AnaCAN. If the state
 * is NOMINAL, this function calls AN_nominalState. If state is SOMETINGWONG, this function calls
 * AN_sometingWongState.
 */
void AN_executeState(){
  static const struct{
    anacan_state (*process)();
  } states[NUM_STATES] =
  {
    [NOMINAL] = {&AN_nominalState},
    [SOMETINGWONG] = {&AN_sometingWongState}
  };
  AN.state = (states[AN.state].process());
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
  uint32_t filteredRearCnt = 0;
  uint32_t filteredFrontCnt = 0;
  if (htim == &htim4){
    AN_shouldExecuteSM = true;
    filteredRearCnt = LP_Filter(&AN_rearFilter, AN_counterR, rearCnt*AN_TIMER_PERIOD);
    filteredFrontCnt = LP_Filter(&AN_frontFilter, AN_counterL, rearCnt*AN_TIMER_PERIOD);
  }
#if AN_DEBUG_MODE
  else if (htim == &htim2)
  {
    AN_counterArrayL[j] = AN_ZERO_SPEED;
    j++;
    if (j>=100){
      j=0;
    }
  }
  else if (htim == &htim5)
  {
    AN_counterArrayR[i] = AN_ZERO_SPEED;
    i++;
    if (i>=100){
      i=0;
    }
  }
  
#else
  else if (htim == &htim2)
  {
  	AN_counterL = AN_ZERO_SPEED;
  }
  else if (htim == &htim5)
  {
  	AN_counterR = AN_ZERO_SPEED;
  }
#endif
}

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim) {
	if (htim == &htim5){
#if AN_DEBUG_MODE
	AN_counterArrayR[i++]=__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_4);
	if (i>=100){
		i=0;
	}
#else
	AN_counterR=__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_4);
#endif
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	}
	if (htim == &htim2){	//Front wheel
#if AN_DEBUG_MODE
	AN_counterArrayL[j++]=__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	if (j>=100){
		j=0;
	}
#else
	AN_counterL=__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
#endif
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
