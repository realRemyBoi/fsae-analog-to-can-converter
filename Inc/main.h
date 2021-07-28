/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "bool.h"
#define NUM_STATES 2

#define AN_PI 3.141593F     //0b5432109876543210
#define AN_WHEEL_RADIUS 0.2286F //[m]
#define AN_MPHTOMS  2.2369F //[m/s]

#define AN_PRESCALER 10.0F
#define AN_CLK_FREQUENCY 50000000.0F  //[Hz]
#define AN_SPEED_FREQUENCY 1000.0F  //[Hz]
#define AN_SPEED_PERIOD (1.0F/AN_SPEED_FREQUENCY) //[s]
#define AN_SPEED_CONVERSION_MS(count, slits) (2.0F*AN_PI*AN_WHEEL_RADIUS/(slits*(count*AN_TIMER_PERIOD))) //[m/s]
#define AN_STATUS_SIZE 4
#define AN_DEBUG_MODE false
#define AN_USE_INTERNAL_CLOCK false
#define AN_MS_CODE_CONVERSION 40.0F/65535.0F
#define FRONT_WHEEL_SLITS 60.0F
#define REAR_WHEEL_SLITS 65.0F
#define FRONT true

typedef enum anacan_states{
	NOMINAL,
	SOMETINGWONG

} anacan_state;

typedef struct s_anacan{
	anacan_state state;
	union{
		struct{
			bool can1Fail:1;
			bool can2Fail:1;
			bool spiFail:1;
		} errorBits;
//		uint8_t allErrorState;
	};
} anacan_s;

anacan_s AN;


void AN_peripheralStartup();
void AN_updateStatus();
void AN_executeState();
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
