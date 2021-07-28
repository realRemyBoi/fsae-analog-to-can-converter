/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
// 1 = right side, 2 = left side.
#define AN_V_REF 2.5F	//[V]
#define AN_RX_SPI_DATA_AUTO 0b0010100001000000
#define AN_RX_SPI_DATA_MANUAL 0b0001101001000000
#define AN_RX_SPI_ROTOR1 0b0001101111000000                          // SEN1, CH7
#define AN_RX_SPI_CALIPER_TEMP1 0b0001101101000000                   // SEN2, CH6
#define AN_RX_SPI_RADIATOR_GEARBOX_TEMP1 0b0001101001000000          // SEN3, CH4 Front: Radiator, Rear: Gearbox
#define AN_RX_SPI_RADIATOR_TEMP2_RESERVE 0b0001100111000000          // SEN4, CH3 Front: Radiator, Rear: Reserve
#define AN_RX_SPI_SHOCK_POT1 0b0001100101000000                      // SEN5, CH2
#define AN_RX_SPI_PITOT_TUBE_RESERVE 0b0001101011000000              // SEN6, CH5 Front: Pitot Tube, Rear: Reserve
#define AN_RX_SPI_ROTOR2 0b0001111001000000                          // SEN7, CH12
#define AN_RX_SPI_CALIPER_TEMP2 0b0001110111000000                   // SEN8, CH11
#define AN_RX_SPI_RADIATOR_TEMP3_GEARBOX_TEMP_2 0b0001110101000000   // SEN9, CH10 Front: Radiator, Rear: Gearbox
#define AN_RX_SPI_RADIATOR_TEMP4_RESERVE 0b0001110011000000          // SEN10, CH9 Front: Radiator, Rear: Reserve
#define AN_RX_SPI_SHOCK_POT2_RESERVE 0b0001110001000000				 // SEN11, CH8 Front: Shock Pot, Rear: Reserve
																	 // CAN works for ALL sensors as of 06/02/21
                                                                     // Shockpot: Brown should be grounded and blue should be 5v
#define TWELVE_2_SIXTEEN (65536.0F/4096.0F)

void SPI_rFastPrepare(uint8_t dataTx[]);
void SPI_lFastPrepare(uint8_t dataTx[]);

void SPI_rSlowPrepare(uint8_t dataTx[]);
void SPI_lSlowPrepare(uint8_t dataTx[]);

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    /**SPI3 GPIO Configuration
    PB0     ------> SPI3_MOSI
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PB0     ------> SPI3_MOSI
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// Obtains data from external ADC via SPI
uint16_t SPI_receive(uint16_t mode){

    uint8_t spiSend[2];
    spiSend[0] = mode >> 8;
    spiSend[1] = mode & 0xFF;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, spiSend, 2, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    for (uint16_t i = 0; i < 500; i++) {}
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, spiSend, 2, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    for (uint16_t i = 0; i < 500; i++) {}
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi1, spiSend, 2, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return (spiSend[0] << 8) | (spiSend[1] & 0xFF);
}

void SPI_rFastPrepare(uint8_t dataTx[]){
  uint16_t spiTxBank[2]={0,0};
  // Shock pots, pitot tube
  // right is sen1-6
  spiTxBank[0] = SPI_receive(AN_RX_SPI_SHOCK_POT1);
  spiTxBank[1] = SPI_receive(AN_RX_SPI_PITOT_TUBE_RESERVE);
  uint16_t volatile adcDataOne = (spiTxBank[0] & 0xFFF) * TWELVE_2_SIXTEEN;
  uint16_t volatile adcDataTwo = (spiTxBank[1] & 0xFFF) * TWELVE_2_SIXTEEN;
  dataTx[2] = adcDataOne >> 8;
  dataTx[3] = adcDataOne & 0xFF;
  dataTx[4] = adcDataTwo >> 8;
  dataTx[5] = adcDataTwo & 0xFF;
}

void SPI_lFastPrepare(uint8_t dataTx[]){
  uint16_t spiTxBank[2]={0,0};
  spiTxBank[0] = SPI_receive(AN_RX_SPI_SHOCK_POT2_RESERVE);
  uint16_t adcDataOne = (spiTxBank[0] & 0xFFF) * TWELVE_2_SIXTEEN;
  uint16_t adcDataTwo = (spiTxBank[1] & 0xFFF) * TWELVE_2_SIXTEEN;
  dataTx[2] = adcDataOne >> 8;
  dataTx[3] = adcDataOne & 0xFF;
  dataTx[4] = adcDataTwo >> 8;
  dataTx[5] = adcDataTwo & 0xFF;
}

void SPI_rSlowPrepare(uint8_t dataTx[]){
  uint16_t spiTxBank[2]={0,0};
  spiTxBank[0] = SPI_receive(AN_RX_SPI_ROTOR1);
  spiTxBank[1] = SPI_receive(AN_RX_SPI_CALIPER_TEMP1);
  uint16_t adcDataOne = (spiTxBank[0] & 0xFFF) * TWELVE_2_SIXTEEN;
  uint16_t adcDataTwo = (spiTxBank[1] & 0xFFF) * TWELVE_2_SIXTEEN;
  dataTx[0] = adcDataOne >> 8;
  dataTx[1] = adcDataOne & 0xFF;
  dataTx[2] = adcDataTwo >> 8;
  dataTx[3] = adcDataTwo & 0xFF;
  spiTxBank[0] = SPI_receive(AN_RX_SPI_RADIATOR_GEARBOX_TEMP1);
  spiTxBank[1] = SPI_receive(AN_RX_SPI_RADIATOR_TEMP2_RESERVE);
  uint16_t adcDataThree = (spiTxBank[0] & 0xFFF) * TWELVE_2_SIXTEEN;
  uint16_t adcDataFour = (spiTxBank[1] & 0xFFF) * TWELVE_2_SIXTEEN;
  dataTx[4] = adcDataThree >> 8;
  dataTx[5] = adcDataThree & 0xFF;
  dataTx[6] = adcDataFour >> 8;
  dataTx[7] = adcDataFour & 0xFF;
}

void SPI_lSlowPrepare(uint8_t dataTx[]){
  uint16_t spiTxBank[2]={0,0};
  spiTxBank[0] = SPI_receive(AN_RX_SPI_ROTOR2);
  spiTxBank[1] = SPI_receive(AN_RX_SPI_CALIPER_TEMP2);
  uint16_t adcDataOne = (spiTxBank[0] & 0xFFF) * TWELVE_2_SIXTEEN;
  uint16_t adcDataTwo = (spiTxBank[1] & 0xFFF) * TWELVE_2_SIXTEEN;
  dataTx[0] = adcDataOne >> 8;
  dataTx[1] = adcDataOne & 0xFF;
  dataTx[2] = adcDataTwo >> 8;
  dataTx[3] = adcDataTwo & 0xFF;
  spiTxBank[0] = SPI_receive(AN_RX_SPI_RADIATOR_TEMP3_GEARBOX_TEMP_2);
  spiTxBank[1] = SPI_receive(AN_RX_SPI_RADIATOR_TEMP4_RESERVE);
  uint16_t adcDataThree = (spiTxBank[0] & 0xFFF) * TWELVE_2_SIXTEEN;
  uint16_t adcDataFour = (spiTxBank[1] & 0xFFF) * TWELVE_2_SIXTEEN;
  dataTx[4] = adcDataThree >> 8;
  dataTx[5] = adcDataThree & 0xFF;
  dataTx[6] = adcDataFour >> 8;
  dataTx[7] = adcDataFour & 0xFF;
}

// Lights up an LED on the ANACAN PCB if SPI_commRegisterTrue returns as 0x14, which
// verifies that the SPI lines are working and the strain-gauge amp is talking to the
// STM32.
void SPI_commRegisterTrue(void){
	int correctRegister = SPI_commRegister();
	if (correctRegister == 0x14)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}

// Reads from the AD7124-8 strain gauge amp ID register. Read value should be 0x14.
// If this function returns 0x14, it verifies that the SPI lines between the STM
// and the AD7124-8 are working. 0x14 is the designated return value according to the
// AD7124-8's datasheet.
uint16_t SPI_commRegister() {
	uint16_t newData;
	uint8_t rxData[1];
	rxData[0] = 0x45; // Address of communications register to read from
	rxData[1] = 0xFF;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Sets Chip Select pin low
	HAL_SPI_Receive(&hspi3, rxData, 2, 100); // Read from Communications register
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	newData = rxData[0] >> 8;
	newData = newData + rxData[1];
	return newData;
}

// Writes to registers of AD7124-8 that require 16-bit commands. First parameter
// indicates address of that register.
void SPI_write16(uint8_t address, uint16_t command) {
	uint8_t txData[5];
	txData[0] = address;
	txData[1] = (command & 0xFF00) >> 8;
	txData[2] = command & 0x00FF;
	txData[3] = 0xFF;
	txData[4] = 0xFF;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Sets Chip Select pin low
	HAL_SPI_Transmit(&hspi3, txData, 5, 100); // Transmit data
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // Sets CS pin high
}

// Writes to registers of AD7124-8 that require 24-bit commands. First parameter
// indicates address of that register.
void SPI_write24(uint8_t address, uint32_t command) {
	uint8_t txData[7];
	txData[0] = address;
	txData[1] = (command & 0xFF0000) >> 16;
	txData[2] = (command & 0x00FF00) >> 8;
	txData[3] = command & 0x0000FF;
	txData[4] = 0xFF;
	txData[5] = 0xFF;
	txData[6] = 0xFF;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Sets Chip Select pin low
	HAL_SPI_Transmit(&hspi3, txData, 7, 100); // Transmit data
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // Sets CS pin high
}

/* Actually reads and returns the ADC values from the data register.
*  The inital ADC value has a 24-bit resolution, so it has to be divided by 2^24
*  or 0x1000000, then multiplied by the chip's voltage reference, and then divided by
*  the chosen ADC gain. This will yield the voltage differential of the strain gauge amplifier.
*  For now, this calculation is commented out, and the raw ADC value is returned instead.
*/
uint32_t SPI_dataRead(int address, int gain) {
	uint8_t rxData[4];
	rxData[0] = address;
	rxData[1] = 0xFF;
	rxData[2] = 0xFF;
	rxData[3] = 0xFF;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi3, rxData, 4, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	int data = (rxData[1] << 16) | rxData[2] << 8 | rxData[3];
	//double ADC_value = ((data / 0x1000000) * V_REF) / gain;
	return data;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
