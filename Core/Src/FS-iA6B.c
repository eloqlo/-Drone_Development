/*
 * FS-iA6B.c
 *
 *  Created on: Jul 30, 2025
 *      Author: jh
 */
#include "FS-iA6B.h"

FSiA6B_iBus iBus;

// Check Sum 계산을 해주는 함수.
unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len){
	unsigned short chksum = 0xffff;
	for (int i=0; i<len-2; i++){
		chksum -= data[i];
	}

	return ((chksum & 0x00ff)==data[30]) && ((chksum>>8)==data[31]);
}

// data를 Parsing 해서 결과를 ibus 구조체에 저장해주는 함수
void iBus_Parsing(unsigned char* data, FSiA6B_iBus* ibus){
	ibus->RH = (data[2] | data[3]<<8) & 0x0fff;		// Right Horizontal : 하위 12비트가 조종기 조작량에 대한 데이터이다.
	ibus->RV = (data[4] | data[5]<<8) & 0x0fff;		// Right Vertical
	ibus->LV = (data[6] | data[7]<<8) & 0x0fff;
	ibus->LH = (data[8] | data[9]<<8) & 0x0fff;
	ibus->SwA = (data[10] | data[11]<<8) & 0x0fff;
	ibus->SwC = (data[12] | data[13]<<8) & 0x0fff;

	ibus->FailSafe = (data[13] >> 4);				// 6번 ch의 상위 4비트만 사용. SwC꺼
}

/* Fail-safe 가 활성화 되었는지 검사한다. */
unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus){
	return iBus->FailSafe != 0;
}

void FSiA6B_UART5_Init(void){
	/*CubeMX로 생성된 UART5_Init 함수*/
	  /* USER CODE BEGIN UART5_Init 0 */

	  /* USER CODE END UART5_Init 0 */

	  LL_USART_InitTypeDef USART_InitStruct = {0};

	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* Peripheral clock enable */
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	  /**UART5 GPIO Configuration
	  PC12   ------> UART5_TX
	  PD2   ------> UART5_RX
	  */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /* UART5 interrupt Init */
	  NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	  NVIC_EnableIRQ(UART5_IRQn);

	  /* USER CODE BEGIN UART5_Init 1 */

	  /* USER CODE END UART5_Init 1 */
	  USART_InitStruct.BaudRate = 115200;
	  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	  LL_USART_Init(UART5, &USART_InitStruct);
	  LL_USART_ConfigAsyncMode(UART5);
	  LL_USART_Enable(UART5);
	  /* USER CODE BEGIN UART5_Init 2 */

	  /* USER CODE END UART5_Init 2 */
}
