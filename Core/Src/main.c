/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno080.h"
#include "quaternion.h"
#include "icm20602.h"
#include "LPS22HH.h"
#include "M8N.h"`
#include "FS-iA6B.h"
#include "AT24C08.h"
#include <string.h>
#include "pid_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char* p, int len)
{
	for(int i=0; i<len; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART6));
		LL_USART_TransmitData8(USART6, *(p+i));
	}
	return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern uint8_t uart6_rx_flag;		// 8bit int (0~255) | extern: 외부 파일의 전역변수를 사용하겠다.
extern uint8_t uart6_rx_data;

extern uint8_t m8n_rx_buf[36];		// message frame 크기와 같은 36byte
extern uint8_t m8n_rx_cplt_flag;

extern uint8_t ibus_rx_buf[36];		// message frame 크기와 같은 36byte
extern uint8_t ibus_rx_cplt_flag;

extern uint8_t uart1_rx_data;

/* 타이머 */
extern uint8_t tim7_1ms_flag;
extern uint8_t tim7_20ms_flag;
extern uint8_t tim7_100ms_flag;
extern uint8_t tim7_1000ms_flag;

/* telemetry 버퍼, 플레그 */
uint8_t telemetry_tx_buf[40];
uint8_t telemetry_rx_buf[20];
uint8_t telemetry_rx_cplt_flag;
uint8_t low_bat_flag;

/* 배터리 ADC 값 */
float batVolt;


unsigned char failsafe_flag = 0;
/*
 * uart6_rx_flag: USART6_IRQHandler 에서, 데이터를 받았다는 표식.
 * uart6_rx_data: 받아진 8bit 데이터. 저장됨 (char)
 * */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
int Is_iBus_Throttle_Min(void);
void ESC_Calibartion(void);
int Is_iBus_Received(void);
void BNO080_Calibration(void);
void Encode_Msg_GPS(unsigned char* telemetry_tx_buf);
void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf);
void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d);
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
	float q[4];
	float quatRadianAcc;
	unsigned short adcVal;
	short gyro_x_offset = -5, gyro_y_offset = 25, gyro_z_offset = -2;
	unsigned char motor_arming_flag = 0;
	unsigned short iBus_SwA_Prev = 0;
	unsigned char iBus_rx_cnt = 0;
	unsigned short ccr1, ccr2, ccr3, ccr4;

	float yaw_haeding_reference;
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
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();	// BNO080
  MX_SPI1_Init();	// ICM20602
  MX_SPI3_Init();	// LPS22HH
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

	/* 장치 초기화 START */
	LL_TIM_EnableCounter(TIM3);				// Buzzer

	LL_USART_EnableIT_RXNE(USART6);			// Debug UART
	LL_USART_EnableIT_RXNE(UART4);			// GPS
	LL_USART_EnableIT_RXNE(UART5);			// FS-iA6B

	LL_TIM_EnableCounter(TIM5);								// Motor PWM : TIM5 활성화.
	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);		// PWM 4개 활성화.
	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);

	HAL_ADC_Start_DMA(&hadc1, &adcVal, 1);	// Battery ADC:  ADC 구조체 주소, ADC 결과 저장할 변수 주소, DMA로 복사할 데이터 개수.

	HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);		// Telemetry:  uart1_rx_data에 1byte 수신되면, 수신완료 callback 함수 호출한다.

	LL_TIM_EnableCounter(TIM7);		// TIM7 활성화.
	LL_TIM_EnableIT_UPDATE(TIM7);		// TIM7 업데이트 인터럽트 --> 1ms 마다 타이머 인터럽트 반복적으로 호출됨.

	// 여기까지 안감.
	printf("Checking Sensor connection..\n\n");

	/* 센서 초기화 */
	if (BNO080_Initialization() != 0){	// SPI2에 대한 초기화, bno080 센서 내부에 대한 초기화.
		printf("\n[!] BNO080 failed. Program Shutting down.");
		while(1){
			LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			TIM3->PSC = 1000;
			HAL_Delay(50);
			TIM3->PSC = 1500;
			HAL_Delay(50);
			LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		}
	}
//	BNO080_enableRotationVector(2500);		// 2500us 마다 센서의 rotation 데이터를 읽어오겠다. (Mag 사용)
	BNO080_enableGameRotationVector(2500);	// 2500us 마다 센서 갱신! (Mag 미사용)

	if (ICM20602_Initialization() != 0){
		printf("\n[!] ICM-20602 failed. Program Shutting dowm...");
		while(1){
			LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			TIM3->PSC = 1000;
			HAL_Delay(50);
			TIM3->PSC = 1500;
			HAL_Delay(50);
			LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		}
	}
	if (LPS22HH_Initialization() != 0){
		printf("\n[!] LPS22HH failed. Program shutting down.");
		while(1){
			LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			TIM3->PSC = 1000;
			HAL_Delay(50);
			TIM3->PSC = 1500;
			HAL_Delay(50);
			LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		}
	}
	M8N_Initialization();

	printf("All Sensors OK!\n\n");

	/* 자이로센서 offset 제거 */
	ICM20602_Writebyte(0x13, (gyro_x_offset * -2)>>8);
	ICM20602_Writebyte(0x14, (gyro_x_offset * -2));
	ICM20602_Writebyte(0x15, (gyro_y_offset * -2)>>8);
	ICM20602_Writebyte(0x16, (gyro_y_offset * -2));
	ICM20602_Writebyte(0x17, (gyro_z_offset * -2)>>8);
	ICM20602_Writebyte(0x18, (gyro_z_offset * -2));

	printf("Loading PID Gain....\n\n");
	/* EEPROM에서 Gain 읽어와서 GCS로 보내주는 부분 */
	// Roll(inner, outer)/Pitch(inner, outer)/Yaw(각도, 각속도)의 PID Gain을 EEPROM에서 읽어와서
	if (EP_PIDGain_Read(0, &roll.in.kp, &roll.in.ki, &roll.in.kd) != 0||
			EP_PIDGain_Read(1, &roll.out.kp, &roll.out.ki, &roll.out.kd) != 0||
			EP_PIDGain_Read(2, &pitch.in.kp, &pitch.in.ki, &pitch.in.kd) != 0||
			EP_PIDGain_Read(3, &pitch.out.kp, &pitch.out.ki, &pitch.out.kd) != 0||
			EP_PIDGain_Read(4, &yaw_heading.kp, &yaw_heading.ki, &yaw_heading.kd) != 0||
			EP_PIDGain_Read(5, &yaw_rate.kp, &yaw_rate.ki, &yaw_rate.kd) != 0)
	{
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

		TIM3->PSC = 1000;
		HAL_Delay(50);
		TIM3->PSC = 1500;
		HAL_Delay(50);
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

		HAL_Delay(500);
		printf("\n[!] Couldn't load PID Gain.\n");
	}
	else
	{
		// Roll
		Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0, roll.in.kp, roll.in.ki, roll.in.kd);	// telemetry protocol로 변경해서
		HAL_UART_Transmit(&huart1, &telemetry_tx_buf, 20, 10);		// Telemetry로 보낸다.
		Encode_Msg_PID_Gain(&telemetry_tx_buf[1], 1, roll.out.kp, roll.out.ki, roll.out.kd);
		HAL_UART_Transmit(&huart1, &telemetry_tx_buf, 20, 10);
		// Pitch
		Encode_Msg_PID_Gain(&telemetry_tx_buf[2], 2, pitch.in.kp, pitch.in.ki, pitch.in.kd);	// telemetry protocol로 변경해서
		HAL_UART_Transmit(&huart1, &telemetry_tx_buf, 20, 10);		// Telemetry로 보낸다.
		Encode_Msg_PID_Gain(&telemetry_tx_buf[3], 3, pitch.out.kp, pitch.out.ki, pitch.out.kd);
		HAL_UART_Transmit(&huart1, &telemetry_tx_buf, 20, 10);
		// Yaw
		Encode_Msg_PID_Gain(&telemetry_tx_buf[4], 4, yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);	// telemetry protocol로 변경해서
		HAL_UART_Transmit(&huart1, &telemetry_tx_buf, 20, 10);		// Telemetry로 보낸다.
		Encode_Msg_PID_Gain(&telemetry_tx_buf[5], 5, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
		HAL_UART_Transmit(&huart1, &telemetry_tx_buf, 20, 10);
		printf("\nAll Gains OK!\n\n");
	}


  /* Wait until Connected with Controller */
  while(Is_iBus_Received() == 0){
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		TIM3->PSC = 3000;
		HAL_Delay(100);
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		HAL_Delay(500);	// -  -  -  -  -
  }

  /* SwC Lower Position --> ESC Calibration */
  if (iBus.SwC == 2000){
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  TIM3->PSC = 1500;
	  HAL_Delay(200);
	  TIM3->PSC = 2000;
	  HAL_Delay(200);
	  TIM3->PSC = 1500;
	  HAL_Delay(200);
	  TIM3->PSC = 2000;
	  HAL_Delay(200);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  ESC_Calibartion();
	  // Wait until Upper SwC --> Start Flight
	  while (iBus.SwC != 1000){
		  Is_iBus_Received();
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  TIM3->PSC = 1500;
		  HAL_Delay(200);
		  TIM3->PSC = 2000;
		  HAL_Delay(200);
		  TIM3->PSC = 1500;
		  HAL_Delay(200);
		  TIM3->PSC = 2000;
		  HAL_Delay(200);
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }
  }
  /* SwC Mid Position --> BNO080 Calibration */
  else if (iBus.SwC == 1500){
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  TIM3->PSC = 1500;
	  HAL_Delay(200);
	  TIM3->PSC = 2000;
	  HAL_Delay(200);
	  TIM3->PSC = 1500;
	  HAL_Delay(200);
	  TIM3->PSC = 2000;
	  HAL_Delay(200);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  BNO080_Calibration();
	  // Wait until Upper SwC --> Start Flight
	  while (iBus.SwC != 1000){
		  Is_iBus_Received();
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  TIM3->PSC = 1500;
		  HAL_Delay(200);
		  TIM3->PSC = 2000;
		  HAL_Delay(200);
		  TIM3->PSC = 1500;
		  HAL_Delay(200);
		  TIM3->PSC = 2000;
		  HAL_Delay(200);
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }
  }

  /* Throttle 내려가있는지 + SwA 내려가있는지 검사 */
  while (Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000){
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		TIM3->PSC = 1000;
		HAL_Delay(50);
		TIM3->PSC = 1500;
		HAL_Delay(50);
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		HAL_Delay(500);
  }


  /* Start Sound */
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  TIM3->PSC = 2000;
  HAL_Delay(300);
  TIM3->PSC = 1500;
  HAL_Delay(300);
  TIM3->PSC = 1000;
  HAL_Delay(300);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  printf("Start!\n\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* 1ms 마다 - PID계산 후 모터 CCR값으로 변환 */
	  if (tim7_1ms_flag == 1)
	  {
		  tim7_1ms_flag = 0;

		  /* Pitch PID 계산 결과가 pitch 구조체에 들어간다. */
		  Double_Roll_Pitch_PID_Calculation(&pitch, (iBus.RV - 1500) * 0.1f, BNO080_Pitch, ICM20602.gyro_x);

		  /* Roll 계산 */
		  Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH - 1500) * 0.1f, BNO080_Roll, ICM20602.gyro_y);

		  /* 오차 누적 방지 */
		  if(iBus.LV < 1030 || motor_arming_flag == 0){
			  Reset_All_PID_Integrator();
		  }

		  /* 좁은 각도는 각도제어, 넓은 각도는 각속도 제어로 control */
		  if (iBus.LH < 1485 || iBus.LH > 1515)
		  {
			  /* Yaw 각속도 제어 결과가 yaw_rate 구조체 변수에 저장된다. */
			  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH - 1500), ICM20602.gyro_z);

			  // Throttle + Pitch(PID) + Roll + Yaw
			  ccr1 = 10500 + 500 + (iBus.LV - 1000)*10 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result;
			  ccr2 = 10500 + 500 + (iBus.LV - 1000)*10 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
			  ccr3 = 10500 + 500 + (iBus.LV - 1000)*10 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result;
			  ccr4 = 10500 + 500 + (iBus.LV - 1000)*10 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
		  }
		  else
		  {
			  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_haeding_reference, BNO080_Yaw, ICM20602.gyro_z);

			  ccr1 = 10500 + 500 + (iBus.LV - 1000)*10 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result;
			  ccr2 = 10500 + 500 + (iBus.LV - 1000)*10 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
			  ccr3 = 10500 + 500 + (iBus.LV - 1000)*10 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result;
			  ccr4 = 10500 + 500 + (iBus.LV - 1000)*10 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
		  }
//		  printf("%f\t %f\n", BNO080_Pitch, ICM20602.gyro_x);	// -
//		  printf("%f\t %f\n", BNO080_Roll, ICM20602.gyro_y);	// -
//		  printf("%f\t %f\n\n", BNO080_Yaw, ICM20602.gyro_z);		// +
	  }

	  /* Motor 구동
	   * SwA 내리면 Arming 된다.
	   * */
	  if (iBus.SwA == 2000 && iBus_SwA_Prev != 2000){
		  if (iBus.LV < 1010){
			  motor_arming_flag = 1;
			  /*Yaw 목표각도 설정 부분*/
			  yaw_haeding_reference = BNO080_Yaw;
		  }
		  else{
			  /* Throttle 내려가있는지 + SwA 내려가있는지 검사 */
			  while (Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000){
				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  TIM3->PSC = 1000;
				  HAL_Delay(70);
				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  HAL_Delay(70);
			  }
		  }
	  }
	  iBus_SwA_Prev = iBus.SwA;

	  if(iBus.SwA != 2000){
		  motor_arming_flag = 0;
	  }

	  if(motor_arming_flag == 1){
		  if (failsafe_flag == 0 && iBus.LV > 1030)
		  {
			  TIM5->CCR1 = ccr1 > 21000 ? 21000 : ccr1 < 11000 ? 11000 : ccr1;
			  TIM5->CCR2 = ccr2 > 21000 ? 21000 : ccr2 < 11000 ? 11000 : ccr2;
			  TIM5->CCR3 = ccr3 > 21000 ? 21000 : ccr3 < 11000 ? 11000 : ccr3;
			  TIM5->CCR4 = ccr4 > 21000 ? 21000 : ccr4 < 11000 ? 11000 : ccr4;
		  }
		  else
		  {
			  // 모터 정지
			  TIM5->CCR1 = 11000;
			  TIM5->CCR2 = 11000;
			  TIM5->CCR3 = 11000;
			  TIM5->CCR4 = 11000;
		  }
	  }
	  else
	  {
		  // 모터 정지
		  TIM5->CCR1 = 10500;
		  TIM5->CCR2 = 10500;
		  TIM5->CCR3 = 10500;
		  TIM5->CCR4 = 10500;
	  }

	  /* Telemetry : GCS --> FC PID 게인 설정 메시지. */
	  /* 1. IRQ Cplt Callback 으로 메시지 20byte 만큼 수신완료.
	   *  --> telemetry_rx_buf[20] 채워짐
	   *  --> telemetry_rx_cplt_flag = 1; */
	  if (telemetry_rx_cplt_flag == 1){
		  telemetry_rx_cplt_flag = 0;

		  if (iBus.SwA == 1000){
			  /* 2. 체크섬 검사 */
			  unsigned char chksum = 0xff;
			  for (int i=0; i<19;i++) {
				  chksum = chksum - telemetry_rx_buf[i];
			  }

			  /* 3. MSG ID 검사(0~5) */
			  if (chksum == telemetry_rx_buf[19]){
				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

				  TIM3->PSC = 1000;
				  HAL_Delay(10);

				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

				  switch(telemetry_rx_buf[2]){
				  /*
				   * 0: GCS에서 받은 PID Gain은 Roll_in 꺼 / EEPROM에 Gain 저장하고 / Telemetry로 저장된 값 전송.
				   * ...
				   * */
				  case 0:
					  /* 4. 받은 Gain Parsing */
					  roll.in.kp = *(float*)&telemetry_rx_buf[3];	// buf에 저장된 값을 float로 읽어오기
					  roll.in.ki = *(float*)&telemetry_rx_buf[7];
					  roll.in.kd = *(float*)&telemetry_rx_buf[11];
					  /* 5. EEPROM에 Gain 저장 */
					  EP_PIDGain_Write(telemetry_rx_buf[2], roll.in.kp, roll.in.ki, roll.in.kd);
					  /* 6. EEPROM에서 Gain 로드 --> kp, ki, kd */
					  EP_PIDGain_Read(telemetry_rx_buf[2], &roll.in.kp, &roll.in.ki, &roll.in.kd);
					  /* 7. PID값 tx_buffer에 저장, Telemetry로 전송 */
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], roll.in.kp, roll.in.ki, roll.in.kd);
					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf, 20);
					  break;
				  case 1:
					  roll.out.kp = *(float*)&telemetry_rx_buf[3];
					  roll.out.ki = *(float*)&telemetry_rx_buf[7];
					  roll.out.kd = *(float*)&telemetry_rx_buf[11];
					  EP_PIDGain_Write(telemetry_rx_buf[2], roll.out.kp, roll.out.ki, roll.out.kd);
					  EP_PIDGain_Read(telemetry_rx_buf[2], &roll.out.kp, &roll.out.ki, &roll.out.kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], roll.out.kp, roll.out.ki, roll.out.kd);
					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf, 20);
					  break;
				  case 2:
					  pitch.in.kp = *(float*)&telemetry_rx_buf[3];
					  pitch.in.ki = *(float*)&telemetry_rx_buf[7];
					  pitch.in.kd = *(float*)&telemetry_rx_buf[11];
					  EP_PIDGain_Write(telemetry_rx_buf[2], pitch.in.kp, pitch.in.ki, pitch.in.kd);
					  EP_PIDGain_Read(telemetry_rx_buf[2], &pitch.in.kp, &pitch.in.ki, &pitch.in.kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], pitch.in.kp, pitch.in.ki, pitch.in.kd);
					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf, 20);
					  break;
				  case 3:
					  pitch.out.kp = *(float*)&telemetry_rx_buf[3];
					  pitch.out.ki = *(float*)&telemetry_rx_buf[7];
					  pitch.out.kd = *(float*)&telemetry_rx_buf[11];
					  EP_PIDGain_Write(telemetry_rx_buf[2], pitch.out.kp, pitch.out.ki, pitch.out.kd);
					  EP_PIDGain_Read(telemetry_rx_buf[2], &pitch.out.kp, &pitch.out.ki, &pitch.out.kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], pitch.out.kp, pitch.out.ki, pitch.out.kd);
					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf, 20);
					  break;
				  case 4:
					  yaw_heading.kp = *(float*)&telemetry_rx_buf[3];
					  yaw_heading.ki = *(float*)&telemetry_rx_buf[7];
					  yaw_heading.kd = *(float*)&telemetry_rx_buf[11];
					  EP_PIDGain_Write(telemetry_rx_buf[2], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
					  EP_PIDGain_Read(telemetry_rx_buf[2], &yaw_heading.kp, &yaw_heading.ki, &yaw_heading.kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf, 20);
					  break;
				  case 5:
					  yaw_rate.kp = *(float*)&telemetry_rx_buf[3];
					  yaw_rate.ki = *(float*)&telemetry_rx_buf[7];
					  yaw_rate.kd = *(float*)&telemetry_rx_buf[11];
					  EP_PIDGain_Write(telemetry_rx_buf[2], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
					  EP_PIDGain_Read(telemetry_rx_buf[2], &yaw_rate.kp, &yaw_rate.ki, &yaw_rate.kd);
					  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_rx_buf[2], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
					  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf, 20);
					  break;
					  /* 0x10 : Gain 요청 */
				  case 0x10:
					  switch(telemetry_rx_buf[3]){
					  case 0:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_tx_buf[3], roll.in.kp, roll.in.ki, roll.in.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 1:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_tx_buf[3], roll.out.kp, roll.out.ki, roll.out.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 2:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_tx_buf[3], pitch.in.kp, pitch.in.ki, pitch.in.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 3:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_tx_buf[3], pitch.out.kp, pitch.out.ki, pitch.out.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 4:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_tx_buf[3], yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 5:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], telemetry_tx_buf[3], yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  break;
					  case 6:
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 0, roll.in.kp, roll.in.ki, roll.in.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 1, roll.out.kp, roll.out.ki, roll.out.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 2, pitch.in.kp, pitch.in.ki, pitch.in.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 3, pitch.out.kp, pitch.out.ki, pitch.out.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 4, yaw_heading.kp, yaw_heading.ki, yaw_heading.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  Encode_Msg_PID_Gain(&telemetry_tx_buf[0], 5, yaw_rate.kp, yaw_rate.ki, yaw_rate.kd);
						  HAL_UART_Transmit(&huart1, &telemetry_tx_buf[0], 20, 10);
						  break;
					  }
					  break;
				  }
			  }
		  }

	  }
	  /* [END] Telemetry : GCS --> FC PID 게인 설정 메시지. */

	  /* Telemetry : 20ms(50Hz) TASK: Roll Pitch Yaw, 고도, FS-i6 스틱 데이터 */
	  if (tim7_20ms_flag == 1 && tim7_100ms_flag != 1){
		  tim7_20ms_flag = 0;

		  Encode_Msg_AHRS(&telemetry_tx_buf[0]);

		  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);		// 20ms(50Hz)동안 함수가 완료되지 않으면 함수를 강제종료한다.

	  }

	  if (tim7_20ms_flag == 1 && tim7_100ms_flag == 1){
		  tim7_20ms_flag = 0;
		  tim7_100ms_flag = 0;

		  Encode_Msg_AHRS(&telemetry_tx_buf[0]);
		  Encode_Msg_GPS(&telemetry_tx_buf[20]);

		  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 40);		// 20ms(50Hz)동안 함수가 완료되지 않으면 함수를 강제종료한다.

	  }

	  /*[1] ADC로 받은 Battery 전압 */
	  batVolt = adcVal * 0.003619f;
	  if(batVolt < 10.0f) {
		  // 10V 미만으로 배터리 떨어지면 flag.
		  low_bat_flag = 1;
	  }
	  else{
		  low_bat_flag = 0;
	  }

	  /*[2] 9축 절대 각도(SPI) */
	  if (BNO080_dataAvailable() == 1)
	  {
		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_1);

		  q[0] = BNO080_getQuatI();
		  q[1] = BNO080_getQuatJ();
		  q[2] = BNO080_getQuatK();
		  q[3] = BNO080_getQuatReal();
		  quatRadianAcc = BNO080_getQuatRadianAccuracy();

		  Quaternion_Update(&q[0]);	// roll, pitch, yaw 값 업데이트

		  /* 부호 반전 */
		  BNO080_Roll = -BNO080_Roll;
		  BNO080_Pitch = -BNO080_Pitch;

//		   printf("%.2f\t%.2f\n", BNO080_Roll, BNO080_Pitch);
//		   printf("%.2f\n", BNO080_Yaw);
	  }

	  /*[3] 6축 각속도 센서(SPI) */
	  if (ICM20602_DataReady() == 1)
	  {
		  /* 자이로 데이터를 받아온다 */
		  ICM20602_Get3AxisGyroRawData(&ICM20602.gyro_x_raw);

		  /* 각속도 단위로 변환한다 */
		  ICM20602.gyro_x = ICM20602.gyro_x_raw * 2000.f / 32768.f;
		  ICM20602.gyro_y = ICM20602.gyro_y_raw * 2000.f / 32768.f;
		  ICM20602.gyro_z = ICM20602.gyro_z_raw * 2000.f / 32768.f;

		  /* 부호 반전 */
		  ICM20602.gyro_x = -ICM20602.gyro_x;
		  ICM20602.gyro_z = -ICM20602.gyro_z;

		  /* UART 데이터 출력 */
		  //  printf("%d,%d,%d\n", ICM20602.gyro_x_raw, ICM20602.gyro_y_raw, ICM20602.gyro_z_raw);
//		    printf("%d,%d,%d\n", (int)(ICM20602.gyro_x*100), (int)(ICM20602.gyro_y*100), (int)(ICM20602.gyro_z*100));
	  }

	  /*[4] 기압 센서: 출력도 느리고(필터), 노이즈도 많다.(SPI) */
	  if (LPS22HH_DataReady() == 1)
	  {
		  LPS22HH_GetPressure(&LPS22HH.pressure_raw);
		  LPS22HH_GetTemperature(&LPS22HH.temperature_raw);

		  LPS22HH.baroAlt = getAltitude2(LPS22HH.pressure_raw/4096.f, LPS22HH.temperature_raw/100.f);
		  // 기압이 m 단위로 baroAlt에 저장됨.
#define C 0.90f
		  LPS22HH.baroAltFilt = LPS22HH.baroAltFilt * C + LPS22HH.baroAlt*(1.0f - C);	// 1차 IIR filter

		   printf("%d,%d\n", (int)(LPS22HH.baroAlt*100), (int)(LPS22HH.baroAltFilt*100));
	  }

	  /*[5] GPS : UBX 메시지 파싱, 체크섬 검사 */
	  if (m8n_rx_cplt_flag == 1)
	  {
		  m8n_rx_cplt_flag = 0;
		  if(M8N_UBX_CHKSUM_Check(&m8n_rx_buf[0], 36) == 1)
		  {
			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
			  M8N_UBX_NAV_POSLLH_Parsing(&m8n_rx_buf[0], &posllh);

			  // printf("LAT: %ld\tLON: %ld\tHeight: %ld\n", posllh.lat, posllh.lon, posllh.height);
		  }
	  }

	  /* [6] 조종기 연결 검사 */
	  if(ibus_rx_cplt_flag == 1){
		  ibus_rx_cplt_flag = 0;
		  if (iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1){
			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);

			  iBus_Parsing(&ibus_rx_buf[0], &iBus);	// ibus_rx_buf 에 담긴 정보를, iBus 구조체에 Parsing 해준다.
			  iBus_rx_cnt++;	// ?
			  if (iBus_isActiveFailsafe(&iBus) == 1){	// 만일 Failsafe
//				  HAL_GPIO_WritePin(GPIOC, LL_GPIO_PIN_0, GPIO_PIN_SET);
				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  failsafe_flag = 1;
			  }
			  else{		// 조종 잘됨
//				  HAL_GPIO_WritePin(GPIOC, LL_GPIO_PIN_0, GPIO_PIN_RESET);
				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  failsafe_flag = 0;

			  }
			  //  printf("%d\t%d\t%d\t%d\t%d\t%d\t\n",
			  //   iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwC);
			  //   HAL_Delay(30);
		  }
	  }


	  /* 1000ms동안 iBus 데이터 수신 안되었다 => Fail-safe 발동 */
//	  if(tim7_1000ms_flag = 1){
//		  tim7_1000ms_flag = 0;
//		  if(iBus_rx_cnt==0){
//			  failsafe_flag = 2;	// motor 정지
//		  }
//		  iBus_rx_cnt = 0;
//	  }

	  /* 조종기 송수신기 연결 끊김, 배터리 전압 10v 미만, SwC 내려가있음 */
//	  if(failsafe_flag == 1 || failsafe_flag == 2 || low_bat_flag == 1 || iBus.SwC == 2000){
	  if(failsafe_flag == 1 || low_bat_flag == 1 || iBus.SwC == 2000){
		  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		  printf("[Debug] failsafe_flag: %d, low_bat_flag: %d, iBus.SwC: %d\n", failsafe_flag, low_bat_flag, iBus.SwC);
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  }
	  else{
		  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		  printf("NO PROBLEM ~ \n");
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI3 GPIO Configuration
  PB3   ------> SPI3_SCK
  PB4   ------> SPI3_MISO
  PB5   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 21-LL_TIM_IC_FILTER_FDIV1_N2;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 10;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB1   ------> TIM3_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 41999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM5);
  LL_TIM_SetClockSource(TIM5, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM5, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM5);
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM5 GPIO Configuration
  PA0-WKUP   ------> TIM5_CH1
  PA1   ------> TIM5_CH2
  PA2   ------> TIM5_CH3
  PA3   ------> TIM5_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /* TIM7 interrupt Init */
  NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM7_IRQn);

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  TIM_InitStruct.Prescaler = 41999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1;
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**UART4 GPIO Configuration
  PC10   ------> UART4_TX
  PC11   ------> UART4_RX
  */
  GPIO_InitStruct.Pin = M8N_TX4_Pin|M8N_RX4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* UART4 interrupt Init */
  NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UART4_IRQn);

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART4, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(UART4);
  LL_USART_Enable(UART4);
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**USART6 GPIO Configuration
  PC6   ------> USART6_TX
  PC7   ------> USART6_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART6 interrupt Init */
  NVIC_SetPriority(USART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART6, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART6);
  LL_USART_Enable(USART6);
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_4
                          |LL_GPIO_PIN_9);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12|LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int Is_iBus_Throttle_Min(void){
	// Receiving iBus data
	if(ibus_rx_cplt_flag == 1){
		ibus_rx_cplt_flag = 0;
		if (iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1){
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			if(iBus.LV < 1010) {
				return 1;
			}
		}
	}

	return 0;
}

void ESC_Calibartion(void){
	TIM5->CCR1 = 21000;
	TIM5->CCR2 = 21000;
	TIM5->CCR3 = 21000;
	TIM5->CCR4 = 21000;
	HAL_Delay(7000);
	TIM5->CCR1 = 10500;
	TIM5->CCR2 = 10500;
	TIM5->CCR3 = 10500;
	TIM5->CCR4 = 10500;
	HAL_Delay(8000);
	HAL_GPIO_WritePin(GPIOC, LL_GPIO_PIN_0, GPIO_PIN_SET);
}

void BNO080_Calibration(void)
{
	//Resets BNO080 to disable All output
	BNO080_Initialization();

	//BNO080/BNO085 Configuration
	//Enable dynamic calibration for accelerometer, gyroscope, and magnetometer
	//Enable Game Rotation Vector output
	//Enable Magnetic Field output
	BNO080_calibrateAll(); //Turn on cal for Accel, Gyro, and Mag
	BNO080_enableGameRotationVector(20000); //Send data update every 20ms (50Hz)
	BNO080_enableMagnetometer(20000); //Send data update every 20ms (50Hz)

	//Once magnetic field is 2 or 3, run the Save DCD Now command
  	printf("Calibrating BNO080. Pull up FS-i6 SWC to end calibration and save to flash\n");
  	printf("Output in form x, y, z, in uTesla\n\n");

	//while loop for calibration procedure
	//Iterates until iBus.SwC is mid point (1500)
	//Calibration procedure should be done while this loop is in iteration.
	while(iBus.SwC == 1500)
	{
		if(BNO080_dataAvailable() == 1)
		{
			//Observing the status bit of the magnetic field output
			float x = BNO080_getMagX();
			float y = BNO080_getMagY();
			float z = BNO080_getMagZ();
			unsigned char accuracy = BNO080_getMagAccuracy();

			float quatI = BNO080_getQuatI();
			float quatJ = BNO080_getQuatJ();
			float quatK = BNO080_getQuatK();
			float quatReal = BNO080_getQuatReal();
			unsigned char sensorAccuracy = BNO080_getQuatAccuracy();

			printf("%f,%f,%f,", x, y, z);
			if (accuracy == 0) printf("Unreliable\t");
			else if (accuracy == 1) printf("Low\t");
			else if (accuracy == 2) printf("Medium\t");
			else if (accuracy == 3) printf("High\t");

			printf("\t%f,%f,%f,%f,", quatI, quatI, quatI, quatReal);
			if (sensorAccuracy == 0) printf("Unreliable\n");
			else if (sensorAccuracy == 1) printf("Low\n");
			else if (sensorAccuracy == 2) printf("Medium\n");
			else if (sensorAccuracy == 3) printf("High\n");

			//Turn the LED and buzzer on when both accuracy and sensorAccuracy is high
			if(accuracy == 3 && sensorAccuracy == 3)
			{
				LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
				TIM3->PSC = 65000; //Very low frequency
				LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			}
			else
			{
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
				LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
			}
		}

		Is_iBus_Received(); //Refreshes iBus Data for iBus.SwC
		HAL_Delay(100);
	}

	//Ends the loop when iBus.SwC is not mid point
	//Turn the LED and buzzer off
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	//Saves the current dynamic calibration data (DCD) to memory
	//Sends command to get the latest calibration status
	BNO080_saveCalibration();
	BNO080_requestCalibrationStatus();

	//Wait for calibration response, timeout if no response
	int counter = 100;
	while(1)
	{
		if(--counter == 0) break;
		if(BNO080_dataAvailable())
		{
			//The IMU can report many different things. We must wait
			//for the ME Calibration Response Status byte to go to zero
			if(BNO080_calibrationComplete() == 1)
			{
				printf("\nCalibration data successfully stored\n");
				LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				TIM3->PSC = 2000;
				HAL_Delay(300);
				TIM3->PSC = 1500;
				HAL_Delay(300);
				LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				HAL_Delay(1000);
				break;
			}
		}
		HAL_Delay(10);
	}
	if(counter == 0)
	{
		printf("\nCalibration data failed to store. Please try again.\n");
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		TIM3->PSC = 1500;
		HAL_Delay(300);
		TIM3->PSC = 2000;
		HAL_Delay(300);
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
		HAL_Delay(1000);
	}

	//BNO080_endCalibration(); //Turns off all calibration
	//In general, calibration should be left on at all times. The BNO080
	//auto-calibrates and auto-records cal data roughly every 5 minutes

	//Resets BNO080 to disable Game Rotation Vector and Magnetometer
	//Enables Rotation Vector
	BNO080_Initialization();
//	BNO080_enableRotationVector(2500); //Send data update every 2.5ms (400Hz)
	BNO080_enableGameRotationVector(2500);
}


int Is_iBus_Received(void){
	if(ibus_rx_cplt_flag == 1){
		ibus_rx_cplt_flag = 0;
		if (iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1){
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			return 1;
		}
	}
	return 0;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static unsigned char cnt = 0;
	/* Telemetry(UART1) 수신받으면 Frame 크기인 20byte 만큼 수신받음. */
	if (huart->Instance == USART1){								// uart1(Telemetry) 채널에서 데이터 수신되면
		HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);		// uart1(Telemetry)에선 다시 1byte 수신 대기
		switch(cnt){
		case 0:
			if(uart1_rx_data == 0x47){
				telemetry_rx_buf[cnt] = uart1_rx_data;
				cnt++;
			}
			break;
		case 1:
			if(uart1_rx_data == 0x53){
				telemetry_rx_buf[cnt] = uart1_rx_data;
				cnt++;
			}
			else
				cnt = 0;			// cnt 0으로 초기화하고 다음 메시지 받을 준비.
			break;
		case 19:
			telemetry_rx_buf[cnt] = uart1_rx_data;
			cnt = 0;
			telemetry_rx_cplt_flag = 1;	// data 수신 완료! main으로 분기
			break;
		default:
			telemetry_rx_buf[cnt] = uart1_rx_data;
			cnt++;
			break;
		}
	}
}

void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf){
	/* FC -> GCS 통신 프레임(8-4강) - 롤, 피치 각도는 x100 해서 보냄.(signed short) */
	telemetry_tx_buf[0] = 0x46;		// 'F'
	telemetry_tx_buf[1] = 0x43;		// 'C'

	telemetry_tx_buf[2] = 0x10;		// AHRS
	// Roll
	telemetry_tx_buf[3] = (short)(BNO080_Roll*100);						// Roll Rotation Angle
	telemetry_tx_buf[4] = ((short)(BNO080_Roll*100))>>8;
	//Pitch(각도)
	telemetry_tx_buf[5] = (short)(BNO080_Pitch*100);						// Pitch Rotation Angle(각도)
	telemetry_tx_buf[6] = ((short)(BNO080_Pitch*100))>>8;
//	telemetry_tx_buf[5] = (short)(ICM20602.gyro_x*100);						// Pitch Rotation Angler Speed(각속도)
//	telemetry_tx_buf[6] = ((short)(ICM20602.gyro_x*100))>>8;
	//Yaw
	telemetry_tx_buf[7] = (unsigned short)(BNO080_Yaw*100);				// Yaw Haeding Angle
	telemetry_tx_buf[8] = ((unsigned short)(BNO080_Yaw*100))>>8;

	telemetry_tx_buf[9] = (short)(LPS22HH.baroAltFilt*10);				// Barometric Altitude
	telemetry_tx_buf[10] = ((short)(LPS22HH.baroAltFilt*10))>>8;

	telemetry_tx_buf[11] = (short)((iBus.RH-1500)*0.1f*100);				// Target Roll angle position
	telemetry_tx_buf[12] = ((short)((iBus.RH-1500)*0.1f*100))>>8;

	telemetry_tx_buf[13] = (short)((iBus.RV-1500)*0.1f*100);				// Target Pitch angle position
	telemetry_tx_buf[14] = ((short)((iBus.RV-1500)*0.1f*100))>>8;

	telemetry_tx_buf[15] = (unsigned short)((iBus.LH-1000)*0.36f*100);	// Target Yaw Heading Position
	telemetry_tx_buf[16] = ((unsigned short)((iBus.LH-1000)*0.36f*100))>>8;

	telemetry_tx_buf[17] = (short)(iBus.LV*10);		// 목표 고도값(조종기 쓰로틀 키값)
	telemetry_tx_buf[18] = ((short)(iBus.LV*10))>>8;

	telemetry_tx_buf[19] = 0xff;

	/* Checksum */
	for (int i=0; i<19; i++){
		telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
	}
}

void Encode_Msg_GPS(unsigned char* telemetry_tx_buf){
	telemetry_tx_buf[0] = 0x46;		// 'F'
	telemetry_tx_buf[1] = 0x43;		// 'C'

	telemetry_tx_buf[2] = 0x11;
	// Roll
	telemetry_tx_buf[3] = posllh.lat;
	telemetry_tx_buf[4] = posllh.lat>>8;
	telemetry_tx_buf[5] = posllh.lat>>16;
	telemetry_tx_buf[6] = posllh.lat>>24;

	telemetry_tx_buf[7] = posllh.lon;
	telemetry_tx_buf[8] = posllh.lon>>8;
	telemetry_tx_buf[9] = posllh.lon>>16;
	telemetry_tx_buf[10] = posllh.lon>>24;

	telemetry_tx_buf[11] = (unsigned short)(batVolt*100);
	telemetry_tx_buf[12] = ((unsigned short)(batVolt*100))>>8;

	telemetry_tx_buf[13] = iBus.SwA == 1000 ? 0 : 1;
	telemetry_tx_buf[14] = iBus.SwC == 1000 ? 0 : iBus.SwC == 1500 ? 1 : 2;

	telemetry_tx_buf[15] = failsafe_flag; 	// 0 , 1 , 2

	telemetry_tx_buf[16] = 0x00;
	telemetry_tx_buf[17] = 0x00;
	telemetry_tx_buf[18] = 0x00;

	telemetry_tx_buf[19] = 0xff;

	/* Checksum */
	for (int i=0; i<19; i++){
		telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
	}
}

void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d){
	telemetry_tx_buf[0] = 0x46;		// 'F'
	telemetry_tx_buf[1] = 0x43;		// 'C'

	telemetry_tx_buf[2] = id;

//	memcpy(&telemetry_tx_buf[3], &p, 4);	// 임시변수 만들어서 메모리 사용량 커짐
//	memcpy(&telemetry_tx_buf[7], &i, 4);	// 7,8,9,10
//	memcpy(&telemetry_tx_buf[11], &d, 4);	// 11,12,13,14
	*(float*)(&telemetry_tx_buf[3]) = p;
	*(float*)(&telemetry_tx_buf[7]) = i;
	*(float*)(&telemetry_tx_buf[11]) = d;

	telemetry_tx_buf[15] = 0x00;
	telemetry_tx_buf[16] = 0x00;
	telemetry_tx_buf[17] = 0x00;
	telemetry_tx_buf[18] = 0x00;

	telemetry_tx_buf[19] = 0xff;

	/* Checksum */
	for (int i=0; i<19; i++){
		telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
