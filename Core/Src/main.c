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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "management.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch) {
HAL_UART_Transmit(&hlpuart1, &ch, 1, 1);
}
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

/* USER CODE BEGIN PV */
uint8_t Rxbuf_from_Ether[Rxbufsize_from_Ether];
uint8_t Rxbuf_from_Ether_UART[Rxbufsize_from_Ether];
uint8_t Rxbuf_from_Ether_temp[Rxbufsize_from_Ether-1]={0};
uint8_t Ether_connect=0,Ether_connect_check=0;
uint16_t cnt_time_tim;
const float32_t rotation_longth=omni_diameter*M_PI;
uint8_t sw_mode;
float yawAngle_temp;
uint16_t cnt_time_50Hz;
uint16_t yawAngle_send;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void maintask_run();
void maintask_stop();
void maintask_emargency();
void maintask_state_stop();
long map(long x, long in_min, long in_max, long out_min, long out_max);
float getAngleDiff(float angle_rad1, float angle_rad2) ;
float normalizeAngle(float angle_rad);
uint32_t HAL_GetTick(void)
 {
   return uwTick;
 }
uint8_t decode_SW(uint16_t SW_data);

float pitchAngle;
float rollAngle;
float yawAngle;
float pitchAngle_rad;
float rollAngle_rad;
float yawAngle_rad;
float acc[3];
float gyro[3];
float acc_comp[3];
float gyro_comp[3];
float IMU_tmp;
char orientation;
float acc_off[3];
float gyro_off[3];

volatile int kick_state,kick_time;
uint8_t data_from_ether[Rxbufsize_from_Ether-1];
uint8_t TX_data_UART[9];
uint16_t Csense[1];
uint16_t Vsense[1];
uint16_t SWdata[1];
uint8_t Emargency;
uint8_t TxData[can_TX_data];
uint8_t RxData[can_RX_data];
uint32_t TxMailbox;
float32_t motor_feedback[5];
float32_t motor_feedback_velocity[5];
float32_t voltage[6];
float32_t Power_voltage[6];
float32_t tempercher[6];
float32_t amplitude[5];
float32_t power_amp;
float32_t vel_surge;
float32_t vel_sway;
float32_t omega;
float32_t drible_power;
float32_t kick_power;
float32_t theta_vision;
float32_t theta_target;
uint8_t chipEN;
uint8_t ball[4];
int16_t mouse[2];
uint8_t error_No[4];
float32_t m1,m2,m3,m4;
float32_t v_round;
float32_t ball_x,ball_y,robot_x,robot_y,robot_x_target,robot_y_target;
uint8_t keeper_EN,stall;
uint16_t cnt_motor;
uint8_t check_motor1,check_motor2,check_motor3,check_motor4,check_power,check_FC;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_FilterTypeDef  sFilterConfig;

TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
UART_HandleTypeDef *huart_xprintf;

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
  MX_FDCAN1_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_ADC5_Init();
  MX_ADC3_Init();
  MX_FDCAN2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  check_motor1=0;
  check_motor2=0;
  check_motor3=0;
  check_motor4=0;
  check_power=0;
  check_FC=0;
  kick_state=0;
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    for(int i=0;i<3;i++){
      actuator_buzzer(20, 20);
    }
    setbuf(stdout, NULL);
    printf("start\r\n");
    can1_init_ibis(&hfdcan1);
    can2_init_ibis(&hfdcan2);

    HAL_FDCAN_Start(&hfdcan1);
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_FDCAN_Start(&hfdcan2);
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_UART_Init(&hlpuart1);
    xprintf_init(&hlpuart1);


    HAL_UART_Init(&huart2);
    HAL_UART_Receive_DMA(&huart2,(uint8_t *)Rxbuf_from_Ether,Rxbufsize_from_Ether);


    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Vsense,1);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)Csense,1);
    HAL_ADC_Start_DMA(&hadc5, (uint32_t *)SWdata,1);


    //while(1){
    	//if(check_motor1==1 && check_motor2==1 && check_motor3==1 && check_motor4==1 &&check_power==1 && check_FC==1){break;}
    	//if(check_power==1 && check_FC==1){break;}
    	//if(check_power==1){break;}
    	//break;
    	//if(HAL_GetTick()>1000){
    	//	Error_Handler();
    	//}
   // }

    actuator_power_ONOFF(0);
    HAL_Delay(20);


    actuator_motor1(0.0,0.0);
    actuator_motor2(0.0,0.0);
    actuator_motor3(0.0,0.0);
    actuator_motor4(0.0,0.0);
    actuator_motor5(0.0,0.0);

    actuator_kicker(1, 1);
    actuator_kicker_voltage(250.0);
    actuator_power_param(1,15.0);//min voltage
    actuator_power_param(2,35.0);//max voltage
    actuator_power_param(3,50.0);//max current
    actuator_power_param(4,90.0);//max temp(fet)
    actuator_power_param(5,90.0);//max temp(solenoid)


    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
    ICM20602_init();
    ICM20602_IMU_calibration2();
    ICM20602_clearAngle();


    //uint8_t senddata_calib[8];
    //can1_send(0x340, senddata_calib);
    //can2_send(0x340, senddata_calib);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    actuator_power_ONOFF(1);


    for(int i=0;i<3;i++){
    	actuator_buzzer(40, 40);
    }

    data_from_ether[Rxbufsize_from_Ether-3] = 0;
    HAL_Delay(500);
    HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 ICM20602_read_IMU_data();
	 sw_mode=15-(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) + (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)<<1)
	 	 					  + (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)<<3) + (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2)<<2));

	 switch (sw_mode){
	 	 case 0:  //main without debug
	 		if(Ether_connect==1){
	 			yawAngle=yawAngle*0.999+(theta_vision*180.0/PI)*0.001;
	 			maintask_run();
	 		}
	 		else{
	 			yawAngle=yawAngle*0.999+(theta_vision*180.0/PI)*0.001;
	 			maintask_state_stop();
	 		}
	 		break;

	 	 case 1:  //main debug
	 		if(Ether_connect==1){
	 			yawAngle=yawAngle*0.999+(theta_vision*180.0/PI)*0.001;
	 			maintask_run();
	 		}
	 		else{
	 			yawAngle=yawAngle*0.999+(theta_vision*180.0/PI)*0.001;
	 			//maintask_run();
	 			maintask_state_stop();
	 		}
	 		break;

	 	 case 2:  //calibration motor
			if(decode_SW(SWdata[0])&0b00010000){
				 uint8_t senddata_calib[8];
				 can1_send(0x310,senddata_calib);//calibration
				 can2_send(0x310,senddata_calib);//calibration
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
			 }
			else{
				omni_move(0.0, 0.0, 0.0,0.0);
				actuator_motor5(0.0,0.0);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
			}
			break;

	 	 case 3:  //motor test
			if(decode_SW(SWdata[0])&0b00000001){
				 omni_move(1.0, 0.0, 0.0,1.0);//fwd
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
			 }
			else if(decode_SW(SWdata[0])&0b00000010){
				 omni_move(-1.0, 0.0, 0.0,1.0);//back
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
			 }
			else if(decode_SW(SWdata[0])&0b00000100){
				 omni_move(0.0, -1.0, 0.0,1.0);//left
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
			 }
			else if(decode_SW(SWdata[0])&0b00001000){
				 omni_move(0.0, 1.0, 0.0,1.0);//right
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
			 }
			else if(decode_SW(SWdata[0])&0b00010000){
				 omni_move(0.0, 0.0, 7.0,1.0);//spin
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
			 }
			else{
				omni_move(0.0, 0.0, 0.0,0.0);
	 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
			}
			actuator_motor5(0.0,0.0);
			break;

	 	 case 4://drible test
	 		 if(decode_SW(SWdata[0])&0b00010000){
	 			actuator_motor5(0.5,1.0);
	 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
			 }
			 else{
				actuator_motor5(0.0,0.0);
	 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
			 }
				omni_move(0.0, 0.0, 0.0,0.0);
	 		 break;

	 	 case 5:
	 		 if(decode_SW(SWdata[0])&0b00010000){
		 		actuator_motor5(0.5,1.0);
	 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
	 			if(ball[0]==1){
					if(kick_state==0){
						actuator_kicker(3, 100);
						kick_state=1;
					}
				}
	 			if(kick_state==1){
	 				kick_time++;
	 				if(kick_time>100){
	 					kick_state=0;
	 					kick_time=0;
	 				}
	 			}

	 		 }
	 		 else{
				 actuator_motor5(0.0,0.0);
	 			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
	 		     actuator_kicker(1, 1);
	 			 actuator_kicker(2, 0);
	 			 actuator_kicker_voltage(250.0);
				 kick_state=0;
				 kick_time=0;
	 		 }
				omni_move(0.0, 0.0, 0.0,0.0);
	 		break;

	 	 case 6:
	 		 if(decode_SW(SWdata[0])&0b00010000){
	 			actuator_motor5(0.5,1.0);
	 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
	 			if(ball[0]==1){
					if(kick_state==0){
						actuator_kicker(3, 100);
						kick_state=1;
					}
				}
				if(kick_state==1){
					kick_time++;
					if(kick_time>100){
						kick_state=0;
						kick_time=0;
					}
				}
	 		 }
	 		 else{
	 			 actuator_motor5(0.0,0.0);
	 			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
	 		     actuator_kicker(1, 1);
	 			 actuator_kicker(2, 1);
	 			 actuator_kicker_voltage(0.0);
					kick_state=0;
					kick_time=0;
	 		 }
				omni_move(0.0, 0.0, 0.0,0.0);
	 		break;

	 	 default:
	 		maintask_stop();
	 		 break;
	}

	 if(cnt_time_tim>50){
	 if(Ether_connect_check != data_from_ether[Rxbufsize_from_Ether-3]){
		 Ether_connect=1;
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
	 }
	 else{
		 Ether_connect=0;
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
	 }
	 Ether_connect_check=data_from_ether[Rxbufsize_from_Ether-3];
	 cnt_time_tim=0;

	 }

	 if(cnt_time_50Hz>50){
       if(sw_mode>0){
    	 //printf(" kicktime=%d, state=%d ",kick_time,kick_state);
		 //printf("data: acc0=%f,acc1=%f,acc2=%f,gyro0=%f,gyro1=%f,gyro2=%f,tmp=%f",acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2],IMU_tmp);
		 //printf(" pich=%f roll=%f yaw=%f",pitchAngle,rollAngle,yawAngle);
    	 printf(" yaw=%f",yawAngle);
		 //printf(" motor0=%.3f motor1=%.3f motor2=%.3f motor3=%.3f",motor_feedback[0],motor_feedback[1],motor_feedback[2],motor_feedback[3]);
		 //printf(" v0=%.3f v1=%.3f v2=%.3f v3=%.3f",voltage[0],voltage[1],voltage[2],voltage[3]);
		 //printf(" t0=%.3f t1=%.3f t2=%.3f t3=%.3f",tempercher[0],tempercher[1],tempercher[2],tempercher[3]);
		 //printf(" A=%.3f",power_amp);
		 //printf(" ball=%d",ball[0]);
		 //printf(" yaw=%f",yawAngle/180.0*M_PI);
		 printf(" connect=%d vel_surge=%.4f vel_sway=%.4f ",Ether_connect,vel_surge,vel_sway);
		 printf(" theta_vision=%.4f theta_AI=%.4f drible_power=%.4f",(theta_vision*180.0/PI),(theta_target*180.0/PI),drible_power);
    	  printf(" v_power=%.3f",Power_voltage[4]);
    	 // printf(" v_charge=%.3f",voltage[5]);
		 printf(" kick_power=%.4f chip=%d",kick_power,chipEN);
		 //printf(" m1=%.5f m2=%.5f m3=%.5f m4=%.5f", m1,m2,m3,m4);
		  //printf(" sw=%d sw=%d",sw_mode,decode_SW(SWdata[0]));
		 //printf(" connect=%d",Ether_connect);
		   /*printf(" AI=%x %x %x %x %x %x %x %x %x %x %x %x %x",data_from_ether[0],data_from_ether[1],data_from_ether[2],
		 	data_from_ether[3],data_from_ether[4], data_from_ether[5],data_from_ether[6],data_from_ether[7],
		 	data_from_ether[8] ,data_from_ether[9],data_from_ether[10],data_from_ether[11],data_from_ether[12]);*/
		 //printf(" data=%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",Rxbuf_from_Ether[0],Rxbuf_from_Ether[1],Rxbuf_from_Ether[2],
		 //		 Rxbuf_from_Ether[3],Rxbuf_from_Ether[4], Rxbuf_from_Ether[5],Rxbuf_from_Ether[6],Rxbuf_from_Ether[7],
		 //		 Rxbuf_from_Ether[8] ,Rxbuf_from_Ether[9],Rxbuf_from_Ether[10],Rxbuf_from_Ether[11],Rxbuf_from_Ether[12]
	     //		,Rxbuf_from_Ether[13],Rxbuf_from_Ether[14]);

		 //printf(" C=%d V=%d SW=%d",Csense[0],Vsense[0],SWdata[0]);
		 //printf(" A=%f",amplitude[4]);
		 printf(" ball:0=%d",ball[0]);
		 printf(" mouse:x=%d, y=%d",mouse[0],mouse[1]);
		 printf("\r\n");
	 }

		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		  cnt_time_50Hz=0;

		  actuator_power_ONOFF(1);
	 }
	 cnt_time_50Hz++;
	 cnt_time_tim++;

	 if(Power_voltage[4]<22.0){
		 actuator_buzzer(100,100);
	 }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GetTick()>2000){
		uint8_t cnt=0;
		while(cnt<100){
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)==1){
				cnt++;
				delayUs(1);
			}
			else{
				break;
			}
		}

		if(cnt>=100){
			Emargency=1;
			printf("Emargency Stop !!!!!!!!!!!!!");
			for(int i=0;i<50;i++){
				maintask_emargency();
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,0);
			NVIC_SystemReset();
			Emargency=0;
		}
		else{
			Emargency=0;
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
   if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	  {
   if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    Error_Handler();
    }
	switch (RxHeader.Identifier){
	//error
	case 0x000:
		error_No[0]=RxData[0];
		error_No[1]=RxData[1];
		Error_Handler();
		break;
	case 0x001:
		error_No[0]=RxData[0];
		error_No[1]=RxData[1];
		maintask_stop();
		break;
	case 0x002:
		break;
	case 0x003:
		break;
	case 0x004:
		break;

	//ball
	case 0x240:
		ball[0]=RxData[0];
		ball[1]=RxData[1];
		ball[2]=RxData[2];
		ball[3]=RxData[3];
		check_FC=1;
		break;

	//power_Voltage
	case 0x210:
		Power_voltage[4]=uchar4_to_float(RxData);
		break;
	case 0x215:
		Power_voltage[5]=uchar4_to_float(RxData);
		break;

	//temperature
	case 0x224:
		tempercher[4]=uchar4_to_float(RxData);
		break;
	case 0x225:
		tempercher[5]=uchar4_to_float(RxData);
		break;

	//amplitude
	case 0x234:
		amplitude[4]=uchar4_to_float(RxData);
		check_power=1;
		break;


	//ETH data
	/*case 0x400:
		data_from_ether[0]=RxData[0];
		data_from_ether[1]=RxData[1];
		data_from_ether[2]=RxData[2];
		data_from_ether[3]=RxData[3];
		data_from_ether[4]=RxData[4];
		data_from_ether[5]=RxData[5];
		data_from_ether[6]=RxData[6];
		data_from_ether[7]=RxData[7];

		vel_surge=((float32_t)(data_from_ether[0]<<8 | data_from_ether[1])-32767.0)/32767.0*7.0;
		vel_sway= ((float32_t)(data_from_ether[2]<<8 | data_from_ether[3])-32767.0)/32767.0*7.0;
		theta_vision=((float32_t)(data_from_ether[4]<<8 | data_from_ether[5])-32767)/32767.0*M_PI;
		theta_target=((float32_t)(data_from_ether[6]<<8 | data_from_ether[7])-32767)/32767.0*M_PI;*/

	break;
	//ETH data2
	case 0x401:
		data_from_ether[8]=RxData[0];
		data_from_ether[9]=RxData[1];
		data_from_ether[10]=RxData[2];
		data_from_ether[11]=RxData[3];
		data_from_ether[12]=RxData[4];
		if(data_from_ether[8]>100){
			chipEN=1;
			data_from_ether[8]=data_from_ether[8]-100;
		}
		else{
			chipEN=0;
		}
		kick_power=(float32_t)data_from_ether[8]/20.0;
		drible_power=(float32_t)data_from_ether[9]/20.0;

		keeper_EN=data_from_ether[10];
	break;

	//mouseXY
		case 0x241:
			mouse[0]=(int16_t)(RxData[0]<<8)|RxData[1];
			mouse[1]=(int16_t)(RxData[2]<<8)|RxData[3];
			break;

		//motor_feedback
		  case 0x200:
			  motor_feedback[0]=uchar4_to_float(RxData);
			  motor_feedback_velocity[0]=motor_feedback[0]*rotation_longth;
			  break;
		  case 0x201:
			  motor_feedback[1]=uchar4_to_float(RxData);
			  motor_feedback_velocity[1]=motor_feedback[1]*rotation_longth;
			  break;
		  case 0x202:
			  motor_feedback[2]=uchar4_to_float(RxData);
			  motor_feedback_velocity[2]=motor_feedback[2]*rotation_longth;
			  break;
		  case 0x203:
			  motor_feedback[3]=uchar4_to_float(RxData);
			  motor_feedback_velocity[3]=motor_feedback[3]*rotation_longth;
			  break;
		  case 0x204:
			  motor_feedback[4]=uchar4_to_float(RxData);
			  motor_feedback_velocity[4]=motor_feedback[3]*rotation_longth;
			  break;


		//temperature
		  case 0x220:
			  tempercher[0]=uchar4_to_float(RxData);
			  break;
		  case 0x221:
			  tempercher[1]=uchar4_to_float(RxData);
			  break;
		  case 0x222:
			  tempercher[2]=uchar4_to_float(RxData);
			  break;
		  case 0x223:
			  tempercher[3]=uchar4_to_float(RxData);
			  break;

	   //amplitude
		  case 0x230:
			  amplitude[0]=uchar4_to_float(RxData);
			  check_motor1=1;
			  break;
		  case 0x231:
			  amplitude[1]=uchar4_to_float(RxData);
			  check_motor2=1;
			  break;
		  case 0x232:
			  amplitude[2]=uchar4_to_float(RxData);
			  check_motor3=1;
			  break;
		  case 0x233:
			  amplitude[3]=uchar4_to_float(RxData);
			  check_motor4=1;
			  break;

	}
	}


}


void maintask_run(){
	//theta_target=0.0;
	omega=(getAngleDiff(theta_target,(yawAngle/180.0*M_PI))*20.0)
			-(getAngleDiff((yawAngle/180.0*M_PI),(yawAngle_temp/180.0*M_PI))*4.5*57.29);

	if(omega>6*M_PI){omega=6*M_PI;}
	if(omega<-6*M_PI){omega=-6*M_PI;}

	omni_move(vel_surge, vel_sway, omega,1.0);
	  if(kick_power>0){
			if(ball[0]==1){
				if(kick_state==0){
				  uint8_t kick_power_param=(float)kick_power*255.0;
				  printf(" kick=%d\r\n",kick_power_param);
				  actuator_kicker(3, (uint8_t)kick_power_param);
				kick_state=1;
				}
			}
			if(kick_state==1){
				kick_time++;
				if(kick_time>100){
					kick_state=0;
					kick_time=0;
				}
			}
	  }

	  if(chipEN==1){
		  actuator_kicker(2, 1);
	  }
	  else{
		  actuator_kicker(2, 0);
	  }
	  actuator_kicker(1, 1);
	  actuator_kicker_voltage(250.0);

	  actuator_motor5(drible_power,1.0);


      uint8_t yawAngle_send_low = ((int)yawAngle+360) & 0x00FF;
      uint8_t yawAngle_send_high = (((int)yawAngle+360) & 0xFF00) >> 8;

	  TX_data_UART[0]=254;
	  TX_data_UART[1]=(uint8_t)yawAngle_send_low;
	  TX_data_UART[2]=(uint8_t)yawAngle_send_high;
	  TX_data_UART[3]=ball[0];
	  TX_data_UART[4]=ball[1];
	  TX_data_UART[5]=chipEN;
	  TX_data_UART[6]=kick_state;
	  TX_data_UART[7]=(uint8_t)Power_voltage[4];
	  HAL_UART_Transmit(&huart2, TX_data_UART, 8,0xff);

	  yawAngle_temp=yawAngle;
}


void maintask_emargency(){
	  actuator_motor1(0.0,0.0);
	  actuator_motor2(0.0,0.0);
	  actuator_motor3(0.0,0.0);
	  actuator_motor4(0.0,0.0);
	  actuator_motor5(0.0,0.0);

	  TX_data_UART[0]=254;
	  TX_data_UART[1]=error_No[0];
	  TX_data_UART[2]=error_No[1];
	  TX_data_UART[3]=error_No[2];
	  TX_data_UART[4]=error_No[3];
	  TX_data_UART[5]=252;
	  TX_data_UART[6]=122;
	  TX_data_UART[7]=200;
	  HAL_UART_Transmit(&huart2, TX_data_UART, 8,0xff);

	  actuator_buzzer(150, 150);

	  uint8_t senddata_error[8];

	  can1_send(0x000, senddata_error);
	  can2_send(0x000, senddata_error);

	  actuator_kicker(1, 0);
	  actuator_kicker_voltage(0.0);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,1);

}


void maintask_state_stop(){

    uint8_t yawAngle_send_low = ((int)yawAngle+360) & 0x00FF;
    uint8_t yawAngle_send_high = (((int)yawAngle+360) & 0xFF00) >> 8;

	  omni_move(0.0, 0.0, 0.0,0.0);
	  actuator_motor5(0.0,0.0);


	  TX_data_UART[0]=254;
	  TX_data_UART[1]=(uint8_t)yawAngle_send_low;
	  TX_data_UART[2]=(uint8_t)yawAngle_send_high;
	  TX_data_UART[3]=error_No[0];
	  TX_data_UART[4]=error_No[1];
	  TX_data_UART[5]=1;
	  TX_data_UART[6]=1;
	  TX_data_UART[7]=(uint8_t)Power_voltage[4];
	  HAL_UART_Transmit(&huart2, TX_data_UART, 8,0xff);


	  actuator_kicker(1, 0);
	  actuator_kicker_voltage(0.0);
}

void maintask_stop(){
	  omni_move(0.0, 0.0, 0.0,0.0);
	  actuator_motor5(0.0,0.0);

      uint8_t yawAngle_send_low = ((int)yawAngle+360) & 0x00FF;
      uint8_t yawAngle_send_high = (((int)yawAngle+360) & 0xFF00) >> 8;

		  omni_move(0.0, 0.0, 0.0,0.0);
		  actuator_motor5(0.0,0.0);


		  TX_data_UART[0]=254;
		  TX_data_UART[1]=(uint8_t)yawAngle_send_low;
		  TX_data_UART[2]=(uint8_t)yawAngle_send_high;
		  TX_data_UART[3]=error_No[0];
		  TX_data_UART[4]=error_No[1];
		  TX_data_UART[5]=0;
		  TX_data_UART[6]=0;
		  TX_data_UART[7]=(uint8_t)Power_voltage[4];
		  HAL_UART_Transmit(&huart2, TX_data_UART, 8,0xff);

	  actuator_kicker(1, 0);
	  actuator_kicker_voltage(0.0);
}


long map(long x, long in_min, long in_max, long out_min, long out_max) {
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

float normalizeAngle(float angle_rad) {
    while (angle_rad > M_PI) {
        angle_rad -= 2.0f * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0f * M_PI;
    }
    return angle_rad;
}

float getAngleDiff(float angle_rad1, float angle_rad2){
    angle_rad1 = normalizeAngle(angle_rad1);
    angle_rad2 = normalizeAngle(angle_rad2);
    if (abs(angle_rad1 - angle_rad2) > M_PI) {
    	if(angle_rad1 > angle_rad2){
    		return angle_rad1 - (angle_rad2 + 2*M_PI);
    	}else{
    		return (angle_rad1 + 2*M_PI) - angle_rad2;
    	}
    } else {
        return angle_rad1 - angle_rad2;
    }
}

uint8_t decode_SW(uint16_t SW_data){
	int data;
	if(SW_data<100){
		data=0b00010000;//C
	}
	else if(SW_data<500 && SW_data>100){
		data=0b00000010;//B
	}
	else if(SW_data<2000 && SW_data>500){
		data=0b00000100;//R
	}
	else if(SW_data<3000 && SW_data>2000){
		data=0b00000001;//F
	}
	else if(SW_data<4000 && SW_data>3000){
		data=0b00001000;//L
	}
	else{
		data=0b00000000;
	}
	return data;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t j = 0;

	while (Rxbuf_from_Ether[j] != 254 &&  j<sizeof(Rxbuf_from_Ether)) {
		j++;
	}
	if(j>=sizeof(Rxbuf_from_Ether)){
		for(uint8_t k=0;k<(sizeof(data_from_ether));k++){
			data_from_ether[k]=0;
		}
	}
	else{
		for (uint8_t k = 0; k < sizeof(data_from_ether); k++) {
			if ((j + k) >= sizeof(data_from_ether)) {
				data_from_ether[k] = Rxbuf_from_Ether[k - (sizeof(data_from_ether) - j)];
			}
			else {
				data_from_ether[k] = Rxbuf_from_Ether[j + k + 1];
			}
		}
	}
	if(data_from_ether[sizeof(data_from_ether)-1]==253){
		for(uint8_t k=0;k<sizeof(data_from_ether);k++){
			Rxbuf_from_Ether_temp[k]=data_from_ether[k];
		}
	}
	else{
		for(uint8_t k=0;k<sizeof(data_from_ether);k++){
			data_from_ether[k]=Rxbuf_from_Ether_temp[k];
		}
	}

	vel_surge=((float32_t)(data_from_ether[0]<<8 | data_from_ether[1])-32767.0)/32767.0*7.0;
	vel_sway= ((float32_t)(data_from_ether[2]<<8 | data_from_ether[3])-32767.0)/32767.0*7.0;
	theta_vision=((float32_t)(data_from_ether[4]<<8 | data_from_ether[5])-32767)/32767.0*M_PI;
	theta_target=((float32_t)(data_from_ether[6]<<8 | data_from_ether[7])-32767)/32767.0*M_PI;

	if(data_from_ether[8]>100){
		chipEN=1;
		data_from_ether[8]=data_from_ether[8]-100;
	}
	else{
		chipEN=0;
	}
	kick_power=(float32_t)data_from_ether[8]/20.0;
	drible_power=(float32_t)data_from_ether[9]/20.0;

	keeper_EN=data_from_ether[10];
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
	while(1){
		maintask_emargency();
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,1);
		actuator_buzzer(200, 200);
		//NVIC_SystemReset();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
