/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
/* Note that stm32f4xx_hal.c line 256 1000-10000 change systemtick */
/* Change the time for interruption */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "usart.h"
#include "stdio.h"
#include "tim.h"
#include "i2c.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "math.h"
#include "pid.h"
#include <string.h>
#include <stdlib.h>  // for atoi()

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	double x;
	double y;
} Vec2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* MOTOR INFORMATION*/
int16_t cmd_current[4] = {0, 0, 0, 0};
int16_t mtr_Rpos_last[4] = {0, 0, 0, 0};
int16_t mtr_Rpos[4] = {0, 0, 0, 0};
int16_t mtr_torque[4] = {0, 0, 0, 0};
int16_t mtr_rpm[4] = {0, 0, 0, 0};

/* FOOT INTERFACE POSITION */
double tmpros[2] = {0.0, 0.0};
double ABSpos[2] = {0.0, 0.0};
int box[4] = {0, 0, 0, 0};
float tmpcounter1 = 0, tmpcounter2 = 0;

/* APF MODE */
double equation[3] = {0.1, 0.0, 0.0};
double goal[2] = {0.0, 0.0};
double tmp_goal[2] = {0.0, 0.0};
double k_att = 20.0;
double k_rep = 2000.0;
double AttractiveForce[2] = {0.0, 0.0};
double RepulsiveForce[2] = {0.0, 0.0};
double tmp[4] = {0, 0, 0, 0};
double lineDir[2] = {0.0, 0.0};
double linePoint1[2] = {-150.0, -150.0};
double linePoint2[2] = {150.0, 150.0};
double U_toatl[2];

double line_Dir[2];
double line_POS1[2];
double dotProduct;
double lengthSquared;
double t;
double closestPoint[2];

/* FOOT INTERFACE HARDWARE CONFIGURATION */
double WheelCircum = 3.1415926535 * (29 ^ 2);
double Reduction_gear = 36;	 // reduction gear on Robomaster M2006
double Encoder_steps = 8191; // number of steps on encoder
int motor_mount_direction[4] = {1, 1, 0, 0};
double gear_ratio[4] = {2.4, 2.4, 0, 0}; // (10/24),(10/24)

/* FOOT INTERFACE SOFTWARE CONFIGURATION */
pid_type_def PID1;
pid_type_def PID2;
const fp32 PID_motor1[3] = {0.4, 0, 0};
const fp32 PID_motor2[3] = {0.4, 0, 0};

// const fp32 PID_motor1[3]= {70,10,0};
// const fp32 PID_motor2[3]= {72.5,30,0};

/* TRANSPARENT MODE CONFIGURATION */
// double Boundary[4] = {20.0,-20.0,20.0,-20.0}; // {XMax,Xmin,Ymax,Ymin} *in mm
double Boundary[4] = {150.0, -150.0, 150.0, -150.0};

/* CIRCULAR BOUNDARY MODE CONFIGURATION */

uint32_t color_num = 500;
int flag = 0;
int led_test = 0;
char str[20];
int str_len = 20;
float Pos_data[2] = {0};
uint16_t Pos_data_len = sizeof(Pos_data);
extern acc *stcAcc;
extern gyro *stcGyro;
extern angle *stcAngle;
float ang = 0;
short aacx, aacy, aacz;
short gyrox, gyroy, gyroz;

int error = 0;
/* USER CODE END PV */

// Receive data from Serial
#define RX_BUFFER_SIZE 20
uint8_t rxBuffer[RX_BUFFER_SIZE] = {0};
uint8_t rxData = 0;
volatile uint8_t rxDataReady = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void closestPointOnLine2D(double Current_pos[2], double linePoint1[2], double linePoint2[2])
{

	// Calculate the direction vector of the line
	line_Dir[0] = linePoint2[0] - linePoint1[0];
	line_Dir[1] = linePoint2[1] - linePoint1[1];
	line_POS1[0] = linePoint2[0] - Current_pos[0];
	line_POS1[1] = linePoint2[1] - Current_pos[1];
	dotProduct = (line_Dir[0] * line_POS1[0]) + (line_Dir[1] * line_POS1[1]);
	lengthSquared = pow(line_Dir[0], 2) + pow(line_Dir[1], 2);
	t = dotProduct / (double)lengthSquared;

	// Calculate the projected point
	closestPoint[0] = linePoint1[0] + (1 - t) * line_Dir[0];
	closestPoint[1] = linePoint1[1] + (1 - t) * line_Dir[1];

	if (closestPoint[0] > linePoint2[0])
		closestPoint[0] = linePoint2[0];
	else if (closestPoint[0] < linePoint1[0])
		closestPoint[0] = linePoint1[0];
	else
		;
	if (closestPoint[1] > linePoint2[1])
		closestPoint[1] = linePoint2[1];
	else if (closestPoint[1] < linePoint1[1])
		closestPoint[1] = linePoint1[1];
	else
		;

	goal[0] = closestPoint[0];
	goal[1] = closestPoint[1];
	return;
}

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void obsatcle(void)
{
}

void Transparent_mode(void)
{
	if (mtr_rpm[0] > 200 && ABSpos[0] < Boundary[0])
	{
		cmd_current[0] = 0;
	}
	else if (mtr_rpm[0] < -200 && ABSpos[0] > Boundary[1])
	{
		cmd_current[0] = -0;
	}
	else if (ABSpos[0] < Boundary[1])
	{
		cmd_current[0] = 3000;
	}
	else if (ABSpos[0] > Boundary[0])
	{
		cmd_current[0] = -3000;
	}
	else
	{
		cmd_current[0] = 0;
	}

	if (mtr_rpm[1] > 200 && ABSpos[1] < Boundary[2])
	{
		cmd_current[1] = 0;
	}
	else if (mtr_rpm[1] < -200 && ABSpos[1] > Boundary[3])
	{
		cmd_current[1] = -0;
	}
	else if (ABSpos[1] < Boundary[3])
	{
		cmd_current[1] = 3000;
	}
	else if (ABSpos[1] > Boundary[2])
	{
		cmd_current[1] = -3000;
	}
	else
	{
		cmd_current[1] = 0;
	}

	CAN_cmd_chassis(cmd_current[0], cmd_current[1], 0, 0);
}

void computeRepulsiveForce()
{
	double threshhold = 50.0;
	double U_rep = 0;
	double distance_xbound;
	double distance_ybound;
	double distance;

	RepulsiveForce[0] = 0;
	RepulsiveForce[1] = 0;

	for (int i = 0; i < 2; i += 1)
	{
		distance_xbound = ABSpos[0] - Boundary[i];
		distance = sqrt(pow(distance_xbound, 2));

		if (distance_xbound <= threshhold)
		{
			U_rep += 0.5 * k_rep * pow(((1 / distance) - (1 / threshhold)), 2);
			RepulsiveForce[0] += k_rep * ((1 / threshhold) - (1 / distance)) * 1 / (pow(distance, 2)) * distance_xbound / distance;
		}

		else
		{
			U_rep += 0;
			RepulsiveForce[0] += 0;
		}
	}
	for (int i = 2; i < 4; i += 1)
	{
		distance_ybound = ABSpos[1] - Boundary[i];
		distance = sqrt(pow(distance_ybound, 2));

		if (distance_ybound <= threshhold)
		{
			U_rep += 0.5 * k_rep * pow(((1 / distance) - (1 / threshhold)), 2);
			RepulsiveForce[1] += k_rep * ((1 / threshhold) - (1 / distance)) * 1 / (pow(distance, 2)) * distance_ybound / distance;
		}

		else
		{
			U_rep += 0;
			RepulsiveForce[1] += 0;
		}
	}
	return;
}

void computeAttractiveForceWithDeadZone()
{
	double deadZone = 10.0;
	double d = sqrt((pow(ABSpos[0] - goal[0], 2)) + (pow(ABSpos[1] - goal[1], 2)));
	if (d >= deadZone)
	{
		AttractiveForce[0] = k_att * (ABSpos[0] - goal[0]) + k_att;
		AttractiveForce[1] = k_att * (ABSpos[1] - goal[1]) + k_att;
	}
	return;
}
void computeAttractiveForcePOnly()
{

	AttractiveForce[0] = k_att * (ABSpos[0] - goal[0]);
	AttractiveForce[1] = k_att * (ABSpos[1] - goal[1]);

	return;
}

void computeAttractiveForce()
{
	double threshhold = 10.0;
	double U_attract;
	double d = sqrt((pow(ABSpos[0] - goal[0], 2)) + (pow(ABSpos[1] - goal[1], 2)));

	if (d <= threshhold)
	{
		// U_attract = 0.5 * k_att * pow(d,2);
		AttractiveForce[0] = k_att * (ABSpos[0] - goal[0]);
		AttractiveForce[1] = k_att * (ABSpos[1] - goal[1]);
	}
	else
	{
		// U_attract = threshhold * k_att * d -(0.5 * k_att * pow(threshhold,2));
		AttractiveForce[0] = k_att * (ABSpos[0] - goal[0]) / threshhold * d;
		AttractiveForce[1] = k_att * (ABSpos[1] - goal[1]) / threshhold * d;
	}

	return;
}

void Circle_mode(void)
{
	goal[0] = 0.0;
	goal[1] = 0.0;
	computeAttractiveForce();
	U_toatl[0] = -AttractiveForce[0] + RepulsiveForce[0];
	U_toatl[1] = -AttractiveForce[1] + RepulsiveForce[1];

	PID_calc(&PID1, tmp[0], -AttractiveForce[0]);
	PID_calc(&PID2, tmp[1], -AttractiveForce[1]);
	tmp[0] = PID1.Pout;
	tmp[1] = PID2.Pout;

	CAN_cmd_chassis(tmp[0], tmp[1], 0, 0);
}

void APF_mode(void)
{

	// closestPointOnLine2D(ABSpos, linePoint1, linePoint2);

	computeRepulsiveForce();
	computeAttractiveForce();
	// computeAttractiveForcePOnly();
	U_toatl[0] = -AttractiveForce[0] - RepulsiveForce[0];
	U_toatl[1] = -AttractiveForce[1] - RepulsiveForce[1];
	PID_calc(&PID1, tmp[0], U_toatl[0]);
	PID_calc(&PID2, tmp[1], U_toatl[1]);
	tmp[0] = PID1.out;
	tmp[1] = PID2.out;

	CAN_cmd_chassis(tmp[0], tmp[1], 0, 0);
}
/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void)
{

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	// MX_TIM5_Init();
	btim_timx_int_init(49, 1679);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_USART1_UART_Init();
	// MX_USART6_UART_Init();
	// MPU_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	can_filter_init();
	PID_init(&PID1, PID_POSITION, PID_motor1, 7000, 7000);
	PID_init(&PID2, PID_POSITION, PID_motor2, 7000, 7000);

	/* USER CODE END 2 */

	/* USER CODE BEGIN WHILE */
	while (1)
	{
		// MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//õٶȴ
		// MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//õ
		// printf("%f, %f, %f \r\n",atan2(aacz, aacy)*180 / 3.14, atan2(aacz, aacx)*180 / 3.14, atan2(aacy, aacx)*180 / 3.14);

		HAL_Delay(50);
		// Transparent_mode();
		// Circle_mode();
		APF_mode();
		HAL_UART_Transmit(&huart1, (uint8_t *)Pos_data, Pos_data_len, 100);
		// HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
		// HAL_Delay(100);
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
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 6;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	error = 1;
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		bsp_toggle_pin();
		//mtr_current = get_chassis_motor_measure_point(0) -> given_current;
		//mtr_rpm = get_chassis_motor_measure_point(0) -> speed_rpm;

	}
}
*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == BTIM_TIMX_INT)
	{
		// HAL_Delay(10);

		box[0] = box[0];
		box[1] = box[1];
		mtr_Rpos[0] = mtr_Rpos[0];
		mtr_Rpos[1] = mtr_Rpos[1];
		ABSpos[0] = ABSpos[0];
		ABSpos[1] = ABSpos[1];
		tmpros[0] = tmpros[0];
		tmpros[1] = tmpros[1];

		for (int i = 0; i < 2; i++)
		{
			mtr_Rpos_last[i] = get_chassis_motor_measure_point(i)->last_ecd;
			mtr_Rpos[i] = get_chassis_motor_measure_point(i)->ecd;
			mtr_torque[i] = get_chassis_motor_measure_point(i)->given_current;
			mtr_rpm[i] = get_chassis_motor_measure_point(i)->speed_rpm;
			// ABSpos[i] = motor_mount_direction[i] * gear_ratio[i] * ((182.2099/36)*(mtr_Rpos[i])/8191 + box[i]*WheelCircum);
			ABSpos[i] = 2 * motor_mount_direction[i] * WheelCircum * gear_ratio[i] / Reduction_gear * (mtr_Rpos[i] / Encoder_steps + box[i]);
			// if (mtr_Rpos_last[i] != mtr_Rpos[i] ){
			if (mtr_Rpos_last[i] < 4000 && mtr_Rpos[i] - mtr_Rpos_last[i] > 4100)
			{
				box[i] -= 1;
			}
			else if (mtr_Rpos_last[i] > 4000 && mtr_Rpos[i] - mtr_Rpos_last[i] < -4100)
			{
				box[i] += 1;
			}
			//}
		}

		PID_calc(&PID1, tmp[0], U_toatl[0]);
		PID_calc(&PID2, tmp[1], U_toatl[1]);
		tmp[0] = PID1.out;
		tmp[1] = PID2.out;

		Pos_data[0] = ABSpos[0];
		Pos_data[1] = ABSpos[1];

		// HAL_UART_Transmit(&huart1, (uint8_t*)str, 20, 100);
		//  ang = (float)stcAcc->a[0] / 32768 * 16;
		// sprintf(str, "%f\r\n", ang);
		// HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
		// HAL_Delay(100);
	}
}

void USART1_IRQHandler(void)  
{
    static uint8_t rxIndex = 0;
    static char rxBuffer[20];  // Buffer for string input
    
    if(huart1.Instance->SR & UART_FLAG_RXNE)
    {
        uint8_t received = huart1.Instance->DR;
        
        // Check for end of transmission (newline or carriage return)
        if(received == '\n' || received == '\r')
        {
            // Null terminate the string
            rxBuffer[rxIndex] = '\0';
            
            // // Convert string to integer
            // int received_value = atoi(rxBuffer);
            
            // // Update k_att with the received value
            // k_att = (double)received_value;
			if (rxBuffer[0] == '+')
			{
				k_att += 10;
			}
			else if (rxBuffer[0] == '-')
			{
				if (k_att > 0)
				{
					k_att -= 10;
				}
			}
            
            // Reset buffer index
            rxIndex = 0;
        }
        // Store character if buffer isn't full
        else if(rxIndex < sizeof(rxBuffer) - 1)
        {
            rxBuffer[rxIndex++] = received;
        }
    }
    else if(huart1.Instance->SR & UART_FLAG_IDLE)
    {
        huart1.Instance->DR; // Clear IDLE flag
        rxIndex = 0; // Reset buffer on IDLE
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
	error = 1;
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
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

