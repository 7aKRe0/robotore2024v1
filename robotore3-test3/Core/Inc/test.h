/*
 * main.h
 *
 *  Created on: Sep 23, 2024
 *      Author: uchuu
 */

// test.h
// test.h
#ifndef INC_TEST_H_
#define INC_TEST_H_

#include <stdio.h>
#include <string.h>

#include <main.h>

#define SENSOR_COUNT 4
#define dt 0.01
#define TIRE 22

// 変数のextern宣言
extern float base_speed;
extern float max_speed;
extern float Kp, Ki, Kd;
extern float previous_error;
extern float integral;
extern float target_speed_L, target_speed_R;
extern float current_speed_L, current_speed_R;
extern float side_r_flag, side_l_flag;
extern float side_l_time, side_r_time;
extern float stop_flag;
extern uint16_t main_sens[];
extern uint16_t side_sens[];
extern float Line1_sens[];
extern float Line2_sens[];
extern float Line3_sens[];
extern float adjusted_speed_L, adjusted_speed_R;
extern float Line1_sens1, Line1_sens2, Line1_sens3, Line1_sens4;
extern float Line2_sens1, Line2_sens2, Line2_sens3, Line2_sens4;
extern int mode, mode_processed;
extern int tape_list[];
extern int section_index;
extern uint32_t last_tape_time;
extern uint32_t current_time, current_ftime, current_2ftime;
extern uint32_t tape_interval;
extern int firstLapComplete;
extern float threshold_0, threshold_1;
extern float min_black_0, min_black_1;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart2;

extern float max_white_a[SENSOR_COUNT];
extern float min_black_a[SENSOR_COUNT];

extern float max_white_b[SENSOR_COUNT];
extern float min_black_b[SENSOR_COUNT];

extern float max_white_0;
extern float max_white_1;

extern float cross_time;
extern float ikiti;
// test.h
void button();
void calibrate_sensors();
void playSound(uint32_t frequency, uint32_t duration, float volume);
void readSensors();
void SpeedControl();

void flag(void);
void flag2(void);

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/




#endif /* INC_TEST_H_ */
