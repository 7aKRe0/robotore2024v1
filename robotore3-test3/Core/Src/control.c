/*
 * control.c
 *
 *  Created on: Sep 23, 2024
 *      Author: uchuu
 */

#include <main.h>
#include <test.h>
#include <stdio.h>
#include <string.h>
#include <math.h>



void controlMotor(double duty_L, double duty_R){
 	  // 左モータの制御
 	      if (duty_L >= 0) {
 	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // PHをHIGH(正転)
 	          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_L);
 	      } else {
 	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // PHをLOW
 	          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -duty_L);
 	      }

 	      // 右モータの制御
 	      if (duty_R >= 0) {
 	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); // PHをHIGH(正転)
 	          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_R);
 	      } else {
 	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // PHをLOW()
 	          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -duty_R);
 	      }

 }



float calculateError() {
	  float Line1_sum = 0.0;
	  float Line2_sum = 0.0;
	  int g1[] = {1,1,1,1};
	  int g2[] = {1,1,1,1};

	  for (int i = 0; i < SENSOR_COUNT; i++){//右
		  Line1_sum += (Line1_sens[i]*g1[i])*3000/(max_white_a[i] - min_black_a[i]);
	  }

	  for (int i = 0; i < SENSOR_COUNT; i++){
		  Line2_sum += (Line2_sens[i]*g2[i])*3000/(max_white_b[i] - min_black_b[i]);
	  }

	  return  Line2_sum - Line1_sum;


}
//
// void Encoder_Read(void) {
//      static int cnt_old_L = 0, cnt_old_R = 0;
//      int cnt_new_L = __HAL_TIM_GET_COUNTER(&htim3);
//      int cnt_new_R = __HAL_TIM_GET_COUNTER(&htim1);
//      int cnt_L = cnt_new_L - cnt_old_L;
//      int cnt_R = cnt_new_R - cnt_old_R;
//
//      // カウンタ値の更新
//      cnt_old_L = cnt_new_L;
//      cnt_old_R = cnt_new_R;
//
//      // カウンタのオーバ�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��フロー
//      if (cnt_L > 32767) cnt_L -= 65536;
//      if (cnt_L < -32767) cnt_L += 65536;
//      if (cnt_R > 32767) cnt_R -= 65536;
//      if (cnt_R < -32767) cnt_R += 65536;
//
//      // 速度の計�?
//      current_speed_L = (3.6 * TIRE * M_PI * cnt_L) / (4096.0 * dt);
//      current_speed_R = (3.6 * TIRE * M_PI * cnt_R) / (4096.0 * dt);
//
//      adjusted_speed_L = current_speed_L * -0.0156;
//      adjusted_speed_R = current_speed_R * -0.0156;
//
//      // ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?バッグ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?報の送信
// //      char scnt[100];
// //      sprintf(scnt, "Speed_L: %f, Speed_R: %f\r\n", adjusted_speed_L, adjusted_speed_R);
// //      HAL_UART_Transmit(&huart2, (uint8_t*)scnt, strlen(scnt) + 1, HAL_MAX_DELAY);
//  }



  void SpeedControl() {
      // 読み取りセンサの値
//      readSens();

      // エラーを計�?
      float error = calculateError();


      // PID制御の計�?
      float derivative = (error - previous_error) / dt;
      integral += error;

      if (integral >= 1000000) integral = 1000000;
      if (integral <= -1000000) integral = -1000000;


      float P =Kp * error;
      float I =Ki * integral;
      float D =Kd * derivative;

      float output = P + I + D;
      previous_error = error;

      target_speed_L = base_speed - output;
      target_speed_R = base_speed + output;

//      Encoder_Read();
//
//      float speed_error_L = target_speed_L - adjusted_speed_L;//+
//      float speed_error_R = target_speed_R - adjusted_speed_R;

       float duty_L = -1*(target_speed_L);
       float duty_R = -1*(target_speed_R);

//      float speed_P_gain = 1.3;//速度のPゲイン

//      float duty_L = -1*(speed_error_L * speed_P_gain);
//      float duty_R = -1*(speed_error_R * speed_P_gain);



      if (duty_L > 2000) duty_L = 2000;
      if (duty_L < -2000) duty_L = -2000;
      if (duty_R > 2000) duty_R = 2000;
      if (duty_R < -2000) duty_R = -2000;

      //float error_speed_L = -0.1*(target_speed_L - current_speed_L);
      //float error_speed_R = -0.1*(target_speed_R - current_speed_R);



      // モータを制御
      controlMotor(duty_L, duty_R);
  }
