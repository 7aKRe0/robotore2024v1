/*
 * sens.c
 *
 *  Created on: Sep 23, 2024
 *      Author: uchuu
 */
#include <main.h>
#include <test.h>
#include <stdio.h>
#include <string.h>


void readSens(void){
 	    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) main_sens,4) != HAL_OK){
 	              Error_Handler();
 	     }
// 	    char msg[100];
       // SピンをHIGHに
 	    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET);
 	    //HAL_Delay(1);
 	    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
// 	      snprintf(msg, sizeof(msg), "ADC Value %d: %u\r\n", i, main_sens[i]);
// 	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 	      Line1_sens[i] = main_sens[i];
 	    }
 	    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) main_sens,4) != HAL_OK){
 	              Error_Handler();
 	     }

 	   //Sをlow
 	    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
 	    //HAL_Delay(1);
 	    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
// 	      snprintf(msg, sizeof(msg), "ADC Value %d: %u\r\n", i+4, main_sens[i]);
// 	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 	      Line2_sens[i] = main_sens[i];//?
 	    }
   }

//void readSens(void){
//    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) main_sens, 4) != HAL_OK){
//        Error_Handler();
//    }
//    char msg[200];  // バッファを増やして一度にまとめて送信
//    int msg_len = 0;
//
//    // SピンをHIGHに
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
//    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
//        msg_len += snprintf(msg + msg_len, sizeof(msg) - msg_len, "i%d: %u, ", i + 1, main_sens[i]);
//        Line1_sens[i] = main_sens[i];
//    }
//    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) main_sens, 4) != HAL_OK){
//        Error_Handler();
//    }
//
//    // SピンをLOWに
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
//    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
//        msg_len += snprintf(msg + msg_len, sizeof(msg) - msg_len, "i%d: %u, ", i + 5, main_sens[i]);
//        Line2_sens[i] = main_sens[i];
//    }
//
//    // 最後の改行を追加して1行で出力
//    snprintf(msg + msg_len, sizeof(msg) - msg_len, "\r\n");
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//}

   void readSens2(){

   	  if (HAL_ADC_Start_DMA(&hadc2,(uint32_t *)side_sens,2) != HAL_OK){
   		  Error_Handler();
   	  }
//    	  	  char msg[100];
//    	        snprintf(msg, sizeof(msg), "ADC Value 0: %u, ADC Value 1: %u\r\n", side_sens[0], side_sens[1]);
//    	        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
   	        Line3_sens[0] = side_sens[0];
   	        Line3_sens[1] = side_sens[1];


     }



   void readSensors(){
   	readSens();
   	readSens2();
   }


   void calibrate_sensors(void) {
	   playSound(1000, 100,0.9);
       for (int i = 0; i < 50000; i++) {//変更
           readSens2();
           readSens();


           for(int k = 0; k < SENSOR_COUNT; k++){
        	   if(Line1_sens[k] < min_black_a[k]){
        		   min_black_a[k] = Line1_sens[k];
        	   }
        	   if(Line1_sens[k] > max_white_a[k] ){
        		   max_white_a[k] = Line1_sens[k];
        	   }


        	   if(Line2_sens[k] < min_black_b[k]){
        		   min_black_b[k] = Line2_sens[k];
        	   }
        	   if(Line2_sens[k] > max_white_b[k] ){
        		   max_white_b[k] = Line2_sens[k];
        	   }
        	   }

           if (Line3_sens[0] < min_black_0) {
               min_black_0 = Line3_sens[0];
           }
           if (Line3_sens[1] < min_black_1) {
               min_black_1 = Line3_sens[1];
           }
           // 黒�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?小�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��に余裕を持たせた閾値を設?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
           threshold_0 = min_black_0 + 50;
           threshold_1 = min_black_1 + 15;//3

       }
       playSound(1000, 100,0.9);



       // ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?バッグ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?報の送信
       char msg[200];
       sprintf(msg, "Black_0(Min)=%f,Threshold_0=%f\r\nBlack_1(Min)=%f,Threshold_1=%f\r\n",
     		  min_black_0, threshold_0, min_black_1, threshold_1);
       HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
   }
