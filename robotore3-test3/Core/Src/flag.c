/*
 * flag.c
 *
 *  Created on: Sep 23, 2024
 *      Author: uchuu
 */
#include <main.h>
#include <test.h>
#include <stdio.h>
#include <string.h>


//void detectTapeAndAdjustSpeed() {
//	  if (mode != 1 && !firstLapComplete) {
//         current_time = HAL_GetTick(); // 現在の時間を取????��?��??��?��???��?��??��?��?
//
//         tape_interval = current_time - last_tape_time;
//         if (tape_interval > 2000) {
//         	tape_list[section_index]= 3;
//         } else{
//         	tape_list[section_index] = 1;
//         }
//         printf("tape_list[%d] = %d\r\n", section_index, tape_list[section_index]);
//         section_index++;
//         last_tape_time = current_time;
//
//	  }
// }
int box = -1;
// void secondLayerRun() {
//     if(section_index > box){
//   	  box++;
//     }
//         if (tape_list[box] == 3) {
//             base_speed = max_speed;
//         } else {
//             // カーブ区間なら�?????��?��??��?��???��?��??��?��常速度
//             base_speed = 600;
//         }
//
//         printf("tape_list[%d] = %d\r\n",box, tape_list[box]);
// }

 uint32_t last_sens_time = 0;
 uint32_t De_last,De_time = 0;

 uint32_t cross_disable_time = 0;
 uint32_t cross_disable_duration = 200; // 無効


 int isSensorDisabled() {
     return (HAL_GetTick() - cross_disable_time < cross_disable_duration);
 }

 void flag(void) {
//	 threshold_0=2000;
//	 threshold_1=3780;
//	 HAL_Delay(50);
	 cross_time = 100;

	 if (isSensorDisabled()) {
		return;
	}

	 if (Line3_sens[0] > threshold_0) { // 左認識
		 side_l_time = HAL_GetTick();
		 side_l_flag = 1;
	 }

	 if (Line3_sens[1] > threshold_1) { // 右認識
		 side_r_time = HAL_GetTick();
		 side_r_flag = 1;
	 }

	 if (side_l_flag == 1 && (HAL_GetTick() - side_l_time >= cross_time - 50) && side_r_flag == 0) {
		 side_l_flag = 0;
//		 printf("111\r\n");
	 }

	 if(side_l_flag == 1){
		 if((HAL_GetTick() - side_l_time < cross_time) && side_r_flag == 1 && (Line3_sens[1] <= threshold_1 || Line3_sens[0] <= threshold_0)){
			 side_r_flag = 0;
			 side_l_flag = 0;
			 cross_disable_time = HAL_GetTick();
//			 playSound(1000, 100,0.9);

//			 printf("333\r\n");
		 }
	 }

	 if (side_r_flag == 1) {
		 if ((HAL_GetTick() - side_r_time < cross_time) && side_l_flag == 1 && (Line3_sens[1] <= threshold_1 || Line3_sens[0] <= threshold_0)) {
			 side_r_flag = 0;
			 side_l_flag = 0;
			 cross_disable_time = HAL_GetTick();
//			 playSound(1000, 100,0.9);

//			 printf("1222\r\n");
		 } else if (HAL_GetTick() - side_r_time >= cross_time && Line3_sens[1] <= threshold_1 && side_l_flag == 0) {
			 stop_flag++;
			 side_r_flag = 0;
			 playSound(1000, 100,0.9);
//			 printf("Stop flag incremented: stop_flag=\r\n");
		 }
	 }

	 if (stop_flag >= 2) {
		 base_speed = 0;
		 SpeedControl();

		 printf("GG stop_flag\r\n");
	//         stop_flag = 0;
	 }
	}




//
//void flag2(void){
//	uint32_t current_2ftime = HAL_GetTick();
//	  if(Line3_sens[0] > threshold_0){
//			if (current_2ftime - last_sens_time > 300) {
//				side_l_flag = 1;
//				side_l_time = 0;
//				playSound(1000, 100,0.9);
//				secondLayerRun();
//				last_sens_time = current_2ftime;
//			}
//	  }else{
//		  return;
//	  }
//	  if(side_l_flag == 1){
//		  side_l_time++;
//	  }
//	  if(side_l_time >= 70){
//		  side_l_time = 0;
//		  side_l_flag = 0;
//		  }
//	  if(Line3_sens[1] > threshold_1){
//		  side_r_flag = 1;
//	  }else{
//		  side_r_flag = 0;
//	  }
//	  if(side_r_flag == 1 && side_l_flag == 0 ){
////		  stop_flag++;
//
//		  De_time = HAL_GetTick();
//		 		  if(De_time - De_last > 80){
//		 			De_last = De_time;
//		 		  }else{
//		 			  return;
//		 		  }
//	  }
//
//	  while(stop_flag >= 2){
//		  //controlMotor(0,0);
//		  base_speed = 0;
//
//			button();
//			if(stop_flag == 0){
//				break;
//			}
//
//
//	  }
// }
