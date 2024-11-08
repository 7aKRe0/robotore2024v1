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
void flag(void){
	//printf("side_r_flag: %d, side_l_flag: %d\n\r", side_r_flag, side_l_flag);
//	printf("side_r_flag: %d, side_l_flag: %d, stop_flag: %d\n\r", side_r_flag, side_l_flag, stop_flag);

	  if(Line3_sens[0] > threshold_0){
		  current_ftime = HAL_GetTick();
			side_l_flag = 1;
			side_l_time = 0;
	  }
	  if(side_l_flag == 1){
		  side_l_time++;
	  }
	  if(side_l_time >= 70){
		  side_l_time = 0;
		  side_l_flag = 0;
		  }
	  if(Line3_sens[1] > threshold_1){
		  side_r_flag = 1;
//		  playSound(1000, 100,0.9);
	  }else{
		  side_r_flag = 0;
	  }
	  if(side_r_flag == 1 && side_l_flag == 0 ){
//			stop_flag++;
//			playSound(1000, 100,0.9);

		  De_time = HAL_GetTick();
		  if(De_time - De_last > 80){

			De_last = De_time;

			if(stop_flag < 2){
			stop_flag++;
			playSound(1000, 100,0.9);
			}
		  }
	  }

	  if(stop_flag >= 2){
		  //controlMotor(0,0);
		  base_speed = 0;


	  }
 }

void flag2(void){
	uint32_t current_2ftime = HAL_GetTick();
	  if(Line3_sens[0] > threshold_0){
			if (current_2ftime - last_sens_time > 300) {
				side_l_flag = 1;
				side_l_time = 0;
				playSound(1000, 100,0.9);
				secondLayerRun();
				last_sens_time = current_2ftime;
			}
	  }else{
		  return;
	  }
	  if(side_l_flag == 1){
		  side_l_time++;
	  }
	  if(side_l_time >= 70){
		  side_l_time = 0;
		  side_l_flag = 0;
		  }
	  if(Line3_sens[1] > threshold_1){
		  side_r_flag = 1;
	  }else{
		  side_r_flag = 0;
	  }
	  if(side_r_flag == 1 && side_l_flag == 0 ){
//		  stop_flag++;

		  De_time = HAL_GetTick();
		 		  if(De_time - De_last > 80){
		 			De_last = De_time;
		 		  }else{
		 			  return;
		 		  }
	  }

	  while(stop_flag >= 2){
		  //controlMotor(0,0);
		  base_speed = 0;

			button();
			if(stop_flag == 0){
				break;
			}


	  }
 }
