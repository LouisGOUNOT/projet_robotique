/*
 * move.c
 *
 *  Created on: 10 mai 2021
 *      Author: clema
 */


#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include <camera/po8030.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <obstacle.h>
#include<pi_regulator.h>


static THD_WORKING_AREA(waMovement, 1024); //mémoire augmentée à cause de panic
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
    	time = chVTGetSystemTime();
    	uint8_t temp = get_target_color();
    	uint8_t demitour_fait=0;
//		chprintf((BaseSequentialStream *)&SD3, "target color vu par pi %d\n", temp);
    	if (temp == 1){

//    		chprintf((BaseSequentialStream *)&SD3, "dans if temp = 1");
			if(get_distance_cm() ){


//				chprintf((BaseSequentialStream *)&SD3, "jesuisdangetlineposition");

				//computes the speed to give to the motors
				//distance_cm is modified by the image processing thread
				speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
				//computes a correction factor to let the robot rotate to be in front of the line
				speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

				//if the line is nearly in front of the camera, don't rotate
				if(abs(speed_correction) < ROTATION_THRESHOLD){
					speed_correction = 0;
				}

				//applies the speed from the PI regulator and the correction for the rotation
//				right_motor_set_speed(0);
//				left_motor_set_speed(0);
				right_motor_set_speed(0.4*speed - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(0.4*speed + ROTATION_COEFF * speed_correction);

				if (get_camera_height()==100){
//					if (speed_correction){
//					right_motor_set_speed(- ROTATION_COEFF * speed_correction);
//					left_motor_set_speed( + ROTATION_COEFF * speed_correction);
//					}
//					else{
						float travel_time = get_distance_cm()*3000/10.32;
						uint8_t i =0;
						while ( i <100){
							chprintf((BaseSequentialStream *)&SD3, "traveltime=%f\n",travel_time);
							i++;
							chThdSleepMilliseconds(10);

						}

						chprintf((BaseSequentialStream *)&SD3, "traveltime=%f\n",travel_time);
						   right_motor_set_speed(800);
						   left_motor_set_speed(800);
							chThdSleepMilliseconds(travel_time);
							right_motor_set_speed(-800);
							left_motor_set_speed(800);
							chThdSleepMilliseconds(820);
						    right_motor_set_speed(800);
						    left_motor_set_speed(800);
							chThdSleepMilliseconds(travel_time);
							set_camera_height(460);
							po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

					//}

				}


				}
//				chprintf((BaseSequentialStream *)&SD3, "vu par time of flight =  %d\n", VL53L0X_get_dist_mm());
//				if (VL53L0X_get_dist_mm() < 80){
////					chprintf((BaseSequentialStream *)&SD3, "rentre dans ime of flight");
//					   right_motor_set_speed(-800);
//					   left_motor_set_speed(800);
//					   chThdSleepMilliseconds(820);
//				}
	}



// Retour apr�s detection couleur
			if((!get_distance_cm()) && obstacle_detected()&&(get_camera_height()==460)){
//				chprintf((BaseSequentialStream *)&SD3, "pas de ligne et obstacle");
//demi tour
			   right_motor_set_speed(-800);
			   left_motor_set_speed(800);
			   chThdSleepMilliseconds(820);
//				retourne sur ligne
//			   po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

			}

//			if ((get_camera_height()==100)&& (demitour_fait ==1)){
//				   right_motor_set_speed(0);
//				   left_motor_set_speed(0);
//				   chThdSleepMilliseconds(1000);
//
//				   float travel_time = get_dist_retour()*1000/10.32;
//					chprintf((BaseSequentialStream *)&SD3, "get dist_retour =  %f\n", get_dist_retour());
//					chprintf((BaseSequentialStream *)&SD3, "travel time =  %f\n", travel_time);
//				   right_motor_set_speed(800);
//				   left_motor_set_speed(800);
//				   chThdSleepMilliseconds(travel_time);
//
//				   right_motor_set_speed(0);
//				   left_motor_set_speed(0);
//				   chThdSleepMilliseconds(1000);
//
//				   po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
//					set_camera_height(460);
//			}

			//si pas de ligne et pas d'obstacle le robot ne bouge pas;
			if((!get_distance_cm()) && (!obstacle_detected())&&(get_camera_height()==460)){
//				chprintf((BaseSequentialStream *)&SD3, "non obstacle et ligne");
				po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				set_camera_height(460);

				set_dist_retour(1.0f);
				right_motor_set_speed(-200);
				left_motor_set_speed(200);
			}

    	//100Hz
    	chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}




void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
