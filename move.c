/*
 * move.c
 *
 * Created on: 15 mai 2021
 * Author: CLément Albert & Louis Gounot
 *
 *
 */


#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
//#include "sensors/VL53L0X/VL53L0X.h"
#include <camera/po8030.h>
#include <leds.h>


#include <main.h>
#include <motors.h>

#include <process_image.h>
#include <obstacle.h>

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}


void demi_tour(void){
	   right_motor_set_speed(-SPEED_ROT);
	   left_motor_set_speed(SPEED_ROT);
	   chThdSleepMilliseconds(TIME_HALF_TURN);
}

static THD_WORKING_AREA(waMovement, 1024); //mÃ©moire augmentÃ©e Ã  cause de panic
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
    	time = chVTGetSystemTime();
    	uint8_t temp = get_target_color();

    	if (temp == 1){
			if(get_distance_cm() ){

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
				right_motor_set_speed(SPEED_COEFF*speed - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(SPEED_COEFF*speed + ROTATION_COEFF * speed_correction);

				set_rgb_led(LED8,255,0,0);
			}

			// Retour après detection couleur
			if((!get_distance_cm()) && obstacle_detected()&&(get_camera_height()==LOW_POS)){
				set_rgb_led(LED8,0,0,255);
				demi_tour();
			}

			//si pas de ligne et pas d'obstacle le robot ne bouge pas;
			if((!get_distance_cm()) && (!obstacle_detected())&&(get_camera_height()==LOW_POS)){
				set_rgb_led(LED8,0,255,255);
				po8030_advanced_config(FORMAT_RGB565, 0, LOW_POS, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				set_camera_height(LOW_POS);
				right_motor_set_speed(-SPEED_ROT/4);
				left_motor_set_speed(SPEED_ROT/4);
			}

    	//100Hz
    	chThdSleepUntilWindowed(time, time + MS2ST(20));
    	}
        else{
        	chThdSleepMilliseconds(6000);
        }
    }
}

void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
