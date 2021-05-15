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
#include <leds.h>


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
				right_motor_set_speed(0.27*speed - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(0.27*speed + ROTATION_COEFF * speed_correction);

				set_rgb_led(LED8,255,0,0);

			}

// Retour apr�s detection couleur
			if((!get_distance_cm()) && obstacle_detected()&&(get_camera_height()==460)){

//demi tour
				set_rgb_led(LED8,0,0,255);

			   right_motor_set_speed(-800);
			   left_motor_set_speed(800);
			   chThdSleepMilliseconds(680);

			}

			//si pas de ligne et pas d'obstacle le robot ne bouge pas;
			if((!get_distance_cm()) && (!obstacle_detected())&&(get_camera_height()==460)){

				set_rgb_led(LED8,0,255,255);

				po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				set_camera_height(460);

				set_dist_retour(1.0f);
				right_motor_set_speed(-200);
				left_motor_set_speed(200);
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
