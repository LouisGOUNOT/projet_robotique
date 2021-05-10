#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
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

//static THD_WORKING_AREA(waPiRegulator, 1024); //mémoire augmentée à cause de panic
//static THD_FUNCTION(PiRegulator, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//    systime_t time;
//
//    int16_t speed = 0;
//    int16_t speed_correction = 0;
//
//    while(1){
//    	time = chVTGetSystemTime();
//    	uint8_t temp = get_target_color();
//		chprintf((BaseSequentialStream *)&SD3, "target color vu par pi %d\n", temp);
//    	if (temp == 1){
//
//    		chprintf((BaseSequentialStream *)&SD3, "dans if temp = 1");
//			if(get_distance_cm() ){
//
//
//				chprintf((BaseSequentialStream *)&SD3, "jesuisdangetlineposition");
//
//				//computes the speed to give to the motors
//				//distance_cm is modified by the image processing thread
//				speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
//				//computes a correction factor to let the robot rotate to be in front of the line
//				speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
//
//				//if the line is nearly in front of the camera, don't rotate
//				if(abs(speed_correction) < ROTATION_THRESHOLD){
//					speed_correction = 0;
//				}
//
//				//applies the speed from the PI regulator and the correction for the rotation
////				right_motor_set_speed(0.4*speed - ROTATION_COEFF * speed_correction);
////				left_motor_set_speed(0.4*speed + ROTATION_COEFF * speed_correction);
//				right_motor_set_speed(0.2*speed - ROTATION_COEFF * speed_correction);
//				left_motor_set_speed(0.2*speed + ROTATION_COEFF * speed_correction);
//			}
//
//
//
//
//			if((!get_distance_cm()) && obstacle_detected()){
//				chprintf((BaseSequentialStream *)&SD3, "pas de ligne et obstacle");
//
//			   right_motor_set_speed(-800);
//			   left_motor_set_speed(800);
//				chThdSleepMilliseconds(820);
//
//			}
//			//si pas de ligne et pas d'obstacle le robot ne bouge pas;
//			else if((!get_distance_cm()) && (!obstacle_detected())){
//				chprintf((BaseSequentialStream *)&SD3, "non obstacle et ligne");
//				right_motor_set_speed(0);
//				left_motor_set_speed(0);
//			}
//    	}
//    	//100Hz
//    	chThdSleepUntilWindowed(time, time + MS2ST(10));
//
//    }
//
//}

//void pi_regulator_start(void){
//	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
//}
