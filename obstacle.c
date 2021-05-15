/*
 * 	obstacle.c
 *
 *  Created on: 15 may 2021
 *  Author: Clément Albert & Louis Gounot
 *
 *  return values if there is an obstacle
 */


#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <leds.h>


#include <main.h>
#include"sensors/proximity.h"
static uint8_t reached_distance=0;

static uint8_t obstacle_detection(void){
	static uint16_t proxi[NUMSENSOR]={0};
	static uint8_t distance_capteurs[NUMSENSOR]={0};



	//linéarize the values send by the IR sensors
	for (uint8_t i=0; i<NUMSENSOR;i++){
    	proxi[i]=get_prox(i);
    	distance_capteurs[i]=530/pow(proxi[i],0.98);
    }


   	//epuck too close from an obstacle
	if((distance_capteurs[FRONT_RIGHT]<DIST_OBS_MAX)||(distance_capteurs[FRONT_LEFT]<DIST_OBS_MAX)){
		return REACHED_FRONT;
	}
	else return 0;
    

       
}

static THD_WORKING_AREA(waObstacle, 256);
static THD_FUNCTION(Obstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){

      	time = chVTGetSystemTime();
      	reached_distance=obstacle_detection();

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}
//send if the epuck is next to an obstacle to the file move.c
 uint8_t obstacle_detected(void){
 	return reached_distance;
 }

 void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}


