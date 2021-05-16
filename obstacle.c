/*
 * ostacle.c
 *
 * Created on: 15 mai 2021
 * Author: Clement Albert & Louis Gounot
 *
 * detection of an obstacle
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <leds.h>

#include <main.h>
#include"sensors/proximity.h"

static uint8_t reached_distance = 0;

static uint8_t obstacle_detection(void) {
	static uint16_t proxi[NUMSENSOR] = { 0 };
	static uint8_t distance_capteurs[NUMSENSOR] = { 0 };

	//linearize the values send by the IR sensors
	for (uint8_t i = 0; i < NUMSENSOR; i++) {
		proxi[i] = get_prox(i);
		distance_capteurs[i] = 530 / pow(proxi[i], 0.98);
	}

	//if a sensor is too close from an obstacle
	if ((distance_capteurs[FRONT_RIGHT] < DIST_OBS_MAX)||
		(distance_capteurs[FRONT_LEFT] < DIST_OBS_MAX)) {

		return REACHED_FRONT;
	}

	else
		return 0;
}

static THD_WORKING_AREA(waObstacle, 256);
static THD_FUNCTION(Obstacle, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	while (1) {

		time = chVTGetSystemTime();
		reached_distance = obstacle_detection();

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}
uint8_t obstacle_detected(void) {
	return reached_distance;
}

void obstacle_start(void) {
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}


