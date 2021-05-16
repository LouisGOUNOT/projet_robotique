/*
 * main.h
 *
 * Created on: 15 mai 2021
 * Author: Clement Albert & Louis Gounot
 *
 * init threads and configs
 */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				20 // black line
#define WIDTH_SLOPE_COLOR		5 // color target
#define MIN_LINE_WIDTH			60// black line
#define MIN_LINE_WIDTH_COLOR	40 // color target
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define SPEED_COEFF				0.27f
#define PXTOCM_BLACK_LINE		1200.0f //experimental value
#define PXTOCM_COLOR			700.0f //experimental value
#define GOAL_DISTANCE 			4.0f// [cm]
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define TRAVELTIMECOEFF 		2500/10.32	//adjusts travel_time, experimental value
#define COMPTE_TOUR_MAX 		640	//experimental value
#define DIST_OBS_MAX            3 // [cm]

#define NUMSENSOR               8
#define REACHED_FRONT           1

//proximity sensors id
#define FRONT_RIGHT             0
#define FRONT_LEFT              7

#define SPEED_ROT				800
#define TIME_HALF_TURN			600 // [Milliseconds]

//Camera
#define LOW_POS					460
#define HIGH_POS				100
#define LINEWIDTH_MODE			0
#define MEAN_MODE				1

//Masks used as camera filters
#define MSK_RED1 				0b11111000
#define	MSK_RED2 				0b00000000
#define MSK_GREEN1 				0b00000111
#define	MSK_GREEN2 				0b11000000
#define MSK_BLUE1 				0b00000000
#define	MSK_BLUE2 				0b00011111


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
