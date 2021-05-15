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
#define WIDTH_SLOPE				20 // 20 avant chez louis peut ne pas marcher, 5 chez clement
#define WIDTH_SLOPE_COLOR		5 // 20 avant chez louis peut ne pas marcher, 5 chez clement
#define MIN_LINE_WIDTH			60// 40 avant chez clement peut ne pas marcher, 60 chez louis
#define MIN_LINE_WIDTH_COLOR	40 // 40 avant chez clement peut ne pas marcher, 60 chez louis
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define SPEED_COEFF				0.50f
#define PXTOCM_BLACK_LINE		1200.0f //experimental value
#define PXTOCM_COLOR			700.0f //experimental value
#define GOAL_DISTANCE 			4.0f
#define MAX_DISTANCE 			30.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#define TRAVELTIMECOEFF 		2700/10.32	//adjusts travel_time, experimental value
#define COMPTE_TOUR_MAX 		640	//experimental value


#define NUMSENSOR               8
#define REACHED_FRONT           1
#define REACHED_FRONT45         2
#define REACHED_LEFT          	3
#define REACHED_RIGHT           4

#define DIST_OBS_MAX            3 // [cm]

//numero des différents capteur de proximité
#define FRONT_RIGHT             0
#define FRONT_RIGHT45           1
#define RIGHT                   2
#define BACK_RIGHT              3
#define BACK_LEFT               4
#define LEFT                    5
#define FRONT_LEFT45            6
#define FRONT_LEFT              7

#define VITESSENULLE			0
#define ROTATIONNULLE			0
#define SPEED_ROT					800
#define TIME_HALF_TURN			600 // [Milliseconds]

//Camera
#define MEAN_CORRECTION  		1.0f //experimental value
#define LOW_POS					460
#define HIGH_POS				100

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
