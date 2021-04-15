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
#define WIDTH_SLOPE				25
#define MIN_LINE_WIDTH			IMAGE_BUFFER_SIZE/20
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570 //experimental value
#define GOAL_DISTANCE 			15
#define MAX_DISTANCE 			30
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
