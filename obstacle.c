#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include"sensors/proximity.h"

//static uint16_t proxi[NUMBER_OF_SENSORS]

uint8_t distance_obstacle_cm(void){

	for (uint8_t i=0; i<NUMBER_OF_SENSORS;i++){
	proxi(i)=get_proxi(i);
	chprintf((BaseSequentialStream *)&SD3, "proximit= %d\n", proxi(i));

	}
}