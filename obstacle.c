#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include"sensors/proximity.h"

static 
uint8_t obstacle_detection(void){
	static uint16_t proxi[NUMSENSOR]=0;
	static uint8_t distance_capteurs[NUMSENSOR]=0;


	//linéarise les valeurs renvoyes par les capteurs de proximité
	for (uint8_t i=0; i<NUMSENSOR;i++){
    	proxi[i]=get_prox(i);
    	distance_capteurs[i]=529,7/pow(proxi[i],0.98);			
    }

   	//quand on arrive devant un objet et que c'est un des capteurs de droitent qui détectent l'objet
    for (int i=0;i<3;i++){
    	if (distance_capteurs[i]<DIST_OBS_MAX)
    		return REACHED_RIGHT;
    }

		//	
    for (int i=5;i<8;i++){
    	if (distance_capteurs[i]<DIST_OBS_MAX)
    		return REACHED_FRONT;
    }    
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
 uint8_t distance_detected(void){
 	return reached_distance;
 }

 void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}


