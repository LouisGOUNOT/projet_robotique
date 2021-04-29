#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include"sensors/proximity.h"
static uint8_t reached_distance=0;

static uint8_t obstacle_detection(void){
	static uint16_t proxi[NUMSENSOR]={0,0,0,0,0,0,0,0};
	static uint8_t distance_capteurs[NUMSENSOR]={0,0,0,0,0,0,0,0};



	//linéarise les valeurs renvoyes par les capteurs de proximité
	for (uint8_t i=0; i<NUMSENSOR;i++){
    	proxi[i]=get_prox(i);
    	distance_capteurs[i]=530/pow(proxi[i],0.98);
    	//chprintf((BaseSequentialStream *)&SD3, "distance_capteurs devant droit%d\n", distance_capteurs[7]);
    }


   	//capteurs trop proches d'un objet
    	if ((distance_capteurs[FRONT_RIGHT]<DIST_OBS_MAX)||(distance_capteurs[FRONT_LEFT]<DIST_OBS_MAX)){
    	    //chprintf((BaseSequentialStream *)&SD3, "distance_capteurs devant droit%d\n et capteurs devant gauche %d\n", distance_capteurs[0],distance_capteurs[7]);
			return REACHED_FRONT;
    	}


    	else if  (distance_capteurs[FRONT_RIGHT45]<DIST_OBS_MAX)
    		 return REACHED_RIGHT45;

    	

    	else if (distance_capteurs[LEFT_RIGHT45]<DIST_OBS_MAX)
    		return REACH_LEFT45;

    	else return 0;
    		//chprintf((BaseSequentialStream *)&SD3, "distance_capteurs devant droit%d\n et capteurs devant gauche %d\n", distance_capteurs[0],distance_capteurs[7]);
    		//return REACHED_FRONT;
    	//else return 0;
    
		//	
    //f//or (int i=5;i<8;i++){
    //	if (distance_capteurs[i]<DIST_OBS_MAX)
    //		return REACHED_LEFT;
    	

   // }   
     	
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
 uint8_t obstacle_detected(void){
 	return reached_distance;
 }

 void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}


