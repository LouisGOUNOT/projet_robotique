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