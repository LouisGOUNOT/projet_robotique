#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include "sensors/proximity.h"
#include "spi_comm.h"
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
  	/** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //Start the SPI Communication
    spi_comm_start();

    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

	//start the threads for the detector of proximity
	proximity_start();
	
	static uint16_t proxi[NUMBER_OF_SENSORS]



    /* Infinite loop. */
    while (1) {
    		for (uint8_t i=0; i<NUMBER_OF_SENSORS;i++){
	proxi(i)=get_proxi(i);
	chprintf((BaseSequentialStream *)&SD3, "proximit= %d\n", proxi(i));
}
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
