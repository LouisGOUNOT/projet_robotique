/*
 * main.c
 *
 * Created on: 15 mai 2021
 * Author: Clement Albert & Louis Gounot
 *
 * init threads and configs
 */
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
#include "sensors/VL53L0X/VL53L0X.h"
#include <spi_comm.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <audio/microphone.h>
#include <leds.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <arm_math.h>


#include <audio_processing.h>
#include <fft.h>
#include <process_image.h>
#include <obstacle.h>
#include <move.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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

//send the FFTs results from the real microphones
#define SEND_FROM_MIC

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    // Inits the Inter Process Communication bus. //
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
    //stars the threads for the pi regulator and the processing of the image
    process_image_start();

    //inits the motors & displacement
    motors_init();
	movement_start();

    //start the threads for the detector of proximity
    proximity_start();
    obstacle_start();

    //start the threads for the audio output
	// Powers ON the alimentation of the speaker
	dac_power_speaker(true);
	dac_start();
	//creates the Melody thread
	playMelodyStart();

	//starts the microphones processing thread.
	//it calls the callback given in parameter when samples are ready
	mic_start(&processAudioData);

	//Unused but does not work if we remove it
	VL53L0X_start();

    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
