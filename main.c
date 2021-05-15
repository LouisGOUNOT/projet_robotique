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


#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <obstacle.h>
#include <move.h>

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
    chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
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

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
//#define DOUBLE_BUFFERING


//static void timer12_start(void){
//    //General Purpose Timer configuration
//    //timer 12 is a 16 bit timer so we can measure time
//    //to about 65ms with a 1Mhz counter
//    static const GPTConfig gpt12cfg = {
//        1000000,        /* 1MHz timer clock in order to measure uS.*/
//        NULL,           /* Timer callback.*/
//        0,
//        0
//    };
//
//    gptStart(&GPTD12, &gpt12cfg);
//    //let the timer count to max value
//    gptStartContinuous(&GPTD12, 0xFFFF);
//}

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
	//enable auto white balance
//	po8030_set_awb(0);
//	//régule contraste avec cste 0< <255
//	po8030_set_contrast(55);
//    //starts timer 12
//    timer12_start();
    //inits the motors
    motors_init();
    //stars the threads for the pi regulator and the processing of the image
//    pi_regulator_start();
    process_image_start();
    //start the threads for the detector of proximity
    proximity_start();

    obstacle_start();
    //temp tab used to store values in complex_float format
    //needed bx doFFT_c
    static complex_float temp_tab[FFT_SIZE];
    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];
	//right_motor_set_speed(-100);
	//left_motor_set_speed(100);
//		//starts the microphones processing thread.
//		//it calls the callback given in parameter when samples are ready
		mic_start(&processAudioData);
	    movement_start();
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
