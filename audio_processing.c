#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <process_image.h>

#include <leds.h>


//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
static uint8_t compteur_rouge = 0 ;
static uint8_t compteur_bleu = 0 ;

#define MIN_VALUE_THRESHOLD	10000

#define MIN_FREQ		22	//we don't analyze before this index to not use resources for nothing
#define FREQ_RED	    26	//406Hz to find red
#define FREQ_GREEN		32	//850Hz noise range that souhld not be detected
#define FREQ_BLUE		38	//602Hz to find blue
#define MAX_FREQ		42	//we don't analyze after this index to not use resources for nothing

//Smalls bandwith for colors
#define FREQ_RED_L		(FREQ_RED-1)
#define FREQ_RED_H		(FREQ_RED+1)
#define FREQ_BLUE_L		(FREQ_BLUE-1)
#define FREQ_BLUE_H		(FREQ_BLUE+1)
// Large bandwidth for noise
#define FREQ_GREEN_L	(FREQ_GREEN-5)
#define FREQ_GREEN_H	(FREQ_GREEN+5)

/*
*	Detects the highest value in a buffer and starts color detection if
*	the detected frequency corresponds to red or blue
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//Target red
	if(max_norm_index >= FREQ_RED_L && max_norm_index <= FREQ_RED_H){

		if (compteur_bleu == 0){
			compteur_rouge++;
		}
		else {
			compteur_bleu = 0;
		}
		//red detected 21 time => start research
		if (compteur_rouge > 20){
			compteur_rouge = 0;
			select_target_color(0);
		}

	}
	//reset if noise detected
	else if(max_norm_index >= FREQ_GREEN_L && max_norm_index <= FREQ_GREEN_H){
		compteur_rouge = 0;
		compteur_bleu = 0;
	}
	//Target blue
	else if(max_norm_index >= FREQ_BLUE_L && max_norm_index <= FREQ_BLUE_H){
		if (compteur_rouge == 0){
			compteur_bleu++;
		}
		else {
			compteur_rouge = 0;
		}
		//blue detected 21 time => start research
		if (compteur_bleu > 20){
			compteur_bleu = 0;
			select_target_color(2);
		}
		po8030_set_rgb_gain(0x50, 0x50,0x00);
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		sound_remote(micLeft_output);
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
