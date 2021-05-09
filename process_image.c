#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <motors.h>
#include <leds.h>

#include <process_image.h>
#include <audio_processing.h>



/*
 * masques de base
 */
//#define MSK_RED1 0b11111000
//#define	MSK_RED2 0b00000000
//#define MSK_GREEN1 0b00000111
//#define	MSK_GREEN2 0b11000000
//#define MSK_BLUE1 0b00000000
//#define	MSK_BLUE2 0b00011111

static float distance_cm = 0.0f; //louis en unint_16
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint8_t target_color= 1; //Target color, 0 = red, 1 = black, 2 = blue // 1 par defaut pour tests
//static uint16_t temp_begin = 0;
//static uint16_t temp_end = 0;
//static uint16_t blueMean = 0;
//static uint16_t redMean = 0;
//static float meanRatio = 0;
//static uint8_t i_red = 0;
//static uint8_t i_blue = 0;
//static uint16_t camera_height = 460; //460 pixels bas louis 100 pour clement

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = 0;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.

		chprintf((BaseSequentialStream *)&SD3, "width= %f\n", width);
	}

	//sets a maximum width or returns the measured width
	chprintf((BaseSequentialStream *)&SD3, "begin= %d\n", begin);
	chprintf((BaseSequentialStream *)&SD3, "end= %d\n", end);
	chprintf((BaseSequentialStream *)&SD3, "width= %d\n", width);
	return width;



}

//uint16_t extract_line_mean(uint8_t *buffer){
//
//	uint16_t i = 0, begin = 0, end = 0, width = 0;
//	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
//	uint32_t mean = 0;
//
//	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;
//
//	//performs an average
//	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
//		mean += buffer[i];
//	}
//	mean /= IMAGE_BUFFER_SIZE;
//
//	do{
//		wrong_line = 0;
//		//search for a begin
//		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
//		{
//			//the slope must at least be WIDTH_SLOPE wide and is compared
//		    //to the mean of the image with an experimental factor: MEAN_CORRECTION
//			if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < MEAN_CORRECTION*mean)
//		    {
//		        begin = i;
//		        stop = 1;
//		    }
//		    i++;
//		}
//		//if a begin was found, search for an end
//		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
//		{
//		    stop = 0;
//
//		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
//		    {
//		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
//		        {
//		            end = i;
//		            stop = 1;
//		        }
//		        i++;
//		    }
//		    //if an end was not found
//		    if (i > IMAGE_BUFFER_SIZE || !end)
//		    {
//		        line_not_found = 1;
//		    }
//		}
//		else//if no begin was found
//		{
//		    line_not_found = 1;
//		}
//
//		//if a line too small has been detected, continues the search
//		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
//			i = end;
//			begin = 0;
//			end = 0;
//			stop = 0;
//			wrong_line = 1;
//		}
//	}while(wrong_line);
//
//	if(line_not_found){
//		begin = 0;
//		end = 0;
//		width = last_width;
//	}else{
//		last_width = width = (end - begin);
//		line_position = (begin + end)/2; //gives the line position.
//	}
//	mean = 0;
//	for(uint16_t i = begin ; i < end ; i++){
//		mean += buffer[i];
//	}
//	temp_end = end;
//	temp_begin = begin;
////	float finalMean = mean/(end-begin);
////	chprintf((BaseSequentialStream *)&SD3, "begin= %d\n", begin);
////	chprintf((BaseSequentialStream *)&SD3, "end= %d\n", end);
////	chprintf((BaseSequentialStream *)&SD3, "finalMean= %d\n", mean);
////	return finalMean;
//	return mean;
//}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;
	//valeurs issues des masques
//	uint8_t red_temp;
//	uint8_t green_temp;
//	uint8_t blue_temp;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();
		if (target_color == 1){

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);
		chprintf((BaseSequentialStream *)&SD3, "lineWidth = %d\n", lineWidth);
		//converts the width into a distance between the robot and the camera
		if(lineWidth>80){
			distance_cm = PXTOCM/lineWidth;
			chprintf((BaseSequentialStream *)&SD3, "distance = %f\n", distance_cm);
		}
		else {
			distance_cm= 0.0;
			chprintf((BaseSequentialStream *)&SD3, "distance = %f\n", distance_cm);

		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
		}
//		else {
//
//			for(uint16_t i = 0; i<IMAGE_BUFFER_SIZE*2; i++){
//				green_temp = (img_buff_ptr[i] & MSK_GREEN1) << 2;
//				blue_temp = (img_buff_ptr[i] & MSK_BLUE1);
//
//				green_temp = (green_temp | ((img_buff_ptr[++i] & MSK_GREEN2) >> 6));
//				blue_temp = (img_buff_ptr[i] & MSK_BLUE2);
//
//				image[i/2] = green_temp + blue_temp;
//			}
//
//			redMean = extract_line_mean(image);
//
//			img_buff_ptr = dcmi_get_last_image_ptr();
//
//			// detecte pixels rouges et verts pour trouver bleu
//			for(uint16_t i = 0; i<IMAGE_BUFFER_SIZE*2; i++){
//				red_temp = (img_buff_ptr[i] & MSK_RED1) >> 3;
//				green_temp = (img_buff_ptr[i] & MSK_GREEN1) << 2;
//				green_temp = (green_temp | ((img_buff_ptr[++i] & MSK_GREEN2) >> 6));
//				image[i/2] = red_temp + green_temp;
//			}
//			blueMean = extract_line_mean(image);
//
//
//			//gets lineWidth in pixels
//			lineWidth = temp_end - temp_begin;
//
//			meanRatio = blueMean/redMean;
//
//
//			// CONDITIONS AVEC ACTIONS SUR LE MOTEUR
//
//			if((blueMean + redMean) > 1){
//				// ratio entre 1 et 5 bleu
//				// si trouve bleu quand c'est demandé alors arrete de tourner et éteint la led et moteur
//				// si pas demandé alors reset le compteur de rouge pour éviter les faux positifs
////					if((meanRatio < 5.5)&&(meanRatio > 0.5)&&(target_color == 2))
//				if((meanRatio < 5.5)&&(meanRatio > 0.5))
//				{
//					if (target_color == 0){
//						i_red = 0;
//					}
//					else{
//						i_blue++;
//						if (i_blue == 5){ //5 est une valeur experimentale a peaufiner pour éviter les erreurs
//							set_rgb_led(LED2,0,0,0);
////							right_motor_set_speed(0);
////							left_motor_set_speed(0);
//							i_blue = 0;
//						}
//					}
//				}
//				// ratio entre 5-5 et 25 rouge
//				// si trouve rouge quand c'est demandé alors arrete de tourner et éteint la led et moteurs
////					else if((meanRatio < 25)&&(meanRatio > 5.5)&&(target_color == 0))
//				else if((meanRatio < 25)&&(meanRatio > 5.5))
//				{
//					if (target_color == 2){
//						i_blue = 0;
//					}
//					else{
//						i_red++;
//						if (i_red == 5){
//							set_rgb_led(LED2,0,0,0);
////							right_motor_set_speed(0);
////							left_motor_set_speed(0);
//							i_red = 0;
//						}
//					}
//				}
//			}
////	//converts the width into a distance between the robot and the camera
////	if(lineWidth){
////		distance_cm = PXTOCM/lineWidth;
////	}
//		}
    }
}

uint16_t get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

//void select_target_color(uint8_t color_id) {
//	switch (color_id){
//			//choisit le rouge et tourne pour le trouver
//			case 0:
//				target_color = 0;
////				right_motor_set_speed(-100);
////				left_motor_set_speed(100);
//				camera_height = 100;
//				po8030_set_awb(0);
//				//régule contraste avec cste 0< <255
//				po8030_set_contrast(55);
//
//			  break;
//			  // suivi ligne noire
//			case 1:
//				target_color = 1;
//				camera_height = 460;
//				po8030_set_awb(1);
//				//régule contraste avec cste 0< <255
//				po8030_set_contrast(74);
//
//			  break;
//			 //choisit le bleu et tourne pour le trouver
//			case 2:
//				target_color = 2;
////				right_motor_set_speed(-100);
////				left_motor_set_speed(100);
//				camera_height = 100;
//				po8030_set_awb(0);
//				//régule contraste avec cste 0< <255
//				po8030_set_contrast(55);
//			  break;
//		}
//}

uint8_t get_target_color(void) {
	return target_color;
}
