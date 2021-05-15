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
#include "memory_protection.h"



/*
 * masques de base A METTRE DANS MAIN.H
 */
#define MSK_RED1 0b11111000
#define	MSK_RED2 0b00000000
#define MSK_GREEN1 0b00000111
#define	MSK_GREEN2 0b11000000
#define MSK_BLUE1 0b00000000
#define	MSK_BLUE2 0b00011111

static float distance_cm = 0.0f; //louis en unint_16
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint8_t target_color= 1; //Target color, 0 = red, 1 = black, 2 = blue // 1 par defaut pour tests
static uint16_t temp_begin = 0;
static uint16_t temp_end = 0;
static uint16_t blueMean = 0;
static uint16_t redMean = 0;
static float meanRatio = 0;
static uint8_t i_red = 0;
static uint8_t i_blue = 0;
static uint16_t camera_height = 460; //460 pixels bas louis 100 pour clement
static float pxtocm = PXTOCM_BLACK_LINE;
static uint16_t dist_retour = 11;
static uint16_t compte_tour = 0;

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

	static uint16_t last_width = PXTOCM_BLACK_LINE/GOAL_DISTANCE;

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

	}

	return width;



}

uint16_t extract_line_mean(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

//	static uint16_t last_width = PXTOCM_BLACK_LINE/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE_COLOR))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image with an experimental factor: MEAN_CORRECTION
			if(buffer[i] > mean && buffer[i+WIDTH_SLOPE_COLOR] < MEAN_CORRECTION*mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE_COLOR) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE_COLOR] < mean)
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
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH_COLOR){
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
	 width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}
	mean = 0;
	for(uint16_t i = begin ; i < end ; i++){
		mean += buffer[i];
	}
	temp_end = end;
	temp_begin = begin;
	return mean;
}

void wait_ms(uint16_t time_ms){
     for(uint32_t i = 0 ; i < 21000000*time_ms/500 ; i++){
         __asm__ volatile ("nop");
     }
}

//Goes to the target then goes back to the black line
void go_to_target(void){

	pxtocm = PXTOCM_COLOR;
	camera_height = 100;
	//travel_time corresponds to the time needed to reach the target at motor speed of 800
	uint16_t travel_time = distance_cm*TRAVELTIMECOEFF;

	right_motor_set_speed(0);
	left_motor_set_speed(0);
	wait_ms(100);
	right_motor_set_speed(800);
	left_motor_set_speed(800);
	wait_ms(travel_time);
	right_motor_set_speed(-800);
	left_motor_set_speed(800);
	wait_ms(675);
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	wait_ms(100);
	right_motor_set_speed(800);
	left_motor_set_speed(800);
	wait_ms(travel_time);
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	wait_ms(100);
//	//goes back on black line tracking mode
//	select_target_color(1);

}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Initialise la hauteur pour ligne noire au démarrage
	po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
//    	Change la hauteur avant de scanner si besoin
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
	uint8_t red_temp;
	uint8_t green_temp;
	uint8_t blue_temp;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    

		img_buff_ptr = dcmi_get_last_image_ptr();

		//Looking for black line
		if ((target_color == 1)&&(camera_height == 460)){

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);
		//converts the width into a distance between the robot and the camera
		if(lineWidth>80){
			distance_cm = pxtocm/lineWidth;
		}
		else {
			distance_cm= 0.0f;
		}

		}
		//Looking for color line
		else {
				compte_tour++;
				for(uint16_t i = 0; i<IMAGE_BUFFER_SIZE*2; i++){
					green_temp = (img_buff_ptr[i] & MSK_GREEN1) << 2;
					blue_temp = (img_buff_ptr[i] & MSK_BLUE1);

					green_temp = (green_temp | ((img_buff_ptr[++i] & MSK_GREEN2) >> 6));
					blue_temp = (img_buff_ptr[i] & MSK_BLUE2);

					image[i/2] = green_temp + blue_temp;
				}

				redMean = extract_line_mean(image);

				img_buff_ptr = dcmi_get_last_image_ptr();

				// detecte pixels rouges et verts pour trouver bleu
				for(uint16_t i = 0; i<IMAGE_BUFFER_SIZE*2; i++){
					red_temp = (img_buff_ptr[i] & MSK_RED1) >> 3;
					green_temp = (img_buff_ptr[i] & MSK_GREEN1) << 2;
					green_temp = (green_temp | ((img_buff_ptr[++i] & MSK_GREEN2) >> 6));
					image[i/2] = red_temp + green_temp;
				}
				blueMean = extract_line_mean(image);

				//gets lineWidth in pixels
				lineWidth = temp_end - temp_begin;
				if(lineWidth>50){
					distance_cm = pxtocm/lineWidth;

				meanRatio = blueMean/redMean;
				}
				else{
					meanRatio = 0;
				}

				if(((blueMean + redMean) > 1)&&(lineWidth>MIN_LINE_WIDTH_COLOR)&&((get_line_position() - (IMAGE_BUFFER_SIZE/2))<50)){

					if((meanRatio > 0)&&(meanRatio  < 3)&&(target_color  == 2))
					{
						if (target_color == 0){
							i_red = 0;
						}
						else{
							i_blue++;
						}
						if (i_blue>2){
							go_to_target();
							compte_tour = COMPTE_TOUR_MAX + 1;
						}
					}

					else if((meanRatio  > 2)&&(target_color  == 0)){
						if (target_color == 2){
							i_blue = 0;
						}
						else{
							i_red++;
						}
						if (i_red>2){
								go_to_target();
								compte_tour = COMPTE_TOUR_MAX + 1;
						}
					}
			}
				//compte_tour is incremented every time an image is captured. When it reaches its maximum, it means 360 degrees were covered
				//without finding the target.
				if (compte_tour > COMPTE_TOUR_MAX){
					compte_tour = 0;
					set_rgb_led(LED2,0,0,0);
					set_rgb_led(LED8,0,0,0);
					//goes back on black line tracking mode
					select_target_color(1);
				}
		}
    }
}


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+1, CaptureImage, NULL);
}

void select_target_color(uint8_t color_id) {
	switch (color_id){
			//choisit le rouge et tourne pour le trouver
			case 0:
				if(target_color == 1){
					right_motor_set_speed(0);
					left_motor_set_speed(0);
					camera_height = 100;
					pxtocm = PXTOCM_COLOR;
					po8030_advanced_config(FORMAT_RGB565, 0, 100, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
					po8030_set_awb(0);
					po8030_set_contrast(55);
			    	//Capture une image pour réactualiser le buffer et éviter de detecter la ligne noire dès le debut
			        chBSemWait(&image_ready_sem);
			        uint8_t *img_buff_ptr = dcmi_get_last_image_ptr();
					target_color = 0;
					set_rgb_led(LED2,255,0,0);
					right_motor_set_speed(-50);
					left_motor_set_speed(50);
				}


			  break;
			  // suivi ligne noire
			case 1:
				target_color = 1;
				camera_height = 460;
				//régule contraste avec cste 0< <255
				pxtocm = PXTOCM_BLACK_LINE;


				//setup la camera
				po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				po8030_set_awb(0);
				po8030_set_contrast(74);
				set_rgb_led(LED4,255,255,255);

			  break;
			 //choisit le bleu et tourne pour le trouver
			case 2:
				if(target_color == 1){
					right_motor_set_speed(0);
					left_motor_set_speed(0);
					camera_height = 100;
					pxtocm = PXTOCM_COLOR;
					po8030_advanced_config(FORMAT_RGB565, 0, 100, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
					po8030_set_awb(0);
					po8030_set_contrast(55);
			    	//Capture une image pour réactualiser le buffer et éviter de detecter la ligne noire dès le debut
			        chBSemWait(&image_ready_sem);
					uint8_t *img_buff_ptr = dcmi_get_last_image_ptr();
					target_color = 2;
					set_rgb_led(LED2,0,0,255);
					right_motor_set_speed(-50);
					left_motor_set_speed(50);

				}

			  break;
		}
}

uint16_t get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}


uint8_t get_target_color(void) {
	return target_color;
}

uint16_t get_dist_retour(void) {
	return dist_retour;
}


void set_dist_retour(float dist){
	dist_retour = dist;
}
void set_camera_height(uint16_t height){
	camera_height=height;
}

uint16_t get_camera_height(void){
	return camera_height;
}




