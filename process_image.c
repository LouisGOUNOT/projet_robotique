#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <motors.h>
#include <leds.h>
#include "memory_protection.h"
#include <audio/audio_thread.h>
#include <audio/play_melody.h>

#include <process_image.h>
#include <audio_processing.h>



static float distance_cm = 0.0f;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle of the image
static uint8_t target_color= 1; // 0 = red, 1 = black, 2 = blue // 1 as default

//Init static variables
static uint16_t temp_begin = 0;
static uint16_t temp_end = 0;
static uint16_t blueMean = 0;
static uint16_t redMean = 0;
static float meanRatio = 0;
static uint8_t i_red = 0;
static uint8_t i_blue = 0;
static uint16_t camera_height = LOW_POS;
static float pxtocm = PXTOCM_BLACK_LINE;
static uint16_t compte_tour = 0;


//music detected color
static const uint16_t color_melody[] = {NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4,};

static const float color_tempo[] = {10, 10, 10, 10, 10, 10,10, 20, 40, 20, 5,};

static const melody_t color={
	.notes = color_melody,
	.tempo = color_tempo,
	.length = sizeof(color_melody)/sizeof(uint16_t),
};

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//wait time_ms milliseconds, used to wait the end of actions
void wait_ms(uint16_t time_ms){
     for(uint32_t i = 0 ; i < 21000000*time_ms/500 ; i++){
         __asm__ volatile ("nop");
     }
}

//stop the robots for 100ms
void stop_100_ms(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	wait_ms(100);
}


//Goes to the target then goes back to the black line
void go_to_target(void){

	pxtocm = PXTOCM_COLOR;
	camera_height = HIGH_POS;
	//travel_time corresponds to the time needed to reach the target at a motor speed of 800
	uint16_t travel_time = distance_cm*TRAVELTIMECOEFF;
	stop_100_ms();
	right_motor_set_speed(800);
	left_motor_set_speed(800);
	wait_ms(travel_time);
	right_motor_set_speed(-800);
	left_motor_set_speed(800);
	wait_ms(675);
	stop_100_ms();
	right_motor_set_speed(800);
	left_motor_set_speed(800);
	wait_ms(travel_time);
}

//set all the settings for a color detection
void reset_high_cam(void){
	camera_height = HIGH_POS;
	pxtocm = PXTOCM_COLOR;
	po8030_advanced_config(FORMAT_RGB565, 0, HIGH_POS, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	po8030_set_awb(0);
	po8030_set_contrast(55);
	//Clears buffer with black line
    chBSemWait(&image_ready_sem);
}

//Makes the robot turn to find a color
void color_rot(void){
	right_motor_set_speed(-50);
	left_motor_set_speed(50);
}

/*
 *  Returns the line's width extracted from the image buffer given if 0 passed
 *  as mode argument. Returns the sum of intensities in the detected line when
 *  1 given.
 *  Returns 0 if line not found
 */
uint16_t extract_line(uint8_t *buffer, uint8_t mode){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0,width_slope = 0,
			min_line_width = 0;
	uint32_t mean = 0;
	if(mode==0){
		width_slope = WIDTH_SLOPE;
		min_line_width = MIN_LINE_WIDTH;
	}
	else{
		width_slope = WIDTH_SLOPE_COLOR;
		min_line_width = MIN_LINE_WIDTH_COLOR;
	}

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - width_slope))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+width_slope] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - width_slope) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-width_slope] < mean)
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
		if(!line_not_found && (end-begin) < min_line_width){
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
	if (mode == 0){
		return width;
	}
	//In color search mode, return the sum of intensity within the detected line
	else{
		mean = 0;
		for(uint16_t i = begin ; i < end ; i++){
			mean += buffer[i];
		}
		temp_end = end;
		temp_begin = begin;
		return mean;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Init low camera for a black line research
	po8030_advanced_config(FORMAT_RGB565, 0, LOW_POS, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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
	//temp variable to save intensities when a mask is applied
	uint8_t red_temp;
	uint8_t green_temp;
	uint8_t blue_temp;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();
		//Looking for black line
		if ((target_color == 1)&&(camera_height == LOW_POS)){

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line(image,LINEWIDTH_MODE);
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
				//Filters green and blue to find red
				for(uint16_t i = 0; i<IMAGE_BUFFER_SIZE*2; i++){
					green_temp = (img_buff_ptr[i] & MSK_GREEN1) << 2;
					blue_temp = (img_buff_ptr[i] & MSK_BLUE1);
					green_temp = (green_temp | ((img_buff_ptr[++i] & MSK_GREEN2) >> 6));
					blue_temp = (img_buff_ptr[i] & MSK_BLUE2);
					//build a new filtered image
					image[i/2] = green_temp + blue_temp;
				}

				redMean = extract_line(image,MEAN_MODE);

				img_buff_ptr = dcmi_get_last_image_ptr();

				//Filters red and green to find blue
				for(uint16_t i = 0; i<IMAGE_BUFFER_SIZE*2; i++){
					red_temp = (img_buff_ptr[i] & MSK_RED1) >> 3;
					green_temp = (img_buff_ptr[i] & MSK_GREEN1) << 2;
					green_temp = (green_temp | ((img_buff_ptr[++i] & MSK_GREEN2) >> 6));
					//build a new filtered image
					image[i/2] = red_temp + green_temp;
				}
				blueMean = extract_line(image,MEAN_MODE);

				//gets lineWidth in pixels
				lineWidth = temp_end - temp_begin;
				if(lineWidth>50){
					distance_cm = pxtocm/lineWidth;
				//Compute the ratio of blue intensity over red's
				meanRatio = blueMean/redMean;
				}
				else{
					meanRatio = 0;
				}

				if(((blueMean + redMean) > 1)&&(lineWidth>MIN_LINE_WIDTH_COLOR)&&((get_line_position() - (IMAGE_BUFFER_SIZE/2))<50)){
					//If blue targeted and detected 3 times, go for it
					if((meanRatio > 0)&&(meanRatio  < 3)&&(target_color  == 2))
					{
						if (target_color == 0){
							i_red = 0;
						}
						else{
							i_blue++;
						}
						if (i_blue>2){
							stop_100_ms();
							playMelody(EXTERNAL_SONG, ML_SIMPLE_PLAY, &color);
							waitMelodyHasFinished();
							go_to_target();
							compte_tour = COMPTE_TOUR_MAX + 1;
						}
					}
					//If red targeted and detected 3 times, go for it
					else if((meanRatio  > 2)&&(target_color  == 0)){
						if (target_color == 2){
							i_blue = 0;
						}
						else{
							i_red++;
						}
						if (i_red>2){
							stop_100_ms();
							playMelody(EXTERNAL_SONG, ML_SIMPLE_PLAY, &color);
							waitMelodyHasFinished();
							go_to_target();
							compte_tour = COMPTE_TOUR_MAX + 1;
						}
					}
			}
				//When it reaches its maximum, it means 360 degrees were covered
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


//getters and setters

/*
 * start detection according to the parameter given
 * 0 => red detection
 * 1 => black line detection
 * 2 => blue detection
 *
 */
void select_target_color(uint8_t color_id) {
	switch (color_id){
			//target 0 = red detection
			case 0:
				if(target_color == 1){
					stop_100_ms();
					reset_high_cam();
					target_color = 0;
					set_rgb_led(LED2,255,0,0);
					color_rot();
				}
			  break;

			 // target 1 = black line detection
			case 1:
				target_color = 1;
				camera_height = LOW_POS;
				pxtocm = PXTOCM_BLACK_LINE;
				//set low camera
				po8030_advanced_config(FORMAT_RGB565, 0, 460, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
				po8030_set_awb(0);
				po8030_set_contrast(74);

			  break;

			 //target 2 = blue detection
			case 2:
				if(target_color == 1){
					stop_100_ms();
					reset_high_cam();
					target_color = 2;
					set_rgb_led(LED2,0,0,255);
					color_rot();
				}

			  break;
		}
}


uint8_t get_target_color(void) {
	return target_color;
}


uint16_t get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}


void set_camera_height(uint16_t height){
	camera_height=height;
}

uint16_t get_camera_height(void){
	return camera_height;
}




