#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

uint16_t get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
void select_target_color(uint8_t color_id);
uint8_t get_target_color(void);
void set_camera_height(uint16_t height);
uint16_t get_camera_height(void);


#endif /* PROCESS_IMAGE_H */
