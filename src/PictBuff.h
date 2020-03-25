#ifndef PICTBUFF_H
#define PICTBUFF_H
void write_jpg_pict_buff(unsigned char byte);
void reset_jpg_pict_buff();
int get_jpg_size();
void *get_jpg_pict_buff();


MLX90621_img_t *createRGBImage(float *temperatures);
MLX90621_img_t *get_rgb_pict_buff(void);
#endif
