#include "Arduino.h"
#include "MLX90621.h"

unsigned char jpg_pict_buff[1024];
int jpg_pict_buff_index=0;
void write_jpg_pict_buff(unsigned char byte) {
  jpg_pict_buff[jpg_pict_buff_index++]=byte;
  //Serial.print(byte);
}
void reset_jpg_pict_buff() {
  jpg_pict_buff_index=0;
}
int get_jpg_size() {
  return jpg_pict_buff_index;
}

void *get_jpg_pict_buff() {
  return jpg_pict_buff;
}


// adapted from http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
void getHeatMapColor(float value, float *red, float *green, float *blue)
{
  const int NUM_COLORS = 5;
  static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,1}, {0,1,0}, {1,1,0}, {1,0,0} };
    // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.
  
  int idx1;        // |-- Our desired color will be between these two indexes in "color".
  int idx2;        // |
  float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.
  
  if(value <= 0)      {  idx1 = idx2 = 0;            }    // accounts for an input <=0
  else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=0
  else
  {
    value = value * (NUM_COLORS-1);        // Will multiply value by 3.
    idx1  = floor(value);                  // Our desired color will be after this index.
    idx2  = idx1+1;                        // ... and before this index (inclusive).
    fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).
  }
    
  *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
  *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
  *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
}


#define MAX_TEMP 35
#define MIN_TEMP 25
void mapDegree2Color(float temp,unsigned char *r,unsigned char *g, unsigned char *b) {
  float rr,gg,bb;
  float temp_n;

  if(temp>MAX_TEMP) temp=MAX_TEMP;
  if(temp<MIN_TEMP) temp=MIN_TEMP;

  temp_n=(temp-MIN_TEMP)/(MAX_TEMP-MIN_TEMP);

  getHeatMapColor(temp_n,&rr,&gg,&bb);
  *r= (unsigned char)round(rr*255);
  *g= (unsigned char)round(gg*255);
  *b= (unsigned char)round(bb*255);
}



enum { img_width = 4, img_height = 16 };
static unsigned char imageBuff[18+img_width*img_height*3];
static MLX90621_img_t ir_img;
MLX90621_img_t *createTGAImage(float *temperatures) {

  unsigned char *pixels= imageBuff+18;
  unsigned char *tga = imageBuff;
  unsigned char *p;
  size_t x, y;

  p = pixels;
  for (y = 0; y < img_height; y++) {
    for (x = 0; x < img_width; x++) {
      mapDegree2Color(temperatures[y*4+x],p,p+1,p+2);
      p+=3;
    }
  }
  tga[2] = 2;
  tga[12] = 255 & img_width;
  tga[13] = 255 & (img_width >> 8);
  tga[14] = 255 & img_height;
  tga[15] = 255 & (img_height >> 8);
  tga[16] = 24;
  tga[17] = 32;

  ir_img.buf=imageBuff;
  ir_img.height=img_height;
  ir_img.width=img_width;
  ir_img.len=img_width*img_height*3+18;

  return &ir_img;
}


MLX90621_img_t *createRGBImage(float *temperatures) {

  unsigned char *pixels= imageBuff;
  unsigned char *p;

  size_t x, y;

  p = pixels;
  for (y = 0; y < img_height; y++) {
    for (x = 0; x < img_width; x++) {
      mapDegree2Color(temperatures[y*4+x],p,p+1,p+2);
      //Serial.printf("%03d %02x %02x %02x \r\n ",y*4+x,p[0],p[1],p[2]);
      p+=3;
    }
  }

  ir_img.buf=imageBuff;
  ir_img.height=img_height;
  ir_img.width=img_width;
  ir_img.len=img_width*img_height*3;

  return &ir_img;
}

MLX90621_img_t *get_rgb_pict_buff(void) {
  return &ir_img;
}


