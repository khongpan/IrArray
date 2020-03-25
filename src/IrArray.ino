#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <heltec.h>
#include <Wire.h>

#include "MLX90621.h"
//#include "toojpeg.h"
#include "PictBuff.h"

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

#define LED 33



void LedOn(){
  digitalWrite(LED,0);
}

void LedOff() {
  digitalWrite(LED,1);
}




void OLEDsetup() {
  //Wire.begin(4,15,10000000);
  Heltec.display->init();
  Heltec.display->clear();
  Heltec.display->flipScreenVertically();
  //Heltec.display->mirrorScreen();
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
}



 



#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
int16_t img_data[SCREEN_WIDTH*SCREEN_HEIGHT];
Gray_Img_t display_img;

#define MIN_TEMP (minT)
#define MAX_TEMP (maxT)
#define T2G(d)  (((d)-MIN_TEMP)*255/(MAX_TEMP-MIN_TEMP))
#define EST(xx,xx1,yy1,xx2,yy2) ( yy1 + (((xx)-(xx1))*((yy2)-(yy1))/((xx2)-(xx1))) )

Gray_Img_t *IrData2ScreenImg()  {
  int x,y,i,j;
  int ox,oy;

  float *t;
  int color;

  float minT=MLX90621GetCaptureMinTemp()+1.8;
  float maxT=MLX90621GetCaptureMaxTemp();
  //float minT=26;
  //float maxT=36;

  if(maxT<=(minT+1)) maxT=minT+1;


  t=MLX90621GetCaptureTemp();

  display_img.height=SCREEN_HEIGHT;
  display_img.width=SCREEN_WIDTH;
  display_img.buf = img_data;
  int i_w=4;
  int i_h=16;

  for (y = 0; y < i_h; y++) {
    //Serial.println("");
    for (x = 0; x < i_w; x++) {

      int16_t ul,ur,ll,lr;

      ul=ll=ur=lr=0;
 
      if( (y < (i_h-1)) && (x < (i_w-1)) ) {
        ul=T2G(t[y*4+x]);
        ll=T2G(t[y*4+x+1]);
        ur=T2G(t[(y+1)*4+x]);
        lr=T2G(t[(y+1)*4+x+1]);
        //Serial.printf("%2.1f,%2.1f,%2.1f,%2.1f ",ul,ur,ll,lr);
        
      }else if ((x==i_w-1) && (y==i_h-1))  {
        ul=T2G(t[y*4+x]);
        ll=T2G(2*t[y*4+x]-t[y*4+x-1]);
        ur=T2G(2*t[y*4+x]-t[(y-1)*4+x]);
        lr=T2G(2*t[y*4+x]-t[(y-1)*4+x-1]);
      }else if (x==i_w-1) {
        ul=T2G(t[y*4+x]);
        ll=T2G(2*t[y*4+x]-t[y*4+x-1]);
        ur=T2G(t[(y+1)*4+x]);
        if(y!=0) {
          lr=T2G(2*t[y*4+x]-t[(y-1)*4+x-1]);
        }else {
          //lr= ul+((ur-ul)+(lr-ul))/2;
          lr=(ur+lr)/2;
        }

        //Serial.printf("%d,%d,%2.1f,%2.1f,%2.1f,%2.1f \r\n",x,y,ul,ll,ur,lr);
      }else if (y==i_h-1) {
        ul=T2G(t[y*4+x]);
        ll=T2G(t[y*4+x+1]);
        ur=T2G(2*t[y*4+x]-t[(y-1)*4+x]);
        lr=T2G(2*t[y*4+x]-t[(y-1)*4+x-1]);
      }else {
        Serial.print("!!!wrong mapping impossible!!!");
      }

  
       //color=T2G(t[y*4+x]);
       //if (color>1) color=1;
       //if (color<0) color=0;

       ox=(15-y)*8;
       oy=x*8;
       for(j=0;j<8;j++) {

         int16_t l=EST(j,0,ul,7,ll);
         int16_t r=EST(j,0,ur,7,lr);
         //Serial.printf("%2.1f,%2.1f ",l,r);

         for(i=0;i<8;i++) {
          color=EST(i,0,l,7,r);
          if(color>255) color=255;
          else if (color<0) color=0;
          display_img.buf[(oy+j)*128+(ox+(7-i))]=color;
         }
       }
       //Serial.println("");
    }
  }

  //for(;;);

  return &display_img;
}


//adapted from JEFworks https://gist.github.com/JEFworks/637308c2a1dd8a6faff7b6264104847a
 Gray_Img_t* DitheringImage(Gray_Img_t *img) {

  int16_t *buf=img->buf;
  int nrow=32;
  int ncol=128;

  #define a(y,x) (buf[(y)*128+(x)])

    for (int i = 0; i < nrow-1; i++) {
      for (int j = 1; j < ncol-1; j++) {
          int p=a(i, j);
          int P;

          if(p>127) P=255; else P=0;
          
          int e = p - P;
          a(i, j) = P;

          /* Floyd-Steinberg dithering */
          // a(i, j+1) = a(i, j+1) + (e * 7/16);
          // a(i+1, j-1) = a(i+1, j-1) + (e * 3/16);
          // a(i+1, j) = a(i+1, j) + (e * 5/16);
          // a(i+1, j+1) = a(i+1, j+1) + (e * 1/16);

          /* False Floyd-Steinberg dithering */
          a(i, j+1) = a(i, j+1) + ((e * 3) / 8);
          a(i+1, j) = a(i+1, j) + ((e * 3) / 8);
          a(i+1, j+1) = a(i+1, j+1) + ((e * 2)/8);
  
          //Serial.printf("%1.0f",a(i,j));
          
      }
      //Serial.println();
      
  }
  return img;

}

void ShowSummary(void) {
  static char str[100];
  float min,max,avg,amb;
  min=MLX90621GetCaptureMinTemp();
  max=MLX90621GetCaptureMaxTemp();
  avg=MLX90621GetCaptureAvgTemp();
  amb=MLX90621GetCaptureTa();
  
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0,10,String("MLX90621 far Ir sensor"));

  Heltec.display->setFont(ArialMT_Plain_24);
  sprintf(str,"%2.1f",avg);
  Heltec.display->drawString(0,26,String(str));

  Heltec.display->setFont(ArialMT_Plain_10);
  sprintf(str,"%2.1f %2.1f",min,max);
  Heltec.display->drawString(48,26,String(str));

  Heltec.display->setFont(ArialMT_Plain_10);
  sprintf(str,"Ta=%2.1f",amb);
  Heltec.display->drawString(0,50,String(str));
    
  Heltec.display->display();

}

void ShowMiniSummary(void) {
  static char str[100];
  float min,max,avg,amb;
  min=MLX90621GetCaptureMinTemp();
  max=MLX90621GetCaptureMaxTemp();
  avg=MLX90621GetCaptureAvgTemp();
  amb=MLX90621GetCaptureTa();
  
  // Heltec.display->clear();
  // Heltec.display->setFont(ArialMT_Plain_10);
  // Heltec.display->drawString(0,10,String("MLX90621 far Ir sensor"));

  Heltec.display->setFont(ArialMT_Plain_24);
  sprintf(str,"%2.1f c",avg);
  Heltec.display->drawString(0,32,String(str));

  Heltec.display->setFont(ArialMT_Plain_10);
  sprintf(str,"%2.1f %2.1f",min,max);
  Heltec.display->drawString(68,32,String(str));

  //Heltec.display->setFont(ArialMT_Plain_10);
  sprintf(str,"Ta=%2.1f",amb);
  Heltec.display->drawString(68,42,String(str));
    
  //Heltec.display->display();

}


void ShowImg() {
  int x,y;
  Gray_Img_t *img;
  //Serial.println("IrData2ScreenImg");
  img=IrData2ScreenImg();
  //Serial.println("DitheringImg");
  img=DitheringImage(img);
  //Serial.println("DisplayImg");

  Heltec.display->clear();
  for(y=0;y<SCREEN_HEIGHT;y++) {
    for(x=0;x<SCREEN_WIDTH;x++) {
      //if ((x+y) % 2 == 0) Heltec.display->setPixel(x,y);
      if(img->buf[y*SCREEN_WIDTH+x]>127)
       Heltec.display->setPixel(SCREEN_WIDTH-x,SCREEN_HEIGHT-y);
       
    }
  }
  //Heltec.display->flipScreenVertically();
  //Heltec.display->mirrorScreen();
  //Heltec.display->setBrightness(255);
  ShowMiniSummary();
  Heltec.display->display();
  
}

void UpdateDisplay() {
  if (digitalRead(0)==0) {
    ShowSummary();
  } else {
    ShowImg();
  }
  

}


void task_camera(void *p) {
  while(1) {
    //Serial.println("Ir image Capture");
    MLX90621Capture();
    createRGBImage(MLX90621GetCaptureTemp());
    delay(10);
  }
}


void task_display(void *p) {
  while(1) {
    //Serial.println("Ir image Capture");
    //MLX90621Capture();
    //Serial.println("OLED update...");
    UpdateDisplay();
    //ShowImg();
    delay(10);
  }
}

void task_main(void *p) {
  esp_task_wdt_delete(NULL);  
  while(1) {
    esp_task_wdt_reset();
    //do_job();
    delay(100);
  }
}


void setup() {

  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  pinMode(0,INPUT); 

  pinMode(LED,OUTPUT_OPEN_DRAIN);
  OLEDsetup();
  //StorageSetup();
  MLX90621Setup();
  //MLX90621Capture();
  //CameraSetup();

  static TaskHandle_t loopTaskHandle = NULL;
  xTaskCreateUniversal(task_camera, "task_camera", 10000, NULL, 1, &loopTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
  xTaskCreateUniversal(task_display, "task_display", 10000, NULL, 1, &loopTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);

  //static TaskHandle_t loopTaskHandle = NULL;
  xTaskCreateUniversal(task_main, "task_main", 50000, NULL, 1, &loopTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);

 
}

void loop() {
  delay(1000);
}
