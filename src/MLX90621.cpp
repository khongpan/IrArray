
//Adapted from https://www.tindie.com/products/onehorse/mlx90621-4-x-16-ir-array/#product-reviews
//https://github.com/kriswiner/MLX90621/blob/master/MLX90621IRArray.ino
#include <Arduino.h>
#include <Wire.h>
//#include "SoftwareWire.h" //partial work
//#include <SoftWire.h>
#include "MLX90621.h"

//#define write(b) write((uint8_t)(b))
//SoftWire Wire2(21,22);
//SoftwareWire Wire2(19,23,true,false);
TwoWire Wire2 = TwoWire(1);
//#define Wire2 Wire1
#define I2C_NOSTOP (false)

// 64, 16-bit words for the 4 x 16 bit array
#define MLX90621_DATA00  0x00 
#define MLX90621_DATA10  0x01 
#define MLX90621_DATA20  0x02 
#define MLX90621_DATA30  0x03 
#define MLX90621_DATA01  0x04 // IRAddress(x,y) = x + 4y, where 0 < x < 3, 0 < y < 15
 
#define MLX90621_DATA64  0x3F

#define MLX90621_PTAT    0x40
#define MLX90621_COMP    0x41
#define MLX90621_CONF    0x92
#define MLX90621_TRIM    0x93

#define MLX90621_ADDRESS 0x60
#define EEPROM_ADDRESS   0x50

/* 
EEPROM
A 2kbit, organized as 256x8 EEPROM is built in the MLX90621. The EEPROM has a separate I2C address SA=0x50 and is
used to store the calibration constants and the configuration of the device.
See: http://ww1.microchip.com/downloads/en/DeviceDoc/21709J.pdf
*/
uint8_t eepromData[256];

// EEPROM data values
#define KS_SCALE          0xC0
#define KS4_EE            0xC4
#define CAL_ACOMMON_L     0xD0
#define CAL_ACOMMON_H     0xD1
#define KT_SCALE          0xD2
#define CAL_ACP_L         0xD3
#define CAL_ACP_H         0xD4
#define CAL_BCP           0xD5
#define CAL_alphaCP_L     0xD6
#define CAL_alphaCP_H     0xD7
#define CAL_TGC           0xD8
#define CAL_AI_SCALE      0xD9
#define CAL_BI_SCALE      0xD9
#define VTH_L             0xDA
#define VTH_H             0xDB
#define KT1_L             0xDC
#define KT1_H             0xDD
#define KT2_L             0xDE
#define KT2_H             0xDF
//Common sensitivity coefficients
#define CAL_A0_L          0xE0
#define CAL_A0_H          0xE1
#define CAL_A0_SCALE      0xE2
#define CAL_DELTA_A_SCALE 0xE3
#define CAL_EMIS_L        0xE4
#define CAL_EMIS_H        0xE5
#define CAL_KSTA_L        0xE6
#define CAL_KSTA_H        0xE7
#define OSC_TRIM          0xF7
#define CHIP_ID           0xF8

uint16_t configuration;
float v_ir_off_comp, ksta, v_ir_tgc_comp, v_ir_comp, alpha_comp;
float tak4, resolution_comp;
int16_t a_common; 
uint16_t k_t1_scale, k_t2_scale, a_i_scale, b_i_scale;
float k_t1, k_t2, emissivity, tgc, alpha_cp, a_cp, b_cp, v_th, tmpTemp, ks_scale, ks4, sx;
uint16_t ptat;
int16_t cpix;
float a_ij, b_ij, alpha_ij;
float avgTemp, minTemp, maxTemp, Tambient, medianTemp;
float temperatures[64]; //Contains the calculated temperatures of each pixel in the array
float sorted_temp[64];
int16_t irData[64];     //Contains the raw IR data from the sensor
uint16_t color;
uint8_t rgb, red, green, blue;



enum refreshRate {  // define IR Array refresh rate
  IRRrate_512Hz = 0x05,
  IRRRate_256Hz,
  IRRRate_128Hz,
  IRRRate_64Hz,
  IRRRate_32Hz,
  IRRRate_16Hz,
  IRRRate_8Hz,
  IRRRate_4Hz,
  IRRRate_2Hz,
  IRRRate_1Hz,  // default
  IRRRate_0_5Hz // 0xFF
};

enum resolution {  // define IR Array resolution
  ADC_15bit = 0,
  ADC_16bit,
  ADC_17bit,
  ADC_18bit
};

// Define configuration
  uint8_t refreshRate = IRRRate_4Hz; // set IR Array refresh rate
  uint8_t resolution =  ADC_18bit;   // set temperature resolution

  
void MLX90621Setup() 
{
  

  Serial.println("Init MLX90621 4 x 16 IR Array sensor");

  avgTemp=maxTemp=minTemp=Tambient=0;
  for(int i=0;i<63;i++) {
    temperatures[i]=i;
  }

  // Initialize i2C bus and check for devices on the bus
  //Wire2.begin(19,23,10000);
  //Wire2.begin(21,22,10000);
  Wire2.begin(13,12,10000);

  //SoftwareWire
  //Wire2.setClock(100000);
  //Wire2.begin();
  
  //SoftWire
  //Wire2.setDelay_us(5);
  //Wire2.setClock(10000);
  //Wire2.enablePullups(true);
  //Wire2.setTxBuffer(txbuff,255);
  //Wire2.setRxBuffer(rxbuff,255);
  //Wire2.begin();
  

  //I2Cscan();

   // Initialization of the MLX90621
    readEEPROM();// Read the entire EEPROM
    Serial.print("EEPROM Chip ID = "); Serial.print(eepromData[CHIP_ID], HEX); Serial.print("  Should be "); Serial.println(0xA6, HEX);

    writeOSCTrim();
    writeConfiguration();
    configuration = readConfiguration();
    Serial.print("Resolution = "); Serial.println((configuration & 0x0030) >> 4);
    Serial.print("Rate = "); Serial.println((configuration & 0x000F) );
    calculateConstants();
}
/* End of setup*/

int tcmp(const void *a, const void *b) {
    return((*(float*)a)- *((float *)b));
}

MLX90621_img_t* MLX90621Capture() 
{
    if(!(readConfiguration() & 0x0400)){ // if POR bit cleared, reinitialize
      readEEPROM();
      writeOSCTrim();
      writeConfiguration();
      //Serial.println("MLX90621 por reset");
    }
    
    readPTAT(); 
    readIR(); // get the raw data
  
    calculateTA(); // get ambient temperature
    //Serial.print("Ambient temperature = "); Serial.println(Tambient);
    readCPIX(); // get pixel corrections
    calculateTO(); // compensate raw data to get 4 x 16 temperatures

    // Get min and max temperatures
    minTemp = 1000.;
    maxTemp =    0.;
    float sumTemp=0;
    for(int x=0; x <16 ; x++){ //go through all the rows
      for(int y=0; y<4; y++){ //go through all the columns
        if(temperatures[y+x*4] > maxTemp) maxTemp = temperatures[y+x*4];
        if(temperatures[y+x*4] < minTemp) minTemp = temperatures[y+x*4];
        sumTemp+=temperatures[y+x*4];
        //sorted_temp[y+x*4]=temperatures[y+x*4];
        //Serial.printf("pixel %02d %2.1f ",y+x*4,temperatures[y+x*4]);
      }
        //Serial.println("");
    }
    avgTemp = sumTemp/(4*16);
    //Serial.print("Min temperature="); Serial.println(minTemp);
    //Serial.print("Max temperature="); Serial.println(maxTemp);
    //Serial.print("Average temperature="); Serial.println(avgTemp);

    //return createTGAImage();
    //qsort(sorted_temp,64,sizeof(float),tcmp);
    medianTemp=sorted_temp[48];
    //return createRGBImage();
    return NULL;

    //delay(100);  // give some time to see the screen values
}


/* End of Loop*/

/*
************************************************************************************************************
* Useful Functions
************************************************************************************************************
*/
  void readEEPROM()
  {// Read the entire EEPROM
    for(int j=0; j<256; j+=32) {
      Wire2.beginTransmission(EEPROM_ADDRESS);
      Wire2.write(j);
      byte rc = Wire2.endTransmission(I2C_NOSTOP);
      Wire2.requestFrom(EEPROM_ADDRESS, 32);
      for (int i = 0; i < 32; i++) {
        eepromData[j+i] = (uint8_t) Wire2.read();
      }
    }
  }
  
void writeConfiguration()
{
    // Write MLX90621 configuration register
    Wire2.beginTransmission(MLX90621_ADDRESS);
    Wire2.write(0x03);                                           // transmit Op Code for Configuration file write
    Wire2.write( ((resolution << 4) | (refreshRate))  - 0x55);   // transmit LSB check
    Wire2.write( ((resolution << 4) | (refreshRate)) );          // transmit LSB data
    Wire2.write(0x46 - 0x55);                                    // transmit MSB check
    Wire2.write(0x46);                                           // transmit MSB, 0x46 default
    Wire2.endTransmission();
}


uint16_t readConfiguration()
{
    //Read MLX90621 configuration register
    Wire2.beginTransmission(MLX90621_ADDRESS);
    Wire2.write(0x02);           // transmit Op Code for register read
    Wire2.write(MLX90621_CONF);  // transmit address for Configuration register
    Wire2.write(0x00);           // transmit address step for Configuration register
    Wire2.write(0x01);           // transmit number of words to be read
    Wire2.endTransmission(I2C_NOSTOP);
    Wire2.requestFrom(MLX90621_ADDRESS, 2);
    uint8_t dataLSB = Wire2.read();  // read LSB of Configuration register
    uint8_t dataMSB = Wire2.read();  // read MSB of Configuration register
    uint16_t data = ( (uint16_t) dataMSB << 8) | dataLSB;
    return data;
}


void   writeOSCTrim() {
  Wire2.beginTransmission(MLX90621_ADDRESS);
  Wire2.write(0x04);
  Wire2.write(eepromData[OSC_TRIM] - 0xAA);
  Wire2.write(eepromData[OSC_TRIM]);
  Wire2.write(0x56);
  Wire2.write(0x00);
  Wire2.endTransmission();
}


void calculateConstants() {
  resolution_comp = pow(2.0, (3 - resolution));
  emissivity = (float)(((uint16_t)eepromData[CAL_EMIS_H] << 8) | eepromData[CAL_EMIS_L]) / 32768.0;
  Serial.print("emmissivity = "); Serial.println(emissivity);
  a_common = ((int16_t)eepromData[CAL_ACOMMON_H] << 8) | eepromData[CAL_ACOMMON_L];
  Serial.print("a_common = "); Serial.println(a_common);
  a_i_scale = (uint16_t)(eepromData[CAL_AI_SCALE] & 0x00F0) >> 4;
  b_i_scale = (uint16_t) eepromData[CAL_BI_SCALE] & 0x000F;
  Serial.print("a_i_scale = "); Serial.println(a_i_scale);
  Serial.print("b_i_scale = "); Serial.println(b_i_scale);
  
  alpha_cp = (float)(((uint16_t)eepromData[CAL_alphaCP_H] << 8) | eepromData[CAL_alphaCP_L]) /
         (pow(2.0, eepromData[CAL_A0_SCALE]) * resolution_comp);
  Serial.print("alpha_cp = "); Serial.println(alpha_cp);
         
  a_cp = (float) ((int16_t)(((int16_t)eepromData[CAL_ACP_H] << 8) | eepromData[CAL_ACP_L])) / resolution_comp;
  Serial.print("a_cp = "); Serial.println(a_cp);
  b_cp = (float) (((int16_t)eepromData[CAL_BCP] << 8) >> 8) / (pow(2.0, (float)b_i_scale) * resolution_comp);
  Serial.print("b_cp = "); Serial.println(b_cp);
  tgc  = (float) (((int16_t)eepromData[CAL_TGC] << 8) >> 8) / 32.0;
  Serial.print("tgc = "); Serial.println(tgc);
  
  ks_scale = eepromData[KS_SCALE] & 0x0F; // use only lower nibble  
  ks4  = (float) (( (int16_t)(eepromData[KS4_EE] << 8) ) >> 8) / pow(2, ks_scale + 8);
  Serial.print("ks4 = "); Serial.println(ks4);
  k_t1_scale = (uint16_t) (eepromData[KT_SCALE] & 0x00F0) >> 4;
  Serial.print("k_t1_scale = "); Serial.println(k_t1_scale);
  k_t2_scale = (uint16_t) (eepromData[KT_SCALE] & 0x000F) + 10;
  Serial.print("k_t2_scale = "); Serial.println(k_t2_scale);
  v_th = (float) (((int16_t)eepromData[VTH_H] << 8) | eepromData[VTH_L]);
  v_th = v_th / resolution_comp;
  Serial.print("v_th = "); Serial.println(v_th);
  k_t1 = (float) (((int16_t)eepromData[KT1_H] << 8) | eepromData[KT1_L]);
  k_t1 /= (pow(2.0, k_t1_scale) * resolution_comp);
  k_t2 = (float) (((int16_t)eepromData[KT2_H]  << 8) | eepromData[KT2_L]);
  k_t2 /= (pow(2.0, k_t2_scale) * resolution_comp);
  Serial.print("k_t1 = "); Serial.println(k_t1);
  Serial.print("k_t2 = "); Serial.println(k_t2);
}

void readPTAT() 
{
  Wire2.beginTransmission(MLX90621_ADDRESS);
  Wire2.write(0x02);
  Wire2.write(0x40);
  Wire2.write(0x00);
  Wire2.write(0x01);
  Wire2.endTransmission(I2C_NOSTOP);
  Wire2.requestFrom(MLX90621_ADDRESS, 2);
  uint8_t ptatLSB  = Wire2.read();
  uint8_t ptatMSB = Wire2.read();
  ptat = ((uint16_t)ptatMSB << 8) | ptatLSB;
}

void readIR() {
  for (int j = 0; j < 64; j += 16) { // Read in blocks of 32 bytes to overcome Wire2 buffer limit
    Wire2.beginTransmission(MLX90621_ADDRESS);
    Wire2.write(0x02);
    Wire2.write(j);
    Wire2.write(0x01);
    Wire2.write(0x20);
    Wire2.endTransmission(I2C_NOSTOP);
    Wire2.requestFrom(MLX90621_ADDRESS, 32);
    for (int i = 0; i < 16; i++) {
      uint8_t pixelDataLSB = Wire2.read();
      uint8_t pixelDataMSB = Wire2.read();
      irData[j + i] = ((int16_t)pixelDataMSB << 8) | pixelDataLSB;
    }
  }
}

float getAmbient() {
  return Tambient;
}

void readCPIX() {
  Wire2.beginTransmission(MLX90621_ADDRESS);
  Wire2.write(0x02);
  Wire2.write(0x41);
  Wire2.write(0x00);
  Wire2.write(0x01);
  Wire2.endTransmission(I2C_NOSTOP);
  Wire2.requestFrom(MLX90621_ADDRESS, 2);
  byte cpixLow = Wire2.read();
  byte cpixHigh = Wire2.read();
  cpix = ((int16_t)cpixHigh << 8) | cpixLow;
}

void calculateTA(void) {
  Tambient = ((-k_t1 + sqrt(sq(k_t1) - (4 * k_t2 * (v_th - (float) ptat))))
      / (2 * k_t2)) + 25.0;
}

void calculateTO() {
  float v_cp_off_comp = (float) cpix - (a_cp + b_cp * (Tambient - 25.0));
  tak4 = pow((float) Tambient + 273.15, 4.0);
  minTemp = 1000, maxTemp = -273.15;
  for (int i = 0; i < 64; i++) {
    a_ij = ((float) a_common + eepromData[i] * pow(2.0, a_i_scale)) / resolution_comp;
    b_ij = (float) (((int16_t)eepromData[0x40 + i] << 8) >> 8) / (pow(2.0, b_i_scale) * resolution_comp);
    v_ir_off_comp = (float) irData[i] - (a_ij + b_ij * (Tambient - 25.0));
    v_ir_tgc_comp = (float) v_ir_off_comp - tgc * v_cp_off_comp;
    float alpha_ij = ((float) (((uint16_t)eepromData[CAL_A0_H] << 8) | eepromData[CAL_A0_L]) / pow(2.0, (float) eepromData[CAL_A0_SCALE]));
    alpha_ij += ((float) eepromData[0x80 + i] / pow(2.0, (float) eepromData[CAL_DELTA_A_SCALE]));
    alpha_ij = alpha_ij / resolution_comp;
    ksta = (float) (((int16_t)eepromData[CAL_KSTA_H] <<8) | eepromData[CAL_KSTA_L]) / pow(2.0, 20.0);
    alpha_comp = (1 + ksta * (Tambient - 25.0)) * (alpha_ij - tgc * alpha_cp);
    v_ir_comp = v_ir_tgc_comp / emissivity;

    sx = ks4*pow(pow(alpha_comp, 3)*v_ir_comp+pow(alpha_comp, 4)*tak4, 0.25);    
    float temperature = pow((v_ir_comp / (alpha_comp*(1 - ks4*273.15) + sx)) + tak4, 0.25) - 273.15;

    temperatures[i] = temperature;
    if (minTemp == 1000 || temperature < minTemp) {
      minTemp = temperature;
    }
    if (maxTemp == -273.15 || temperature > maxTemp) {
      maxTemp = temperature;
    }
  }
}


// I2C scan function
void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire2.beginTransmission(address);
    error = Wire2.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

// I2C read/write functions for the MLX90621 and EEPROM

  void writeWord(uint8_t address, uint8_t subAddress, uint16_t data)
{
  Wire2.beginTransmission(address);  // Initialize the Tx buffer
  Wire2.write(subAddress);           // Put slave register address in Tx buffer
  Wire2.write((data & 0xFF00) << 8); // Put data MSB in Tx buffer
  Wire2.write(data & 0x00FF);        // Put data LSB in Tx buffer
  Wire2.endTransmission();           // Send the Tx buffer
}

  uint16_t readWord(uint8_t address, uint8_t subAddress)
{
  uint16_t data; // `data` will store the register data   
  Wire2.beginTransmission(address);  // Initialize the Tx buffer
  Wire2.write(subAddress);           // Put slave register address in Tx buffer
  Wire2.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire2.requestFrom(address, (size_t) 1);   // Read one word from slave register address 
  data = Wire2.read();                      // Fill Rx buffer with result
  return data; 
}

  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire2.beginTransmission(address);  // Initialize the Tx buffer
  Wire2.write(subAddress);           // Put slave register address in Tx buffer
  Wire2.write(data);                 // Put data in Tx buffer
  Wire2.endTransmission();           // Send the Tx buffer
}

  uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire2.beginTransmission(address);         // Initialize the Tx buffer
  Wire2.write(subAddress);                  // Put slave register address in Tx buffer
  Wire2.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire2.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire2.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire2.beginTransmission(address);   // Initialize the Tx buffer
  Wire2.write(subAddress);            // Put slave register address in Tx buffer
  Wire2.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire2.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire2.available()) {
        dest[i++] = Wire2.read(); }         // Put read results in the Rx buffer
}





//MLX90621_img_t *MLX90621GetCaptureImage(void) {
//  return &ir_img;
//}
float *MLX90621GetCaptureTemp(void) {
  return temperatures;
}

float MLX90621GetCaptureAvgTemp(void) {
  return avgTemp;
}
float MLX90621GetCaptureMinTemp(void) {
  return minTemp;
}
float MLX90621GetCaptureMaxTemp(void) {
  return maxTemp;
}
float MLX90621GetCaptureTa(void) {
  return Tambient;
}
float MLX90621GetCaptureMedianTemp(void) {
  return medianTemp;
}