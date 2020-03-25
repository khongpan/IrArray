#ifndef MLX90621_H

#define MLX90621_H


typedef struct {
    uint8_t * buf;              /*!< Pointer to the pixel data */
    size_t len;                 /*!< Length of the buffer in bytes */
    size_t width;               /*!< Width of the buffer in pixels */
    size_t height;              /*!< Height of the buffer in pixels */
} MLX90621_img_t;

typedef struct {
    float * buf;              /*!< Pointer to the pixel data */
    size_t len;                 /*!< Length of the buffer in bytes */
    size_t width;               /*!< Width of the buffer in pixels */
    size_t height;              /*!< Height of the buffer in pixels */
} Normalized_Img_t;

typedef struct {
    int16_t * buf;              /*!< Pointer to the pixel data */
    size_t len;                 /*!< Length of the buffer in bytes */
    size_t width;               /*!< Width of the buffer in pixels */
    size_t height;              /*!< Height of the buffer in pixels */
} Gray_Img_t;

void readEEPROM();
void writeConfiguration();
uint16_t readConfiguration();
void   writeOSCTrim();
void calculateConstants();
void readPTAT();
void readIR();
float getAmbient();
void readCPIX();
void calculateTA(void);
void calculateTO();
void I2Cscan();
void writeWord(uint8_t address, uint8_t subAddress, uint16_t data);
uint16_t readWord(uint8_t address, uint8_t subAddress);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
MLX90621_img_t *createTGAImage(void);
MLX90621_img_t *createRGBImage(void);

void MLX90621Setup();
MLX90621_img_t* MLX90621Capture();
float *MLX90621GetCaptureTemp(void);
MLX90621_img_t *MLX90621GetCaptureImage(void);
float MLX90621GetCaptureAvgTemp(void);
float MLX90621GetCaptureMinTemp(void);
float MLX90621GetCaptureMaxTemp(void);
float MLX90621GetCaptureTa(void);
float MLX90621GetCaptureMedianTemp(void);



#endif