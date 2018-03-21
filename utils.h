
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __utils_H
#define __utils_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define DELTA_ANGLE    20
#define CENTER_ANGLE  (90)
#define RIGHT_ANGLE   (CENTER_ANGLE + DELTA_ANGLE)
#define LEFT_ANGLE    (CENTER_ANGLE - DELTA_ANGLE)

typedef uint8_t U8;
typedef uint16_t U16;//0011 010_   
  
extern uint32_t     g_writtenSampleCnt;
extern HAL_StatusTypeDef  g_statusHal;   
extern uint8_t     g_fileOpen;

extern int16_t    g_leftChVol;
extern int16_t    g_rightChVol;

extern int32_t    g_Timer ; 
extern int32_t    g_Period;
extern int g_maxFiFoLen;

extern U8 g_status, g_resuls, globalError;


extern uint32_t g_dma_iteration;
extern uint32_t g_proc_iteration;

typedef uint32_t U32;
typedef uint16_t U16;
typedef uint8_t U8;

typedef int32_t I32;
typedef int16_t I16;
typedef int8_t I8;

#ifdef  RECORD_SOUND 
#define AMMOUNT_OF_STEREO_SAMPLES   (29250)  // 29250< x  <  29500 
#endif //RECORD_SOUND 
    
#ifdef SOUND_ORIENTATION
#define AMMOUNT_OF_SAMPLES   (32*300)  // 32*450< x  <32*460   12384  
#endif  

   
#ifdef  RECORD_SOUND 
 
U8 initTxtFile();
U8 writeSample(int16_t g_leftChVol, int16_t g_rightChVol); 
void updateMaxLen(int16_t g_procLen);
void  updateMaxLeftSample(int16_t leftChVol);
void  updateMaxRightSample(int16_t rightChVol);
U8  writeSRAMMemory2SDMemory(uint8_t *array, uint32_t ammountOfByte);

#endif //RECORD_SOUND 

void errorAlarm();
void putSampleInFiFo(int16_t g_leftChVol , int16_t g_rightChVol);
void performCorrelation();
void procSamples(int16_t *array, int16_t ammountOfSamples, int16_t indx);
   
  
#ifdef __cplusplus
}
#endif
#endif /* __utils_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
