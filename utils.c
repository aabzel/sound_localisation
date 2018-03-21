

#include "utils.h"
#include "fifo.h"
#include "gpio.h"
#include <math.h>
#include "Kalnem_filter_API.h"

#ifdef RECORD_SOUND  
  #include "fatfs.h"
  #include "sdio.h"
  #include "i2s.h"
#endif 


#include <stdio.h>

uint32_t g_dma_iteration=0;
uint32_t g_proc_iteration=0;

int16_t g_maxLeftChVol=0;
int16_t g_maxRightChVol=0;

int g_curSample_i=0, g_maxProcLen, g_maxFiFoLen;
U8 g_status, g_resuls, globalError;
HAL_StatusTypeDef  g_statusHal;

uint32_t     g_writtenSampleCnt=0;

#ifdef  RECORD_SOUND 
UINT g_ByteWritten=0 ; 
uint8_t g_testBuffer[16] = "Start:\n";//hex 
UINT g_txLen=0,g_n=0;
#endif //RECORD_SOUND 




uint8_t     g_fileOpen=0;

int16_t    g_leftChVol=0;
int16_t    g_rightChVol=0;

int32_t    g_Timer =32*500; 
int32_t    g_Period=200;

float g_waveFrontAngleFiltered = 0;

int g_error     =0;
int g_maxInd    =0;
float g_timeDiff=0;
float g_waveFrontAngle         = 0;
float g_rightAngle =  RIGHT_ANGLE; 
float g_leftAngle  =  LEFT_ANGLE; 


float pi  =3.14159;
float m_Vs=331.0;
float m_L =0.19;


#ifdef SOUND_ORIENTATION 
float calcTimeDiffAmp(int m_maxInd)
{
  float m_dt,m_ts;
  m_ts  = 0.00003125;
  m_dt  = (m_maxInd-20)*m_ts;
  return m_dt;
}

float calcAngleAmpDeg(float m_dt )
{
  float acosParam=0;
  float m_thetaRad=0;
  float m_thetaDeg=0;
  
  acosParam=(m_dt*m_Vs)/m_L;
  if( 1.0< acosParam)
  {
    acosParam=1.0;
  }

  if(acosParam<-1)
  {
    acosParam=-1.0;
  }

  m_thetaRad=acos(acosParam);
  m_thetaDeg=(180.0*m_thetaRad)/pi;
  return m_thetaDeg;
}


void putSampleInFiFo(int16_t g_leftChVol , int16_t g_rightChVol)
{
    fifo_push(&objFifoLeft0, g_leftChVol);
    fifo_push(&objFifoLeft1, g_leftChVol);
    fifo_push(&objFifoLeft2, g_leftChVol);
    fifo_push(&objFifoLeft3, g_leftChVol);
    fifo_push(&objFifoLeft4, g_leftChVol);
    fifo_push(&objFifoLeft5, g_leftChVol);
    fifo_push(&objFifoLeft6, g_leftChVol);
    fifo_push(&objFifoLeft7, g_leftChVol);
    fifo_push(&objFifoLeft8, g_leftChVol);
    fifo_push(&objFifoLeft9, g_leftChVol);
    fifo_push(&objFifoLeft10, g_leftChVol);
    fifo_push(&objFifoLeft11, g_leftChVol);
    fifo_push(&objFifoLeft12, g_leftChVol);
    fifo_push(&objFifoLeft13, g_leftChVol);
    fifo_push(&objFifoLeft14, g_leftChVol);
    fifo_push(&objFifoLeft15, g_leftChVol);
    fifo_push(&objFifoLeft16, g_leftChVol);
    fifo_push(&objFifoLeft17, g_leftChVol);
    fifo_push(&objFifoLeft18, g_leftChVol);
    fifo_push(&objFifoLeft19, g_leftChVol);
    fifo_push(&objFifoLeft20, g_leftChVol);
    
    
    fifo_push(&objFifoRight0, g_rightChVol);
    fifo_push(&objFifoRight1, g_rightChVol);
    fifo_push(&objFifoRight2, g_rightChVol);
    fifo_push(&objFifoRight3, g_rightChVol);
    fifo_push(&objFifoRight4, g_rightChVol);
    fifo_push(&objFifoRight5, g_rightChVol);
    fifo_push(&objFifoRight6, g_rightChVol);
    fifo_push(&objFifoRight7, g_rightChVol);
    fifo_push(&objFifoRight8, g_rightChVol);
    fifo_push(&objFifoRight9, g_rightChVol);
    fifo_push(&objFifoRight10, g_rightChVol);
    fifo_push(&objFifoRight11, g_rightChVol);
    fifo_push(&objFifoRight12, g_rightChVol);
    fifo_push(&objFifoRight13, g_rightChVol);
    fifo_push(&objFifoRight14, g_rightChVol);
    fifo_push(&objFifoRight15, g_rightChVol);
    fifo_push(&objFifoRight16, g_rightChVol);
    fifo_push(&objFifoRight17, g_rightChVol);
    fifo_push(&objFifoRight18, g_rightChVol);
    fifo_push(&objFifoRight19, g_rightChVol);
    fifo_push(&objFifoRight20, g_rightChVol);
    
}

void performCorrelation()
{
      resultOfCrossCorelatin[0] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft20,  0,  20, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[1] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft19,  0,  19, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[2] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft18,  0,  18, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[3] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft17,  0,  17, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[4] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft16,  0,  16, WORK_FIFO_LEN);// 5
      resultOfCrossCorelatin[5] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft15,  0,  15, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[6] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft14,  0,  14, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[7] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft13,  0,  13, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[8] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft12,  0,  12, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[9] =crosCorrelatinOne(&objFifoRight0,  &objFifoLeft11,  0,  11, WORK_FIFO_LEN);// 10
      resultOfCrossCorelatin[10]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft10,  0,  10, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[11]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft9,  0,  9, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[12]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft8,  0,  8, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[13]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft7,  0,  7, WORK_FIFO_LEN); 
      resultOfCrossCorelatin[14]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft6,  0,  6, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[15]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft5,  0,  5, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[16]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft4,  0,  4, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[17]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft3,  0,  3, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[18]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft2,  0,  2, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[19]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft1,  0,  1, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[20]=crosCorrelatinOne(&objFifoRight0,  &objFifoLeft0,  0,  0, WORK_FIFO_LEN);  // L0R0     
      resultOfCrossCorelatin[21]=crosCorrelatinOne(&objFifoRight1,  &objFifoLeft0,  1,  0, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[22]=crosCorrelatinOne(&objFifoRight2,  &objFifoLeft0,  2,  0, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[23]=crosCorrelatinOne(&objFifoRight3,  &objFifoLeft0,  3,  0, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[24]=crosCorrelatinOne(&objFifoRight4,  &objFifoLeft0,  4,  0, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[25]=crosCorrelatinOne(&objFifoRight5,  &objFifoLeft0,  5,  0, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[26]=crosCorrelatinOne(&objFifoRight6,  &objFifoLeft0,  6,  0, WORK_FIFO_LEN);  //
      resultOfCrossCorelatin[27]=crosCorrelatinOne(&objFifoRight7,  &objFifoLeft0,  7,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[28]=crosCorrelatinOne(&objFifoRight8,  &objFifoLeft0,  8,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[29]=crosCorrelatinOne(&objFifoRight9,  &objFifoLeft0,  9,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[30]=crosCorrelatinOne(&objFifoRight10, &objFifoLeft0,  10,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[31]=crosCorrelatinOne(&objFifoRight11, &objFifoLeft0,  11,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[32]=crosCorrelatinOne(&objFifoRight12, &objFifoLeft0,  12,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[33]=crosCorrelatinOne(&objFifoRight13, &objFifoLeft0,  13,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[34]=crosCorrelatinOne(&objFifoRight14, &objFifoLeft0,  14,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[35]=crosCorrelatinOne(&objFifoRight15, &objFifoLeft0,  15,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[36]=crosCorrelatinOne(&objFifoRight16, &objFifoLeft0,  16,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[37]=crosCorrelatinOne(&objFifoRight17, &objFifoLeft0,  17,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[38]=crosCorrelatinOne(&objFifoRight18, &objFifoLeft0,  18,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[39]=crosCorrelatinOne(&objFifoRight19, &objFifoLeft0,  19,  0, WORK_FIFO_LEN);
      resultOfCrossCorelatin[40]=crosCorrelatinOne(&objFifoRight20, &objFifoLeft0,  20,  0, WORK_FIFO_LEN);
      
      g_maxInd                 = getMaxInd(resultOfCrossCorelatin, 41);
      g_timeDiff               = calcTimeDiffAmp(g_maxInd);
      g_waveFrontAngle         = calcAngleAmpDeg(g_timeDiff );
      g_waveFrontAngleFiltered = calcOutput(g_waveFrontAngle);
      
      if(0<= g_waveFrontAngleFiltered &&  g_waveFrontAngleFiltered<g_leftAngle)
      {
        //Left g_leftAngle
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
      }
      else if(g_rightAngle< g_waveFrontAngleFiltered &&  g_waveFrontAngleFiltered<=180)
      {
        //right g_rightAngle
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
      }
      else if(g_leftAngle<= g_waveFrontAngleFiltered &&  g_waveFrontAngleFiltered<=g_rightAngle)
      {
        // center
        g_error=0;
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET); 
      }
      else
      {
        //error
        g_error=1;
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
      }
      HAL_GPIO_TogglePin(LED2_BOARD_PORT, LED2_BOARD_PIN);
}

#endif



#ifdef RECORD_SOUND 
uint8_t g_buffer[80] ;
U8 writeSample(int16_t leftChVol, int16_t rightChVol)
{
  if(g_fileOpen)
  {
    g_n=sprintf (g_buffer, "%d %d\n", leftChVol, rightChVol);
    g_txLen = strlen(g_buffer);
    
    HAL_GPIO_WritePin(LED4_BOARD_PORT, LED4_BOARD_PIN, GPIO_PIN_SET);
    
    g_res = f_write(&g_testFile, g_buffer, g_txLen, &g_ByteWritten    );
    
    HAL_GPIO_WritePin(LED4_BOARD_PORT, LED4_BOARD_PIN, GPIO_PIN_RESET);
    
    if(FR_OK !=g_res)
    {
       return 2;   
    }
    if(g_txLen!=g_ByteWritten)
    {
      return 3;   
    }
  }else{
    return 1; 
  }
  g_writtenSampleCnt++;
  return 0;
}

U8  writeSRAMMemory2SDMemory(uint8_t *array, uint32_t ammountOfByte)
{
  if(g_fileOpen)
  {
    HAL_GPIO_WritePin(LED4_BOARD_PORT, LED4_BOARD_PIN, GPIO_PIN_SET);
    
    
    g_res = f_write(&g_testFile, array, ammountOfByte, &g_ByteWritten );
    
    HAL_GPIO_WritePin(LED4_BOARD_PORT, LED4_BOARD_PIN, GPIO_PIN_RESET);
    if(FR_OK !=g_res)
    {
       return 2;   
    }
    
    if(ammountOfByte!=g_ByteWritten)
    {
      return 3;
    }
    

  }else{
    return 1; 
  }
  
  g_writtenSampleCnt +=ammountOfByte;
  return 0;
}



U8 initTxtFile()
{
  uint8_t stat;

  DWORD curWrCursor;
  strcpy(g_testBuffer,"0 0\n");
  g_res = f_mount(&fileSystem, SD_Path, 1);
  if(FR_OK ==g_res)
  { 
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

    stat= openTxtFile();
    if(!stat)
    {
      g_fileOpen=1;

      curWrCursor = f_size (  &g_testFile   );
      g_res = f_lseek (   &g_testFile,    curWrCursor   );
      if(FR_OK !=g_res)
      {
         return 4;
      }
      // writebin file to boost

      //g_txLen = strlen(g_testBuffer);
      g_testBuffer[0]=0;
      g_testBuffer[1]=0;
      g_testBuffer[2]=0;
      g_testBuffer[3]=0;
      g_txLen = 4;
      g_res = f_write(&g_testFile, g_testBuffer, g_txLen, &g_ByteWritten  );
      if(FR_OK !=g_res)
      {
         return 3;
      }
      

    }else{
      return 2;
    }
  }else{
    return 1;
  }
  return 0;
}

#endif 



void procSamples(int16_t *array, int16_t ammountOfSamples, int16_t indx)
{
  //indx=0
  //0 1  2 3 4 5 6 7 8 9
  //1 3  5 7 9 11
  // array bytes
#ifdef RECORD_SOUND
  //memset((uint8_t *)g_backUpArray, (uint8_t *) 0x00, 4*ammountOfSamples);
  //memcpy((uint8_t *)g_backUpArray, (uint8_t *) &array[indx*2], 4*ammountOfSamples);
  //g_resuls = writeSRAMMemory2SDMemory((uint8_t *) g_backUpArray, 4*ammountOfSamples);
  g_resuls = writeSRAMMemory2SDMemory((uint8_t *) &array[indx*2], 4*ammountOfSamples);
  if(g_resuls)
  {
      g_btnPressed=1;
      globalError=g_resuls;
      //errorAlarm();
  }
#endif //RECORD_SOUND
  memset(&array[indx*2],0x00, 4*ammountOfSamples);
  
#ifdef SOUND_ORIENTATION  
  int curSample=0;
  
  for(g_curSample_i=0; g_curSample_i<ammountOfSamples; g_curSample_i++)
  {
    g_leftChVol  = array[g_curSample_i*2  +indx*2];
    curSample++;
    g_rightChVol = array[g_curSample_i*2+1+indx*2];
    curSample++;
    HAL_GPIO_TogglePin(LED2_BOARD_PORT, LED2_BOARD_PIN);
    
    //write to SD card
    updateMaxLeftSample(g_leftChVol);
    updateMaxRightSample(g_rightChVol);
    
    putSampleInFiFo( g_leftChVol ,  g_rightChVol);
      
    g_Timer--;
    if(!g_Timer)
    {
      g_Timer = g_Period;
      performCorrelation();
    }
  }
#endif  // SOUND_ORIENTATION
}

void updateMaxLen(int16_t g_procLen)
{
  if(g_maxProcLen<g_procLen)
  {
    g_maxProcLen=g_procLen;
  }
}

void  updateMaxLeftSample(int16_t leftChVol)
{
  if(g_maxLeftChVol<leftChVol)
  {
    g_maxLeftChVol=leftChVol;
  }
}

void  updateMaxRightSample(int16_t rightChVol)
{
  if(g_maxRightChVol<rightChVol)
  {
    g_maxRightChVol=rightChVol;
  }
}
    



void errorAlarm()
{
  while(1)
  {
      HAL_GPIO_TogglePin(LED1_BOARD_PORT, LED1_BOARD_PIN);
      HAL_GPIO_TogglePin(LED2_BOARD_PORT, LED2_BOARD_PIN);
      HAL_GPIO_TogglePin(LED3_BOARD_PORT, LED3_BOARD_PIN);
      HAL_GPIO_TogglePin(LED4_BOARD_PORT, LED4_BOARD_PIN);
      HAL_Delay(100);
  }

}