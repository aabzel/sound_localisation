/**
  ******************************************************************************
  * File Name          : iir_filter.c
  */

#ifdef DIGITAL_MIC_ECHO_EFFECT    
  
    
    
/* Includes ------------------------------------------------------------------*/
#include "iir_filter.h"
//#include < string.h > 
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure iir_filter                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
int32_t g_fifoArray[SIZE_OF_FIFO];
int32_t g_curElem    =0;
int32_t g_sizeofFifo =0;
int32_t valfromFifo  =0; 
int32_t valtoFifo    =0;

int16_t g_threshold=10000;// -32k ... +32k
/* USER CODE END 1 */

void initFirFifo()
{
  valfromFifo=0; 
  g_curElem=0;
  valtoFifo=0;
  memset(g_fifoArray,0, sizeof(g_fifoArray));
  g_sizeofFifo = sizeof(g_fifoArray)/sizeof(g_fifoArray[0]);
  
}

void putFIFOval(int32_t inVal)
{
    if(g_curElem < g_sizeofFifo)
    {
        g_fifoArray[g_curElem] = inVal;
    }
    else
    {// щелчок
        g_curElem = 0;
        g_fifoArray[g_curElem] = inVal;
    }
    g_curElem++;
}

int32_t getFIFOVal()
{
    if(g_curElem < (g_sizeofFifo-1))
    {
        return g_fifoArray[g_curElem+1];
    }
    else
    {
        return g_fifoArray[0];
    }
}

void iirFilter(int16_t inVal, int16_t  *outVal)
{
  valfromFifo = getFIFOVal();
  *outVal = (inVal + valfromFifo);
  valtoFifo = (*outVal)*2/5; 
  putFIFOval( valtoFifo);
  
}

void distortion(int16_t inVal, int16_t  *outVal)
{
  if(g_threshold<inVal)
  {
    *outVal=g_threshold;
    return;
  }else if(inVal<-g_threshold)
  {
    *outVal=-g_threshold;
    return;
  }else
  {
    *outVal=inVal;
  }
}

#endif

