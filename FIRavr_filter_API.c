//FIRavr_filter_API.c

#ifdef SOUND_ORIENTATION  


#include "FIRavr_filter_API.h"


 U16 curWritePtr_1;
 I16 values_1[ORDER_OF_FILTER];

 U16 curWritePtr_2;
 I16 values_2[ORDER_OF_FILTER];

void FIRaverageInit_1() 
{
  curWritePtr_1 = 0; //initialize so that we start to read at index 0
  for (int i  = 0; i<ORDER_OF_FILTER; i++) 
  {
    values_1[i] = 0; // to have a nice start up, fill the array with 0's
  }
}

void resetFIR_1() 
{
  curWritePtr_1 = 0; //initialize so that we start to read at index 0
  for (int i = 0; i<ORDER_OF_FILTER; i++) 
  {
    values_1[i] = 0; // to have a nice start up, fill the array with 0's
  }
}

I16 calcFilterOutput_1(I16 inputSample)
{
  I32 sumOfelements = 0;
  
  // input new sample into shift register
  if (curWritePtr_1 < ORDER_OF_FILTER)
  {
    //curPoss :0, 1, 2, ...(ORDER_OF_FILTER-1)
    values_1[curWritePtr_1] = inputSample;
    curWritePtr_1++;
  }
  else
  {
    curWritePtr_1         = 0;
    values_1[curWritePtr_1] = inputSample;
  }
  
  for (int k = 0; k < ORDER_OF_FILTER; k++)
  {
    sumOfelements = sumOfelements + values_1[k];
  }

  sumOfelements = (sumOfelements / ORDER_OF_FILTER);
  return sumOfelements;
}




void FIRaverageInit_2() 
{
  curWritePtr_2 = 0; //initialize so that we start to read at index 0
  for (int i  = 0; i<ORDER_OF_FILTER; i++) 
  {
    values_2[i] = 0; // to have a nice start up, fill the array with 0's
  }
}

void resetFIR_2() 
{
  curWritePtr_2 = 0; //initialize so that we start to read at index 0
  for (int i = 0; i<ORDER_OF_FILTER; i++) 
  {
    values_2[i] = 0; // to have a nice start up, fill the array with 0's
  }
}

I16 calcFilterOutput_2(I16 inputSample)
{
  I32 sumOfelements = 0;
  
  // input new sample into shift register
  if (curWritePtr_2 < ORDER_OF_FILTER)
  {
    //curPoss :0, 1, 2, ...(ORDER_OF_FILTER-1)
    values_2[curWritePtr_2] = inputSample;
    curWritePtr_2++;
  }
  else
  {
    curWritePtr_2         = 0;
    values_2[curWritePtr_2] = inputSample;
  }
  
  for (int k = 0; k < ORDER_OF_FILTER; k++)
  {
    sumOfelements = sumOfelements + values_2[k];
  }

  sumOfelements = (sumOfelements / ORDER_OF_FILTER);
  return sumOfelements;
}

#endif  
