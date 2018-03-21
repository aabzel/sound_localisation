//FIRavr_filter_API.h
#ifndef FIRAVR_FILTER_API_H
#define FIRAVR_FILTER_API_H 


typedef int I32;
typedef short int I16;
typedef unsigned short int U16;
  
#define ORDER_OF_FILTER 50//40 80


extern U16 curWritePtr_1;
extern I16 values_1[ORDER_OF_FILTER];

extern U16 curWritePtr_2;
extern I16 values_2[ORDER_OF_FILTER];


void FIRaverageInit_1(); 
I16 calcFilterOutput_1(I16 in);
void resetFIR_1();

void FIRaverageInit_2(); 
I16 calcFilterOutput_2(I16 in);
void resetFIR_2();




#endif  //  FIRAVR_FILTER_API_H
