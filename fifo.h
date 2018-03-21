
#ifndef __fifo_H
#define __fifo_H

#ifdef __cplusplus
 extern "C" {
#endif
   
   
#ifdef SOUND_ORIENTATION 
   
   
#include "stm32f4xx_hal.h"

#define  WORK_FIFO_LEN (60)  //10 20


typedef enum {
    FIFO_FALSE   = 2,
    FIFO_TRUE    = 1,
    FIFO_SUCCESS = 0,
    FIFO_ERROR   = 3
} fifo_result_t;

typedef struct 
{
    int16_t *buffer;
    uint16_t size;
    uint16_t curWr;
} fifo_t;


typedef struct  {
  unsigned int getData: 1;
  unsigned int backUpDone: 1;
}status;
   
extern status flag;



extern fifo_t objFifoLeft0;
extern fifo_t objFifoLeft1;
extern fifo_t objFifoLeft2;
extern fifo_t objFifoLeft3;
extern fifo_t objFifoLeft4;
extern fifo_t objFifoLeft5;
extern fifo_t objFifoLeft6;
extern fifo_t objFifoLeft7;
extern fifo_t objFifoLeft8;
extern fifo_t objFifoLeft9;
extern fifo_t objFifoLeft10;
extern fifo_t objFifoLeft11;
extern fifo_t objFifoLeft12;
extern fifo_t objFifoLeft13;
extern fifo_t objFifoLeft14;
extern fifo_t objFifoLeft15;
extern fifo_t objFifoLeft16;
extern fifo_t objFifoLeft17;
extern fifo_t objFifoLeft18;
extern fifo_t objFifoLeft19;
extern fifo_t objFifoLeft20;

extern fifo_t objFifoRight0;
extern fifo_t objFifoRight1;
extern fifo_t objFifoRight2;
extern fifo_t objFifoRight3;
extern fifo_t objFifoRight4;
extern fifo_t objFifoRight5;
extern fifo_t objFifoRight6;
extern fifo_t objFifoRight7;
extern fifo_t objFifoRight8;
extern fifo_t objFifoRight9;
extern fifo_t objFifoRight10;
extern fifo_t objFifoRight11;
extern fifo_t objFifoRight12;
extern fifo_t objFifoRight13;
extern fifo_t objFifoRight14;
extern fifo_t objFifoRight15;
extern fifo_t objFifoRight16;
extern fifo_t objFifoRight17;
extern fifo_t objFifoRight18;
extern fifo_t objFifoRight19;
extern fifo_t objFifoRight20;



extern fifo_t backUpObjFifoLeft0 ;
extern fifo_t backUpObjFifoLeft1 ;
extern fifo_t backUpObjFifoLeft2 ;
extern fifo_t backUpObjFifoLeft3 ;
extern fifo_t backUpObjFifoLeft4 ;
extern fifo_t backUpObjFifoLeft5 ;
extern fifo_t backUpObjFifoLeft6 ;
extern fifo_t backUpObjFifoLeft7 ;
extern fifo_t backUpObjFifoLeft8 ;
extern fifo_t backUpObjFifoLeft9 ;
extern fifo_t backUpObjFifoLeft10 ;
extern fifo_t backUpObjFifoLeft11 ;
extern fifo_t backUpObjFifoLeft12 ;
extern fifo_t backUpObjFifoLeft13 ;
extern fifo_t backUpObjFifoLeft14 ;
extern fifo_t backUpObjFifoLeft15 ;
extern fifo_t backUpObjFifoLeft16 ;
extern fifo_t backUpObjFifoLeft17 ;
extern fifo_t backUpObjFifoLeft18 ;
extern fifo_t backUpObjFifoLeft19 ;
extern fifo_t backUpObjFifoLeft20 ;

extern fifo_t backUpObjFifoRight0 ;
extern fifo_t backUpObjFifoRight1 ;
extern fifo_t backUpObjFifoRight2 ;
extern fifo_t backUpObjFifoRight3 ;
extern fifo_t backUpObjFifoRight4 ;
extern fifo_t backUpObjFifoRight5 ;
extern fifo_t backUpObjFifoRight6 ;
extern fifo_t backUpObjFifoRight7 ;
extern fifo_t backUpObjFifoRight8 ;
extern fifo_t backUpObjFifoRight9 ;
extern fifo_t backUpObjFifoRight10 ;
extern fifo_t backUpObjFifoRight11 ;
extern fifo_t backUpObjFifoRight12 ;
extern fifo_t backUpObjFifoRight13 ;
extern fifo_t backUpObjFifoRight14 ;
extern fifo_t backUpObjFifoRight15 ;
extern fifo_t backUpObjFifoRight16 ;
extern fifo_t backUpObjFifoRight17 ;
extern fifo_t backUpObjFifoRight18 ;
extern fifo_t backUpObjFifoRight19 ;
extern fifo_t backUpObjFifoRight20 ;

extern int16_t fifoBuffLeft0[0+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft1[1+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft2[2+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft3[3+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft4[4+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft5[5+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft6[6+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft7[7+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft8[8+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft9[9+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft10[10+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft11[11+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft12[12+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft13[13+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft14[14+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft15[15+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft16[16+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft17[17+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft18[18+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft19[19+WORK_FIFO_LEN];
extern int16_t fifoBuffLeft20[20+WORK_FIFO_LEN];


extern int16_t fifoBuffRight0[0+WORK_FIFO_LEN];
extern int16_t fifoBuffRight1[1+WORK_FIFO_LEN];
extern int16_t fifoBuffRight2[2+WORK_FIFO_LEN];
extern int16_t fifoBuffRight3[3+WORK_FIFO_LEN];
extern int16_t fifoBuffRight4[4+WORK_FIFO_LEN];
extern int16_t fifoBuffRight5[5+WORK_FIFO_LEN];
extern int16_t fifoBuffRight6[6+WORK_FIFO_LEN];
extern int16_t fifoBuffRight7[7+WORK_FIFO_LEN];
extern int16_t fifoBuffRight8[8+WORK_FIFO_LEN];
extern int16_t fifoBuffRight9[9+WORK_FIFO_LEN];
extern int16_t fifoBuffRight10[10+WORK_FIFO_LEN];
extern int16_t fifoBuffRight11[11+WORK_FIFO_LEN];
extern int16_t fifoBuffRight12[12+WORK_FIFO_LEN];
extern int16_t fifoBuffRight13[13+WORK_FIFO_LEN];
extern int16_t fifoBuffRight14[14+WORK_FIFO_LEN];
extern int16_t fifoBuffRight15[15+WORK_FIFO_LEN];
extern int16_t fifoBuffRight16[16+WORK_FIFO_LEN];
extern int16_t fifoBuffRight17[17+WORK_FIFO_LEN];
extern int16_t fifoBuffRight18[18+WORK_FIFO_LEN];
extern int16_t fifoBuffRight19[19+WORK_FIFO_LEN];
extern int16_t fifoBuffRight20[20+WORK_FIFO_LEN];


extern int16_t backUpFifoBuffLeft0[0+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft1[1+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft2[2+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft3[3+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft4[4+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft5[5+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft6[6+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft7[7+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft8[8+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft9[9+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft10[10+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft11[11+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft12[12+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft13[13+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft14[14+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft15[15+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft16[16+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft17[17+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft18[18+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft19[19+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffLeft20[20+WORK_FIFO_LEN];


extern int16_t backUpFifoBuffRight0[0+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight1[1+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight2[2+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight3[3+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight4[4+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight5[5+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight6[6+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight7[7+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight8[8+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight9[9+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight10[10+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight11[11+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight12[12+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight13[13+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight14[14+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight15[15+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight16[16+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight17[17+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight18[18+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight19[19+WORK_FIFO_LEN];
extern int16_t backUpFifoBuffRight20[20+WORK_FIFO_LEN];




int8_t getMaxInd(int64_t *inArray, int size);

fifo_result_t fifo_init(fifo_t * fifo, int16_t * buffer, int16_t size);
fifo_result_t fifo_push(fifo_t * fifo, int16_t data);
fifo_result_t fifo_get(fifo_t *fifo, int16_t *data, int16_t idx);
fifo_result_t fifo_pop(fifo_t *fifo, int16_t *data);

int64_t crosCorrelatinOne(fifo_t *fifo1, fifo_t *fifo2,  int startIdx1,  int startIdx2 , int sizeOfarray);

extern int64_t resultOfCrossCorelatin[41];
//extern float resultOfCrossCorelatin[41];


#endif  // SOUND_ORIENTATION

#ifdef __cplusplus
}
#endif

#endif /* __fifo_H */
