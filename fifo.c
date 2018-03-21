
//   File Name          : fifo.c
#include "fifo.h"



status flag;




fifo_t objFifoLeft0;

int16_t fifoBuffLeft0[0+WORK_FIFO_LEN];


fifo_result_t fifo_init(fifo_t * fifo, int16_t *buffer, int16_t size)
{
  if (NULL != fifo && NULL != buffer && 0 < size) 
  {
    fifo->buffer = buffer;
    fifo->size   = size;
    fifo->curWr   = 0;

    return FIFO_SUCCESS;
  } 
  else 
  {
    return FIFO_ERROR;
  }
}


fifo_result_t fifo_push(fifo_t * fifo, int16_t data)
{
  if (NULL != fifo ) 
  {
    if( fifo->size <=  fifo->curWr)
    {
      fifo->curWr=0;
    }
    fifo->buffer[fifo->curWr]=data;
    fifo->curWr=fifo->curWr+1;
    return FIFO_SUCCESS;
  } 
  else 
  {
    return FIFO_ERROR;
  }
}


fifo_result_t fifo_get(fifo_t *fifo, int16_t *data, int16_t idx)
{ 
  if (NULL == fifo) 
  {
    return FIFO_ERROR;
  }

  if(fifo->size <= idx)
  {
    return FIFO_ERROR;
  }

  if(idx < fifo->curWr)
  {
    *data = fifo->buffer[fifo->curWr-1-idx];
  }
  else
  {
    *data = fifo->buffer[fifo->size-1-idx+fifo->curWr];
  }
  return FIFO_SUCCESS;
}


fifo_result_t fifo_pop(fifo_t *fifo, int16_t *data)
{
  if (NULL != fifo) 
  {
    return fifo_get(fifo,  data,  fifo->size-1 );
  }
  else
  {
    return FIFO_ERROR;
  }
}

void initFifo()
{
  fifo_result_t fifo_result; 
  
  fifo_result= fifo_init(&objFifoLeft0, fifoBuffLeft0,   sizeof(fifoBuffLeft0)/sizeof(fifoBuffLeft0[0]) );

  fifo_result= fifo_init(&objFifoRight0, fifoBuffRight0,   sizeof(fifoBuffRight0)/sizeof(fifoBuffRight0[0]) );
 
  fifo_result= fifo_init(&backUpObjFifoLeft0,  backUpFifoBuffLeft0,   sizeof( backUpFifoBuffLeft0)/sizeof( backUpFifoBuffLeft0[0]) );

  fifo_result= fifo_init(&backUpObjFifoRight0,  backUpFifoBuffRight0,   sizeof( backUpFifoBuffRight0)/sizeof( backUpFifoBuffRight0[0]) );

}

