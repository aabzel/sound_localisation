#ifndef __iir_filter_H
#define __iir_filter_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#define SIZE_OF_FIFO  4000    //  25000 32000

void iirFilter(int16_t inVal, int16_t  *outVal);
void initFifo();

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */
