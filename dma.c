#include "dma.h"

DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_i2s2_ext_tx;

void MX_DMA_Init(void) {
    __DMA1_CLK_ENABLE();
 
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
 
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}
