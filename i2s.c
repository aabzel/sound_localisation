/**
  ******************************************************************************
  * File Name          : I2S.c
  * Description        : This file provides code for the configuration
  *                      of the I2S instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2s.h"

#include "gpio.h"
#include "dma.h"
#include "utils.h"

#ifdef SOUND_ORIENTATION  
#include "FIRavr_filter_API.h"
#include "fifo.h"
#endif 

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef status1, status2;
uint8_t i2sTxDone=0;
uint8_t i2sRxDone=0;

int16_t i2sDataIn[AMMOUNT_OF_STEREO_SAMPLES*2];
//int16_t g_backUpArray[AMMOUNT_OF_STEREO_SAMPLES*2];
//int16_t i2sDataOut[AMMOUNT_OF_SAMPLES*2];
int16_t  irrOutVal;

int16_t    g_volThreshold=100;
    
/* USER CODE END 0 */

I2S_HandleTypeDef hi2s2;


/* I2S2 init function */
void MX_I2S2_Init(void)
{
  hi2s2.Instance            = SPI2;
  hi2s2.Init.Mode           = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard       = I2S_STANDARD_PHILLIPS;
  hi2s2.Init.DataFormat     = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput     = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq      = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL           = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource    = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  
  HAL_I2S_Init(&hi2s2);
}


void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2s->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __SPI2_CLK_ENABLE();
  
    /**I2S2 GPIO Configuration    
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB14     ------> I2S2_ext_SD
    PC3      ------> I2S2_SD 
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin       = GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_14;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2S2ext;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_spi2_rx.Instance       = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel   = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc    = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_rx.Init.Mode          = DMA_CIRCULAR;
    hdma_spi2_rx.Init.Priority      = DMA_PRIORITY_LOW;
    hdma_spi2_rx.Init.FIFOMode      = DMA_FIFOMODE_ENABLE;
    hdma_spi2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_spi2_rx.Init.MemBurst      = DMA_MBURST_SINGLE;
    hdma_spi2_rx.Init.PeriphBurst   = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_spi2_rx);

    __HAL_LINKDMA(hi2s, hdmarx, hdma_spi2_rx);

    hdma_i2s2_ext_tx.Instance       = DMA1_Stream4;
    hdma_i2s2_ext_tx.Init.Channel   = DMA_CHANNEL_2;
    hdma_i2s2_ext_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2s2_ext_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s2_ext_tx.Init.MemInc    = DMA_MINC_ENABLE;
    hdma_i2s2_ext_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_ext_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2_ext_tx.Init.Mode          = DMA_CIRCULAR;
    hdma_i2s2_ext_tx.Init.Priority      = DMA_PRIORITY_LOW;
    hdma_i2s2_ext_tx.Init.FIFOMode      = DMA_FIFOMODE_ENABLE;
    hdma_i2s2_ext_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2s2_ext_tx.Init.MemBurst      = DMA_MBURST_SINGLE;
    hdma_i2s2_ext_tx.Init.PeriphBurst   = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_i2s2_ext_tx);

    __HAL_LINKDMA(hi2s,hdmatx,hdma_i2s2_ext_tx);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef* hi2s)
{
  if(hi2s->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __SPI2_CLK_DISABLE();
  
    /**I2S2 GPIO Configuration    
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB14     ------> I2S2_ext_SD
    PC3      ------> I2S2_SD 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hi2s->hdmarx);
    HAL_DMA_DeInit(hi2s->hdmatx);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(SPI2_IRQn);

  }
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(SPI2==hi2s->Instance)
  {
    HAL_GPIO_TogglePin(LED1_BOARD_PORT, LED1_BOARD_PIN);
    i2sRxDone=1;
    g_dma_iteration++;
#ifdef DIGITAL_MIC_ECHO_EFFECT    
    iirFilter(i2sDataIn[0], &irrOutVal);
    i2sDataOut[0]=irrOutVal;
    i2sDataOut[1]=irrOutVal;
    status1= HAL_I2SEx_TransmitReceive_DMA(&hi2s2, i2sDataOut, i2sDataIn, 2);
#endif

#ifdef DIGITAL_MIC_ECHO    
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    i2sDataOut[0]=i2sDataIn[0];
    i2sDataOut[1]=i2sDataIn[1];
    status1= HAL_I2SEx_TransmitReceive_DMA(&hi2s2, i2sDataOut, i2sDataIn, 2);
#endif

#ifdef LINE_DIGITAL_ECHO
    i2sDataOut[0]=i2sDataIn[0];
    i2sDataOut[1]=i2sDataIn[1];
#endif

#ifdef IIR_FILTER 
    iirFilter(i2sDataIn[0], &irrOutVal);
    i2sDataOut[0]=irrOutVal;
    i2sDataOut[1]=irrOutVal;
#endif    

#ifdef DISTORTION 
    distortion(i2sDataIn[0], &irrOutVal);
    i2sDataOut[0]=irrOutVal;
    i2sDataOut[1]=irrOutVal;
#endif
  }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(SPI2==hi2s->Instance)
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
    i2sTxDone=1;
  }
}


void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
  if(SPI2==hi2s->Instance)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
  }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
