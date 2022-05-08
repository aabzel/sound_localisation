

#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "gpio.h"

#include "utils.h"

#ifdef RECORD_SOUND  
  #include "fatfs.h"
  #include "sdio.h"
#endif 
  


    
#ifdef DIGITAL_MIC_ECHO_EFFECT    
  #include "iir_filter.h"
#endif

#ifdef SOUND_ORIENTATION
#include "fifo.h"
#include "FIRavr_filter_API.h"
#include "Kalnem_filter_API.h"
#endif





/* USER CODE BEGIN Includes */
//float g_timeDiff=0;
//float g_waveFrontAngle         = 0;
//float g_waveFrontAngleFiltered = 0;
//int g_maxInd=0;
//int g_error=0;
/* USER CODE END Includes */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t g_addres ;
int g_RxSamplePtr=0, g_procLen=0, g_ProcPtr=0,  g_dmaPtr=0;

void procSamples(int16_t *array, int16_t ammountOfSamples, int16_t indx);


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef status;
  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

#ifdef DIGITAL_MIC_ECHO_EFFECT    
   initFirFifo();
#endif


  g_writtenSampleCnt=0;

#ifdef SOUND_ORIENTATION    
  g_maxFiFoLen=AMMOUNT_OF_STEREO_SAMPLES;
  flag.backUpDone=0;
  flag.getData=0;
  /* Initialize all configured peripherals */
  
  initFifo();
  
  int i;
    
  for(int i=0; i<sizeof(resultOfCrossCorelatin)/sizeof(resultOfCrossCorelatin[0]); i++)
  {
    resultOfCrossCorelatin[i]=0;
  }
#endif 
  
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();

#ifdef RECORD_SOUND  
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
#endif  
  HAL_Delay(1000);
  g_writtenSampleCnt=0;
  
#ifdef SOUND_ORIENTATION  
  //initFirFifo();
  kalmenFilterInit(0.1);
  FIRaverageInit_1();
  FIRaverageInit_2();
  flag.getData=1;
#endif  
  
  /* USER CODE BEGIN 2 */
  writeCodecRegs();
  
  //i2sDataOut[0]=0xaaaa;
  //i2sDataOut[1]=0x5555;
  
#ifdef  RECORD_SOUND 
  g_status = initTxtFile();
  if(g_status){
    errorAlarm();
  }
#endif //RECORD_SOUND   

#ifdef DIGITAL_MIC_ECHO    
    status1= HAL_I2SEx_TransmitReceive_DMA(&hi2s2, i2sDataOut, i2sDataIn, 2);
#endif

#ifdef DIGITAL_MIC_ECHO_EFFECT    
    status1= HAL_I2SEx_TransmitReceive_DMA(&hi2s2, i2sDataOut, i2sDataIn, 2);
#endif    

#ifdef LINE_DIGITAL_ECHO
    status1= HAL_I2SEx_TransmitReceive_DMA(&hi2s2, i2sDataOut, i2sDataIn, 2);
#endif        
    
#ifdef SOUND_ORIENTATION  
  status1= HAL_I2SEx_TransmitReceive_DMA(&hi2s2, i2sDataOut, i2sDataIn, AMMOUNT_OF_STEREO_SAMPLES*2);
#endif  

#ifdef RECORD_SOUND  
  g_statusHal= HAL_I2SEx_TransmitReceive_DMA(&hi2s2, i2sDataIn, i2sDataIn, AMMOUNT_OF_STEREO_SAMPLES*2);
  if(HAL_OK != g_statusHal)
  {
    errorAlarm();
  }
#endif  
  //0x4002605C
  g_addres =(uint32_t) &hdma_spi2_rx.Instance->NDTR;
  while (1)
  {
    g_dmaPtr      = hdma_spi2_rx.Instance->NDTR;//0....AMMOUNT_OF_STEREO_SAMPLES*2
    g_RxSamplePtr = AMMOUNT_OF_STEREO_SAMPLES-(g_dmaPtr/2 );//?
      //g_RxSamplePtr 0 1 2 3 4 5 ... AMMOUNT_OF_STEREO_SAMPLES
    if(g_ProcPtr <= g_RxSamplePtr)
    {
        g_procLen = g_RxSamplePtr - g_ProcPtr;
        updateMaxLen(g_procLen);
        
        procSamples(i2sDataIn, g_procLen, g_ProcPtr);
        g_procLen=0;
        g_ProcPtr = g_RxSamplePtr;
    }
    else if(g_RxSamplePtr < g_ProcPtr)
    {
        g_procLen =AMMOUNT_OF_STEREO_SAMPLES -g_ProcPtr;
        updateMaxLen(g_procLen);
        
        procSamples(i2sDataIn, g_procLen, g_ProcPtr);
        g_ProcPtr = 0;
        g_procLen = 0;
        
        HAL_GPIO_TogglePin(LED3_BOARD_PORT, LED3_BOARD_PIN);
        g_proc_iteration++;
        
        g_procLen  = g_RxSamplePtr;
        updateMaxLen(g_procLen);
        
        procSamples(i2sDataIn, g_procLen, 0);
        g_ProcPtr = g_RxSamplePtr;
        g_procLen = 0;
    }
    else
    {
    }
      
    if(g_btnPressed)
    {
        g_btnPressed=0;
        if(g_fileOpen)
        { 
          #ifdef  RECORD_SOUND 
          g_res = f_close(&g_testFile);
          if(FR_OK ==g_res)
          {
           int a=0;
           g_fileOpen=0;
          } 
          #endif //RECORD_SOUND 
        }
    }

  }// while(1)



}


void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 100;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
 
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_5);
 
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



#ifdef USE_FULL_ASSERT


void assert_failed(uint8_t* file, uint32_t line)
{


}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
