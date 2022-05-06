
#include "stm32f4xx_hal.h"


void HAL_MspInit(void)
{

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);

  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);

  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);

  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);


  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);


}


