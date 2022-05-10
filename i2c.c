

#include "i2c.h"

#include "gpio.h"
#include "utils.h"

U8 chipAddrWr=0x34;  //   0011 0100      

//To control the WM8731/L on the 2-wire bus the master control device must initiate
//a data transfer by  establishing a start condition, defined by a high to low transition
//on SDIN while SCLK remains high.  This indicates that an address and data transfer will 
//follow. All peripherals on the 2-wire bus respond  to the start condition and shift in
//the next eight bits (7-bit address + R/W bit). The transfer is MSB  first. The 7-bit address 
//consists of a 6-bit base address + a single programmable bit to select one of  two available
//addresses for this device (see table 24). 

//If the correct address is received and the R/W  bit is ‘0’, indicating a write, 
//then the WM8731/L will respond by pulling SDIN low on the next clock  pulse (ACK). 
//The WM8731/L is a write only device and will only respond to the R/W bit indicating a  write.
//If the address is not recognised the device will return to the idle condition and wait for a new 
//start condition and valid address.

// line stereo i2s 32khz 32 bit



#ifdef ANALOG_LINE_STEREO_ECHO
U16 reg0_leftLineIn          = 0x1F00;
U16 reg1_rightLineIn         = 0x1F02;
U16 reg2_leftHeadphoneOut    = 0xFF04;
U16 reg3_rightHeadphoneOut   = 0xfF06;
U16 reg4_analogAudioPathCntrl= 0x2A08;
U16 reg5_DigitalAudioPathControl =0x2A08;
U16 reg6_powerDownControl    = 0x020c;//
U16 reg7_digitalAudioInterfaceFormat = 0x4e0e;
U16 reg8_samplingControl     = 0x1810;
U16 reg9_activeControl       = 0x0112;
U16 reg15_resetRegister      = 0x001E;
#endif

#ifdef LINE_DIGITAL_ECHO
U16 reg0_leftLineIn          = 0x1700;
U16 reg1_rightLineIn         = 0x1702;
U16 reg2_leftHeadphoneOut    = 0xf904;
U16 reg3_rightHeadphoneOut   = 0xf906;
U16 reg4_analogAudioPathCntrl= 0x1208;
U16 reg5_DigitalAudioPathControl =0x000a;
U16 reg6_powerDownControl    = 0x020c;
U16 reg7_digitalAudioInterfaceFormat =0x020e ; 
U16 reg8_samplingControl     =0x1810 ;
U16 reg9_activeControl       =0x0112 ;
U16 reg15_resetRegister      =0x001e ;
#endif


#ifdef LINE_DIGITAL_READ
U16 reg0_leftLineIn          = 0x1700;
U16 reg1_rightLineIn         = 0x1702;
U16 reg2_leftHeadphoneOut    = 0xf904;
U16 reg3_rightHeadphoneOut   = 0xf906;
U16 reg4_analogAudioPathCntrl= 0x1208;
U16 reg5_DigitalAudioPathControl =0x000a;
U16 reg6_powerDownControl    = 0x020c;
U16 reg7_digitalAudioInterfaceFormat =0x020e ; 
U16 reg8_samplingControl     =0x1810 ;
U16 reg9_activeControl       =0x0112 ;
U16 reg15_resetRegister      =0x001e ;
#endif


#ifdef DIGITAL_MIC_ECHO_EFFECT
U16 reg0_leftLineIn          = 0x9300;
U16 reg1_rightLineIn         = 0x9302;
U16 reg2_leftHeadphoneOut    = 0xF904;
U16 reg3_rightHeadphoneOut   = 0xf906;
U16 reg4_analogAudioPathCntrl= 0x1408;
U16 reg5_DigitalAudioPathControl =0x130A;//0x110A;  120a

U16 reg6_powerDownControl    = 0x010C;//
U16 reg7_digitalAudioInterfaceFormat = 0x120e; 
U16 reg8_samplingControl     = 0x1810;
U16 reg9_activeControl       = 0x0112;
U16 reg15_resetRegister      = 0x001E;
#endif


#ifdef DIGITAL_MIC_ECHO
U16 reg0_leftLineIn          = 0x9300;
U16 reg1_rightLineIn         = 0x9302;
U16 reg2_leftHeadphoneOut    = 0xF904;
U16 reg3_rightHeadphoneOut   = 0xf906;
U16 reg4_analogAudioPathCntrl= 0x1408;
U16 reg5_DigitalAudioPathControl =0x130A;//0x110A;  120a

U16 reg6_powerDownControl    = 0x010C;//
U16 reg7_digitalAudioInterfaceFormat = 0x120e; 
U16 reg8_samplingControl     = 0x1810;
U16 reg9_activeControl       = 0x0112;
U16 reg15_resetRegister      = 0x001E;
#endif


#ifdef ANALOG_MIC_ECHO
U16 reg0_leftLineIn          = 0x9700;
U16 reg1_rightLineIn         = 0x9702;
U16 reg2_leftHeadphoneOut    = 0xFF04;
U16 reg3_rightHeadphoneOut   = 0xfF06;
U16 reg4_analogAudioPathCntrl= 0x2D08;
U16 reg5_DigitalAudioPathControl =0x100A;//0x2A08;  // 
U16 reg6_powerDownControl    = 0x000C;//
U16 reg7_digitalAudioInterfaceFormat = 0x020e;
U16 reg8_samplingControl     = 0x1810;
U16 reg9_activeControl       = 0x0112;
U16 reg15_resetRegister      = 0x001E;
#endif


void writeCodecRegs(){
  globalError=0;
  //HAL_Delay(200);
  //status = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg15_resetRegister, 2);
  do
  {
    HAL_Delay(100);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg15_resetRegister, 2);
  }  while(HAL_OK != g_statusHal);
  

  do
  {
    globalError=1;
    HAL_Delay(500);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg0_leftLineIn, 2);
  }
  while(HAL_OK!=g_statusHal);
 
  do
  {
    globalError=2;
    HAL_Delay(100);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg1_rightLineIn, 2);
  }
  while(HAL_OK!=g_statusHal);
 
  
  do
  {
    globalError=3;
    HAL_Delay(200);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg2_leftHeadphoneOut, 2);
  }
  while(HAL_OK!=g_statusHal);
  

  
  do
  {
    globalError=4;
    HAL_Delay(400);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg3_rightHeadphoneOut, 2);

  }
  while(HAL_OK!=g_statusHal);
  
  do
  {
    globalError=5;
    HAL_Delay(100);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg4_analogAudioPathCntrl, 2);

  }
  while(HAL_OK!=g_statusHal);
  

  
  
  do
  {
    globalError=6;
    HAL_Delay(100); 
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg5_DigitalAudioPathControl, 2);

  }
  while(HAL_OK!=g_statusHal);
  

  
  
  do
  {
    globalError=7;
    HAL_Delay(500);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg6_powerDownControl, 2);

  }
  while(HAL_OK!=g_statusHal);
  

  
  
  do
  {
    globalError=8;
    HAL_Delay(100);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg7_digitalAudioInterfaceFormat, 2);

  }
  while(HAL_OK!=g_statusHal);
  

  
  
  do
  {
    globalError=9;
    HAL_Delay(100);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg8_samplingControl, 2);

  }
  while(HAL_OK!=g_statusHal);
  

  
  
  do
  {
    globalError=10;
    HAL_Delay(100);
    g_statusHal = HAL_I2C_Master_Transmit_IT(&hi2c1,(uint16_t) chipAddrWr, (uint8_t*) &reg9_activeControl, 2);

  }
  while(HAL_OK!=g_statusHal);
  

  
  globalError=11;
  HAL_Delay(100); 
  
}

//void readCodecRegs()
//{
//  uint8_t pData[20];
//  for(i=0;i<15; i+=2)
//  {
//    HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(&hi2c1, (uint16_t) chipAddr, &pData[i], 2);
//  }
//
//}


I2C_HandleTypeDef hi2c1;


void MX_I2C1_Init(void){
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 1000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
  
  HAL_I2C_Init(&hi2c1);

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c){
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)  {
 
    /**I2C1 GPIO Configuration    
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL 
    */
    GPIO_InitStruct.Pin   = I2C1_SDA_PB7_AF4_Pin | I2C1_SCL_PB8_AF4_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __I2C1_CLK_ENABLE();
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c){
  if(hi2c->Instance==I2C1)  {
    __I2C1_CLK_DISABLE();
    /**I2C1 GPIO Configuration    
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL 
    */
    HAL_GPIO_DeInit(GPIOB, I2C1_SDA_PB7_AF4_Pin|I2C1_SCL_PB8_AF4_Pin);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  }
} 

