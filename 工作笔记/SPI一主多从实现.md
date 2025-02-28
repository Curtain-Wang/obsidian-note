## 背景
博主参与的项目中，有个需要接OLED屏的功能，打算用SPI通信去实现，然后原本的板子上已经有一个从机和SPI有连接了，故将原本的一主一从的架构改成了一主多从。
## 一主多从架构图

![[Pasted image 20250228154646.png]]

## 相关代码
### SPI初始化代码
```
void Init_SPI1(void)
{
  stc_gpio_cfg_t GpioInitStruct;
  stc_spi_cfg_t  SpiInitStruct;
  
  DDL_ZERO_STRUCT(GpioInitStruct);
  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
  //----------SPI0引脚配置:主机----------------
  GpioInitStruct.enDir = GpioDirOut;           ///< 端口方向配置
  GpioInitStruct.enDrv = GpioDrvH;             ///< 端口驱动能力配置
  GpioInitStruct.enPu = GpioPuEnable;
  Gpio_Init(GpioPortB, GpioPin13,&GpioInitStruct);       //配置引脚PB13作为SPI1_SCK
  Gpio_SetAfMode(GpioPortB, GpioPin13,GpioAf1);          //配置引脚PB13作为SPI1_SCK
  Gpio_Init(GpioPortB, GpioPin15,&GpioInitStruct);       //配置引脚PB15作为SPI1_MOSI 
  Gpio_SetAfMode(GpioPortB, GpioPin15,GpioAf1);          //配置引脚PB15作为SSPI1_MOSI
  Gpio_Init(GpioPortB, GpioPin12,&GpioInitStruct);       //配置引脚PB12作为SPI1_CS（FLASH）
  Gpio_Init(GpioPortE, GpioPin4, &GpioInitStruct);      //配置引脚PE4作为SPI1_CS（OLED）
//  Gpio_SetAfMode(GpioPortB, GpioPin12,GpioAf1);          //配置引脚PB12作为SPI1_CS
  GpioInitStruct.enDir = GpioDirIn;
  Gpio_Init(GpioPortB, GpioPin14,&GpioInitStruct);
  Gpio_SetAfMode(GpioPortB, GpioPin14,GpioAf1);          //配置引脚PB14作为SPI1_MISO
  Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi1,TRUE);
  //-------------SPI1模块配置：主机----------------
  SpiInitStruct.enSpiMode = SpiMskMaster;                //配置位主机模式
  SpiInitStruct.enPclkDiv = SpiClkMskDiv64;              //波特率：32M fsys/16
  SpiInitStruct.enCPHA    = SpiMskCphafirst;             //第一边沿采样
  SpiInitStruct.enCPOL    = SpiMskcpollow;               //极性为低
  Spi_Init(M0P_SPI1, &SpiInitStruct);
}
```
### GPIO初始化相关代码
```
  stc_gpio_cfg_t  stcGpioCfg;
  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
  //-------------输出端口配置-------------
  DDL_ZERO_STRUCT(stcGpioCfg); 
  stcGpioCfg.enDir = GpioDirOut;
  stcGpioCfg.enDrv = GpioDrvH;
  stcGpioCfg.enPu  = GpioPuEnable;                               //上拉开启
  stcGpioCfg.enPd  = GpioPdDisable;                              //下拉关闭
  Gpio_Init(GpioPortE,GpioPin3,&stcGpioCfg);                     //OLED 的DC引脚 1-数据 0-指令
  Gpio_Init(GpioPortE,GpioPin2,&stcGpioCfg);                     //OLED 的RES引脚 0复位 1不变
  P_OLED_RES = 1;               //OLED默认不复位
  P_OLED_DC = 0;                //默认指令模式
  P_OLED_CS = 1;                //默认高电平 不选
  P_FlashCS = 1;                //默认高电平 不选
```
### 主机发送OLED数据相关代码
```
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
  u8 i;			  
  if(cmd)
  {
    OLED_DC_Set(); //设置DC为高电平表示写数据
  }else
  {
    OLED_DC_Clr(); //设置DC为低电平表示写指令
  }
  P_FlashCS = 1;   //设置Flash不被片选
  OLED_CS_Clr();   //拉低OLED片选引脚
  Spi_SetCS(M0P_SPI1, FALSE);    //提示主机开始片选
  while(FALSE == M0P_SPI1->STAT_f.TXE){;}   //等待发送缓冲区为空
  M0P_SPI1->DATA = dat; 	            //发送数据
  while(FALSE == M0P_SPI1->STAT_f.RXNE){;} //等到收到从机回复的数据
  dat = M0P_SPI1->DATA;                    //取出从机回复的数据
  while(FALSE == M0P_SPI1->STAT_f.TXE){;}   //等待发送缓冲区为空
  while(TRUE == M0P_SPI1->STAT_f.BUSY){;}   //等待SPI总线空闲
  OLED_CS_Set();                            //OLED片选引脚置高
  Spi_SetCS(M0P_SPI1, TRUE);                //主机结束片选
  OLED_DC_Set();   	                        //设置DC为高电平表示写数据
}
```
PS: 这里有个很关键的点，就是虽然我只是往从机发数据，但是<font color="red">必须要将从机返回的数据取出</font>。由于发送缓冲区和接收缓冲区用的都是M0P_SPI1->DATA，如果不取出从机返回的数据的话，可能会影响下一次主机数据的发送。