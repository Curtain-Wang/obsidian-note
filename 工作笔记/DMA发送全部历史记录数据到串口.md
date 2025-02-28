## 背景
E1109项目中，有个读取全部历史记录的功能，如果下位机在main中将全部历史记录单纯地通过串口传输会比较占用cpu资源，影响main中别的功能。最后商量得出以下实现方案：

**定义两个发送缓冲区DMATxbuf1和DMATxbuf2，以及这两个发送缓冲区的标志DMATxbuf1Flag和DMATxbuf2Flag，发送缓冲区的标志取值有三种：可搬运、可发送和发送中。主main中负责搬运数据到这两个发送缓冲区中以及如果存在可发送的缓冲区且不存在发送中的缓冲区，那么主main开启一下这个DMA的通道使能。在DMA传输完成中断中，需要更新刚刚发送缓冲区的标志即可。**
## 相关代码

### 初始化串口和DMA
```
void InitDebug_Sub(void)
{
//  stc_gpio_cfg_t stcGpioCfg;
//  stc_uart_cfg_t    stcCfg;
  /***********************RAM口初始化******************************************/
  Rxput_232 = 0;
  Rxget_232 = 0;
  Txput_232 = 0;
  Txget_232 = 0;
  TXing_232 = 0;
    /***********************GPIO口初始化**********************************************/      
  stc_gpio_cfg_t stcGpioCfg;
  DDL_ZERO_STRUCT(stcGpioCfg);
  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
   /***********TX************/
  stcGpioCfg.enDir =  GpioDirOut;
  Gpio_Init(GpioPortA,GpioPin0,&stcGpioCfg);
  Gpio_SetAfMode(GpioPortA,GpioPin0,GpioAf2); //配置PC10为LPUART1_TX
  /**********RX**************/
  stcGpioCfg.enDir =  GpioDirIn;
  Gpio_Init(GpioPortA,GpioPin1,&stcGpioCfg);
  Gpio_SetAfMode(GpioPortA,GpioPin1,GpioAf2); //配置PC11为LPUART1_RX
  /***********************串口初始化**********************************************/    
  stc_lpuart_cfg_t  stcCfg;
  DDL_ZERO_STRUCT(stcCfg);
  Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart1,TRUE);  //<外设模块时钟使能  
  stcCfg.enStopBit = LPUart1bit;                   ///<1停止位    
  stcCfg.enMmdorCk = LPUartEven;                   ///<偶校验
  stcCfg.stcBaud.enSclkSel = LPUartMskPclk;        ///<传输时钟源
  stcCfg.stcBaud.u32Sclk = Sysctrl_GetPClkFreq();  ///<PCLK获取
  stcCfg.stcBaud.enSclkDiv = LPUartMsk4Or8Div;     ///<采样分频
  stcCfg.stcBaud.u32Baud = 38400;                   ///<波特率
  stcCfg.enRunMode = LPUartMskMode1;               ///<工作模式1，异步模式全双工
  LPUart_Init(M0P_LPUART1, &stcCfg);
  /***********************DMA初始化**********************************************/    
  Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE); //开启DMA外设时钟
  stc_dma_cfg_t dmaCfg;
  //CONFB配置
  dmaCfg.enMode = DmaMskBlock;  //Block传输
  dmaCfg.enTransferWidth = DmaMsk8Bit; // 传输宽度 8bit（适用于 UART）
  dmaCfg.enSrcAddrMode = DmaMskSrcAddrInc;  // 源地址递增
  dmaCfg.enDstAddrMode = DmaMskDstAddrFix;  // 目标地址固定
  dmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable;//使能源地址重载
  dmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadEnable;//使能目标地址重载
  dmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable; //使能BC/TC重载
  dmaCfg.enTransferMode = DmaMskOneTransfer;  // 单次传输模式
  //CONFA配置
  dmaCfg.u16BlockSize = 1;  // 每次传输 1 字节
  dmaCfg.u16TransferCnt = 131;  //实际传输的时候会设置长度
  dmaCfg.enRequestNum = DmaLpUart1TxTrig;                //触发源配置：LPUART1 SBUF为空
  
  dmaCfg.u32SrcAddress = (uint32_t)&DMATxbuf1.DATA[0];      // 源地址：发送缓冲区
  dmaCfg.u32DstAddress = (uint32_t)&M0P_LPUART1->SBUF; // 目标地址：LPUART1 发送寄存器
  dmaCfg.enPriority = DmaMskPriorityFix;  // 固定优先级
  
  Dma_InitChannel(DmaCh1, &dmaCfg);     //初始化DMA通道
  Dma_EnableChannelIrq(DmaCh1); //使能DMA传输完成中断
  NVIC_EnableIRQ(DMAC_IRQn);    //使能NVIC DMA中断
  sendingHistoryRemaingTime = 0; //初始化发送历史记录剩余时间为0
  carryHistoryFinishFlag = 1;   //初始化不需要搬运历史记录
  DMATxbuf1Flag = 1;            //初始化DMA缓冲区1可搬运
  DMATxbuf2Flag = 1;            //初始化DMA缓冲区2可搬运
  /**********************LPUART 中断使能*********************************/
  LPUart_ClrStatus(M0P_LPUART1,LPUartRC);          //<清接收中断请求
  LPUart_ClrStatus(M0P_LPUART1,LPUartTC);          //<清发送中断请求
  LPUart_EnableIrq(M0P_LPUART1,LPUartRxIrq);       //<使能接收中断
  LPUart_DisableIrq(M0P_LPUART1,LPUartTxIrq);      //发送完成之后关闭发送中断
  //  LPUart_EnableIrq(M0P_LPUART1,LPUartTxIrq);       //<不使能发送中断，
  EnableNvic(LPUART1_IRQn,IrqLevel3,TRUE);         //<系统中断使能
  LPUart_EnableFunc(M0P_LPUART1,LPUartDmaTxFunc);  //DMA发送LPUART使能
  P_RD_DBG = 0;//初始化调试口485方向为接收
```

### 收到获取全部指令时
```
  case 12:      //读取全部历史记录
    sendingHistoryRemaingTime = 2000; //设置发送超时时间，同时也标记着此时在传输历史及记录
    FlashReadSP = W25QX_RECSTART_ADDR; //初始化历史记录读取地址为开始地址
    //重置两个缓冲区状态为可搬运
    DMATxbuf1Flag = 1;
    DMATxbuf2Flag = 1;
    carryHistoryFinishFlag = 0;//重置历史记录没有搬运完
    break;
```
### 主main中的搬运、发送历史数据
```
  //发送历史记录期间且历史记录没有搬运完
  if(sendingHistoryRemaingTime)
  {
    //尝试搬运历史记录到缓冲区
    if(!carryHistoryFinishFlag)
    {
      carryHistory();
    }
    //尝试并触发DMA发送
    dmaSendHistoryRecord();
  }
```

### 搬运、发送历史数据方法详情
```
void carryHistory()
{
  typedef_DMATXBUF* dmaTxbuf;
  u8 carryBufType = 0; //记录搬的是哪个缓冲区
  //选择可搬运的缓冲区
  if(DMATxbuf1Flag == 1)
  {
    dmaTxbuf = &DMATxbuf1;
    carryBufType = 1;
  }
  else if(DMATxbuf2Flag == 1)
  {
    dmaTxbuf = &DMATxbuf2;
    carryBufType = 2;
  }else
  {
    //目前没有可搬运的
    return;
  }
  u8 count = 0;
  dmaTxbuf->LEN = 0;
  while(count < 3 && !carryHistoryFinishFlag)
  {
    u8 cmd2=0;
    u8 buf[300];
    structHistory_TypeDef* ptr = (structHistory_TypeDef*)buf;
    //历史记录已经读完了
    if(FlashReadSP>=W25QX_RECEND_ADDR)
    {
      cmd2 = 0x01;
      FlashReadSP = W25QX_RECSTART_ADDR;     //首条记录
      carryHistoryFinishFlag = 1;
    }
    W25QX_BufferRead((u8*)&ptr->HistoryData,FlashReadSP,sizeof(structEEP_GET_HISTOR_TypeDef)); //读取记录
    if(ptr->HistoryData.FLAG != 0x7e)
    {
      FlashReadSP += 4096;
      FlashReadSP &= 0xfffff000;
      W25QX_BufferRead((u8*)&ptr->HistoryData,FlashReadSP,sizeof(structEEP_GET_HISTOR_TypeDef)); //读取记录索引
      if(ptr->HistoryData.FLAG != 0x7e) 
      {
        cmd2 = 0x01;
	    FlashReadSP = W25QX_RECSTART_ADDR;     //首条记录
	    carryHistoryFinishFlag = 1;
      }
    }
    ptr->CMD1 = 0x8c;
    ptr->CMD2 = cmd2;
    ptr->ADDR = 0x01;
    ptr->SoftVer = SOFTVER;      
    ptr->checksum = CRC16(buf,sizeof(structHistory_TypeDef)-2);
    dmaTxbuf->DATA[dmaTxbuf->LEN++] = 0x10;
    dmaTxbuf->DATA[dmaTxbuf->LEN++] = 0x02;
    for(u16 i = 0; i < sizeof(structHistory_TypeDef); i++)
    {
      u8 qq = buf[i];
      dmaTxbuf->DATA[dmaTxbuf->LEN++] = qq;
      /* -----0x10加发一个 -----*/
      if(qq == DLE) dmaTxbuf->DATA[dmaTxbuf->LEN++] = DLE;
    }
    dmaTxbuf->DATA[dmaTxbuf->LEN++] = DLE;
    dmaTxbuf->DATA[dmaTxbuf->LEN++] = ETX;
    dmaTxbuf->DATA[dmaTxbuf->LEN++] = 0x16; 
    count++;
    FlashReadSP += 128;
  }
  //说明搬运了一些历史记录，需要发送，更新标记
  if(count > 0)
  {
    //更新缓冲区1的标志
    if(carryBufType == 1)
    {
      DMATxbuf1Flag = 2;//缓冲区1可发送
    }
    if(carryBufType == 2)
    {
      DMATxbuf2Flag = 2;//缓冲区2可发送
    }
    //说明还有数据待发送，更新过期时间
    sendingHistoryRemaingTime = 2000;
  }
}

void dmaSendHistoryRecord()
{
  //存在发送中的数据
  if(DMATxbuf1Flag == 3 || DMATxbuf2Flag == 3)
  {
    return;
  }
  if(DMATxbuf1Flag == 2)
  {
    UART_DMA_Send_History(1);
  }else if(DMATxbuf2Flag == 2)
  {
    UART_DMA_Send_History(2);
  }
}
```

### DMA发送中断处理函数
```
void DMAC_IRQHandler(void)
{
#if (INT_CALLBACK_ON == INT_CALLBACK_DMAC)    
    Dmac_IRQHandler();
#endif 
        // DMA 中断处理代码
    if (M0P_DMAC->CONFB1_f.STAT == 5)  // 传输完成标志101
    {
        // 清除传输完成标志
        M0P_DMAC->CONFB1_f.STAT = 0;  
        //更新缓冲区标志
        if(DMATxbuf1Flag == 3)
        {
          DMATxbuf1Flag = 1;
        }else if (DMATxbuf2Flag == 3)
        {
          DMATxbuf2Flag = 1;
        }
    }
    M0P_DMAC->CONFB1_f.STAT = 0; 
}
```