// 专业尚阳
/* 包含头文件 */
/********************************************************************/
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_int.h"
#include "hal_timer_32k.h"
#include "hal_joystick.h"
#include "hal_button.h"
#include "hal_board.h"
#include "hal_rf.h"
#include "hal_assert.h"
#include "util_lcd.h"
#include "basic_rf.h"
#include "per_test.h"
/********************************************************************/
/*******************************************************************/

#define MODE_SEND               // 屏蔽时：接收器
                                  // 不屏蔽时：发送器

/* 应用状态 */
/********************************************************************/
#define IDLE                      0
#define TRANSMIT_PACKET           1
/********************************************************************/

/* 本地变量 */
/********************************************************************/
static basicRfCfg_t basicRfConfig;
static perTestPacket_t txPacket;
static perTestPacket_t rxPacket;
static volatile uint8 appState;
static volatile uint8 appStarted;

/********************************************************************/

/* 本地函数 */
/********************************************************************/
static void appTimerISR(void);
static void appStartStop(void);
static void appTransmitter();
static void appReceiver();
void UartTX_Send_String();
void initUART();
/********************************************************************/



/*********************************************************************
 * 函数名称：appTimerISR
 * 功    能：32KHz定时器中断服务程序。更新应用状态为发送。
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void appTimerISR(void)
{
  appState = TRANSMIT_PACKET;
}


/*********************************************************************
 * 函数名称：appStartStop
 * 功    能：启动或停止32KHz定时器以便启动或停止数据包的传输。
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void appStartStop(void)
{
  appStarted ^= 1;

  if(appStarted) 
  {
    halTimer32kIntEnable();
  }
  else
  {
    halTimer32kIntDisable();
  }
}


/*********************************************************************
 * 函数名称：appConfigTimer
 * 功    能：配置本应用的中断。使用32KHz定时器。
 * 入口参数：rate  定时器中断的频率。该值必须在1到32768HZ之间。
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void appConfigTimer(uint16 rate)
{
  halTimer32kInit(TIMER_32K_CLK_FREQ/rate);
  halTimer32kIntConnect(&appTimerISR);
}


/*********************************************************************
 * 函数名称：appReceiver
 * 功    能：接收器的应用代码。控制器进入无限循环。
 * 入口参数：basicRfConfig  Basic RF 配置数据
 *           rxPacket       perTestPacket_t类型变量
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void appReceiver()
{
  uint32 segNumber=0;                              // 数据包序列号 
  int16 perRssiBuf[RSSI_AVG_WINDOW_SIZE] = {0};    // 存储RSSI的环形缓冲区
  uint8 perRssiBufCounter = 0;                     // 计数器用于RSSI缓冲区统计
  perRxStats_t rxStats = {0,0,0,0};                // 接收状态 
  int16 rssi;
  uint8 resetStats=FALSE;
  int16 MyDate[10];            //串口数据串数字
   initUART();     // 初始化串口
#ifdef INCLUDE_PA
  uint8 gain;

  // 选择增益 (仅SK - CC2590/91模块有效)
  gain =appSelectGain();
  halRfSetGain(gain);
#endif
    
  /* 初始化Basic RF */
  basicRfConfig.myAddr = RX_ADDR;
  if(basicRfInit(&basicRfConfig)==FAILED) 
  {
    HAL_ASSERT(FALSE);
  }
  basicRfReceiveOn();

  /* 在LCD上显示相关信息 */
  //halLcdClear();
    
  //halLcdWriteCharString(0,HAL_LCD_LINE_1, "Mode:Receiver");
  //halLcdWriteCharString(0,HAL_LCD_LINE_3, "Ready");
 
  /* 主循环 */
  while (TRUE) 
  {
    while(!basicRfPacketIsReady());  // 等待新的数据包
    if(basicRfReceive((uint8*)&rxPacket, MAX_PAYLOAD_LENGTH, &rssi)>0) 
    {
      halLedSet(1);  // 点亮LED1
      //halLedSet(2);  // 点亮LED2
			
      UINT32_NTOH(rxPacket.seqNumber);  // 改变接收序号的字节顺序
      segNumber = rxPacket.seqNumber;
            
      /* 若果统计被复位，设置期望收到的数据包序号为已经收到的数据包序号 */     
      if(resetStats)
      {
        rxStats.expectedSeqNum = segNumber;
        
        resetStats=FALSE;
      }
        
      rxStats.rssiSum -= perRssiBuf[perRssiBufCounter];  // 从sum中减去旧的RSSI值
      perRssiBuf[perRssiBufCounter] =  rssi;  // 存储新的RSSI值到环形缓冲区，之后它将被加入sum

      rxStats.rssiSum += perRssiBuf[perRssiBufCounter];  // 增加新的RSSI值到sum
       MyDate[4] = rssi;           ////
       MyDate[3] = rxStats.rssiSum;////
      if(++perRssiBufCounter == RSSI_AVG_WINDOW_SIZE) 
      {
        perRssiBufCounter = 0;      
      }

      /* 检查接收到的数据包是否是所期望收到的数据包 */
      if(rxStats.expectedSeqNum == segNumber)  // 是所期望收到的数据包 
      {
        MyDate[0] = rxStats.expectedSeqNum;////
        rxStats.expectedSeqNum++;
      
      }

      else if(rxStats.expectedSeqNum < segNumber)  // 不是所期望收到的数据包（收到的数据包的序号大于期望收到的数据包的序号）
      {                                            // 认为丢包
        rxStats.lostPkts += segNumber - rxStats.expectedSeqNum;
        MyDate[2] = rxStats.lostPkts;///
        rxStats.expectedSeqNum = segNumber + 1;
        MyDate[0] = rxStats.expectedSeqNum;///
      }
      else  // 不是所期望收到的数据包（收到的数据包的序号小于期望收到的数据包的序号）
      {     // 认为是一个新的测试开始，复位统计变量
        rxStats.expectedSeqNum = segNumber + 1;
        MyDate[0] = rxStats.expectedSeqNum;///
        rxStats.rcvdPkts = 0;
        rxStats.lostPkts = 0;
      }
      MyDate[1] = rxStats.rcvdPkts;///
      rxStats.rcvdPkts++;
      
      UartTX_Send_String(MyDate,5);
      halMcuWaitMs(300);
      halLedClear(1);   //熄灭LED1
      halLedClear(2);   //熄灭LED2
      halMcuWaitMs(300);
    }
  }
}


/*********************************************************************
 * 函数名称：appTransmitter
 * 功    能：发送器的应用代码。控制器进入无限循环。
 * 入口参数：basicRfConfig  Basic RF 配置数据
 *           txPacket       perTestPacket_t类型变量
 *           appState       保存应用状态
 *           appStarted     控制传输的启动和停止
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void appTransmitter()
{
  uint32 burstSize=0;
  uint32 pktsSent=0;
  uint8 appTxPower;
  uint8 n;

  /* 初始化Basic RF */
  basicRfConfig.myAddr = TX_ADDR;
  if(basicRfInit(&basicRfConfig)==FAILED) 
  {
    HAL_ASSERT(FALSE);
  }

  /* 设置输出功率 */
  //appTxPower = appSelectOutputPower();
  halRfSetTxPower(2);//HAL_RF_TXPOWER_4_DBM
//  halRfSetTxPower(appTxPower);

  /* 设置进行一次测试所发送的数据包数量 */
  //burstSize = appSelectBurstSize();
  burstSize = 100000;
  /* Basic RF在发送数据包前关闭接收器，在发送完一个数据包后打开接收器 */
  basicRfReceiveOff();

  /* 配置定时器和IO */
  //n= appSelectRate();
  appConfigTimer(0xC8);
  //halJoystickInit();

  /* 初始化数据包载荷 */
  txPacket.seqNumber = 0;
  for(n = 0; n < sizeof(txPacket.padding); n++) 
  {
    txPacket.padding[n] = n;
  }

  /*显示相关信息 */
  //halLcdClear();
  //halLcdWriteCharString(0,HAL_LCD_LINE_1, "Mode:Transmitter");
  //halLcdWriteCharString(0,HAL_LCD_LINE_2, "CENTER to start/stop");

  /* 主循环 */
  while (TRUE) 
  {
    //while(!halJoystickPushed());  // 等待用户启动应用
    //appStartStop();
    

    //while(appStarted) 
    //{
      //if( halJoystickPushed()) 
      //{
      //  appStartStop();
      //}

      if (pktsSent < burstSize) 
      {
        //if( appState == TRANSMIT_PACKET ) 
        //{
          UINT32_HTON(txPacket.seqNumber);  // 改变发送序号的字节顺序
          basicRfSendPacket(RX_ADDR, (uint8*)&txPacket, PACKET_SIZE);

          /* 在增加序号前将字节顺序改回为主机顺序 */
          UINT32_NTOH(txPacket.seqNumber);
          txPacket.seqNumber++;

          pktsSent++;
          //utilLcdDisplayValue(HAL_LCD_LINE_3, "Sent: ", (int32)pktsSent, NULL);
          appState = IDLE;
          halLedToggle(1);   //切换LED1的亮灭状态
          halLedToggle(2);   //切换LED2的亮灭状态
          halMcuWaitMs(1000);
        //}
      }
      //else
        //appStarted = !appStarted;
    //}

    /* 复位统计和序号 */
    pktsSent = 0;
    //txPacket.seqNumber = 0;
    //halLcdClear();
    //halLedClear(2); //熄灭LED3(LED_Y)
    //halLcdWriteCharString(0,HAL_LCD_LINE_1, "Mode:Transmitter");
    //halLcdWriteCharString(0,HAL_LCD_LINE_2, "CENTER to start/stop");
  }
}


/*********************************************************************
 * 函数名称：main
 * 功    能：丢包率测试实验的main函数入口
 * 入口参数：basicRfConfig  Basic RF 配置数据
 *           appState       保存应用状态
 *           appStarted     控制传输的启动和停止
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void main (void)
{
  uint8 appMode;
  uint8 i;

  appState = IDLE;     // 初始化应用状态为空闲
  appStarted = FALSE;  // 初始化启动标志位FALSE
	
  /* 初始化Basic RF */
  basicRfConfig.panId = PAN_ID;      // 初始化个域网ID
  basicRfConfig.ackRequest = FALSE;  // 不需要确认

  halBoardInit();  // SK-SmartRF05EB评估板外设初始化

  //utilPrintLogo("PER Tester");  // 在LCD上显示Splash屏幕和Logo
  
  /* 初始化hal_rf */
  if(halRfInit()==FAILED) 
    HAL_ASSERT(FALSE);
    
  /* 快速闪烁8次LED_G、LEG_R、LEG_Y */
  for(i = 0; i < 16; i++)
  {
    halLedToggle(1);  // 切换LED_G的亮灭状态
    halLedToggle(2);  // 切换LED_R的亮灭状态
    halLedToggle(3);  // 切换LED_Y的亮灭状态
    halMcuWaitMs(50); // 延时大约50ms
  }
    
  halLedSet(1);  // LED1(LED_G)指示灯亮，指示设备已上电运行
  halLedClear(2);
  //halLedClear(3);

  //halLcdWriteCharString(0, HAL_LCD_LINE_6, "Press S2 to enter.");
  
  //while (halButtonPushed() != HAL_BUTTON_1); // 等待用户按下S2(BUTTON)键进入菜单
  halMcuWaitMs(350);
  //halLcdClear();

  basicRfConfig.channel = 0x0B;//appSelectChannel();  // 设置信道
  //appMode = appSelectMode();  // 选择运行模式
  
#ifdef MODE_SEND
  appMode = MODE_TX;
#else
  appMode = MODE_RX;
#endif  
  
  /* 发送器模式 */
  if(appMode == MODE_TX) 
  {
    appTransmitter();
   
  }
  /* 接收器模式 */
  else if(appMode == MODE_RX) 
  {
    appReceiver();
  }
    
  HAL_ASSERT(FALSE);
}


