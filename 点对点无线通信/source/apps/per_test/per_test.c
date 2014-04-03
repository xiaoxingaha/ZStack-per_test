/* 包含头文件 */
/********************************************************************/
#include "hal_led.h"
#include "hal_int.h"
#include "hal_timer_32k.h"
#include "hal_board.h"
#include "hal_rf.h"
#include "hal_assert.h"
#include "basic_rf.h"
#include "per_test.h"
/********************************************************************/
/*******************************************************************/

//#define MODE_SEND               // 屏蔽时：接收器
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
static void appTransmitter();
static void appReceiver();
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

    /* 初始化Basic RF */
    basicRfConfig.myAddr = RX_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) 
    {
        HAL_ASSERT(FALSE);
    }
    basicRfReceiveOn();

    /* 主循环 */
    while (TRUE) 
    {
        while(!basicRfPacketIsReady());  // 等待新的数据包
        if(basicRfReceive((uint8*)&rxPacket, MAX_PAYLOAD_LENGTH, &rssi)>0) 
        {
            halLedSet(1);  // 点亮LED1
                              
            UINT32_NTOH(rxPacket.seqNumber);  // 改变接收序号的字节顺序
            segNumber = rxPacket.seqNumber;
                  
            /* 若果统计被复位，设置期望收到的数据包序号为已经收到的数据包序号 */     
            if(resetStats)
            {
                rxStats.expectedSeqNum = segNumber;   
                resetStats=FALSE;
            }
              
            rxStats.rssiSum -= perRssiBuf[perRssiBufCounter];  // 从sum中减去旧的RSSI值
            perRssiBuf[perRssiBufCounter] =  rssi;             // 存储新的RSSI值到环形缓冲区，之后它将被加入sum
      
            rxStats.rssiSum += perRssiBuf[perRssiBufCounter];  // 增加新的RSSI值到sum
            if(++perRssiBufCounter == RSSI_AVG_WINDOW_SIZE) 
            {
              perRssiBufCounter = 0;      
            }
      
            /* 检查接收到的数据包是否是所期望收到的数据包 */
            if(rxStats.expectedSeqNum == segNumber)  // 是所期望收到的数据包 
            {
                rxStats.expectedSeqNum++;
            }  
            else if(rxStats.expectedSeqNum < segNumber)  // 不是所期望收到的数据包（收到的数据包的序号大于期望收到的数据包的序号）
            {                                            // 认为丢包
              rxStats.lostPkts += segNumber - rxStats.expectedSeqNum;
              rxStats.expectedSeqNum = segNumber + 1;
            }
            else  // 不是所期望收到的数据包（收到的数据包的序号小于期望收到的数据包的序号）
            {     // 认为是一个新的测试开始，复位统计变量
              rxStats.expectedSeqNum = segNumber + 1;
              rxStats.rcvdPkts = 0;
              rxStats.lostPkts = 0;
            }
            rxStats.rcvdPkts++;
            
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
    uint8 n;
  
    /* 初始化Basic RF */
    basicRfConfig.myAddr = TX_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) 
    {
      HAL_ASSERT(FALSE);
    }
  
    halRfSetTxPower(2);      //HAL_RF_TXPOWER_4_DBM  设置输出功率
  
    burstSize = 100000;       //设置进行一次测试所发送的数据包数量
    basicRfReceiveOff();      //Basic RF在发送数据包前关闭接收器，在发送完一个数据包后打开接收器
  
    appConfigTimer(0xC8);     //配置定时器和IO
  
    /* 初始化数据包载荷 */
    txPacket.seqNumber = 0;
    for(n = 0; n < sizeof(txPacket.padding); n++) 
    {
      txPacket.padding[n] = n;
    }
    
    /* 主循环 */
    while (TRUE) 
    {
        if (pktsSent < burstSize) 
        {
            UINT32_HTON(txPacket.seqNumber);  // 改变发送序号的字节顺序
            basicRfSendPacket(RX_ADDR, (uint8*)&txPacket, PACKET_SIZE);
  
            UINT32_NTOH(txPacket.seqNumber);    //在增加序号前将字节顺序改回为主机顺序
            txPacket.seqNumber++;
  
            pktsSent++;
            appState = IDLE;
            halLedToggle(1);   //切换LED1的亮灭状态
            halLedToggle(2);   //切换LED2的亮灭状态
            halMcuWaitMs(1000);
        }
  
        pktsSent = 0;   // 复位统计和序号
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
    uint8 i;
  
    appState = IDLE;           // 初始化应用状态为空闲
    appStarted = FALSE;        // 初始化启动标志位FALSE
          
    /* 初始化Basic RF */
    basicRfConfig.panId = PAN_ID;        // 初始化个域网ID
    basicRfConfig.ackRequest = FALSE;    // 不需要确认
  
    halBoardInit();  
    
    if(halRfInit()==FAILED)      //初始化hal_rf
      HAL_ASSERT(FALSE);
      
    /* 快速闪烁8次led1,led2 */
    for(i = 0; i < 16; i++)
    {
      halLedToggle(1);  // 切换led1的亮灭状态
      halLedToggle(2);  // 切换led2的亮灭状态
      halMcuWaitMs(50); // 延时大约50ms
    }
      
    halLedSet(1);       // led1指示灯亮，指示设备已上电运行
    halLedClear(2);
  
    basicRfConfig.channel = 0x0B;       // 设置信道
    
#ifdef MODE_SEND
    appTransmitter();     // 发送器模式       
#else
    appReceiver();        // 接收器模式    
#endif  

    HAL_ASSERT(FALSE);
}


