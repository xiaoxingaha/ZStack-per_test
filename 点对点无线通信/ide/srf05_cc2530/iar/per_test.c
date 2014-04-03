// רҵ����
/* ����ͷ�ļ� */
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

#define MODE_SEND               // ����ʱ��������
                                  // ������ʱ��������

/* Ӧ��״̬ */
/********************************************************************/
#define IDLE                      0
#define TRANSMIT_PACKET           1
/********************************************************************/

/* ���ر��� */
/********************************************************************/
static basicRfCfg_t basicRfConfig;
static perTestPacket_t txPacket;
static perTestPacket_t rxPacket;
static volatile uint8 appState;
static volatile uint8 appStarted;

/********************************************************************/

/* ���غ��� */
/********************************************************************/
static void appTimerISR(void);
static void appStartStop(void);
static void appTransmitter();
static void appReceiver();
void UartTX_Send_String();
void initUART();
/********************************************************************/



/*********************************************************************
 * �������ƣ�appTimerISR
 * ��    �ܣ�32KHz��ʱ���жϷ�����򡣸���Ӧ��״̬Ϊ���͡�
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
static void appTimerISR(void)
{
  appState = TRANSMIT_PACKET;
}


/*********************************************************************
 * �������ƣ�appStartStop
 * ��    �ܣ�������ֹͣ32KHz��ʱ���Ա�������ֹͣ���ݰ��Ĵ��䡣
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
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
 * �������ƣ�appConfigTimer
 * ��    �ܣ����ñ�Ӧ�õ��жϡ�ʹ��32KHz��ʱ����
 * ��ڲ�����rate  ��ʱ���жϵ�Ƶ�ʡ���ֵ������1��32768HZ֮�䡣
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
static void appConfigTimer(uint16 rate)
{
  halTimer32kInit(TIMER_32K_CLK_FREQ/rate);
  halTimer32kIntConnect(&appTimerISR);
}


/*********************************************************************
 * �������ƣ�appReceiver
 * ��    �ܣ���������Ӧ�ô��롣��������������ѭ����
 * ��ڲ�����basicRfConfig  Basic RF ��������
 *           rxPacket       perTestPacket_t���ͱ���
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
static void appReceiver()
{
  uint32 segNumber=0;                              // ���ݰ����к� 
  int16 perRssiBuf[RSSI_AVG_WINDOW_SIZE] = {0};    // �洢RSSI�Ļ��λ�����
  uint8 perRssiBufCounter = 0;                     // ����������RSSI������ͳ��
  perRxStats_t rxStats = {0,0,0,0};                // ����״̬ 
  int16 rssi;
  uint8 resetStats=FALSE;
  int16 MyDate[10];            //�������ݴ�����
   initUART();     // ��ʼ������
#ifdef INCLUDE_PA
  uint8 gain;

  // ѡ������ (��SK - CC2590/91ģ����Ч)
  gain =appSelectGain();
  halRfSetGain(gain);
#endif
    
  /* ��ʼ��Basic RF */
  basicRfConfig.myAddr = RX_ADDR;
  if(basicRfInit(&basicRfConfig)==FAILED) 
  {
    HAL_ASSERT(FALSE);
  }
  basicRfReceiveOn();

  /* ��LCD����ʾ�����Ϣ */
  //halLcdClear();
    
  //halLcdWriteCharString(0,HAL_LCD_LINE_1, "Mode:Receiver");
  //halLcdWriteCharString(0,HAL_LCD_LINE_3, "Ready");
 
  /* ��ѭ�� */
  while (TRUE) 
  {
    while(!basicRfPacketIsReady());  // �ȴ��µ����ݰ�
    if(basicRfReceive((uint8*)&rxPacket, MAX_PAYLOAD_LENGTH, &rssi)>0) 
    {
      halLedSet(1);  // ����LED1
      //halLedSet(2);  // ����LED2
			
      UINT32_NTOH(rxPacket.seqNumber);  // �ı������ŵ��ֽ�˳��
      segNumber = rxPacket.seqNumber;
            
      /* ����ͳ�Ʊ���λ�����������յ������ݰ����Ϊ�Ѿ��յ������ݰ���� */     
      if(resetStats)
      {
        rxStats.expectedSeqNum = segNumber;
        
        resetStats=FALSE;
      }
        
      rxStats.rssiSum -= perRssiBuf[perRssiBufCounter];  // ��sum�м�ȥ�ɵ�RSSIֵ
      perRssiBuf[perRssiBufCounter] =  rssi;  // �洢�µ�RSSIֵ�����λ�������֮������������sum

      rxStats.rssiSum += perRssiBuf[perRssiBufCounter];  // �����µ�RSSIֵ��sum
       MyDate[4] = rssi;           ////
       MyDate[3] = rxStats.rssiSum;////
      if(++perRssiBufCounter == RSSI_AVG_WINDOW_SIZE) 
      {
        perRssiBufCounter = 0;      
      }

      /* �����յ������ݰ��Ƿ����������յ������ݰ� */
      if(rxStats.expectedSeqNum == segNumber)  // ���������յ������ݰ� 
      {
        MyDate[0] = rxStats.expectedSeqNum;////
        rxStats.expectedSeqNum++;
      
      }

      else if(rxStats.expectedSeqNum < segNumber)  // �����������յ������ݰ����յ������ݰ�����Ŵ��������յ������ݰ�����ţ�
      {                                            // ��Ϊ����
        rxStats.lostPkts += segNumber - rxStats.expectedSeqNum;
        MyDate[2] = rxStats.lostPkts;///
        rxStats.expectedSeqNum = segNumber + 1;
        MyDate[0] = rxStats.expectedSeqNum;///
      }
      else  // �����������յ������ݰ����յ������ݰ������С�������յ������ݰ�����ţ�
      {     // ��Ϊ��һ���µĲ��Կ�ʼ����λͳ�Ʊ���
        rxStats.expectedSeqNum = segNumber + 1;
        MyDate[0] = rxStats.expectedSeqNum;///
        rxStats.rcvdPkts = 0;
        rxStats.lostPkts = 0;
      }
      MyDate[1] = rxStats.rcvdPkts;///
      rxStats.rcvdPkts++;
      
      UartTX_Send_String(MyDate,5);
      halMcuWaitMs(300);
      halLedClear(1);   //Ϩ��LED1
      halLedClear(2);   //Ϩ��LED2
      halMcuWaitMs(300);
    }
  }
}


/*********************************************************************
 * �������ƣ�appTransmitter
 * ��    �ܣ���������Ӧ�ô��롣��������������ѭ����
 * ��ڲ�����basicRfConfig  Basic RF ��������
 *           txPacket       perTestPacket_t���ͱ���
 *           appState       ����Ӧ��״̬
 *           appStarted     ���ƴ����������ֹͣ
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
static void appTransmitter()
{
  uint32 burstSize=0;
  uint32 pktsSent=0;
  uint8 appTxPower;
  uint8 n;

  /* ��ʼ��Basic RF */
  basicRfConfig.myAddr = TX_ADDR;
  if(basicRfInit(&basicRfConfig)==FAILED) 
  {
    HAL_ASSERT(FALSE);
  }

  /* ����������� */
  //appTxPower = appSelectOutputPower();
  halRfSetTxPower(2);//HAL_RF_TXPOWER_4_DBM
//  halRfSetTxPower(appTxPower);

  /* ���ý���һ�β��������͵����ݰ����� */
  //burstSize = appSelectBurstSize();
  burstSize = 100000;
  /* Basic RF�ڷ������ݰ�ǰ�رս��������ڷ�����һ�����ݰ���򿪽����� */
  basicRfReceiveOff();

  /* ���ö�ʱ����IO */
  //n= appSelectRate();
  appConfigTimer(0xC8);
  //halJoystickInit();

  /* ��ʼ�����ݰ��غ� */
  txPacket.seqNumber = 0;
  for(n = 0; n < sizeof(txPacket.padding); n++) 
  {
    txPacket.padding[n] = n;
  }

  /*��ʾ�����Ϣ */
  //halLcdClear();
  //halLcdWriteCharString(0,HAL_LCD_LINE_1, "Mode:Transmitter");
  //halLcdWriteCharString(0,HAL_LCD_LINE_2, "CENTER to start/stop");

  /* ��ѭ�� */
  while (TRUE) 
  {
    //while(!halJoystickPushed());  // �ȴ��û�����Ӧ��
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
          UINT32_HTON(txPacket.seqNumber);  // �ı䷢����ŵ��ֽ�˳��
          basicRfSendPacket(RX_ADDR, (uint8*)&txPacket, PACKET_SIZE);

          /* ���������ǰ���ֽ�˳��Ļ�Ϊ����˳�� */
          UINT32_NTOH(txPacket.seqNumber);
          txPacket.seqNumber++;

          pktsSent++;
          //utilLcdDisplayValue(HAL_LCD_LINE_3, "Sent: ", (int32)pktsSent, NULL);
          appState = IDLE;
          halLedToggle(1);   //�л�LED1������״̬
          halLedToggle(2);   //�л�LED2������״̬
          halMcuWaitMs(1000);
        //}
      }
      //else
        //appStarted = !appStarted;
    //}

    /* ��λͳ�ƺ���� */
    pktsSent = 0;
    //txPacket.seqNumber = 0;
    //halLcdClear();
    //halLedClear(2); //Ϩ��LED3(LED_Y)
    //halLcdWriteCharString(0,HAL_LCD_LINE_1, "Mode:Transmitter");
    //halLcdWriteCharString(0,HAL_LCD_LINE_2, "CENTER to start/stop");
  }
}


/*********************************************************************
 * �������ƣ�main
 * ��    �ܣ������ʲ���ʵ���main�������
 * ��ڲ�����basicRfConfig  Basic RF ��������
 *           appState       ����Ӧ��״̬
 *           appStarted     ���ƴ����������ֹͣ
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void main (void)
{
  uint8 appMode;
  uint8 i;

  appState = IDLE;     // ��ʼ��Ӧ��״̬Ϊ����
  appStarted = FALSE;  // ��ʼ��������־λFALSE
	
  /* ��ʼ��Basic RF */
  basicRfConfig.panId = PAN_ID;      // ��ʼ��������ID
  basicRfConfig.ackRequest = FALSE;  // ����Ҫȷ��

  halBoardInit();  // SK-SmartRF05EB�����������ʼ��

  //utilPrintLogo("PER Tester");  // ��LCD����ʾSplash��Ļ��Logo
  
  /* ��ʼ��hal_rf */
  if(halRfInit()==FAILED) 
    HAL_ASSERT(FALSE);
    
  /* ������˸8��LED_G��LEG_R��LEG_Y */
  for(i = 0; i < 16; i++)
  {
    halLedToggle(1);  // �л�LED_G������״̬
    halLedToggle(2);  // �л�LED_R������״̬
    halLedToggle(3);  // �л�LED_Y������״̬
    halMcuWaitMs(50); // ��ʱ��Լ50ms
  }
    
  halLedSet(1);  // LED1(LED_G)ָʾ������ָʾ�豸���ϵ�����
  halLedClear(2);
  //halLedClear(3);

  //halLcdWriteCharString(0, HAL_LCD_LINE_6, "Press S2 to enter.");
  
  //while (halButtonPushed() != HAL_BUTTON_1); // �ȴ��û�����S2(BUTTON)������˵�
  halMcuWaitMs(350);
  //halLcdClear();

  basicRfConfig.channel = 0x0B;//appSelectChannel();  // �����ŵ�
  //appMode = appSelectMode();  // ѡ������ģʽ
  
#ifdef MODE_SEND
  appMode = MODE_TX;
#else
  appMode = MODE_RX;
#endif  
  
  /* ������ģʽ */
  if(appMode == MODE_TX) 
  {
    appTransmitter();
   
  }
  /* ������ģʽ */
  else if(appMode == MODE_RX) 
  {
    appReceiver();
  }
    
  HAL_ASSERT(FALSE);
}


