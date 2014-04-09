/* ����ͷ�ļ� */
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

//#define MODE_SEND               // ����ʱ��������
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
static void appTransmitter();
static void appReceiver();
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

    /* ��ʼ��Basic RF */
    // basicRfConfig�Ķ��壺static basicRfCfg_t basicRfConfig;
    // basicRfCfg_t ������͵Ľṹ�����ɶ�ò����
    basicRfConfig.myAddr = RX_ADDR;                   
    if(basicRfInit(&basicRfConfig)==FAILED)           // �ж�Э��ջ��ʼ���Ƿ�ɹ�
    {
        HAL_ASSERT(FALSE);
    }
    basicRfReceiveOn();   // Turns on receiver on radio

    /* ��ѭ�� */
    while (TRUE) 
    {
        // ����ֵһֱΪ1��ֻ�е���⵽���ݰ�ʱ���Ż���0��������ѭ��
        while(!basicRfPacketIsReady());   // �ȴ��µ����ݰ�
        if(basicRfReceive((uint8*)&rxPacket, MAX_PAYLOAD_LENGTH, &rssi)>0) 
        {
            halLedSet(1);  // ����LED1
             
            // rxPacket�Ķ��壺static perTestPacket_t rxPacket;
            // perTestPacket_t��PER test packet format
            UINT32_NTOH(rxPacket.seqNumber);  // �ı������ŵ��ֽ�˳��
            segNumber = rxPacket.seqNumber;
                  
            /* ���ͳ�Ʊ���λ�����������յ������ݰ����Ϊ�Ѿ��յ������ݰ���� */     
            if(resetStats)
            {
                rxStats.expectedSeqNum = segNumber;   
                resetStats=FALSE;
            }
              
            rxStats.rssiSum -= perRssiBuf[perRssiBufCounter];  // ��sum�м�ȥ�ɵ�RSSIֵ
            perRssiBuf[perRssiBufCounter] =  rssi;  // �洢�µ�RSSIֵ�����λ�������֮������������sum
      
            rxStats.rssiSum += perRssiBuf[perRssiBufCounter];  // �����µ�RSSIֵ��sum
            if(++perRssiBufCounter == RSSI_AVG_WINDOW_SIZE) 
            {
              perRssiBufCounter = 0;      
            }
      
            /* �����յ������ݰ��Ƿ����������յ������ݰ� */
            if(rxStats.expectedSeqNum == segNumber)  // ���������յ������ݰ� 
            {
                rxStats.expectedSeqNum++;
            }  
            else if(rxStats.expectedSeqNum < segNumber)  // �����������յ������ݰ����յ������ݰ�����Ŵ��������յ������ݰ�����ţ�
            {                                            // ��Ϊ����
              rxStats.lostPkts += segNumber - rxStats.expectedSeqNum;
              rxStats.expectedSeqNum = segNumber + 1;
            }
            else  // �����������յ������ݰ����յ������ݰ������С�������յ������ݰ�����ţ�
            {     // ��Ϊ��һ���µĲ��Կ�ʼ����λͳ�Ʊ���
              rxStats.expectedSeqNum = segNumber + 1;
              rxStats.rcvdPkts = 0;
              rxStats.lostPkts = 0;
            }
            rxStats.rcvdPkts++;
            
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
    uint8 n;
  
    /* ��ʼ��Basic RF */
    basicRfConfig.myAddr = TX_ADDR;     // ���ڵ��ַ��ֵ
    
    // ��ʼ�� basicRF���ݽṹ�����������Ĳ������ýṹ��
    if(basicRfInit(&basicRfConfig)==FAILED) 
    {
      HAL_ASSERT(FALSE);
    }
  
    halRfSetTxPower(2);      //HAL_RF_TXPOWER_4_DBM  �����������
  
    burstSize = 100000;       //���ý���һ�β��������͵����ݰ�����
    basicRfReceiveOff();      //Basic RF�ڷ������ݰ�ǰ�رս��������ڷ�����һ�����ݰ���򿪽�����
  
    appConfigTimer(0xC8);     //���ö�ʱ����IO
  
    /* ��ʼ�����ݰ��غ� */
    txPacket.seqNumber = 0;
    for(n = 0; n < sizeof(txPacket.padding); n++) 
    {
      txPacket.padding[n] = n;
    }
    
    /* ��ѭ�� */
    while (TRUE) 
    {
        if (pktsSent < burstSize) 
        {
            UINT32_HTON(txPacket.seqNumber);  // �ı䷢����ŵ��ֽ�˳��
            basicRfSendPacket(RX_ADDR, (uint8*)&txPacket, PACKET_SIZE);
  
            UINT32_NTOH(txPacket.seqNumber);    //���������ǰ���ֽ�˳��Ļ�Ϊ����˳��
            txPacket.seqNumber++;
  
            pktsSent++;
            appState = IDLE;
            halLedToggle(1);   //�л�LED1������״̬
            halLedToggle(2);   //�л�LED2������״̬
            halMcuWaitMs(1000);
        }
  
        pktsSent = 0;   // ��λͳ�ƺ����
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
    uint8 i;
  
    appState = IDLE;           // ��ʼ��Ӧ��״̬Ϊ����
    appStarted = FALSE;        // ��ʼ��������־λFALSE
          
    /* ��ʼ��Basic RF */
    // basicRfConfig �Ķ��壺static basicRfCfg_t basicRfConfig;
    // basicRfCfg_t �ṹ��Ϊ��Ƶ��·��ʼ����صĲ������ýṹ��,�������£�
    /*
    typedef struct {
        uint16 myAddr;        // 16λ�̵�ַ���ڵ�ĵ�ַ���������ھ������е�IP��ַ
        uint16 panId;         // �ڵ��PAN ID�������ID�ţ���������"������������IP��ַ"
        uint8 channel;        // RFͨ����������11-26֮��
        uint8 ackRequest;     // Ŀ��ȷ�Ͼ��� true
        #ifdef SECURITY_CCM   // �Ƿ���ܣ�Ԥ������ȡ���˼���
        uint8* securityKey;
        uint8* securityNonce;
        #endif
    } basicRfCfg_t;
    */
    basicRfConfig.panId = PAN_ID;        // ��ʼ��������ID
    basicRfConfig.ackRequest = FALSE;    // ����Ҫȷ��
  
    halBoardInit();     // ��·��ĳ�ʼ��
    
    if(halRfInit()==FAILED)      //��ʼ��hal_rf
      HAL_ASSERT(FALSE);
      
    /* ������˸8��led1,led2 */
    for(i = 0; i < 16; i++)
    {
      halLedToggle(1);  // �л�led1������״̬
      halLedToggle(2);  // �л�led2������״̬
      halMcuWaitMs(50); // ��ʱ��Լ50ms
    }
      
    halLedSet(1);       // led1ָʾ������ָʾ�豸���ϵ�����
    halLedClear(2);
  
    basicRfConfig.channel = 0x0B;       // �����ŵ�
    
#ifdef MODE_SEND
    appTransmitter();     // ������ģʽ       
#else
    appReceiver();        // ������ģʽ    
#endif  

    HAL_ASSERT(FALSE);
}


