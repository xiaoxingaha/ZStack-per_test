/***********************************************************************************
  Filename:     basic_rf.c

  Description:  Basic RF library

***********************************************************************************/
// רҵ����
/***********************************************************************************
* INCLUDES
*/
#include "hal_int.h"
// Using halMcuWaitUs()��hal_mcu.h����һЩ��ʱ����
#include "hal_mcu.h"    

#include "hal_rf.h"
// ͨ����̵ĺ꣬�Լ�Ӳ��RF��ʼ����SECURITY_CCM�����������ư�ȫMAC��
#ifdef SECURITY_CCM     
#include "hal_rf_security.h"
#endif

// basic_rf.h�ж�����RF�����õ����ݽṹ
#include "basic_rf.h"
#ifdef SECURITY_CCM
#include "basic_rf_security.h"
#endif

#include "util.h"       // Using min()
#include "string.h"

/***********************************************************************************
* CONSTANTS AND DEFINES
*/

// Packet and packet part lengths
#define PKT_LEN_MIC                         8
#define PKT_LEN_SEC                         PKT_LEN_UNSEC + PKT_LEN_MIC
#define PKT_LEN_AUTH                        8
#define PKT_LEN_ENCR                        24

// Packet overhead ((frame control field, sequence number, PAN ID,
// destination and source) + (footer))
// Note that the length byte itself is not included included in the packet length
// MPDU�ĳ��Ⱥ꣬��2�ֽڎ������� + 1�ֽ��������к� + 2�ֽ�PAN ID 
// + 2�ֽ�Ŀ���ַ + 2�ֽ�Դ��ַ + 2�ֽ�MACβ��
#define BASIC_RF_PACKET_OVERHEAD_SIZE       ((2 + 1 + 2 + 2 + 2) + (2))

// MPDU�����Ч�غɵĳ��ȣ�����FIFOֻ��128�ֽڿռ䣩��
// BASIC_RF_AUX_HDR_LENGTH �� BASIC_RF_LEN_MIC
// �Ǹ�����ȫͷ�궨��ĳ��ȷֱ���5��8�����Խ�������Ϊ0�������߱���ȫ����
// BASIC_RF_PACKET_OVERHEAD_SIZE ����һ���궨�壬MPDU�г����غ�������ֽ���
#define BASIC_RF_MAX_PAYLOAD_SIZE	     (127 - BASIC_RF_PACKET_OVERHEAD_SIZE - \
    BASIC_RF_AUX_HDR_LENGTH - BASIC_RF_LEN_MIC)
#define BASIC_RF_ACK_PACKET_SIZE	        5
#define BASIC_RF_FOOTER_SIZE                2
#define BASIC_RF_HDR_SIZE                   10

// The time it takes for the acknowledgment packet to be received after the
// data packet has been transmitted.
#define BASIC_RF_ACK_DURATION		        (0.5 * 32 * 2 * ((4 + 1) + (1) + (2 + 1) + (2)))
#define BASIC_RF_SYMBOL_DURATION	        (32 * 0.5)

// The length byte
#define BASIC_RF_PLD_LEN_MASK               0x7F

// Frame control field
#define BASIC_RF_FCF_NOACK                  0x8841
#define BASIC_RF_FCF_ACK                    0x8861
#define BASIC_RF_FCF_ACK_BM                 0x0020
#define BASIC_RF_FCF_BM                     (~BASIC_RF_FCF_ACK_BM)
#define BASIC_RF_SEC_ENABLED_FCF_BM         0x0008

// Frame control field LSB
#define BASIC_RF_FCF_NOACK_L                LO_UINT16(BASIC_RF_FCF_NOACK)
#define BASIC_RF_FCF_ACK_L                  LO_UINT16(BASIC_RF_FCF_ACK)
#define BASIC_RF_FCF_ACK_BM_L               LO_UINT16(BASIC_RF_FCF_ACK_BM)
#define BASIC_RF_FCF_BM_L                   LO_UINT16(BASIC_RF_FCF_BM)
#define BASIC_RF_SEC_ENABLED_FCF_BM_L       LO_UINT16(BASIC_RF_SEC_ENABLED_FCF_BM)

// Auxiliary Security header
#define BASIC_RF_AUX_HDR_LENGTH             5
#define BASIC_RF_LEN_AUTH                   BASIC_RF_PACKET_OVERHEAD_SIZE + \
    BASIC_RF_AUX_HDR_LENGTH - BASIC_RF_FOOTER_SIZE
#define BASIC_RF_SECURITY_M                 2
#define BASIC_RF_LEN_MIC                    8
#ifdef SECURITY_CCM
#undef BASIC_RF_HDR_SIZE
#define BASIC_RF_HDR_SIZE                   15
#endif

// Footer
#define BASIC_RF_CRC_OK_BM                  0x80

/***********************************************************************************
* TYPEDEFS
*/
// The receive struct
// ����֡��Ϣ
typedef struct {
    uint8 seqNumber;            // ֡���
    uint16 srcAddr;             // Դ��ַ
    uint16 srcPanId;            // Դ�ڵ��PANID
    int8 length;                // ֡����
    uint8* pPayload;            // ��ָ��ָ��֡���ݼ������غ�����
    uint8 ackRequest;           // ֡�������Ӧ��λ��Ϣ
    int8 rssi;                  // �����ź�ǿ��ָʾ
    volatile uint8 isReady;     // ͨ��CRCУ�飬���ݽ�����ɣ��ñ�־λ���к�����ȡ����
    uint8 status;
} basicRfRxInfo_t;

// Tx state
// ����״̬��Ϣ
typedef struct {
    uint8 txSeqNumber;           // ֡���
    volatile uint8 ackReceived;  // ACK�Ƿ�������
    uint8 receiveOn;             // �Ƿ��ڽ���״̬
    uint32 frameCounter;         // ����֡����
} basicRfTxState_t;


// Basic RF packet header (IEEE 802.15.4)
// BasicRf ����֡ͷ(IEEE 802.15.4)
typedef struct {
    uint8   packetLength;         // ֡����
    uint8   fcf0;                 // Frame control field LSB,�Ͱ�λ
    uint8   fcf1;                 // Frame control field MSB,�߰�λ
    uint8   seqNumber;            // ֡���
    uint16  panId;                // PANID������ID
    uint16  destAddr;             // Ŀ�ĵ�ַ
    uint16  srcAddr;              // Դ��ַ
    #ifdef SECURITY_CCM           // ��ȫѡ��
    uint8   securityControl;
    uint8  frameCounter[4];
    #endif
} basicRfPktHdr_t;


/***********************************************************************************
* LOCAL VARIABLES
*/
static basicRfRxInfo_t  rxi=      { 0xFF }; // Make sure sequence numbers are
static basicRfTxState_t txState=  { 0x00 }; // initialised and distinct.

static basicRfCfg_t* pConfig;
static uint8 txMpdu[BASIC_RF_MAX_PAYLOAD_SIZE+BASIC_RF_PACKET_OVERHEAD_SIZE+1];
static uint8 rxMpdu[128];

/***********************************************************************************
* GLOBAL VARIABLES
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/


/***********************************************************************************
* @fn          basicRfBuildHeader
*
* @brief       Builds packet header according to IEEE 802.15.4 frame format
*              ����802.15.4 Э�鹹������֡ͷ��

* @param       buffer - Pointer to buffer to write the header    // MPDU��buffer
*              destAddr - destination short address              // Ŀ�Ķ̵�ַ��
*              payloadLength - length of higher layer payload    // �غ����ݳ��ȣ�
*
* @return      uint8 - length of header  // ����֡ͷ����(֡��MFR)����BASIC_RF_HDR_SIZE��
*/
static uint8 basicRfBuildHeader(uint8* buffer, uint16 destAddr, uint8 payloadLength)
{
    // ����������Ͱ�MPDU�����ֶεĿռ䶼�������
    basicRfPktHdr_t *pHdr;           // ����һ��ָ��MAC֡ͷ�ṹ��ָ�룻
    uint16 fcf;                      // frame control field���洢֡�����������Ϣ������

    // ��ΪpHdr Ϊ�ֲ�������������ҪӰ�쵽ȫ�ֵı��� buffer��
    // ���Ծͽ��Լ�����ľֲ�֡ͷָ��ָ�� ȫ�ֵ�MPDU buffer 
    pHdr= (basicRfPktHdr_t*)buffer;  // ֡ͷbuffer ָ�� MPDU buffer��

/**************Populate packet header,�������Ϣ����֡ͷbuffer*******/
    // ֡ͷ���ȶ���Ϊ:
    // #define BASIC_RF_PACKET_OVERHEAD_SIZE       ((2 + 1 + 2 + 2 + 2) + (2))
    // ���Ӧ��ϵΪ����FCF+SeqNum+PANID+DestAddr+SrcAddr��+��FCS����
    // Ȼ��֡ͷ��Ϣ ��ֵ��֡ͷ buffer��
    // pHdr->packetLength ΪMPDU������ length
    // payloadLength Ϊ��Ч�غɵĳ��ȣ���Ҫ�������ݵĳ���
    pHdr->packetLength = payloadLength + BASIC_RF_PACKET_OVERHEAD_SIZE;
    // pHdr->frameControlField = pConfig->ackRequest ? BASIC_RF_FCF_ACK : BASIC_RF_FCF_NOACK;
    // pConfig->ackRequest �� per_test�� mani()�������Ѿ���ֵ�ˡ�
    fcf= pConfig->ackRequest ? BASIC_RF_FCF_ACK : BASIC_RF_FCF_NOACK;
    // �ֱ�õ�16λ��Frame Control Field�ĵͰ�λ�͸߰�λ
    // #define HI_UINT16(a) (((uint16)(a) >> 8) & 0xFF)
    // #define LO_UINT16(a) ((uint16)(a) & 0xFF)
    pHdr->fcf0 = LO_UINT16(fcf);
    pHdr->fcf1 = HI_UINT16(fcf);
    // �õ���������
    pHdr->seqNumber= txState.txSeqNumber;
    pHdr->panId= pConfig->panId;
    pHdr->destAddr= destAddr;
    pHdr->srcAddr= pConfig->myAddr;

/*********************************************************************/
    #ifdef SECURITY_CCM

    // Add security to FCF, length and security header
    pHdr->fcf0 |= BASIC_RF_SEC_ENABLED_FCF_BM_L;
    pHdr->packetLength += PKT_LEN_MIC;
    pHdr->packetLength += BASIC_RF_AUX_HDR_LENGTH;

    pHdr->securityControl= SECURITY_CONTROL;
    pHdr->frameCounter[0]=   LO_UINT16(LO_UINT32(txState.frameCounter));
    pHdr->frameCounter[1]=   HI_UINT16(LO_UINT32(txState.frameCounter));
    pHdr->frameCounter[2]=   LO_UINT16(HI_UINT32(txState.frameCounter));
    pHdr->frameCounter[3]=   HI_UINT16(HI_UINT32(txState.frameCounter));

    #endif

    // Make sure bytefields are network byte order
    // �ߵ�λ�任�����ߵ����ݴ���Ϊ�ȵ�λ���λ������������Ϊ���ߴ�����׼����
    // �����Ǵ��С�˵�����
    UINT16_HTON(pHdr->panId);
    UINT16_HTON(pHdr->destAddr);
    UINT16_HTON(pHdr->srcAddr);

    // ������PHR+MHR�ĺͣ����ظ�hdrLength��
    // 1�ֽ�PHR��2�ֽ�FCF��1�ֽ�sequence��2�ֽ�PanID��2�ֽ�destAddr��2�ֽ�srcAddr
    return BASIC_RF_HDR_SIZE;
}


/***********************************************************************************
* @fn          basicRfBuildMpdu
*
* @brief       Builds mpdu (MAC header + payload) according to IEEE 802.15.4
*              frame format
*              ����802.15.4Э���֡�ṹ ���� MPDU��MAC֡ͷ+���غ����ݣ�
               ������AUTOCRC = 1��FCS�����ֶ�д��TXFIFO�����Ժ��Ե��ռ䳤����Ҫ��������

* @param       destAddr - Destination short address         // Ŀ�ĵ�ַ
*              pPayload - pointer to buffer with payload    // ���غ�����buffer
*              payloadLength - length of payload buffer     // ���غ����ݳ���
*
* @return      uint8 - length of mpdu          // MPDU�ĳ���= MAC Hdr + MAC Payload��
*/
static uint8 basicRfBuildMpdu(uint16 destAddr, uint8* pPayload, uint8 payloadLength)
{
    uint8 hdrLength, n;
    
    // hdrLength ������PHR+MHR�ĺͣ�Ҳ���� BASIC_RF_HDR_SIZE
    // ����MAC��� header
    hdrLength = basicRfBuildHeader(txMpdu, destAddr, payloadLength);

    // ����Ч�غ����ݷ����� MHR�����MPDU��Ԫ��
    for(n=0;n<payloadLength;n++)
    {
        txMpdu[hdrLength+n] = pPayload[n];
    }
    // total mpdu length
    // ����ע�����㣺1.ΪʲôҪ����PHR��2.Ϊʲô������FCS
    // 1.��Ϊ�ڽ��շ��Ǳߣ���������Ҫ֪������ packet�� length,�������Ϣ�ͷ��� PHR��
    // 2.FCS�������ֽڼ������к�������Ӳ���Զ�����
    return hdrLength + payloadLength; 
}


/***********************************************************************************
* @fn          basicRfRxFrmDoneIsr
*
* @brief       Interrupt service routine for received frame from radio
*              (either data or acknowlegdement)
*              // �������ź��ɽ���֡ʱִ���жϷ����ӳ���
*
* @param       rxi - file scope variable info extracted from the last incoming
*                    frame
               // ���һ�ν���֡��Ϣ
*              txState - file scope variable that keeps tx state info
               // ����״̬��Ϣ�����жϷ����ӳ�������Ҫ�ڽ��� ACK���漰
*
* @return      none
*/
// �����жϷ������
static void basicRfRxFrmDoneIsr(void)
{
    basicRfPktHdr_t *pHdr;      // ֡ͷ�ṹ��ָ��
    uint8 *pStatusWord;         // �洢��RSSI��CRC��Ϣ
    #ifdef SECURITY_CCM
    uint8 authStatus=0;
    #endif

    // Map header to packet buffer
    // ֡ͷָ��ָ�� �����յ�MPDU���ݵ�Ԫ��ʹ pHdr�ĵ�ַ�� rxMpdu��ַʵ����һ��
    // �ṹ����ǿ������ת������ʼ��ַ��ָ��
    pHdr= (basicRfPktHdr_t*)rxMpdu;

    // IM_FIFOP�жϽ�ֹ����ֹRF���ж�
    // �ر�ͨ��RF�ж� �� RXPKTDONE�жϣ�
    // Clear interrupt and disable new RX frame done interrupt
    halRfDisableRxInterrupt();

    // ʹ�����ж�  
    // Enable all other interrupt sources (enables interrupt nesting)
    halIntOn();

    // ��ȡ MPDU�ĳ���ֵ
    // Read payload length. 
    // �����Ǵ��͹��������ݰ��еĵ�һ�����ݣ�Ҳ���� MPDUǰ�渽�ӵ�һ���ֽڵ� Frame Length
    // Frame Length�е���Ϣ���� MPDU�� length.Ϊ�˺���׼ȷ��ȡ���ݰ�����Ч���ȵ����ݶ�����
    // �����һ�������ǵ�ַ &pHdr->packetLength�����Ժ����ڵ� pDataֵ�Ϳ���д�뵽�õ�ַ�� \
       Ҳ���� rxMpdu������
    halRfReadRxBuf(&pHdr->packetLength,1);
    // ��󳤶�Ϊ127�ֽڣ��ʺ��������Чλ���õ�������ʵֵ
    pHdr->packetLength &= BASIC_RF_PLD_LEN_MASK; // Ignore MSB
    
/*********************Acknowledge Frame Format********************************************
ע��ACKһ����6���ֽڣ���1���ֽڴ洢����Frame Length��Ҳ���Ǻ��汣�浽 packetLength�е�ֵ
    �������5���ֽھ���ACK�����岿�֣�����mpdu�������д洢��Ҳ��6���ֽڣ�����ǰ���Length
    
*/
    // ���ֻ��5���ֽڳ��ȣ����֡ΪӦ��֡
    // ��Ϊֻ��ACK֡�����ݳ���Ϊ5�����Կ���ͨ�� ֡�������жϸ�֡�����ͣ�����֡ or Ӧ��֡
    // Is this an acknowledgment packet?
    // Only ack packets may be 5 bytes in total.
    if (pHdr->packetLength == BASIC_RF_ACK_PACKET_SIZE) {

        // Read the packet
        // ���������� ͨ��RFD�ӻ�����һ���ֽ�һ���ֽڶ�����д�뵽 rxMpdu������
        // ע�⣺����RFD������д��ʱ�� rxMpdu[1]��ʼ����ΪrxMpdu[0]�����ǰ���ȡ�� Length
        halRfReadRxBuf(&rxMpdu[1], pHdr->packetLength);

        // ����������ĸ�4λ�͵�4λ�������� ���������ʽ ת��Ϊ ���ظ�ʽ
        // Make sure byte fields are changed from network to host byte order
    	UINT16_NTOH(pHdr->panId);
    	UINT16_NTOH(pHdr->destAddr);
    	UINT16_NTOH(pHdr->srcAddr);
        #ifdef SECURITY_CCM
        UINT32_NTOH(pHdr->frameCounter);
        #endif
/****************Format of the Frame Control Field (FCF)********************
        Bits:               5 
        description:        Acknowledge request
      
*/       
        // ��ȡӦ��֡֡��Ӧ��λ��Ϣ������Ӧ��֡��FCF��λӦ��Ϊ0��
        // BASIC_RF_FCF_ACK_BM_L ��Ӧ�Ĵ�����Ϊ0x0020 \
           fcf0ָ����FCF�ĵͰ�λ����� &�������ֵ��Ϊ��ȡ bit5��Ӧ��ֵ
        // ���ﲻ�������"!!"����
        rxi.ackRequest = !!(pHdr->fcf0 & BASIC_RF_FCF_ACK_BM_L);

        // ָ��ָ���֡��FCS Ҳ����CRCУ��
        // ��Ϊһ����6���ֽڣ�FCSռ������rxMpduΪͷָ��
        // Read the status word and check for CRC OK
        pStatusWord= rxMpdu + 4;

        // ��CRCУ��λ��֡���������txState.ackReceived ����Ϊ TRUE��\
           ��ʾ���ݷ��ͼ��Է��ڵ������ɣ�
        // ����FCS��ôУ�黹û��ϸ��
        // Indicate the successful ACK reception if CRC and sequence number OK
        if ((pStatusWord[1] & BASIC_RF_CRC_OK_BM) && (pHdr->seqNumber == txState.txSeqNumber)) {
            txState.ackReceived = TRUE;
        }

        // No, it is data
    } else {

        // It is assumed that the radio rejects packets with invalid length.
        // Subtract the number of bytes in the frame overhead to get actual payload.
        // Ĭ�Ϸ�Ӧ��֡���ȵ����ݽ�Ϊ��Ч����
        // �������ֵ��ȥ��MHR+MFR�������Ч���س��ȣ�
        rxi.length = pHdr->packetLength - BASIC_RF_PACKET_OVERHEAD_SIZE;

        #ifdef SECURITY_CCM
        rxi.length -= (BASIC_RF_AUX_HDR_LENGTH + BASIC_RF_LEN_MIC);
        authStatus = halRfReadRxBufSecure(&rxMpdu[1], pHdr->packetLength, rxi.length,
                                        BASIC_RF_LEN_AUTH, BASIC_RF_SECURITY_M);
        #else
        // ���������� ͨ��RFD�ӻ�����һ���ֽ�һ���ֽڶ�����
        // ���ͣ�ͨ��RFD����TXFIFO buffer�����գ�ͨ��RFD��RXFIFO buffer��ȡ
        halRfReadRxBuf(&rxMpdu[1], pHdr->packetLength);
        #endif

        // ����������ĸ�4λ�͵�4λ���� ���� ���������ʽ ת��Ϊ ���ظ�ʽ��
        // Make sure byte fields are changed from network to host byte order
    	UINT16_NTOH(pHdr->panId);
    	UINT16_NTOH(pHdr->destAddr);
    	UINT16_NTOH(pHdr->srcAddr);
        #ifdef SECURITY_CCM
        UINT32_NTOH(pHdr->frameCounter);
        #endif

        // ��ȡ��֡��Ӧ��λ��Ϣ��������Ͳ���ACK�����
        rxi.ackRequest = !!(pHdr->fcf0 & BASIC_RF_FCF_ACK_BM_L);

        // ��ȡԴ��ַ��
        // Read the source address
        rxi.srcAddr= pHdr->srcAddr;

        // ��ȡ���غ����ݣ�ָ��ָ�� Mpdu��Ӧ�� payload����
        // Read the packet payload
        rxi.pPayload = rxMpdu + BASIC_RF_HDR_SIZE;

        // ��ȡ֡У���е�RSSI��CRC�����Ϣ��
        // ָ��ֱ��ָ�� FCSͷ��
        // Read the FCS to get the RSSI and CRC
        pStatusWord= rxi.pPayload+rxi.length;
        #ifdef SECURITY_CCM
        pStatusWord+= BASIC_RF_LEN_MIC;
        #endif
        // ��ȡRSSI��ֵ��
        rxi.rssi = pStatusWord[0];

/******************************** FCS ************************************************
FCS1: 
        RSSI(The RSSI value is measured over the first eight symbols following the SFD.)
        RSSI��ֵ��SFD��ǰ��λ����(�������)
FCS2: 
        bit 7: CRC OK -- indicates whether the FCS is correct (1) or incorrect (0).
*/        
        // Notify the application about the received data packet if the CRC is OK
        // Throw packet if the previous packet had the same sequence number
        // ����CRCУ��λ���ж��Ƿ� Ϊ��Ч����֡���� &BASIC_RF_CRC_OK_BM �Ĳ���
        // ���У���Ҫ���֡��Ž�����֤�����������֡ Ҫ����Ӧ�������� \
           ������֡���ͽ��������ϲ�������֡�������������ͬ����ô�����ظ�����֡���к����Ĵ�����Ҫ��������
        // ���� rxi.seqNumber��ָ��һ֡������֡��ţ�pHdr->seqNumber��ʾ��һ֡������֡���
          // ����������ͽڵ�ȷ�����ݱ���ȷ���գ����Բ��������������Σ�\
          ������ Ӧ���Ƿ�ɹ� ���жϳ����Ƿ���ж����ظ����ͣ�
        if( (pStatusWord[1] & BASIC_RF_CRC_OK_BM) && (rxi.seqNumber != pHdr->seqNumber) ) {
            // If security is used check also that authentication passed
            #ifdef SECURITY_CCM
            if( authStatus==SUCCESS ) {
                if ( (pHdr->fcf0 & BASIC_RF_FCF_BM_L) ==
                    (BASIC_RF_FCF_NOACK_L | BASIC_RF_SEC_ENABLED_FCF_BM_L)) {
                        rxi.isReady = TRUE;
                }
            }
            #else
            // ������յ���FCF������ACKλ��������˵����ȷ���գ������սڵ�״̬ ����ΪTRUE��
            // ����BASIC_RF_FCF_NOACK_L = 0x61��ʾPANIDһ�¶��Ҹ�֡����Ϊ ����֡��
            // ����֡��ͨ����֤���ж�������1.CRC = OK ��2.SeqNumber����һ�εĲ�ͬ��3.����PANID �� ֡���� һ�£�
            if ( ((pHdr->fcf0 & (BASIC_RF_FCF_BM_L)) == BASIC_RF_FCF_NOACK_L) ) {
                rxi.isReady = TRUE;
            }              
            #endif
        }
        
        // ��������������ǰ��µ� pHdr->seqNumber֡��Ÿ����ɵ�֡��� rxi.seqNumber
        // ����֡Ҫ����Ӧ��������Ӧ��ɹ� ����һ֡SeqNumber�Լ� ��\
          ��PANID �� ֡����һ�µ������£���һ֡���еĺ��ڲ�����Ȼ��Ӧ�𲻳ɹ� ��\
            ��һ֡SeqNumber���� ��rxi.isReadyʼ��ΪFALSE��һ֡�޷����к��ڴ���\
            ��������Ϊ��Ӧ�ڵ㷢�͵�����ʼ���޷����գ�     
        // ����֡Ҫ����Ӧ����������һ֡SeqNumberʼ���Լӣ�������ɶ�Ӧ�ڵ㷢�͵������޷����յ����⣻
        // SeqNumber�ڵƿ�����������ʼ���� ACK�ĳɹ�����������ģ����Ӧ����������ACK�Ƿ�ɹ���ɵ�������      
        rxi.seqNumber = pHdr->seqNumber;
    }
  
    // Enable RX frame done interrupt again
    // �ر����жϣ����� basicRfReceive()�� ������������� ������ʼ�ܣ�
    halIntOff();
    // ʼ��ͨ��RF�ж� �� RXPKTDONE�жϣ�
    halRfEnableRxInterrupt();
}


/***********************************************************************************
* GLOBAL FUNCTIONS
*/

/***********************************************************************************
* @fn          basicRfInit
*
* @brief       Initialise basic RF datastructures. Sets channel, short address and
*              PAN id in the chip and configures interrupt on packet reception
*              // ��ʼ��BasicRF���ݽṹ,�磺ͨ��ѡ�񡢶̵�ַ��PANID�������жϵ����ã�
*
* @param       pRfConfig - pointer to BASIC_RF_CONFIG struct.
*                          This struct must be allocated by higher layer
*              txState - file scope variable that keeps tx state info        //����״̬��Ϣ��
*              rxi - file scope variable info extracted from the last incoming
*                    frame                    //���µ�������֡��Ϣ��
*
* @return      none
*/
/* basicRfInit()���´�����ʾ���ú�������RF���򵥳�ʼ����ͨ��ѡ��PANID�����ڵ��ַ�������ã�
*  ���ΪRF�����жϣ�����һ������ָ��basicRfRxFrmDoneIsr��
*/
uint8 basicRfInit(basicRfCfg_t* pRfConfig)    // Э��ĳ�ʼ��
{
    // Rf��ʼ��������Rf���Ƽ������ã���ѡ��PAģ�����ã�ʼ�շ���Success��
    if (halRfInit()==FAILED)
        return FAILED;

    // �ر����ж�
    halIntOff();

    // Set the protocol(Э��) configuration(����)
    pConfig = pRfConfig;                       // ָ�����������Ϣ
    rxi.pPayload   = NULL;                     // ��ձ��ڵ�Ľ����غ����ݣ�

    txState.receiveOn = TRUE;                  // halRfInit()�п������գ�
    txState.frameCounter = 0;                  // ����֡����ֵ��

/*******************�����Ƕ���Ƶ��ʼ�����ò�����һ���Ĵ�������**********/
    // Set channel
    // �������ͨ����д����ؼĴ�����
    halRfSetChannel(pConfig->channel);         
    
    // Write the short address and the PAN ID to the CC2520 RAM
    // ������ı��ڵ��ַд����ؼĴ�����
    // #define SHORT_ADDR0     XREG( 0x6174 )
    // #define SHORT_ADDR1     XREG( 0x6175 )
    halRfSetShortAddr(pConfig->myAddr);        

    // �������PANIDд����ؼĴ�����
    // #define PAN_ID0         XREG( 0x6172 )
    // #define PAN_ID1         XREG( 0x6173 )
    halRfSetPanId(pConfig->panId);             
/****************************************************************************/    
    
    // if security is enabled, write key and nonce
    #ifdef SECURITY_CCM
    basicRfSecurityInit(pConfig);
    #endif

    // Set up receive interrupt (received data or acknowlegment)
    // �������õ������߽����жϣ����ݵĲ���Ϊ�жϺ���ָ��
    // ������жϺ���ָ�������Ӧ���жϺ�����Ҳ���������жϳ���
    // basicRfRxFrmDoneIsr ����жϺ����ķ���ֵΪָ�룬����������ָ�뵱������
    halRfRxInterruptConfig(basicRfRxFrmDoneIsr); 

    // �������жϣ�
    halIntOn();

    return SUCCESS;
}


/***********************************************************************************
* @fn          basicRfSendPacket
*
* @brief       Send packet
*
* @param       destAddr - destination short address   
               // Ŀ�ĵ�ַ

*              pPayload - pointer to payload buffer. This buffer must be
*                         allocated by higher layer.  
               // ��ҪMAC�����ϲ���Ҫ���͵�����(ָ�������)

*              length - length of payload   
               // Ҫ�������ݵĳ���

*              txState - file scope variable that keeps tx state info   
               // ����״̬��Ϣ

*              mpdu - file scope variable. Buffer for the frame to send
               // �����ݽ��з�֡Ϊ�����Э�����ݵ�Ԫ��
*
* @return      basicRFStatus_t - SUCCESS or FAILED
*/
uint8 basicRfSendPacket(uint16 destAddr, uint8* pPayload, uint8 length)
{
    uint8 mpduLength;
    uint8 status;

    // Turn on receiver if its not on
    // ��֤�豸���ڽ���״̬�����ʼֵ��halRfInit()�п������ղ��� basicRfInit()�б���ֵΪTRUE��
    if(!txState.receiveOn) {
        halRfReceiveOn();
    }

    // Check packet length
    // ȡ��С����Ч���ݳ��ȣ�������ɱ䳤���򣬿ɱ䳤��ֵ�Ǻ����õ�.���磺����͸�������ݳ��ȣ�
    // ��������غ�Ϊ #define BASIC_RF_MAX_PAYLOAD_SIZE (127 - BASIC_RF_PACKET_OVERHEAD_SIZE 
    // - BASIC_RF_AUX_HDR_LENGTH - BASIC_RF_LEN_MIC)
    // ��������Ϊ��ȫѡ��ĸ�����Ϣ���ɸ�����Ҫ���е�����
    length = min(length, BASIC_RF_MAX_PAYLOAD_SIZE);

    // Wait until the transceiver is idle
    // ����SFD��TX_Active ״̬λ���ж��豸�Ƿ��ڿ���״̬��
    // SFD״̬λΪ0˵���豸Ŀǰ�޷����޽��գ�
    // �ȴ���һ�η��ͽ���
    halRfWaitTransceiverReady();

    // Turn off RX frame done interrupt to avoid interference on the SPI interface
    // ��ֹ2591��ͻ������
    // ��ֹ�����жϣ���Ϊ��ȫ������ͨ��RFD������TXFIFO��һ���͵ģ���ʱ���ʹ�ܽ����ж�
    halRfDisableRxInterrupt();

    // ����Ŀ�ĵ�ַ���غ����ݼ�������Ϣ���з�֡��
    // ����MAC��� MPDU�������� MPDU�ĳ���
    // ����� mpduLength�ǰ������� MPDU�ĳ��ȣ��ټ���ǰ��һ���ֽڵ� Frame Length
    // Ҳ�������������������ʵ����Ч�����ݰ��ĳ���
    mpduLength = basicRfBuildMpdu(destAddr, pPayload, length);

    #ifdef SECURITY_CCM
    halRfWriteTxBufSecure(txMpdu, mpduLength, length, BASIC_RF_LEN_AUTH, BASIC_RF_SECURITY_M);
    txState.frameCounter++;     // Increment frame counter field
    #else
    // ʹ��ISFLUSHTX()���TXFIFO �����IRQ_TXDONE�����ж������־λ��
    // ��MPDU�������һ���ֽ�һ���ֽڵ�ͨ��RFDд��TXFIFO�����ͳ�ȥ
    halRfWriteTxBuf(txMpdu, mpduLength);
    #endif

    // Turn on RX frame done interrupt for ACK reception
    // ������ʼ�ܽ����жϣ�Ϊ������ɺ��Զ��������ģʽ ���� ACK��׼���Թ�����
    // ����Ϊ���ͽڵ��Ҳ�����ACK�Ļ����ⲿ����䶼����ʡ��ȥ��
    halRfEnableRxInterrupt();

    // Send frame with CCA. return FAILED if not successful
    // �жϷ�����û�гɹ�
    if(halRfTransmit() != SUCCESS) {
        status = FAILED;
    }

    // Wait for the acknowledge to be received, if any
    // �������ACK�����ڷ�����ɺ���еȴ�580��s
    /* 
    ʵ�ʲ����з���7���ֽڵ����ݣ������ڵ��Ⱥ���A��B��������֡��
    ��������֡�ļ�������Ҫ��С��440��s+580��s+330��s 
    (���Թ���^_^)��ʱ����Sniffer���ܲ�׽��A��Ӧ��֡����Щ�����ʱ��
    �ɽڵ����׼���ͽ���ʱ�䣿������ȷ�����ǽ��ܽڵ�����Ⱥ���������֡��ʱ����Ҫ����580��s��
    ����̫�̲�����ȷ���պ�һ������֡������ͨ������CCA��������ͻ��
    */
    if (pConfig->ackRequest) {
        txState.ackReceived = FALSE;

        // We'll enter RX automatically, so just wait until we can be sure that the ack reception should have finished
        // The timeout consists of a 12-symbol turnaround time, the ack packet duration, and a small margin
        // �ȴ�Ӧ��֡��ʱ��
        halMcuWaitUs((12 * BASIC_RF_SYMBOL_DURATION) + (BASIC_RF_ACK_DURATION) + (2 * BASIC_RF_SYMBOL_DURATION) + 10);

        // If an acknowledgment has been received (by RxFrmDoneIsr), the ackReceived flag should be set
        status = txState.ackReceived ? SUCCESS : FAILED;

    } else {
        status = SUCCESS;
    }

    // Turn off the receiver if it should not continue to be enabled
    // �������Ҫ����������رս���
    if (!txState.receiveOn) {
        halRfReceiveOff();
    }

    if(status == SUCCESS) {
        txState.txSeqNumber++;
    }

#ifdef SECURITY_CCM
    halRfIncNonceTx();          // Increment nonce value
#endif

    return status;

}


/***********************************************************************************
* @fn          basicRfPacketIsReady
*
* @brief       Check if a new packet is ready to be read by next higher layer
*
* @param       none
*
* @return      uint8 - TRUE if a packet is ready to be read by higher layer
*/
uint8 basicRfPacketIsReady(void)
{
    return rxi.isReady;
}


/**********************************************************************************
* @fn          basicRfReceive
*
* @brief       Copies the payload of the last incoming packet into a buffer
*              // ���յ������ݸ��Ƶ� buffer��
               // ��������Basic RF�������֡����Ϊ�����յ����ݺ�RSSIֵ�仺����

* @param       pRxData - pointer to data buffer to fill. This buffer must be
*                        allocated by higher layer.
*              len - Number of bytes to read in to buffer
*              rxi - file scope variable holding the information of the last
*                    incoming packet
*
* @return      uint8 - number of bytes actually copied into buffer
*/
uint8 basicRfReceive(uint8* pRxData, uint8 len, int16* pRssi)
{
    // Accessing shared variables -> this is a critical region
    // Critical region start
    // �ر��ж�
    halIntOff();
    // memcpy��������װ�����ˣ�û��֪������ʵ�ֹ���
    // ʵ�ֵĹ����ǽ����߷��͹����� payload������ݷŵ��Լ������ RxData��
    memcpy(pRxData, rxi.pPayload, min(rxi.length, len));
    // pRssi �ź�ǿ�ȵļ��
    if(pRssi != NULL) {
        if(rxi.rssi < 128){
            *pRssi = rxi.rssi - halRfGetRssiOffset();
        }
        else{
            *pRssi = (rxi.rssi - 256) - halRfGetRssiOffset();
        }
    }
    rxi.isReady = FALSE;
    halIntOn();

    // Critical region end

    return min(rxi.length, len);
}


/**********************************************************************************
* @fn          basicRfGetRssi
*
* @brief       Copies the payload of the last incoming packet into a buffer
*
* @param       none

* @return      int8 - RSSI value
*/
int8 basicRfGetRssi(void)
{
    if(rxi.rssi < 128){
        return rxi.rssi - halRfGetRssiOffset();
    }
    else{
        return (rxi.rssi - 256) - halRfGetRssiOffset();
    }
}

/***********************************************************************************
* @fn          basicRfReceiveOn
*
* @brief       Turns on receiver on radio
*
* @param       txState - file scope variable
*
* @return      none
*/
void basicRfReceiveOn(void)
{
    txState.receiveOn = TRUE;
    halRfReceiveOn();             // Turn receiver on
}


/***********************************************************************************
* @fn          basicRfReceiveOff
*
* @brief       Turns off receiver on radio
*
* @param       txState - file scope variable
*
* @return      none
*/
void basicRfReceiveOff(void)
{
    txState.receiveOn = FALSE;
    halRfReceiveOff();
}


/***********************************************************************************
  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/

