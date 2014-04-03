/***********************************************************************************
  Filename:     basic_rf.c

  Description:  Basic RF library

***********************************************************************************/
// 专业尚阳
/***********************************************************************************
* INCLUDES
*/
#include "hal_int.h"
// Using halMcuWaitUs()，hal_mcu.h中是一些延时函数
#include "hal_mcu.h"    

#include "hal_rf.h"
// 通道编程的宏，以及硬件RF初始化，SECURITY_CCM宏是用来控制安全MAC的
#ifdef SECURITY_CCM     
#include "hal_rf_security.h"
#endif

// basic_rf.h中定义了RF配置用的数据结构
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
// MPDU的长度宏，（2字节幀控制域 + 1字节数据序列号 + 2字节PAN ID 
// + 2字节目标地址 + 2字节源地址 + 2字节MAC尾）
#define BASIC_RF_PACKET_OVERHEAD_SIZE       ((2 + 1 + 2 + 2 + 2) + (2))
#define BASIC_RF_MAX_PAYLOAD_SIZE	        (127 - BASIC_RF_PACKET_OVERHEAD_SIZE - \
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
// 接收帧信息
typedef struct {
    uint8 seqNumber;            // 帧序号
    uint16 srcAddr;             // 源地址
    uint16 srcPanId;            // 源节点的PANID
    int8 length;                // 帧长度
    uint8* pPayload;            // 该指针指向帧数据即：净载荷数据
    uint8 ackRequest;           // 帧控制域的应答位信息
    int8 rssi;                  // 接收信号强度指示
    volatile uint8 isReady;     // 通过CRC校验，数据接收完成，该标志位进行后续读取操作
    uint8 status;
} basicRfRxInfo_t;

// Tx state
// 发送状态信息
typedef struct {
    uint8 txSeqNumber;           // 帧序号
    volatile uint8 ackReceived;  // ACK是否接收完成
    uint8 receiveOn;             // 是否处于接收状态
    uint32 frameCounter;         // 发送帧计数
} basicRfTxState_t;


// Basic RF packet header (IEEE 802.15.4)
// BasicRf 帧头(IEEE 802.15.4)
typedef struct {
    uint8   packetLength;         // 帧长度
    uint8   fcf0;                 // Frame control field LSB
    uint8   fcf1;                 // Frame control field MSB
    uint8   seqNumber;            // 帧序号
    uint16  panId;                // PANID，网络ID
    uint16  destAddr;             // 目的地址
    uint16  srcAddr;              // 源地址
    #ifdef SECURITY_CCM           // 安全选项
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
*              根据802.15.4 协议构建帧头；

* @param       buffer - Pointer to buffer to write the header    // MPDU的buffer
*              destAddr - destination short address              // 目的短地址；
*              payloadLength - length of higher layer payload    // 载荷数据长度；
*
* @return      uint8 - length of header  // 帧头长度(包括MFR)，即BASIC_RF_HDR_SIZE；
*/
static uint8 basicRfBuildHeader(uint8* buffer, uint16 destAddr, uint8 payloadLength)
{
    basicRfPktHdr_t *pHdr;           // 帧头指针；
    uint16 fcf;                      // 存储帧控制域相关信息变量；

    pHdr= (basicRfPktHdr_t*)buffer;  // 帧头buffer 指向 MPDU buffer；

    // Populate packet header
    // 将相关信息 存至 帧头buffer；
    // 帧头长度定义为
    // #define BASIC_RF_PACKET_OVERHEAD_SIZE       ((2 + 1 + 2 + 2 + 2) + (2))
    // 其对应关系为（（FCF+SeqNum+PANID+DestAddr+SrcAddr）+（FCS））
    // 然后将帧头信息 赋值给帧头 buffer；
    pHdr->packetLength = payloadLength + BASIC_RF_PACKET_OVERHEAD_SIZE;
    //pHdr->frameControlField = pConfig->ackRequest ? BASIC_RF_FCF_ACK : BASIC_RF_FCF_NOACK;
    fcf= pConfig->ackRequest ? BASIC_RF_FCF_ACK : BASIC_RF_FCF_NOACK;
    pHdr->fcf0 = LO_UINT16(fcf);
    pHdr->fcf1 = HI_UINT16(fcf);
    pHdr->seqNumber= txState.txSeqNumber;
    pHdr->panId= pConfig->panId;
    pHdr->destAddr= destAddr;
    pHdr->srcAddr= pConfig->myAddr;

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
    // 高低位变换；无线电数据传输为先低位后高位，这样做是在为无线传输做准备？？
    UINT16_HTON(pHdr->panId);
    UINT16_HTON(pHdr->destAddr);
    UINT16_HTON(pHdr->srcAddr);

    return BASIC_RF_HDR_SIZE;
}


/***********************************************************************************
* @fn          basicRfBuildMpdu
*
* @brief       Builds mpdu (MAC header + payload) according to IEEE 802.15.4
*              frame format
*              根据802.15.4协议的帧结构 构建 MPDU（MAC帧头+净载荷数据，
               而由于AUTOCRC = 1则FCS不必手动写入TXFIFO，可以忽略但空间长度需要保留）；

* @param       destAddr - Destination short address         // 目的地址
*              pPayload - pointer to buffer with payload    // 净载荷数据buffer
*              payloadLength - length of payload buffer     // 净载荷数据长度
*
* @return      uint8 - length of mpdu          // MPDU的长度= MAC Hdr + MAC Payload；
*/
static uint8 basicRfBuildMpdu(uint16 destAddr, uint8* pPayload, uint8 payloadLength)
{
    uint8 hdrLength, n;
    
    // hdrLength 这里是MHR+MFR的和，也就是 BASIC_RF_HDR_SIZE
    hdrLength = basicRfBuildHeader(txMpdu, destAddr, payloadLength);

    // 将净载荷数据放置于 MHR 后组成MPDU；
    for(n=0;n<payloadLength;n++)
    {
        txMpdu[hdrLength+n] = pPayload[n];
    }
    return hdrLength + payloadLength; // total mpdu length
}


/***********************************************************************************
* @fn          basicRfRxFrmDoneIsr
*
* @brief       Interrupt service routine for received frame from radio
*              (either data or acknowlegdement)
*
* @param       rxi - file scope variable info extracted from the last incoming
*                    frame
*              txState - file scope variable that keeps tx state info
*
* @return      none
*/
static void basicRfRxFrmDoneIsr(void)
{
    basicRfPktHdr_t *pHdr;
    uint8 *pStatusWord;
    #ifdef SECURITY_CCM
    uint8 authStatus=0;
    #endif

    // Map header to packet buffer
    pHdr= (basicRfPktHdr_t*)rxMpdu;

    // Clear interrupt and disable new RX frame done interrupt
    halRfDisableRxInterrupt();

    // Enable all other interrupt sources (enables interrupt nesting)
    halIntOn();

    // Read payload length.
    halRfReadRxBuf(&pHdr->packetLength,1);
    pHdr->packetLength &= BASIC_RF_PLD_LEN_MASK; // Ignore MSB
    
    // Is this an acknowledgment packet?
    // Only ack packets may be 5 bytes in total.
    if (pHdr->packetLength == BASIC_RF_ACK_PACKET_SIZE) {

        // Read the packet
        halRfReadRxBuf(&rxMpdu[1], pHdr->packetLength);

        // Make sure byte fields are changed from network to host byte order
    	UINT16_NTOH(pHdr->panId);
    	UINT16_NTOH(pHdr->destAddr);
    	UINT16_NTOH(pHdr->srcAddr);
        #ifdef SECURITY_CCM
        UINT32_NTOH(pHdr->frameCounter);
        #endif

        rxi.ackRequest = !!(pHdr->fcf0 & BASIC_RF_FCF_ACK_BM_L);

        // Read the status word and check for CRC OK
        pStatusWord= rxMpdu + 4;

        // Indicate the successful ACK reception if CRC and sequence number OK
        if ((pStatusWord[1] & BASIC_RF_CRC_OK_BM) && (pHdr->seqNumber == txState.txSeqNumber)) {
            txState.ackReceived = TRUE;
        }

        // No, it is data
    } else {

        // It is assumed that the radio rejects packets with invalid length.
        // Subtract the number of bytes in the frame overhead to get actual payload.

        rxi.length = pHdr->packetLength - BASIC_RF_PACKET_OVERHEAD_SIZE;

        #ifdef SECURITY_CCM
        rxi.length -= (BASIC_RF_AUX_HDR_LENGTH + BASIC_RF_LEN_MIC);
        authStatus = halRfReadRxBufSecure(&rxMpdu[1], pHdr->packetLength, rxi.length,
                                        BASIC_RF_LEN_AUTH, BASIC_RF_SECURITY_M);
        #else
        halRfReadRxBuf(&rxMpdu[1], pHdr->packetLength);
        #endif

        // Make sure byte fields are changed from network to host byte order
    	UINT16_NTOH(pHdr->panId);
    	UINT16_NTOH(pHdr->destAddr);
    	UINT16_NTOH(pHdr->srcAddr);
        #ifdef SECURITY_CCM
        UINT32_NTOH(pHdr->frameCounter);
        #endif

        rxi.ackRequest = !!(pHdr->fcf0 & BASIC_RF_FCF_ACK_BM_L);

        // Read the source address
        rxi.srcAddr= pHdr->srcAddr;

        // Read the packet payload
        rxi.pPayload = rxMpdu + BASIC_RF_HDR_SIZE;

        // Read the FCS to get the RSSI and CRC
        pStatusWord= rxi.pPayload+rxi.length;
        #ifdef SECURITY_CCM
        pStatusWord+= BASIC_RF_LEN_MIC;
        #endif
        rxi.rssi = pStatusWord[0];

        // Notify the application about the received data packet if the CRC is OK
        // Throw packet if the previous packet had the same sequence number
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
            if ( ((pHdr->fcf0 & (BASIC_RF_FCF_BM_L)) == BASIC_RF_FCF_NOACK_L) ) {
                rxi.isReady = TRUE;
            }              
            #endif
        }
        rxi.seqNumber = pHdr->seqNumber;
    }
  
    // Enable RX frame done interrupt again
    halIntOff();
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
*              // 初始化BasicRF数据结构,如：通道选择、短地址、PANID及接收中断的配置；
*
* @param       pRfConfig - pointer to BASIC_RF_CONFIG struct.
*                          This struct must be allocated by higher layer
*              txState - file scope variable that keeps tx state info        //发送状态信息；
*              rxi - file scope variable info extracted from the last incoming
*                    frame                    //最新的所接收帧信息；
*
* @return      none
*/
/* basicRfInit()如下代码所示，该函数仅对RF做简单初始化、通道选择、PANID、本节点地址进行配置，
*  最后为RF接收中断，声明一个函数指针basicRfRxFrmDoneIsr；
*/
uint8 basicRfInit(basicRfCfg_t* pRfConfig)    // 协议的初始化
{
    // Rf初始化，启用Rf的推荐简单配置，可选的PA模块配置，始终返回Success；
    if (halRfInit()==FAILED)
        return FAILED;

    // 关闭总中断
    halIntOff();

    // Set the protocol(协议) configuration(配置)
    pConfig = pRfConfig;                       // 指向相关配置信息
    rxi.pPayload   = NULL;                     // 清空本节点的接收载荷数据；

    txState.receiveOn = TRUE;                  // halRfInit()中开启接收；
    txState.frameCounter = 0;                  // 发送帧计数值；

    // Set channel
    halRfSetChannel(pConfig->channel);         // 将定义的通道号写入相关寄存器；

    // Write the short address and the PAN ID to the CC2520 RAM
    halRfSetShortAddr(pConfig->myAddr);        // 将定义的本节点地址写入相关寄存器；
                                               // #define SHORT_ADDR0     XREG( 0x6174 )
                                               // #define SHORT_ADDR1     XREG( 0x6175 )

    halRfSetPanId(pConfig->panId);             // 将定义的PANID写入相关寄存器；
                                               // #define PAN_ID0         XREG( 0x6172 )
                                               // #define PAN_ID1         XREG( 0x6173 )

    // if security is enabled, write key and nonce
    #ifdef SECURITY_CCM
    basicRfSecurityInit(pConfig);
    #endif

    // Set up receive interrupt (received data or acknowlegment)
    halRfRxInterruptConfig(basicRfRxFrmDoneIsr); // 对函数指针进行赋值，关联相应的中断函数，
                                                 // 也就是声明中断程序；

    halIntOn();                                  // 开启总中断；

    return SUCCESS;
}


/***********************************************************************************
* @fn          basicRfSendPacket
*
* @brief       Send packet
*
* @param       destAddr - destination short address   
               // 目的地址

*              pPayload - pointer to payload buffer. This buffer must be
*                         allocated by higher layer.  
               // 需要MAC层以上产生要发送的数据(指针或数组)

*              length - length of payload   
               // 要发送数据的长度

*              txState - file scope variable that keeps tx state info   
               // 发送状态信息

*              mpdu - file scope variable. Buffer for the frame to send
               // 对数据进行封包为物理层协议数据单元；
*
* @return      basicRFStatus_t - SUCCESS or FAILED
*/
uint8 basicRfSendPacket(uint16 destAddr, uint8* pPayload, uint8 length)
{
    uint8 mpduLength;
    uint8 status;

    // Turn on receiver if its not on
    // 保证设备处于接收状态，其初始值在halRfInit()中开启接收并在 basicRfInit()中被赋值为TRUE；
    if(!txState.receiveOn) {
        halRfReceiveOn();
    }

    // Check packet length
    // 取最小的有效数据长度，类似与可变长度域，可变长度值是很有用的.例如：串口透传的数据长度；
    // 最大数据载荷为 #define BASIC_RF_MAX_PAYLOAD_SIZE (127 - BASIC_RF_PACKET_OVERHEAD_SIZE 
    // - BASIC_RF_AUX_HDR_LENGTH - BASIC_RF_LEN_MIC)
    // 后面两项为安全选项的附加信息，可根据需要自行调整；
    length = min(length, BASIC_RF_MAX_PAYLOAD_SIZE);

    // Wait until the transceiver is idle
    // 根据SFD和TX_Active 状态位来判定设备是否处于空闲状态；
    // SFD状态位为0说明设备目前无发送无接收；
    halRfWaitTransceiverReady();

    // Turn off RX frame done interrupt to avoid interference on the SPI interface
    // 防止2591冲突？？？
    halRfDisableRxInterrupt();

    // 根据目的地址、载荷数据及长度信息进行封包；
    mpduLength = basicRfBuildMpdu(destAddr, pPayload, length);

    #ifdef SECURITY_CCM
    halRfWriteTxBufSecure(txMpdu, mpduLength, length, BASIC_RF_LEN_AUTH, BASIC_RF_SECURITY_M);
    txState.frameCounter++;     // Increment frame counter field
    #else
    // 使用ISFLUSHTX()清空TXFIFO 并清除IRQ_TXDONE中断溢出标志位，将MPDU一个字节一个字节的写入RFD；
    halRfWriteTxBuf(txMpdu, mpduLength);
    #endif

    // Turn on RX frame done interrupt for ACK reception
    // 仅仅是始能接收中断，为发送完成后自动进入接收模式 接收 ACK做准备性工作；
    // 仅作为发送节点且不启用ACK的话，这部分语句都可以省略去；
    halRfEnableRxInterrupt();

    // Send frame with CCA. return FAILED if not successful
    if(halRfTransmit() != SUCCESS) {
        status = FAILED;
    }

    // Wait for the acknowledge to be received, if any
    // 如果启用ACK，则在发送完成后进行等待580μs
    /* 
    实际测试中发送7个字节的数据，两个节点先后发送A和B两个数据帧，
    两个数据帧的间隔大概需要不小于440μs+580μs+330μs 
    (粗略估计^_^)的时间间隔Sniffer才能捕捉到A的应答帧（这些多出的时间
    由节点程序准备和结束时间？）可以确定的是接受节点接收先后两个数据帧的时间间隔要大于580μs，
    间间隔太短不能正确接收后一个数据帧，可以通过启用CCA解决这个冲突；
    */
    if (pConfig->ackRequest) {
        txState.ackReceived = FALSE;

        // We'll enter RX automatically, so just wait until we can be sure that the ack reception should have finished
        // The timeout consists of a 12-symbol turnaround time, the ack packet duration, and a small margin
        halMcuWaitUs((12 * BASIC_RF_SYMBOL_DURATION) + (BASIC_RF_ACK_DURATION) + (2 * BASIC_RF_SYMBOL_DURATION) + 10);

        // If an acknowledgment has been received (by RxFrmDoneIsr), the ackReceived flag should be set
        status = txState.ackReceived ? SUCCESS : FAILED;

    } else {
        status = SUCCESS;
    }

    // Turn off the receiver if it should not continue to be enabled
    // 如果不需要继续接收则关闭接收
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
*
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
    halIntOff();
    memcpy(pRxData, rxi.pPayload, min(rxi.length, len));
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
    halRfReceiveOn();
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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

