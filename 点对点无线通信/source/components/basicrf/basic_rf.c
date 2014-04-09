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

// MPDU最大有效载荷的长度（利用FIFO只有128字节空间），
// BASIC_RF_AUX_HDR_LENGTH 和 BASIC_RF_LEN_MIC
// 是辅助安全头宏定义的长度分别是5和8，可以将其设置为0，即不具备安全功能
// BASIC_RF_PACKET_OVERHEAD_SIZE 即上一个宏定义，MPDU中除净载荷外的总字节数
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
// BasicRf 数据帧头(IEEE 802.15.4)
typedef struct {
    uint8   packetLength;         // 帧长度
    uint8   fcf0;                 // Frame control field LSB,低八位
    uint8   fcf1;                 // Frame control field MSB,高八位
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
*              根据802.15.4 协议构建数据帧头；

* @param       buffer - Pointer to buffer to write the header    // MPDU的buffer
*              destAddr - destination short address              // 目的短地址；
*              payloadLength - length of higher layer payload    // 载荷数据长度；
*
* @return      uint8 - length of header  // 数据帧头长度(帧括MFR)，即BASIC_RF_HDR_SIZE；
*/
static uint8 basicRfBuildHeader(uint8* buffer, uint16 destAddr, uint8 payloadLength)
{
    // 下面这两句就把MPDU所有字段的空间都分配好了
    basicRfPktHdr_t *pHdr;           // 声明一个指向MAC帧头结构的指针；
    uint16 fcf;                      // frame control field，存储帧控制域相关信息变量；

    // 因为pHdr 为局部变量，但他又要影响到全局的变量 buffer，
    // 所以就将自己定义的局部帧头指针指向 全局的MPDU buffer 
    pHdr= (basicRfPktHdr_t*)buffer;  // 帧头buffer 指向 MPDU buffer；

/**************Populate packet header,将相关信息存至帧头buffer*******/
    // 帧头长度定义为:
    // #define BASIC_RF_PACKET_OVERHEAD_SIZE       ((2 + 1 + 2 + 2 + 2) + (2))
    // 其对应关系为（（FCF+SeqNum+PANID+DestAddr+SrcAddr）+（FCS））
    // 然后将帧头信息 赋值给帧头 buffer；
    // pHdr->packetLength 为MPDU整个的 length
    // payloadLength 为有效载荷的长度，即要发送数据的长度
    pHdr->packetLength = payloadLength + BASIC_RF_PACKET_OVERHEAD_SIZE;
    // pHdr->frameControlField = pConfig->ackRequest ? BASIC_RF_FCF_ACK : BASIC_RF_FCF_NOACK;
    // pConfig->ackRequest 在 per_test的 mani()函数里已经赋值了。
    fcf= pConfig->ackRequest ? BASIC_RF_FCF_ACK : BASIC_RF_FCF_NOACK;
    // 分别得到16位的Frame Control Field的低八位和高八位
    // #define HI_UINT16(a) (((uint16)(a) >> 8) & 0xFF)
    // #define LO_UINT16(a) ((uint16)(a) & 0xFF)
    pHdr->fcf0 = LO_UINT16(fcf);
    pHdr->fcf1 = HI_UINT16(fcf);
    // 得到数据序列
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
    // 高低位变换；无线电数据传输为先低位后高位，这样做是在为无线传输做准备？
    // 可能是大端小端的问题
    UINT16_HTON(pHdr->panId);
    UINT16_HTON(pHdr->destAddr);
    UINT16_HTON(pHdr->srcAddr);

    // 这里是PHR+MHR的和，返回给hdrLength。
    // 1字节PHR，2字节FCF，1字节sequence，2字节PanID，2字节destAddr，2字节srcAddr
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
    
    // hdrLength 这里是PHR+MHR的和，也就是 BASIC_RF_HDR_SIZE
    // 构建MAC层的 header
    hdrLength = basicRfBuildHeader(txMpdu, destAddr, payloadLength);

    // 将有效载荷数据放置于 MHR后组成MPDU单元；
    for(n=0;n<payloadLength;n++)
    {
        txMpdu[hdrLength+n] = pPayload[n];
    }
    // total mpdu length
    // 这里注意两点：1.为什么要算上PHR；2.为什么不算上FCS
    // 1.因为在接收方那边，他首先需要知道整个 packet的 length,而这个信息就放在 PHR中
    // 2.FCS的两个字节检验序列好像是由硬件自动生成
    return hdrLength + payloadLength; 
}


/***********************************************************************************
* @fn          basicRfRxFrmDoneIsr
*
* @brief       Interrupt service routine for received frame from radio
*              (either data or acknowlegdement)
*              // 当无线信号由接收帧时执行中断服务子程序
*
* @param       rxi - file scope variable info extracted from the last incoming
*                    frame
               // 最近一次接收帧信息
*              txState - file scope variable that keeps tx state info
               // 发送状态信息，在中断服务子程序中主要在接收 ACK中涉及
*
* @return      none
*/
// 接收中断服务程序
static void basicRfRxFrmDoneIsr(void)
{
    basicRfPktHdr_t *pHdr;      // 帧头结构体指针
    uint8 *pStatusWord;         // 存储读RSSI和CRC信息
    #ifdef SECURITY_CCM
    uint8 authStatus=0;
    #endif

    // Map header to packet buffer
    // 帧头指针指向 所接收的MPDU数据单元，使 pHdr的地址和 rxMpdu地址实际上一样
    // 结构体间的强制类型转换，起始地址的指向；
    pHdr= (basicRfPktHdr_t*)rxMpdu;

    // IM_FIFOP中断禁止，禁止RF总中断
    // 关闭通用RF中断 和 RXPKTDONE中断；
    // Clear interrupt and disable new RX frame done interrupt
    halRfDisableRxInterrupt();

    // 使能总中断  
    // Enable all other interrupt sources (enables interrupt nesting)
    halIntOn();

    // 读取 MPDU的长度值
    // Read payload length. 
    // 这里是传送过来的数据包中的第一个数据，也就是 MPDU前面附加的一个字节的 Frame Length
    // Frame Length中的信息就是 MPDU的 length.为了后面准确获取数据包中有效长度的数据而服务。
    // 这里第一个参数是地址 &pHdr->packetLength，所以函数内的 pData值就可以写入到该地址中 \
       也就是 rxMpdu数组中
    halRfReadRxBuf(&pHdr->packetLength,1);
    // 最大长度为127字节，故忽略最高有效位，得到长度真实值
    pHdr->packetLength &= BASIC_RF_PLD_LEN_MASK; // Ignore MSB
    
/*********************Acknowledge Frame Format********************************************
注：ACK一共有6个字节，第1个字节存储的是Frame Length，也就是后面保存到 packetLength中的值
    而后面的5个字节就是ACK的主体部分，所以mpdu的数组中存储的也是6个字节，包括前面的Length
    
*/
    // 如果只有5个字节长度，则此帧为应答帧
    // 因为只有ACK帧的数据长度为5，所以可以通过 帧长度来判断该帧的类型：数据帧 or 应答帧
    // Is this an acknowledgment packet?
    // Only ack packets may be 5 bytes in total.
    if (pHdr->packetLength == BASIC_RF_ACK_PACKET_SIZE) {

        // Read the packet
        // 将接收数据 通过RFD从缓存中一个字节一个字节读出，写入到 rxMpdu数组中
        // 注意：这里RFD的数据写入时从 rxMpdu[1]开始，因为rxMpdu[0]存的是前面获取的 Length
        halRfReadRxBuf(&rxMpdu[1], pHdr->packetLength);

        // 将网络参数的高4位和低4位交换，由 无线网络格式 转换为 本地格式
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
        // 读取应答帧帧的应答位信息，对于应答帧的FCF该位应该为0；
        // BASIC_RF_FCF_ACK_BM_L 对应的处理结果为0x0020 \
           fcf0指的是FCF的低八位，因此 &上上面的值即为获取 bit5对应的值
        // 这里不清楚两个"!!"何意
        rxi.ackRequest = !!(pHdr->fcf0 & BASIC_RF_FCF_ACK_BM_L);

        // 指针指向该帧的FCS 也就是CRC校验
        // 因为一共有6个字节，FCS占两个，rxMpdu为头指针
        // Read the status word and check for CRC OK
        pStatusWord= rxMpdu + 4;

        // 若CRC校验位和帧序号无误，则将txState.ackReceived 设置为 TRUE，\
           表示数据发送及对方节点接收完成；
        // 具体FCS怎么校验还没仔细看
        // Indicate the successful ACK reception if CRC and sequence number OK
        if ((pStatusWord[1] & BASIC_RF_CRC_OK_BM) && (pHdr->seqNumber == txState.txSeqNumber)) {
            txState.ackReceived = TRUE;
        }

        // No, it is data
    } else {

        // It is assumed that the radio rejects packets with invalid length.
        // Subtract the number of bytes in the frame overhead to get actual payload.
        // 默认非应答帧长度的数据皆为有效数据
        // 长度域的值减去（MHR+MFR）获得有效负载长度；
        rxi.length = pHdr->packetLength - BASIC_RF_PACKET_OVERHEAD_SIZE;

        #ifdef SECURITY_CCM
        rxi.length -= (BASIC_RF_AUX_HDR_LENGTH + BASIC_RF_LEN_MIC);
        authStatus = halRfReadRxBufSecure(&rxMpdu[1], pHdr->packetLength, rxi.length,
                                        BASIC_RF_LEN_AUTH, BASIC_RF_SECURITY_M);
        #else
        // 将接收数据 通过RFD从缓存中一个字节一个字节读出；
        // 发送：通过RFD传入TXFIFO buffer；接收：通过RFD从RXFIFO buffer获取
        halRfReadRxBuf(&rxMpdu[1], pHdr->packetLength);
        #endif

        // 将网络参数的高4位和低4位交换 ，由 无线网络格式 转换为 本地格式；
        // Make sure byte fields are changed from network to host byte order
    	UINT16_NTOH(pHdr->panId);
    	UINT16_NTOH(pHdr->destAddr);
    	UINT16_NTOH(pHdr->srcAddr);
        #ifdef SECURITY_CCM
        UINT32_NTOH(pHdr->frameCounter);
        #endif

        // 读取该帧的应答位信息，具体解释参照ACK中这句
        rxi.ackRequest = !!(pHdr->fcf0 & BASIC_RF_FCF_ACK_BM_L);

        // 读取源地址；
        // Read the source address
        rxi.srcAddr= pHdr->srcAddr;

        // 读取净载荷数据，指针指到 Mpdu相应的 payload部分
        // Read the packet payload
        rxi.pPayload = rxMpdu + BASIC_RF_HDR_SIZE;

        // 读取帧校验中的RSSI和CRC相关信息；
        // 指针直接指到 FCS头部
        // Read the FCS to get the RSSI and CRC
        pStatusWord= rxi.pPayload+rxi.length;
        #ifdef SECURITY_CCM
        pStatusWord+= BASIC_RF_LEN_MIC;
        #endif
        // 读取RSSI的值；
        rxi.rssi = pStatusWord[0];

/******************************** FCS ************************************************
FCS1: 
        RSSI(The RSSI value is measured over the first eight symbols following the SFD.)
        RSSI的值由SFD的前八位决定(个人理解)
FCS2: 
        bit 7: CRC OK -- indicates whether the FCS is correct (1) or incorrect (0).
*/        
        // Notify the application about the received data packet if the CRC is OK
        // Throw packet if the previous packet had the same sequence number
        // 根据CRC校验位来判定是否 为有效数据帧，即 &BASIC_RF_CRC_OK_BM 的操作
        // 其中，还要求对帧序号进行验证，如果该数据帧 要求有应答的情况下 \
           在数据帧发送接收流程上不完整，帧序号连续两次相同的那么不对重复数据帧进行后续的处理，需要被丢弃；
        // 这里 rxi.seqNumber是指上一帧的数据帧序号，pHdr->seqNumber表示下一帧的数据帧序号
          // 另外如果发送节点确定数据被正确接收，可以采用连续发送两次，\
          并根据 应答是否成功 来判断程序是否进行二次重复发送；
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
            // 如果接收到的FCF满足无ACK位的条件，说明正确接收，将接收节点状态 设置为TRUE；
            // 其中BASIC_RF_FCF_NOACK_L = 0x61表示PANID一致而且该帧类型为 数据帧；
            // 数据帧的通过验证的判断条件：1.CRC = OK ；2.SeqNumber和上一次的不同；3.保持PANID 和 帧类型 一致；
            if ( ((pHdr->fcf0 & (BASIC_RF_FCF_BM_L)) == BASIC_RF_FCF_NOACK_L) ) {
                rxi.isReady = TRUE;
            }              
            #endif
        }
        
        // 这里做的事情就是把新的 pHdr->seqNumber帧序号赋给旧的帧序号 rxi.seqNumber
        // 数据帧要求有应答的情况，应答成功 →下一帧SeqNumber自加 →\
          （PANID 和 帧类型一致的条件下）下一帧进行的后期操作；然而应答不成功 →\
            下一帧SeqNumber保持 →rxi.isReady始终为FALSE下一帧无法进行后期处理，\
            表现现象为对应节点发送的数据始终无法接收！     
        // 数据帧要求无应答的情况，下一帧SeqNumber始终自加，不会造成对应节点发送的数据无法接收的问题；
        // SeqNumber在灯开关例程作用始终是 ACK的成功与否来决定的，因此应该善于利用ACK是否成功完成的条件！      
        rxi.seqNumber = pHdr->seqNumber;
    }
  
    // Enable RX frame done interrupt again
    // 关闭总中断，会在 basicRfReceive()中 复制完接收数据 后重新始能；
    halIntOff();
    // 始能通用RF中断 和 RXPKTDONE中断；
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

/*******************下面是对射频初始化配置参数的一个寄存器设置**********/
    // Set channel
    // 将定义的通道号写入相关寄存器；
    halRfSetChannel(pConfig->channel);         
    
    // Write the short address and the PAN ID to the CC2520 RAM
    // 将定义的本节点地址写入相关寄存器；
    // #define SHORT_ADDR0     XREG( 0x6174 )
    // #define SHORT_ADDR1     XREG( 0x6175 )
    halRfSetShortAddr(pConfig->myAddr);        

    // 将定义的PANID写入相关寄存器；
    // #define PAN_ID0         XREG( 0x6172 )
    // #define PAN_ID1         XREG( 0x6173 )
    halRfSetPanId(pConfig->panId);             
/****************************************************************************/    
    
    // if security is enabled, write key and nonce
    #ifdef SECURITY_CCM
    basicRfSecurityInit(pConfig);
    #endif

    // Set up receive interrupt (received data or acknowlegment)
    // 这里配置的是无线接收中断，传递的参数为中断函数指针
    // 这里的中断函数指针关联相应的中断函数，也就是声明中断程序；
    // basicRfRxFrmDoneIsr 这个中断函数的返回值为指针，这里就用这个指针当做参数
    halRfRxInterruptConfig(basicRfRxFrmDoneIsr); 

    // 开启总中断；
    halIntOn();

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
               // 对数据进行封帧为物理层协议数据单元；
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
    // 等待上一次发送结束
    halRfWaitTransceiverReady();

    // Turn off RX frame done interrupt to avoid interference on the SPI interface
    // 防止2591冲突？？？
    // 禁止接收中断，因为是全部数据通过RFD都传入TXFIFO才一起发送的，那时候才使能接收中断
    halRfDisableRxInterrupt();

    // 根据目的地址、载荷数据及长度信息进行封帧；
    // 构建MAC层的 MPDU，并返回 MPDU的长度
    // 这里的 mpduLength是包括整个 MPDU的长度，再加上前面一个字节的 Frame Length
    // 也就是在整个传输过程中实际有效的数据包的长度
    mpduLength = basicRfBuildMpdu(destAddr, pPayload, length);

    #ifdef SECURITY_CCM
    halRfWriteTxBufSecure(txMpdu, mpduLength, length, BASIC_RF_LEN_AUTH, BASIC_RF_SECURITY_M);
    txState.frameCounter++;     // Increment frame counter field
    #else
    // 使用ISFLUSHTX()清空TXFIFO 并清除IRQ_TXDONE发送中断溢出标志位，
    // 将MPDU里的数据一个字节一个字节的通过RFD写入TXFIFO，发送出去
    halRfWriteTxBuf(txMpdu, mpduLength);
    #endif

    // Turn on RX frame done interrupt for ACK reception
    // 仅仅是始能接收中断，为发送完成后自动进入接收模式 接收 ACK做准备性工作；
    // 仅作为发送节点且不启用ACK的话，这部分语句都可以省略去；
    halRfEnableRxInterrupt();

    // Send frame with CCA. return FAILED if not successful
    // 判断发送有没有成功
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
        // 等待应答帧的时间
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
*              // 把收到的数据复制到 buffer中
               // 接收来自Basic RF层的数据帧，并为所接收的数据和RSSI值配缓冲区

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
    // 关闭中断
    halIntOff();
    // memcpy函数被封装起来了，没法知道具体实现过程
    // 实现的功能是将无线发送过来的 payload里的数据放到自己定义的 RxData中
    memcpy(pRxData, rxi.pPayload, min(rxi.length, len));
    // pRssi 信号强度的检测
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

