/**
 *   @file  cppi_drv.h
 *
 *   @brief   
 *      This is the CPPI Low Level Driver include file.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  \par
*/


/** @defgroup CPPI_LLD_API CPPI
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 *
 * @subsection References
 *   -# CPPI Functional Specification 
 *
 * @subsection Assumptions
 *    
 */
#ifndef CPPI_DRV_H_
#define CPPI_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <c6678.h>


/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_cppidma_global_config.h>
#include <ti/csl/cslr_cppidma_rx_channel_config.h>
#include <ti/csl/cslr_cppidma_rx_flow_config.h>
#include <ti/csl/cslr_cppidma_tx_channel_config.h>
#include <ti/csl/cslr_cppidma_tx_scheduler_config.h>
#include <ti/csl/csl_cppi.h>

#include <ti/drv/cppi/cppiver.h>

/* QMSS LLD includes */
#include <qmss_drv.h>
/**
@defgroup CPPI_LLD_SYMBOL  CPPI Low Level Driver Symbols Defined
@ingroup CPPI_LLD_API
*/
/**
@defgroup CPPI_LLD_ENUM  CPPI Low Level Driver Enums
@ingroup CPPI_LLD_API
*/
/**
@defgroup CPPI_LLD_DATASTRUCT  CPPI Low Level Driver Data Structures
@ingroup CPPI_LLD_API
*/
/**
@defgroup CPPI_LLD_FUNCTION  CPPI Low Level Driver Functions
@ingroup CPPI_LLD_API
*/
/**
@defgroup CPPI_LLD_OSAL  CPPI Low Level Driver OSAL Functions
@ingroup CPPI_LLD_API
*/

/**
@addtogroup CPPI_LLD_SYMBOL
@{
*/

/** Used as input parameter when queue number is 
 * not known and not specified */
#define CPPI_PARAM_NOT_SPECIFIED            -1

/** CPPI Low level Driver return and Error Codes */
/** CPPI successful return code */
#define CPPI_SOK                            0
/** CPPI Error Base */       
#define CPPI_LLD_EBASE                      (-128)
/** CPPI CPDMA not yet initialized */
#define CPPI_CPDMA_NOT_INITIALIZED          (CPPI_LLD_EBASE-1)
/** CPPI invalid parameter */
#define CPPI_INVALID_PARAM                  (CPPI_LLD_EBASE-2)
/** CPPI Rx/Tx channel not yet enabled */
#define CPPI_CHANNEL_NOT_OPEN               (CPPI_LLD_EBASE-3)
/** CPPI Rx flow not yet enabled */
#define CPPI_FLOW_NOT_OPEN                  (CPPI_LLD_EBASE-4)
/** CPPI Tx channels are still open. 
 * All Tx channels should be closed 
 * before calling CPPI_close */
#define CPPI_TX_CHANNELS_NOT_CLOSED         (CPPI_LLD_EBASE-5)
/** CPPI Rx channels are still open. 
 * All Rx channels should be closed 
 * before calling CPPI_close */
#define CPPI_RX_CHANNELS_NOT_CLOSED         (CPPI_LLD_EBASE-6)
/** CPPI Rx flows are still open. 
 * All Rx flows should be closed 
 * before calling CPPI_close */
#define CPPI_RX_FLOWS_NOT_CLOSED            (CPPI_LLD_EBASE-7)

/** Queue Manager subsystem memory region not enabled */
#define CPPI_QMSS_MEMREGION_NOT_INITIALIZED (CPPI_LLD_EBASE-8)
/** Queue open error */
#define CPPI_QUEUE_OPEN_ERROR               (CPPI_LLD_EBASE-9)
/** CPPI extended packet information block not present in descriptor */
#define CPPI_EPIB_NOT_PRESENT               (CPPI_LLD_EBASE-10)
/** CPPI protocol specific data not present in descriptor */
#define CPPI_PSDATA_NOT_PRESENT             (CPPI_LLD_EBASE-11)
/** CPPI CPDMA instances are still open. 
 * All CPDMA instances should be closed 
 * before calling CPPI_exit */
#define CPPI_CPDMA_NOT_CLOSED               (CPPI_LLD_EBASE-12)

/**
@}
*/


/** Monolithic descriptor extended packet information block size */
#define CPPI_MONOLITHIC_DESC_EPIB_SIZE      20
/** Host descriptor extended packet information block size */
#define CPPI_HOST_DESC_EPIB_SIZE            16

/**************************************************************************
 * STRUCTURE -  Cppi_ListNode
 **************************************************************************
 *	The structure defines a LIST NODE structure that contains links to the
 *	previous and next element in the list.
 **************************************************************************/
typedef struct Cppi_ListNode
{
	void*	p_next;		/* Pointer to the next element in the list. */
    void*   p_prev;     /* Pointer to the prev element in the list. */
} Cppi_ListNode;



/**
@addtogroup CPPI_LLD_ENUM
@{
*/

/** 
 * @brief CPPI Channel type 
 */
typedef enum
{
    /** Receive Channel */
    Cppi_ChType_RX_CHANNEL = 0,
    /** Transmit Channel */
    Cppi_ChType_TX_CHANNEL
}Cppi_ChType;

/** 
 * @brief CPPI Channel Enable
 */
typedef enum
{
    /** Disable Channel */
    Cppi_ChState_CHANNEL_DISABLE = 0,
    /** Enable Channel */
    Cppi_ChState_CHANNEL_ENABLE 
}Cppi_ChState;

/** 
 * @brief CPPI Wait after Channel Teardown
 */
typedef enum
{
    /** No wait */
    Cppi_Wait_NO_WAIT = 0,
    /** Wait */
    Cppi_Wait_WAIT
}Cppi_Wait;


/**
@}
*/

/** @addtogroup CPPI_LLD_DATASTRUCT
@{ 
*/

/** 
 * @brief CPPI global configuration structure
 */
typedef struct
{
    /** CPDMA this configuration belongs to */
    Cppi_CpDma      dmaNum;
    /** Maximum supported Rx Channels */
    uint32_t          maxRxCh;
    /** Maximum supported Tx Channels */
    uint32_t          maxTxCh;
    /** Maximum supported Rx Flows */
    uint32_t          maxRxFlow;
    /** Priority for all Rx transactions of this CPDMA */
    uint8_t           rxPriority;
    /** Priority for all Tx transactions of this CPDMA */
    uint8_t           txPriority;

    /** Base address for the CPDMA overlay registers */

    /** Global Config registers */
    CSL_Cppidma_global_configRegs       *gblCfgRegs;
    /** Tx Channel Config registers */
    CSL_Cppidma_tx_channel_configRegs   *txChRegs;
    /** Rx Channel Config registers */
    CSL_Cppidma_rx_channel_configRegs   *rxChRegs;
    /** Tx Channel Scheduler registers */
    CSL_Cppidma_tx_scheduler_configRegs *txSchedRegs;
    /** Rx Flow Config registers */
    CSL_Cppidma_rx_flow_configRegs      *rxFlowRegs;
}Cppi_GlobalConfigParams;

/** 
 * @brief CPPI CPDMA configuration structure
 */
typedef struct
{
    /** CPDMA configuring control registers */
    Cppi_CpDma                  dmaNum;

    /** This field sets the depth of the write arbitration FIFO which stores write transaction information
     * between the command arbiter and write data arbiters in the Bus Interface Unit. Setting this field to smaller 
     * values will cause prevent the CDMAHP from having an excess of write transactions outstanding whose data is 
     * still waiting to be transferred.
     * System performance can suffer if write commands are allowed to be issued long before the corresponding 
     * write data will be transferred.  This field allows the command count to be optimized based on system dynamics
     *
     * Valid range is 1 to 32. If writeFifoDepth field is set to 0, this field will not be configured. The reset/default value is 20.
     */
    uint8_t                     writeFifoDepth;
    /** This field sets the timeout duration in clock cycles.  This field controls the minimum 
     * amount of time that an Rx channel will be required to wait when it encounters a buffer starvation 
     * condition and the Rx error handling bit is set to 1 (packet is to be preserved - no discard).  
     * If the Rx error handling bit in the flow table is cleared, this field will have no effect on the Rx operation.  
     * When this field is set to 0, the Rx engine will not force an Rx channel to wait after encountering a starvation 
     * event (the feature is disabled).  When this field is set to a value other than 0, the Rx engine will force any 
     * channel whose associated flow had the Rx error handling bit asserted and which encounters starvation to wait for 
     * at least the specified # of clock cycles before coming into context again to retry the access to the QM
     */ 
    uint16_t                    timeoutCount;
    /** The QM N Queues Region Base Address Register is used to provide a programmable 
     * pointer to the base address of the queues region in Queue Manager N in the system 
     */

    /** Queue Manager 0 base address register */
    volatile uint32_t            qm0BaseAddress;
    /** Queue Manager 1 base address register */
    volatile uint32_t            qm1BaseAddress;
    /** Queue Manager 2 base address register */
    volatile uint32_t            qm2BaseAddress;
    /** Queue Manager 3 base address register */
    volatile uint32_t            qm3BaseAddress;
}Cppi_CpDmaInitCfg;

/** 
 * @brief CPPI transmit channel configuration structure
 */
typedef struct
{
    /** Channel number */
    /** If channelNum is set to CPPI_PARAM_NOT_SPECIFIED then the next 
     * available channel will be allocated */
    int32_t           channelNum;
    /** Enable Tx Channel on creation. If not set use CPPI_channelEnable() API to enable it later */
    Cppi_ChState    txEnable;
    /** Tx scheduling priority for channelNum */
    uint8_t           priority;
    /** Tx Filter Software Info.  This field controls whether or not the DMA controller will pass the 
     * extended packet information fields (if present) from the descriptor to the back end application.
     * 0 - DMA controller will pass extended packet info fields if they are present in the descriptor
     * 1 - DMA controller will filter extended packet info fields
     */
    uint16_t             filterEPIB;
    /** Filter Protocol Specific Words. This field controls whether or not the DMA controller will 
     * pass the protocol specific words (if present) from the descriptor to the back end application.
     * 0 - DMA controller will pass PS words if present in descriptor
     * 1 - DMA controller will filter PS words
     */
    uint16_t             filterPS;
    /**
     * AIF Specific Monolithic Packet Mode. This field when set indicates that all monolithic packets 
     * which will be transferred on this channel will be formatted in an optimal configuration as needed 
     * by the Antenna Interface Peripheral.  The AIF configuration uses a fixed descriptor format which 
     * includes the 3 mandatory descriptor info words, a single Protocol Specific Word and data 
     * immediately following (data offset = 16).
     */
    uint16_t             aifMonoMode;
}Cppi_TxChInitCfg;

/** 
 * @brief CPPI receive channel configuration structure
 */
typedef struct
{
    /** Channel number */
    /** If channelNum is set to CPPI_PARAM_NOT_SPECIFIED then the next 
     * available channel will be allocated */
    int32_t           channelNum;
    /** Enable Rx Channel on creation. If not set use CPPI_channelEnable() API to enable it later */
    Cppi_ChState    rxEnable;
}Cppi_RxChInitCfg;

/** 
 * @brief CPPI receive flow configuration structure
 */
typedef struct 
{
    /** Rx flow configuration register A */

    /** flow ID number */
    /** If flowIdNum is set to CPPI_PARAM_NOT_SPECIFIED then the next available flow ID will be allocated */
    int16_t           flowIdNum;
    /** This field indicates the default receive queue that this channel should use */
    uint16_t          rx_dest_qnum;
    /** This field indicates the default receive queue manager that this channel should use */
    uint16_t          rx_dest_qmgr;
    /** This field specifies the number of bytes that are to be skipped in the SOP buffer before beginning 
     * to write the payload or protocol specific bytes(if they are in the sop buffer).  This value must
     * be less than the minimum size of a buffer in the system */
    uint16_t          rx_sop_offset;
    /** This field controls where the Protocol Specific words will be placed in the Host Mode CPPI data structure 
     * 0 - protocol specific information is located in descriptor 
     * 1 - protocol specific information is located in SOP buffer */
    uint16_t             rx_ps_location;
    /** This field indicates the descriptor type to use 1 = Host, 2 = Monolithic */
    uint8_t           rx_desc_type;
    /** This field controls the error handling mode for the flow and is only used when channel errors occurs 
     * 0 = Starvation errors result in dropping packet and reclaiming any used descriptor or buffer resources 
     * back to the original queues/pools they were allocated to
     * 1 = Starvation errors result in subsequent re-try of the descriptor allocation operation.  
     */
    uint16_t             rx_error_handling;
    /** This field controls whether or not the Protocol Specific words will be present in the Rx Packet Descriptor 
     * 0 - The port DMA will set the PS word count to 0 in the PD and will drop any PS words that are presented 
     * from the back end application.
     * 1 - The port DMA will set the PS word count to the value given by the back end application and will copy 
     * the PS words from the back end application to the location 
     */
    uint16_t             rx_psinfo_present;
    /** This field controls whether or not the Extended Packet Info Block will be present in the Rx Packet Descriptor.  
     * 0 - The port DMA will clear the Extended Packet Info Block Present bit in the PD and will drop any extended 
     * packet info words that are presented from the back end application. 
     * 1 - The port DMA will set the Extended Packet Info Block Present bit in the PD and will copy any extended packet
     * info words that are presented across the Rx streaming interface into the extended packet info words in the descriptor.
     * If no extended packet info words are presented from the back end application, the port DMA will overwrite the fields with zeroes.
     */
    uint16_t             rx_einfo_present;

    /** Rx flow configuration register B */

    /** This is the value to insert into bits 7:0 of the destination tag if the rx_dest_tag_lo_sel is set to 1 */
    uint8_t           rx_dest_tag_lo;
    /** This is the value to insert into bits 15:8 of the destination tag if the rx_dest_tag_hi_sel is set to 1 */
    uint8_t           rx_dest_tag_hi;
    /** This is the value to insert into bits 7:0 of the source tag if the rx_src_tag_lo_sel is set to 1 */
    uint8_t           rx_src_tag_lo;
    /** This is the value to insert into bits 15:8 of the source tag if the rx_src_tag_hi_sel is set to 1 */
    uint8_t           rx_src_tag_hi;    

    /** Rx flow configuration register C */
    /** This bits control whether or not the flow will compare the packet size received from the back end application 
     * against the rx_size_thresh0 fields to determine which FDQ to allocate the SOP buffer from.  
     * The bits in this field is encoded as follows:
     * 0 = Do not use the threshold.
     * 1 = Use the thresholds to select SOP FDQ rx_fdq0_sz0_qnum/rx_fdq0_sz0_qmgr.
     */
    uint8_t             rx_size_thresh0_en;
    /** This bits control whether or not the flow will compare the packet size received from the back end application 
     * against the rx_size_thresh1 fields to determine which FDQ to allocate the SOP buffer from.  
     * The bits in this field is encoded as follows:
     * 0 = Do not use the threshold.
     * 1 = Use the thresholds to select SOP FDQ rx_fdq0_sz1_qnum/rx_fdq0_sz1_qmgr.
     */
    uint8_t             rx_size_thresh1_en;
        /** This bits control whether or not the flow will compare the packet size received from the back end application 
     * against the rx_size_thresh2 fields to determine which FDQ to allocate the SOP buffer from.  
     * The bits in this field is encoded as follows:
     * 0 = Do not use the threshold.
     * 1 = Use the thresholds to select SOP FDQ rx_fdq0_sz2_qnum/rx_fdq0_sz2_qmgr.
     */
    uint8_t             rx_size_thresh2_en;

    /** This field specifies the source for bits 7:0 of the source tag field in word 1 of the output PD.
     * This field is encoded as follows:
     * 0 = do not overwrite
     * 1 = overwrite with value given in rx_dest_tag_lo
     * 2 = overwrite with flow_id[7:0] from back end application
     * 3 = RESERVED
     * 4 = overwrite with dest_tag[7:0] from back end application
     * 5 = overwrite with dest_tag[15:8] from back end application
     * 6-7 = RESERVED
     */    
    uint8_t           rx_dest_tag_lo_sel;
    /** This field specifies the source for bits 15:8 of the source tag field in the word 1 of the output PD.
     * This field is encoded as follows:
     * 0 = do not overwrite
     * 1 = overwrite with value given in rx_dest_tag_hi
     * 2 = overwrite with flow_id[7:0] from back end application
     * 3 = RESERVED
     * 4 = overwrite with dest_tag[7:0] from back end application
     * 5 = overwrite with dest_tag[15:8] from back end application
     * 6-7 = RESERVED
     */
    uint8_t           rx_dest_tag_hi_sel;
    /** This field specifies the source for bits 7:0 of the source tag field in the output packet descriptor.
     * This field is encoded as follows:
     * 0 = do not overwrite
     * 1 = overwrite with value given in rx_src_tag_lo
     * 2 = overwrite with flow_id[7:0] from back end application
     * 3 = RESERVED
     * 4 = overwrite with src_tag[7:0] from back end application
     * 5 = RESERVED
     * 6-7 = RESERVED
     */
    uint8_t           rx_src_tag_lo_sel;
    /** This field specifies the source for bits 15:8 of the source tag field in the output packet descriptor.
     * This field is encoded as follows:
     * 0 = do not overwrite
     * 1 = overwrite with value given in rx_src_tag_hi
     * 2 = overwrite with flow_id[7:0] from back end application
     * 3 = RESERVED
     * 4 = overwrite with src_tag[7:0] from back end application
     * 5 = RESERVED
     * 6-7 = RESERVED
     */
    uint8_t           rx_src_tag_hi_sel;    
 
    /** Rx flow configuration register D */

    /** This field specifies which Free Descriptor Queue should be used for the 2nd Rx buffer in a host type packet */
    uint16_t          rx_fdq1_qnum;
    /** This field specifies which Queue Manager should be used for the 2nd Rx buffer in a host type packet */
    uint16_t          rx_fdq1_qmgr;
    /** This field specifies which Free Descriptor Queue should be used for the 1st Rx buffer in a packet whose 
     * size is less than or equal to the rx_size0 value */
    uint16_t          rx_fdq0_sz0_qnum;
    /** This field specifies which Queue Manager should be used for the 1st Rx buffer in a packet whose size 
     * is less than or equal to the rx_size0 value */
    uint16_t          rx_fdq0_sz0_qmgr;

    /** Rx flow configuration register E */

    /** This field specifies which Free Descriptor Queue should be used for the 4th or later Rx
     *  buffers in a host type packet */
    uint16_t          rx_fdq3_qnum;
    /** This field specifies which Queue Manager should be used for the 4th or later Rx buffers 
     * in a host type packet */
    uint16_t          rx_fdq3_qmgr;
    /** This field specifies which Free Descriptor Queue should be used for the 3rd Rx buffer in a host type packet */
    uint16_t          rx_fdq2_qnum;
    /** This field specifies which Queue Manager should be used for the 3rd Rx buffer in a host type packet */
    uint16_t          rx_fdq2_qmgr;

    /** Rx flow configuration register F */

    /** This value is left shifted by 5 bits and compared against the packet size to determine which free descriptor 
     * queue should be used for the SOP buffer in the packet.  If the  packet size is greater than the rx_size_thresh0 
     * but is less than or equal to the value given in this threshold, the DMA controller in the port will allocate the 
     * SOP buffer from the queue given by the rx_fdq0_sz1_qmgr and rx_fdq0_sz1_qnum fields. 
     * If enabled, this value must be greater than the value given in the rx_size_thresh0 field. This field is optional.
     */
    uint16_t          rx_size_thresh1;
    /** This value is left shifted by 5 bits and compared against the packet size to determine which free descriptor 
     * queue should be used for the SOP buffer in the packet.  If the packet size is less than or equal to the value 
     * given in this threshold, the DMA controller in the port will allocate the SOP buffer from the queue given by 
     * the rx_fdq0_sz0_qmgr and rx_fdq0_sz0_qnum fields. This field is optional.
     */
    uint16_t          rx_size_thresh0;
    
    /** Rx flow configuration register G */

    /** This field specifies which Queue should be used for the 1st Rx buffer in a packet whose size is 
     * less than or equal to the rx_size0 value */
    uint16_t          rx_fdq0_sz1_qnum;
    /** This field specifies which Queue Manager should be used for the 1st Rx buffer in a packet whose size 
     * is less than or equal to the rx_size0 value */
    uint16_t          rx_fdq0_sz1_qmgr;
    /** This value is left shifted by 5 bits and compared against the packet size to determine which free descriptor 
     * queue should be used for the SOP buffer in the packet.  If the  packet size is less than or equal to the value
     * given in this threshold, the DMA controller in the port will allocate the SOP buffer from the queue given by the 
     * rx_fdq0_sz2_qmgr and rx_fdq0_sz2_qnum fields.
     * If enabled, this value must be greater than the value given in the rx_size_thresh1 field. This field is optional.
     */
    uint16_t  		rx_size_thresh2;

    /** Rx flow configuration register H */

    /** This field specifies which Free Descriptor Queue should be used for the 1st Rx buffer in a
     * packet whose size is less than or equal to the rx_size3 value */
    uint16_t          rx_fdq0_sz3_qnum;
    /** This field specifies which Free Descriptor Queue Manager should be used for the 1st Rx buffer in a 
     * packet whose size is less than or equal to the rx_size3 value */
    uint16_t          rx_fdq0_sz3_qmgr;
    /** This field specifies which Free Descriptor Queue should be used for the 1st Rx buffer in a packet 
     * whose size is less than or equal to the rx_size2 value */
    uint16_t          rx_fdq0_sz2_qnum;
    /** This field specifies which Free Descriptor Queue Manager should be used for the 1st Rx buffer in a packet 
     * whose size is less than or equal to the rx_size2 value */
    uint16_t          rx_fdq0_sz2_qmgr;
}Cppi_RxFlowCfg;

/** 
 * @brief CPPI return result
 */
typedef int32_t   Cppi_Result;

/** 
 * @brief CPPI handle
 */
typedef uint32_t  *Cppi_Handle;

/** 
 * @brief CPPI channel handle
 */
typedef uint32_t  *Cppi_ChHnd;

/** 
 * @brief CPPI receive flow handle
 */
typedef uint32_t  *Cppi_FlowHnd;

/** 
@} 
*/

/**
 * @brief CPPI descriptor types
 */
typedef enum
{
    /** Host descriptor */
    Cppi_DescType_HOST = 0,
    /** Monolithic descriptor */
    Cppi_DescType_MONOLITHIC = 2
}Cppi_DescType;

/**
 * @brief Packet return policy
 */
typedef enum
{
    /** Return entire packet */
    Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET = 0,
    /** Return one buffer at a time */
    Cppi_ReturnPolicy_RETURN_BUFFER
}Cppi_ReturnPolicy;

/**
 * @brief protocol specific information location
 */
typedef enum
{
    /** protocol specific information is located in descriptor */
    Cppi_PSLoc_PS_IN_DESC = 0,
    /** protocol specific information is located in SOP buffer */
    Cppi_PSLoc_PS_IN_SOP
}Cppi_PSLoc;

/**
 * @brief extended packet information block
 */
typedef enum
{
    /** extended packet information block is not present in descriptor */
    Cppi_EPIB_NO_EPIB_PRESENT = 0,
    /** extended packet information block is present in descriptor  */
    Cppi_EPIB_EPIB_PRESENT
}Cppi_EPIB;

/**
 * @brief Descriptor resource management
 */
typedef enum
{
    /** LLD initializes the descriptors with specified values */
    Cppi_InitDesc_INIT_DESCRIPTOR = 0,
    /** LLD does not initialize the descriptor with specified values */
    Cppi_InitDesc_BYPASS_INIT
}Cppi_InitDesc;

/**
@}
*/

/** @addtogroup CPPI_LLD_DATASTRUCT
@{
*/

/**
 * @brief CPPI host descriptor configuration structure
 */
typedef struct
{
    /** Indicates return policy for the packet.
     * Valid only for host descriptor */
    Cppi_ReturnPolicy       returnPolicy;
    /** Indicates protocol specific location CPPI_PS_DESC - located in descriptor, CPPI_PS_SOP - located in SOP buffer
     * Valid only for host descriptor */
    Cppi_PSLoc              psLocation;
}Cppi_HostDescCfg;

/**
 * @brief CPPI monolithic descriptor configuration structure
 */
typedef struct
{
    /** Byte offset from byte 0 of monolithic descriptor to the location where the valid data begins */
    uint32_t                  dataOffset;
}Cppi_MonolithicDescCfg;

/**
 * @brief CPPI descriptor configuration structure
 */
typedef struct
{
    /** Memory Region corresponding to the descriptor. */
    Qmss_MemRegion          memRegion;
    /** Number of descriptors that should be configured with value below */
    uint32_t                  descNum;
    /** Queue where the descriptor is stored. If destQueueNum is set to QMSS_PARAM_NOT_SPECIFIED then the next
     * available queue of type Qmss_QueueType will be allocated */
    int32_t                   destQueueNum;
    /** If destQueueNum is set to QMSS_PARAM_NOT_SPECIFIED then the next available queue of type
     * Qmss_QueueType will be allocated */
    Qmss_QueueType          queueType;

    /** Descriptor configuration parameters */
    /** Indicates if the descriptor should be initialized with parameters listed below */
    Cppi_InitDesc           initDesc;

    /** Type of descriptor - Host or Monolithic */
    Cppi_DescType           descType;
    /** Indicates return Queue Manager and Queue Number. If both qMgr and qNum in returnQueue is
     * set to QMSS_PARAM_NOT_SPECIFIED then the destQueueNum is configured in returnQueue of the descriptor */
    Qmss_Queue              returnQueue;
    /** Indicates how the CPDMA returns descriptors to free queue */
    Qmss_Location           returnPushPolicy;
    /** Indicates presence of EPIB */
    Cppi_EPIB               epibPresent;

    /** Union contains configuration that should be initialized in for host or monolithic descriptor.
     * The configuration for host or monolithic descriptor is choosen based on "descType" field.
     * The approriate structure fields must be specified if "initDesc" field is set to CPPI_INIT_DESCRIPTOR. */
    union{
    /** Host descriptor configuration parameters */
    Cppi_HostDescCfg        host;
    /** Monolithic  descriptor configuration parameters */
    Cppi_MonolithicDescCfg  mono;
    }cfg;
}Cppi_DescCfg;

/**
 * @brief CPPI descriptor Word 1 Tag information
 */
typedef struct {
    uint8_t srcTagHi;
    uint8_t srcTagLo;
    uint8_t destTagHi;
    uint8_t destTagLo;
}Cppi_DescTag;

/**
 * @brief CPPI host descriptor layout
 */
typedef struct {
    /** Descriptor type, packet type, protocol specific region location, packet length */
    uint32_t          descInfo;
    /** Source tag, Destination tag */
    uint32_t          tagInfo;
    /** EPIB present, PS valid word count, error flags, PS flags, return policy, return push policy,
     * packet return QM number, packet return queue number */
    uint32_t          packetInfo;
    /** Number of valid data bytes in the buffer */
    uint32_t          buffLen;
    /** Byte aligned memory address of the buffer associated with this descriptor */
    uint32_t          buffPtr;
    /** 32-bit word aligned memory address of the next buffer descriptor */
    uint32_t          nextBDPtr;
    /** Completion tag, original buffer size */
    uint32_t          origBufferLen;
    /** Original buffer pointer */
    uint32_t          origBuffPtr;
    /** Optional EPIB word0 */
    uint32_t          timeStamp;
    /** Optional EPIB word1 */
    uint32_t          softwareInfo0;
    /** Optional EPIB word2 */
    uint32_t          softwareInfo1;
    /** Optional EPIB word3 */
    uint32_t          softwareInfo2;
    /** Optional protocol specific data */
    uint32_t          psData;
}Cppi_HostDesc;

/**
 * @brief CPPI monolithic descriptor layout
 */
typedef struct {
    /** Descriptor type, packet type, data offset, packet length */
    uint32_t          descInfo;
    /** Source tag, Destination tag */
    uint32_t          tagInfo;
    /** EPIB present, PS valid word count, error flags, PS flags, return push policy,
     * packet return QM number, packet return queue number */
    uint32_t          packetInfo;
    /** NULL word to align the extended packet words to a 128 bit boundary */
    uint32_t          Reserved;
    /** Optional EPIB word0 */
    uint32_t          timeStamp;
    /** Optional EPIB word1 */
    uint32_t          softwareInfo0;
    /** Optional EPIB word2 */
    uint32_t          softwareInfo1;
    /** Optional EPIB word3 */
    uint32_t          softwareInfo2;
    /** Optional protocol specific data */
    uint32_t          psData;
}Cppi_MonolithicDesc;

/**
 * @brief CPPI descriptor
 */
typedef union {
    /** Host descriptor */
    Cppi_HostDesc       *ptrHostDesc;
    /** Monolithic descriptor */
    Cppi_MonolithicDesc *ptrMonoDesc;
}Cppi_Desc;


/* Channel Object */
typedef struct
{
    /* List of channel objects */
    Cppi_ListNode       links;
    /* Channel number */
    uint8_t             channelNum;
    /* CPDMA opening the channel */
    Cppi_CpDma          dmaNum;
    /* Channel Type - Rx or Tx */
    Cppi_ChType         chType;
    /* Number of times Channel is opened */
    uint8_t               refCnt;
    /* Pointer back to the CPDMA Object that opened this channel */
    struct Cppi_DMAObj  *dmaObjHnd;
}Cppi_ChObj;

/* Flow Object */
typedef struct
{
    /* List of flow objects */
    Cppi_ListNode       links;
    /* Channel number */
    uint8_t             flowId;
    /* CPDMA opening the channel */
    Cppi_CpDma          dmaNum;
    /* Number of times Channel is opened */
    uint8_t             refCnt;
    /* Pointer back to the CPDMA Object that opened this channel */
    struct Cppi_DMAObj  *dmaObjHnd;
}Cppi_FlowObj;

/* CPDMA Object */
typedef struct Cppi_DMAObj
{
    /* CPDMA this object belongs to */
    Cppi_CpDma              dmaNum;
    /* Reference count, the number of times CPDMA called cppi_init */
    uint8_t                 refCnt;
    /* Tx channel reference count, the number of times Tx channel was opened */
    uint8_t                 txChCnt;
    /* Rx channel reference count, the number of times Rx channel was opened */
    uint8_t                 rxChCnt;
    /* Rx flow reference count, the number of times Rx flow was configured */
    uint8_t                 rxFlowCnt;

    /* Depth of write arbitration FIFO */
    uint8_t                 writeFifoDepth;
	/* Minimum amount of time in clock cycles that an Rx channel will be required to wait when it
     * encounters a buffer starvation */
    uint16_t                timeoutCount;
    /* Queue Manager 0 base address register */
    volatile uint32_t       qm0BaseAddress;
    /* Queue Manager 1 base address register */
    volatile uint32_t       qm1BaseAddress;
    /* Queue Manager 2 base address register */
    volatile uint32_t       qm2BaseAddress;
    /* Queue Manager 3 base address register */
    volatile uint32_t       qm3BaseAddress;

    /* Base address for the CPDMA overlay registers */

    /* Global Config registers */
    CSL_Cppidma_global_configRegs       *gblCfgRegs;
    /* Rx Channel Config registers */
    CSL_Cppidma_rx_channel_configRegs   *rxChRegs;
    /* Tx Channel Config registers */
    CSL_Cppidma_tx_channel_configRegs   *txChRegs;
    /* Rx Flow Config registers */
    CSL_Cppidma_rx_flow_configRegs      *rxFlowRegs;
    /* Tx Channel Scheduler registers */
    CSL_Cppidma_tx_scheduler_configRegs *txSchedRegs;

    /* Maximum supported Rx Channels */
    uint8_t                 maxRxCh;
    /* Maximum supported Tx Channels */
    uint8_t                 maxTxCh;
    /* Maximum supported Rx Flows */
    uint8_t                 maxRxFlow;
    /* Priority for all Rx transactions of this CPDMA */
    uint8_t                 rxPriority;
    /* Priority for all Tx transactions of this CPDMA */
    uint8_t                 txPriority;
    /* Allocated Rx channels */
    Uint32                  rxChMask[5];
    /* Allocated Tx channels */
    Uint32                  txChMask[5];
    /* Allocated Rx flows */
    Uint32                  rxFlowMask[5];
    /* Rx Channel Handles */
    Cppi_ChObj              *rxChHnd;
    /* Tx Channel Handles */
    Cppi_ChObj              *txChHnd;
    /* Rx Flow Handles */
    Cppi_FlowObj            *rxFlowHnd;
}Cppi_DMAObj;

/* CPPI Object */
typedef struct
{
    /* CPDMA handle */
    Cppi_DMAObj             dmaCfg[CPPI_MAX_CPDMA];
}Cppi_Obj;


/**************************************cppi_osal.h************************/
/** @addtogroup CPPI_LLD_OSAL
 @{ */
void Radar_Osal_cppiBeginMemAccess (void *ptr, uint32_t size);
void Radar_Osal_cppiEndMemAccess (void *ptr, uint32_t size);


/**
 * @brief   The macro is used by the CPPI LLD to indicate that a block
 * of memory is about to be accessed. If the memory block is cached then
 * this indicates that the application would need to ensure that the cache
 * is updated with the data from the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_cppiBeginMemAccess (void *ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Radar_Cppi_osalBeginMemAccess     Radar_Osal_cppiBeginMemAccess

/* CPPI LLD OSAL Critical section and cache coherency APIs are used without redefinition in TEST application API */
#define Radar_Cppi_osalCsEnter            Radar_Osal_cppiCsEnter
#define Radar_Cppi_osalCsExit             Radar_Osal_cppiCsExit

/**
 * @brief   The macro is used by the CPPI LLD to indicate that the block of
 * memory has finished being accessed. If the memory block is cached then the
 * application would need to ensure that the contents of the cache are updated
 * immediately to the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_cppiEndMemAccess (void *ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Radar_Cppi_osalEndMemAccess       Radar_Osal_cppiEndMemAccess



void Radar_Cppi_setDescType(Cppi_Desc *descAddr, Cppi_DescType descType);
Cppi_DescType Radar_Cppi_getDescType (Cppi_Desc *descAddr);
uint32_t Radar_Cppi_getDescError (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setData (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t *buffAddr, uint32_t buffLen);
void Radar_Cppi_getData (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t **buffAddr, uint32_t *buffLen);
void Radar_Cppi_setDataLen (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t buffLen);
uint32_t Radar_Cppi_getDataLen (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_linkNextBD (Cppi_DescType descType, Cppi_Desc *descAddr, Cppi_Desc *nextBD);
Cppi_Desc* Radar_Cppi_getNextBD (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setOriginalBufInfo (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t *buffAddr, uint32_t buffLen);
void Radar_Cppi_getOriginalBufInfo (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t **buffAddr, uint32_t *buffLen);
void Radar_Cppi_setPacketType (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t packetType);
uint8_t Radar_Cppi_getPacketType (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setTimeStamp (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t timeStamp);
Cppi_Result Radar_Cppi_getTimeStamp (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t *timeStamp);
void Radar_Cppi_setSoftwareInfo (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t *infoAddr);
Cppi_Result Radar_Cppi_getSoftwareInfo (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t **infoAddr);
void Radar_Cppi_setSoftwareInfo0 (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t value);
uint32_t Radar_Cppi_getSoftwareInfo0 (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setSoftwareInfo1 (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t value);
uint32_t Radar_Cppi_getSoftwareInfo1 (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setSoftwareInfo2 (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t value);
uint32_t Radar_Cppi_getSoftwareInfo2 (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setPSData (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t *dataAddr, uint32_t dataLen);
Cppi_Result Radar_Cppi_getPSData (Cppi_DescType descType, Cppi_PSLoc location, Cppi_Desc *descAddr, uint8_t **dataAddr, uint32_t *dataLen);
void Radar_Cppi_setPSLen (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t dataLen);
uint32_t Radar_Cppi_getPSLen (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setPacketLen (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t packetLen);
uint32_t Radar_Cppi_getPacketLen (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setPSLocation (Cppi_DescType descType, Cppi_Desc *descAddr, Cppi_PSLoc location);
Cppi_PSLoc Radar_Cppi_getPSLocation (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setPSFlags (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t psFlags);
uint8_t Radar_Cppi_getPSFlags (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setOrigBufferpooIndex (Cppi_DescType descType, Cppi_Desc *descAddr, uint8_t poolIndex);
uint8_t Radar_Cppi_getOrigBufferpooIndex (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_incrementRefCount (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_decrementRefCount (Cppi_DescType descType, Cppi_Desc *descAddr);
uint8_t Radar_Cppi_getRefCount (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setDataOffset (Cppi_DescType descType, Cppi_Desc *descAddr, uint32_t dataOffset);
uint32_t Radar_Cppi_getDataOffset (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setReturnPolicy (Cppi_DescType descType, Cppi_Desc *descAddr, Cppi_ReturnPolicy returnPolicy);
Cppi_ReturnPolicy Radar_Cppi_getReturnPolicy (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setReturnPushPolicy (Cppi_DescType descType, Cppi_Desc *descAddr, Qmss_Location returnPushPolicy);
Qmss_Location Radar_Cppi_getReturnPushPolicy (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setReturnQueue (Cppi_DescType descType, Cppi_Desc *descAddr, Qmss_Queue queue);
Qmss_Queue Radar_Cppi_getReturnQueue (Cppi_DescType descType, Cppi_Desc *descAddr);
void Radar_Cppi_setTag (Cppi_DescType descType, Cppi_Desc *descAddr, Cppi_DescTag *tag);
Cppi_DescTag Radar_Cppi_getTag (Cppi_DescType descType, Cppi_Desc *descAddr);

Qmss_QueueHnd Radar_Cppi_initDescriptor (Cppi_DescCfg *descCfg, uint32_t *numAllocated);

/* Exported functions */
Cppi_Result Radar_Cppi_init (Cppi_GlobalConfigParams *cppiGblCfgParams);
Cppi_Result Radar_Cppi_exit (void);
Cppi_Handle Radar_Cppi_open (Cppi_CpDmaInitCfg *initCfg);
Cppi_Result Radar_Cppi_close (Cppi_Handle hnd);
Cppi_ChHnd Radar_Cppi_txChannelOpen (Cppi_Handle hnd, Cppi_TxChInitCfg *cfg, uint8_t *isAllocated);
Cppi_ChHnd Radar_Cppi_rxChannelOpen (Cppi_Handle hnd, Cppi_RxChInitCfg *cfg, uint8_t *isAllocated);
Cppi_Result Radar_Cppi_channelEnable (Cppi_ChHnd hnd);
Cppi_Result Radar_Cppi_channelDisable (Cppi_ChHnd hnd);
Cppi_Result Radar_Cppi_channelTeardown (Cppi_ChHnd hnd, Cppi_Wait wait);
Cppi_Result Radar_Cppi_channelClose (Cppi_ChHnd hnd);
Cppi_Result Radar_Cppi_channelPause (Cppi_ChHnd hnd);
Cppi_Result Radar_Cppi_channelStatus (Cppi_ChHnd hnd);
Cppi_FlowHnd Radar_Cppi_configureRxFlow (Cppi_Handle hnd, Cppi_RxFlowCfg *cfg, uint8_t *isAllocated);
Cppi_Result Radar_Cppi_closeRxFlow (Cppi_FlowHnd hnd);
uint32_t Radar_Cppi_getChannelNumber (Cppi_ChHnd hnd);
uint32_t Radar_Cppi_getFlowId (Cppi_FlowHnd hnd);
Cppi_Result Radar_Cppi_setCpdmaLoopback (Cppi_Handle hnd, uint8_t loopback);
Cppi_Result Radar_Cppi_getCpdmaLoopback (Cppi_Handle hnd);
uint32_t Radar_Cppi_getVersion (void);
const unsigned char* Radar_Cppi_getVersionStr (void);



void Radar_cppi_list_add (Cppi_ListNode** ptr_list, Cppi_ListNode* ptr_node);
Cppi_ListNode* Radar_cppi_list_remove (Cppi_ListNode** ptr_list);
Cppi_ListNode* Radar_cppi_list_get_head (Cppi_ListNode** ptr_list);
Cppi_ListNode* Radar_cppi_list_get_next (Cppi_ListNode* ptr_list);
int Radar_cppi_list_remove_node (Cppi_ListNode** ptr_list, Cppi_ListNode* ptr_remove);
void Radar_cppi_list_cat (Cppi_ListNode** ptr_dst, Cppi_ListNode** ptr_src);


Ptr Radar_Osal_cppiCsEnter (void);
void Radar_Osal_cppiCsExit (Ptr CsHandle);


#ifdef __cplusplus
}
#endif

#endif /* CPPI_DRV_H_ */

