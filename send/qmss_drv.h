/**
 *   @file  qmss_drv.h
 *
 *   @brief   
 *      This is the Queue Manager Sub System Low Level Driver include file.
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


/** @defgroup QMSS_LLD_API QMSS
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 *
 * @subsection References
 *   -# QMSS Functional Specification 
 *
 * @subsection Assumptions
 *    
 */
#ifndef QMSS_DRV_H_
#define QMSS_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

/* QMSS LLD includes */
#include <c6678.h>

#include <string.h>
/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_qm_config.h>
#include <ti/csl/cslr_qm_descriptor_region_config.h>
#include <ti/csl/cslr_qm_queue_management.h>
#include <ti/csl/cslr_qm_queue_status_config.h>
#include <ti/csl/cslr_qm_intd.h>
#include <ti/csl/cslr_pdsp.h>
#include <ti/csl/cslr_qm_qos_pdsp.h>
#include <ti/csl/csl_qm_queue.h>
#include <ti/csl/cslr_mcdma.h>
#include <ti/csl/cslr_cp_timer16.h>

/* QMSS includes */
#include <ti/drv/qmss/qmssver.h>



/*******************************qmss_acc.h********************************/
/**
@addtogroup QMSS_LLD_SYMBOL
@{
*/

/** QMSS accumulator return and Error Codes */
/** QMSS accumulator idle return code */
#define QMSS_ACC_IDLE                               0
/** QMSS accumulator successful return code */
#define QMSS_ACC_SOK                                1
/** QMSS accumulator invalid command return code */
#define QMSS_ACC_INVALID_COMMAND                    2
/** QMSS accumulator invalid channel return code */
#define QMSS_ACC_INVALID_CHANNEL                    3
/** QMSS accumulator channel not active return code */
#define QMSS_ACC_CHANNEL_NOT_ACTIVE                 4
/** QMSS accumulator channel already active */
#define QMSS_ACC_CHANNEL_ALREADY_ACTIVE             5
/** QMSS accumulator invalid queue number */
#define QMSS_ACC_INVALID_QUEUE_NUMBER               6

/**
@}
*/

/***********************************qmss_qm.h*******************************/
/**
@defgroup QMSS_LLD_SYMBOL  QMSS Low Level Driver Symbols Defined
@ingroup QMSS_LLD_API
*/
/**
@defgroup QMSS_LLD_ENUM  QMSS Low Level Driver Enums
@ingroup QMSS_LLD_API
*/
/**
@defgroup QMSS_LLD_DATASTRUCT  QMSS Low Level Driver Data Structures
@ingroup QMSS_LLD_API
*/
/**
@defgroup QMSS_LLD_FUNCTION  QMSS Low Level Driver Functions
@ingroup QMSS_LLD_API
*/
/**
@defgroup QMSS_LLD_OSAL  QMSS Low Level Driver OSAL Functions
@ingroup QMSS_LLD_API
*/

/**
@addtogroup QMSS_LLD_SYMBOL
@{
*/

/** Internal Linking RAM offset */
#define QMSS_LINKING_RAM_OFFSET                     0x80000
/** Internal Linking RAM default size */
#define QMSS_LINKING_RAM_REGION_0_DEFAULT_SIZE      0x3FFF

/** Used as input parameter when queue number is
 * not known and not specified */
#define QMSS_PARAM_NOT_SPECIFIED                    -1

/** QMSS Low level Driver return and Error Codes */
/** QMSS successful return code */
#define QMSS_SOK                                    0
/** QMSS Error Base */
#define QMSS_LLD_EBASE                              (-128)
/** QMSS LLD invalid parameter */
#define QMSS_INVALID_PARAM                          (QMSS_LLD_EBASE-1)
/** QMSS LLD not initialized */
#define QMSS_NOT_INITIALIZED                        (QMSS_LLD_EBASE-2)
/** QMSS LLD queue open error */
#define QMSS_QUEUE_OPEN_ERROR                       (QMSS_LLD_EBASE-3)
/** QMSS memory region not initialized */
#define QMSS_MEMREGION_NOT_INITIALIZED              (QMSS_LLD_EBASE-4)
/** QMSS memory region already initialized */
#define QMSS_MEMREGION_ALREADY_INITIALIZED          (QMSS_LLD_EBASE-5)
/** QMSS memory region invalid parameter */
#define QMSS_MEMREGION_INVALID_PARAM                (QMSS_LLD_EBASE-6)
/** QMSS maximum number of allowed descriptor are already configured */
#define QMSS_MAX_DESCRIPTORS_CONFIGURED             (QMSS_LLD_EBASE-7)
/** QMSS Specified memory region index is invalid or no memory regions are available */
#define QMSS_MEMREGION_INVALID_INDEX                (QMSS_LLD_EBASE-8)
/** QMSS memory region overlap */
#define QMSS_MEMREGION_OVERLAP                      (QMSS_LLD_EBASE-9)
/** QMSS memory region not in acscending order */
#define QMSS_MEMREGION_ORDERING                     (QMSS_LLD_EBASE-10)
/** QMSS PDSP firmware download failure */
#define QMSS_FIRMWARE_DOWNLOAD_FAILED               (QMSS_LLD_EBASE-11)

/** QMSS maximum number of memory regions */
#define QMSS_MAX_MEM_REGIONS                        20
#define QMSS_MAX_PDSP                               2

/** Macro to get the descriptor pointer if the popped descriptor contains the descriptor size.
 * If Qmss_queuePushDescSize() API is used to push a descriptor onto a queue, the descriptor when
 * popped will have the descriptor size information in the lower 4 bits. This macro is provided to
 * clear out the size information */
#define QMSS_DESC_PTR(desc)                         ((uint32_t)(desc) & 0xFFFFFFF0)

/** Macro to get the descriptor size if the popped descriptor contains the descriptor size.
 * If Qmss_queuePushDescSize() API is used to push a descriptor onto a queue, the descriptor when
 * popped will have the descriptor size information in the lower 4 bits. This macro is provided to
 * obtain the size information. Minimum size is 16 bytes. Maximum size is 256 bytes */
#define QMSS_DESC_SIZE(desc)                        ((((uint32_t)(desc) & 0x0000000F)+ 1) << 4)

/**
@}
*/

/****************************************qmss_qos.h******************************/

/**
@addtogroup QMSS_LLD_SYMBOL
@{
*/

/** QMSS QoS PDSP number. QoS PDSP is downloaded to PDSP 1 */
#define QMSS_QOS_PDSP_NUM                           1
#define QMSS_QOS_MAX_CLUSTERS                       8
#define QMSS_QOS_MAX_QUEUES                         64
#define QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT          9

/* Specifications for Round Robin cluster */
#define QMSS_QOS_MAX_QUE_RR_HIGH_PRI                4
#define QMSS_QOS_MAX_QUE_RR_LOW_PRI                 4
#define QMSS_QOS_MAX_QUE_RR_EGRESS                  1

/** QMSS QoS commands */
#define QMSS_QCMD_GET_QUEUE_BASE                    0x80
#define QMSS_QCMD_SET_QUEUE_BASE                    0x81
#define QMSS_QCMD_TIMER_CONFIG                      0x82
#define QMSS_QCMD_ENABLE_CLUSTER                    0x83

/** QMSS QoS return and Error Codes */
/** QMSS QoS successful return code */
#define QCMD_RETCODE_SUCCESS                        1
/** QMSS QoS invalid command return code */
#define QMSS_QCMD_INVALID_COMMAND                   2
/** QMSS QoS invalid index return code */
#define QMSS_QCMD_INVALID_INDEX                     3
/** QMSS QoS invalid option return code */
#define QMSS_QCMD_INVALID_OPTION                    4
/** QMSS QoS invalid cluster mode */
#define QMSS_QCMD_INVALID_MODE                      100
/** QMSS QoS invalid round robin high priority q num */
#define QMSS_QCMD_INVALID_RR_HIGH_Q                 101
/** QMSS QoS invalid round robin low priority q num */
#define QMSS_QCMD_INVALID_RR_LOW_Q                  102
/** QMSS QoS invalid round robin low priority q num */
#define QMSS_QCMD_INVALID_RR_EGRESS_Q               103

/**
@}
*/
#define NUM_HOST_DESC               32
#define SIZE_HOST_DESC              48
/****************************************qmss_qos.h******************************/
/**
@addtogroup QMSS_LLD_ENUM
@{
*/

/**
 * @brief QoS cluster mode
 */
typedef enum
{
    /** Modified Token Bucket Mode */
    Qmss_QosMode_TokenBucket,
    /** Round Robin Mode */
    Qmss_QosMode_RoundRobin
} Qmss_QosMode;

/**
@}
*/

/** @addtogroup QMSS_LLD_DATASTRUCT
@{
*/

/**
 * @brief QoS queue configuration structure
 */
typedef struct
{
    /** Queue manger and Queue index of the forwarding queue */
    uint16_t              egressQueNum;
    /** The amount of forwarding byte credit that the queue receives every 25us */
    uint16_t              iterationCredit;
    /** The maximum amount of forwarding byte credit that the queue is allowed to
     * hold at the end of the timer iteration. Any credit over the maximum limit
     * is added to a global pool */
    uint32_t              maxCredit;
    /** The size in bytes at which point the QOS queue is considered to be congested */
    uint32_t              congestionThreshold;
} Qmss_QosQueueCfg;


/*******************************qmss_acc.h********************************/
/**
@addtogroup QMSS_LLD_ENUM
@{
*/

/**
 * @brief Accumulator channel commands
 */
typedef enum
{
    /** Accumulator command to disable channel */
    Qmss_AccCmd_DISABLE_CHANNEL = 0x80,
    /** Accumulator command to enable channel */
    Qmss_AccCmd_ENABLE_CHANNEL = 0x81,
    /** Accumulator command to configure timer constant */
    Qmss_AccCmd_CONFIG_TIMER_CONSTANT = 0x82,
    /** Accumulator command to configure reclamation queue */
    Qmss_AccCmd_CONFIG_RECLAIM_QUEUE = 0x83,
    /** Accumulator command to configure diversion queue */
    Qmss_AccCmd_CONFIG_DIVERSION_QUEUE = 0x84

}Qmss_AccCmdType;

/**
 * @brief Accumulator configuration - interrupt pacing mode
 */
typedef enum
{
    /** Interrupt on entry threshold count only */
    Qmss_AccPacingMode_NONE = 0,
    /** Time delay since last interrupt */
    Qmss_AccPacingMode_LAST_INTERRUPT,
    /** Time delay since first new packet */
    Qmss_AccPacingMode_FIRST_NEW_PACKET,
    /** Time delay since last new packet */
    Qmss_AccPacingMode_LAST_NEW_PACKET
}Qmss_AccPacingMode;

/**
 * @brief Accumulator configuration - list entry size
 */
typedef enum
{
    /** 'D' register only (4 byte entries)
     * Word 0 : Packet Descriptor Pointer
     */
    Qmss_AccEntrySize_REG_D = 0,
    /** 'C,D' registers (8 byte entries)
     * Word 0 : Packet Length (as reported by queue manager)
     * Word 1 : Packet Descriptor Pointer
     */
    Qmss_AccEntrySize_REG_CD,
    /** 'A,B,C,D' registers (16 byte entries)
     * Word 0 : Packet Count on Queue (when read)
     * Word 1 : Byte Count on Queue (when read)
     * Word 2 : Packet Length (as reported by queue manager)
     * Word 3 : Packet Descriptor Pointer
     */
    Qmss_AccEntrySize_REG_ABCD
}Qmss_AccEntrySize;

/**
 * @brief Accumulator configuration - list count mode
 */
typedef enum
{
    /** NULL Terminate Mode - The last list entry is used to store a NULL pointer
     * record (NULL terminator) to mark the end of list. In either case there is room for one less
     * list entry in a page than is actually specified by the host.
     */
    Qmss_AccCountMode_NULL_TERMINATE = 0,
    /** Entry Count Mode - The first list entry is used to store the total list entry
     * count (not including the length entry).
     */
    Qmss_AccCountMode_ENTRY_COUNT
}Qmss_AccCountMode;


/**
 * @brief Accumulator configuration - Multi-Queue Mode
 */
typedef enum
{
    /** Single Queue Mode - The channel monitors a single queue. */
    Qmss_AccQueueMode_SINGLE_QUEUE = 0,
    /** Multi-Queue Mode - The channel monitors up to 32 queues starting at the supplied base queue index. */
    Qmss_AccQueueMode_MULTI_QUEUE
}Qmss_AccQueueMode;

/**
@}
*/

/***********************************qmss_qm.h*******************************/
/**
@addtogroup QMSS_LLD_ENUM
@{
*/

/**
 * @brief location where the packet is queued
 */
typedef enum
{
    /** Queue packet to the tail of the queue. Default behavior. */
    Qmss_Location_TAIL = 0,
    /** Queue packet to the head of the queue. */
    Qmss_Location_HEAD
}Qmss_Location;

/**
 * @brief Descriptor resource management
 */
typedef enum
{
    /** LLD doesnot manage the descriptors. The caller should manage them. */
    Qmss_ManageDesc_UNMANAGED_DESCRIPTOR = 0,
    /** LLD manages the descriptors. The descriptors are reclaimed using
     * the QMSS_initDescriptor() or CPPI_initDescriptor() APIs
     * */
    Qmss_ManageDesc_MANAGE_DESCRIPTOR
}Qmss_ManageDesc;

/**
 * @brief Queue Manager's memory regions
 */
typedef enum
{
    /** Memory region not specified. LLD allocates the next available memory region */
    Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED = -1,
    /** Configure memory region0. */
    Qmss_MemRegion_MEMORY_REGION0 = 0,
    /** Configure memory region 1. */
    Qmss_MemRegion_MEMORY_REGION1,
    /** Configure memory region 2. */
    Qmss_MemRegion_MEMORY_REGION2,
    /** Configure memory region 3. */
    Qmss_MemRegion_MEMORY_REGION3,
    /** Configure memory region 4. */
    Qmss_MemRegion_MEMORY_REGION4,
    /** Configure memory region 5. */
    Qmss_MemRegion_MEMORY_REGION5,
    /** Configure memory region 6. */
    Qmss_MemRegion_MEMORY_REGION6,
    /** Configure memory region 7. */
    Qmss_MemRegion_MEMORY_REGION7,
    /** Configure memory region 8. */
    Qmss_MemRegion_MEMORY_REGION8,
    /** Configure memory region 9. */
    Qmss_MemRegion_MEMORY_REGION9,
    /** Configure memory region 10. */
    Qmss_MemRegion_MEMORY_REGION10,
    /** Configure memory region 11. */
    Qmss_MemRegion_MEMORY_REGION11,
    /** Configure memory region 12. */
    Qmss_MemRegion_MEMORY_REGION12,
    /** Configure memory region 13. */
    Qmss_MemRegion_MEMORY_REGION13,
    /** Configure memory region 14. */
    Qmss_MemRegion_MEMORY_REGION14,
    /** Configure memory region 15. */
    Qmss_MemRegion_MEMORY_REGION15,
    /** Configure memory region 16. */
    Qmss_MemRegion_MEMORY_REGION16,
    /** Configure memory region 17. */
    Qmss_MemRegion_MEMORY_REGION17,
    /** Configure memory region 18. */
    Qmss_MemRegion_MEMORY_REGION18,
    /** Configure memory region 19. */
    Qmss_MemRegion_MEMORY_REGION19
}Qmss_MemRegion;

/**
 * @brief PDSP ID
 */
typedef enum
{
    /** PDSP 1 */
    Qmss_PdspId_PDSP1 = 0,
    /** PDSP 2 */
    Qmss_PdspId_PDSP2
}Qmss_PdspId;

/**
 * @brief INTD interrupt types
 */
typedef enum
{
    /** Interrupt generated for the high priority accumulator.
     * 32 interrupts are generated in response to events in the 32 high-priority queues.
     */
    Qmss_IntdInterruptType_HIGH = 0,
    /** Interrupt generated for the low priority accumulator.
     * 16 interrupts are generated in response to events in the 512 low-priority queues.
     */
    Qmss_IntdInterruptType_LOW,
    /** Interrupt generated for QMSS CDMA.
     * 2 interrupts are generated for buffer descriptor starvation event on
     * receive SOP (start of packet) and MOP (middle of packet) for any of the receive DMA units in the CDMA.
     */
    Qmss_IntdInterruptType_CDMA
}Qmss_IntdInterruptType;

/**
@}
*/

/** @addtogroup QMSS_LLD_DATASTRUCT
@{
*/

/**
 * @brief Queue definition
 */
typedef struct
{
    /** Queue manager number */
    int32_t qMgr;
    /** Queue number within Queue Manager */
    int32_t qNum;
}Qmss_Queue;

/**
 * @brief Queue definition
 */
typedef struct
{
    /** Queue manager number */
    int32_t     startIndex;
    /** Queue number within Queue Manager */
    int32_t     maxNum;
}Qmss_QueueNumRange;



/****************************************qmss_qos.h******************************/
/**
 * @brief QoS cluster configuration structure for Modified Token Bucket
 */
typedef struct
{
    /** The maximum amount of global credit allowed to carry over to the next queue.
     * Excess global credit is discarded */
    uint32_t              maxGlobalCredit;
    /** The number of QOS queues in this cluster. Valid range is 1 to QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT */
    uint8_t               qosQueCnt;
    /** The queue index (0 to 63) of each QOS queue in the cluster listed
     * in priority order. These queue indices are relative to the configured QOS
     * queue base index
     * Ensure that the queue base passed into @ref Qmss_setQosQueueBase supports
     * the size of the queue index provided (eg some devices allocate fewer than 64
     * queues).
     */
    uint8_t               qosQueNum[QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT];
    /** This 9-bit mask contains 1 bit for each QOS queue in the cluster.
     * When this bit is set for its corresponding QOS queue, iteration credit is treated
     * as "real time" scheduling and does not scale when the egress queue become congested */
    uint16_t              qosQueRTFlags;
    /** The total number of egress queues sampled to obtain the egress queue congestion estimation.
     * Valid range is 1 to QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT */
    uint8_t               egressQueCnt;
    /** The Queue manger and Queue index of every egress queue enumerated in Egress Queue Count.
     * These queue indices are absolute index values */
    Qmss_Queue          egressQueNum[QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT];
    /** Each QOS cluster is configured with four egress congestion threshold values.
     * Iteration credit is assigned to each queue in the cluster depending on the egress
     * congestion, and the value of these four congestion thresholds.
     *
     * It is implemented as shown below:
     *
     * Egress Queue Congestion (Backlog) Level  |   QOS Queue Credit Assigned
     * From no backlog to Threshold 1	        |   Double credit
     * Between Threshold 1 and Threshold 2	    |   Normal credit
     * Between Threshold 2 and Threshold 3	    |   Half credit
     * Between Threshold 3 and Threshold 4	    |   Quarter credit
     * Over Threshold 4	                        |   No credit
     *
     */
    /** Egress Congestion Threshold point 1 */
    uint32_t              egressCongestionThreshold1;
    /** Egress Congestion Threshold point 2 */
    uint32_t              egressCongestionThreshold2;
    /** Egress Congestion Threshold point 3 */
    uint32_t              egressCongestionThreshold3;
    /** Egress Congestion Threshold point 4 */
    uint32_t              egressCongestionThreshold4;
} Qmss_QosClusterCfgTB;

/**
@}
*/

/**
 * @brief QoS cluster configuration structure for Round Robin
 */
typedef struct
{
    /** The maximum amount of global credit allowed to carry over to the next queue.
     * Excess global credit is discarded */
    uint32_t              maxGlobalCredit;

    /** The number of high priority QOS queues in this cluster. Valid value is 4. */
    uint8_t               qosQueHighCnt;

    /** The queue index (0 to 63) of each QOS queue in the high priority
     * round robin group. These queue indices are relative to the configured
     * QOS queue base index. These fields must be set to 56, 57, 58, and 59
     * respectively.  Ensure that the base address passed into
     * @ref Qmss_setQosQueueBase supports 64 queues.
     */
    uint8_t               qosQueNumHigh[QMSS_QOS_MAX_QUE_RR_HIGH_PRI];

    /** The number of high priority QOS queues in this cluster. Valid value is 4. */
    uint8_t               qosQueLowCnt;

    /** The queue index (0 to 63) of each QOS queue in the low priority
     * round robin group. These queue indices are relative to the configured
     * QOS queue base index. These fields must be set to 60, 61, 62, and 63
     * respectively.  Ensure that the base address passed into
     * @ref Qmss_setQosQueueBase supports 64 queues.
     */
    uint8_t               qosQueNumLow[QMSS_QOS_MAX_QUE_RR_LOW_PRI];

    /** This field holds the value of a packet size adjustment that can be
     * applied to each packet. For example, setting this value to 24
     * can adjust for the preamble, inter-packet gap, and CRC for packets
     * without CRC being sent over Ethernet. This adjustment value is
     * applied across all queues. */
    uint16_t              sizeAdjust;

    /** The total number of egress queues sampled to obtain the egress
     * queue congestion estimation.  Valid value is 1.
     */
    uint8_t               egressQueCnt;

    /** The Queue manger and Queue index of every (1) egress queue enumerated
     * in Egress Queue Count.
     * These queue indices are absolute index values */
    Qmss_Queue            egressQueNum[QMSS_QOS_MAX_QUE_RR_EGRESS];

    /** This is the per timer tick real time iteration credit for the cluster.
     * (The iteration credit specified in each of the round robin queues is
     * ignored.) */
    uint32_t              iterationCredit;

    /** This is the max number of bytes allowed to reside in the egress
     * queue(s). Note that packets will be written until this threshold is
     * crossed, so the actual number of bytes queued can be larger. */
    uint32_t              maxEgressBacklog;

    /** This 8-bit mask contains 1 bit for each QOS queue in the cluster.
     * When this bit is set for its corresponding QOS queue, the queue
     * is disabled for forwarding. */
    uint32_t              queueDisableMask;
} Qmss_QosClusterCfgRR;

/**
@}
*/

/**
 * @brief QoS cluster configuration structure
 */
typedef struct
{
    /** Select Modified Token Bucket or Round Robin mode */
    Qmss_QosMode mode;
    union {
        /** configuration for Modified Token Bucket mode */
        Qmss_QosClusterCfgTB cfgTB;
        /** configuration for Round Robin mode */
        Qmss_QosClusterCfgRR cfgRR;
    } u;
} Qmss_QosClusterCfg;
/**
@}
*/

/*******************************qmss_acc.h********************************/
/** @addtogroup QMSS_LLD_DATASTRUCT
@{
*/

/**
 * @brief Accumulator Command interface structure
 */
typedef struct
{
    /** Accumulator channel affected (0-47) */
    uint8_t               channel;
    /** Accumulator channel command - Qmss_AccCmd_ENABLE_CHANNEL : Enable channel
     * Qmss_AccCmd_DISABLE_CHANNEL : Disable channel */
    Qmss_AccCmdType     command;
    /** This field specifies which queues are to be included in the queue group.
     * Bit 0 corresponds to the base queue index, and bit 31 corresponds to the base
     * queue index plus 31. For any bit set in this mask, the corresponding queue index
     * is included in the monitoring function.
     *
     * This field is ignored in single-queue mode.*/
    uint32_t              queueEnMask;
    /** Physical pointer to list ping/pong buffer. NULL when channel disabled */
    uint32_t              listAddress;
    /** Queue Manager and Queue Number index to monitor. This serves as a base queue index when the
     * channel in multi-queue mode, and must be a multiple of 32 when multi-queue mode is enabled. */
    uint16_t              queMgrIndex;
    /** Max entries per list buffer page */
    uint16_t              maxPageEntries;
    /** Number of timer ticks to delay interrupt */
    uint16_t              timerLoadCount;
    /** Interrupt pacing mode. Specifies when the interrupt should be trigerred */
    Qmss_AccPacingMode  interruptPacingMode;
    /** List entry size. Specifies the size of each data entry */
    Qmss_AccEntrySize   listEntrySize;
    /** List count Mode. The number of entries in the list */
    Qmss_AccCountMode   listCountMode;
    /** Queue mode. Moitor single or multiple queues */
    Qmss_AccQueueMode   multiQueueMode;
} Qmss_AccCmdCfg;

/**
@}
*/

/* Internal data structure to write accumulator command */
typedef struct
{
    /* Channel, Command */
    uint32_t              word0;
    /* Queue enable mask */
    uint32_t              word1;
    /* List buffer physical address */
    uint32_t              word2;
    /* Queue manager and base index, Maximum page entries */
    uint32_t              word3;
    /* Timer Load Count, Configuration */
    uint32_t              word4;
} Qmss_AccCmd;


/***********************************qmss_qm.h*******************************/
/**
 * @brief descriptor configuration structure
 */
typedef struct
{
    uint32_t          memRegion;
    /** Number of descriptors that should be allocated */
    uint32_t          descNum;
    /** Queue where the descriptor is stored. If QueueNum is set to QMSS_PARAM_NOT_SPECIFIED then the next
     * available queue of type Qmss_QueueType will be allocated */
    int32_t           destQueueNum;
    /** If QueueNum is set to QMSS_PARAM_NOT_SPECIFIED then the next available queue of type
     * Qmss_QueueType will be allocated */
    Qmss_QueueType  queueType;
}Qmss_DescCfg;

/**
 * @brief Memory region configuration information structure
 */
typedef struct
{
    /** The base address of descriptor region. Note the
     * descriptor Base address must be specified in ascending memory order
     * */
    uint32_t          *descBase;
    /** Size of each descriptor in the memory region. Must be a multiple of 16 */
    uint32_t          descSize;
    /** Number of descriptors in the memory region.
     * Must be a minimum of 32.
     * Must be 2^(5 or greater)
     * Maximum supported value 2^20
     * */
    uint32_t          descNum;

    /** Memory Region corresponding to the descriptor.
     * At init time this field must have a valid memory region
     * index (0 to Maximum number of memory regions supported).
     *
     * At runtime this field is used to either
     *      * set to Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED, in this case the LLD
     *      * will decide which memory region to use.
     *      * OR
     *      * specify the descriptor memory region, must be a valid memory
     *      * region index (0 to Maximum number of memory regions supported).
     */
    Qmss_MemRegion  memRegion;
    /** Flag control whether the descriptors are managed
     * by LLD or by the caller allocating descriptor memory */
    Qmss_ManageDesc manageDescFlag;
    /** Used to leave holes by configuring dummy regions which can be later
     * configured with actual values. Must be calculated and a correct startIndex must be
     * specified if memRegion value is valid (0 to Maximum number of memory regions supported). */
    uint32_t          startIndex;
} Qmss_MemRegInfo;

/**
 * @brief Memory region configuration information structure of all memory regions
 */
typedef struct
{
    /** Descriptor information for each CPDMA passed during cppi_Init */
    Qmss_MemRegInfo memRegInfo[QMSS_MAX_MEM_REGIONS];
    /** Current descriptor count. Sum of descriptors in all memory regions */
    uint32_t          currDescCnt;
} Qmss_MemRegCfg;

/**
 * @brief QMSS PDSP firmware download information structure
 */
typedef struct
{
    /** ID of the PDSP to download this firmware to */
    Qmss_PdspId     pdspId;
    /** Pointer to the firmware image, If the firmware pointer is NULL, LLD will not
     * download the firmware */
    void            *firmware;
    /** Size of firmware in bytes */
    uint32_t          size;
}Qmss_PdspCfg;

/**
 * @brief QMSS configuration structure
 */
typedef struct
{
    /** Base address of Linking RAM 0. LLD will configure linking RAM0 address to internal linking RAM
     * address if a value of zero is specified. */
    uint32_t          linkingRAM0Base;
    /** Linking RAM 0 Size. LLD will configure linking RAM0 size to maximum internal linking RAM
     * size if a value of zero is specified*/
    uint32_t          linkingRAM0Size;
    /** Base address of Linking RAM 1. Depends on RAM 0 Size and total number of
     * descriptors. If linkingRAM1Base is zero then linkingRAM0Size must be large
     * enough to store all descriptors in the system */
    uint32_t          linkingRAM1Base;
    /** Maximum number of descriptors in the system. Should be equal to less than
     * the RAM0+RAM1 size */
    uint32_t          maxDescNum;
    /** PDSP firmware to download. If the firmware pointer is NULL, LLD will not download the firmware */
    Qmss_PdspCfg    pdspFirmware[QMSS_MAX_PDSP];
}Qmss_InitCfg;

/**
 * @brief QMSS Global configuration structure definition
 */
typedef struct
{
    /** Maximum number of queue Managers */
    uint32_t                                maxQueMgr;
    /** Maximum number of queues */
    uint32_t                                maxQue;

    /** Queue start index and maximum number of queues of each queue type */
    Qmss_QueueNumRange                      maxQueueNum[25];

    /** Base address for the CPDMA overlay registers */

    /** QM Global Config registers */
    CSL_Qm_configRegs                       *qmConfigReg;
    /** QM Descriptor Config registers */
    CSL_Qm_descriptor_region_configRegs     *qmDescReg;
    /** QM queue Management registers, accessed via CFG port */
    CSL_Qm_queue_managementRegs             *qmQueMgmtReg;
    /** QM queue Management Proxy registers, accessed via CFG port */
    CSL_Qm_queue_managementRegs             *qmQueMgmtProxyReg;
    /** QM queue status registers */
    CSL_Qm_queue_status_configRegs          *qmQueStatReg;
    /** QM INTD registers */
    CSL_Qm_intdRegs                         *qmQueIntdReg;
    /** QM PDSP command register */
    volatile uint32_t                       *qmPdspCmdReg[QMSS_MAX_PDSP];
    /** QM PDSP control register */
    CSL_PdspRegs                            *qmPdspCtrlReg[QMSS_MAX_PDSP];
    /** QM PDSP IRAM register */
    volatile uint32_t                       *qmPdspIRamReg[QMSS_MAX_PDSP];
    /** QM Status RAM */
    CSL_Qm_Queue_Status                     *qmStatusRAM;
    /** QM Linking RAM register */
    volatile uint32_t                       *qmLinkingRAMReg;
    /** QM McDMA register */
    CSL_McdmaRegs                           *qmMcDMAReg;
    /** QM Timer16 register */
    CSL_Cp_timer16Regs                      *qmTimer16Reg[2];
    /** QM queue Management registers, accessed via DMA port */
    CSL_Qm_queue_managementRegs             *qmQueMgmtDataReg;
    /** QM queue Management Proxy registers, accessed via DMA port */
    CSL_Qm_queue_managementRegs             *qmQueMgmtProxyDataReg;

}Qmss_GlobalConfigParams;



/* QMSS Global object definition. */
typedef struct
{
    /** Store the configuration structure passed during Qmss_init */
    Qmss_GlobalConfigParams             Radar_qmssGblCfgParams;
    /** Store the intialization structure passed during Qmss_init */
    Qmss_InitCfg                        initCfg;
    /** Current Memory regions configuration */
    Qmss_MemRegInfo                     memRegInfo[QMSS_MAX_MEM_REGIONS];
    /** General purpose source queue handles */
    int32_t                             descQueue[QMSS_MAX_MEM_REGIONS];
    /** Current descriptor count */
    uint32_t                            currDescCnt;
}Qmss_GlobalObj;
/**
 * @brief Queue handle
 */
typedef int32_t   Qmss_QueueHnd;

/**
 * @brief QMSS return result
 */
typedef int32_t   Qmss_Result;

/**
 * @brief Handle used in the "Fast Push" set of APIs
 */
typedef uint32_t* Qmss_QueuePushHnd;


/****************************qmss_mgmt.h**********************************/
/**********************************************************************
 ************************** Externs *********************************
 **********************************************************************/

/* QMSS Local object */
extern Qmss_GlobalConfigParams  Radar_qmssLObj;



/**********************************qmss_osal.h***************************/
/**********************************************************************
 ************************* Extern Declarations ************************
 **********************************************************************/
void Radar_Osal_qmssBeginMemAccess (void *ptr, uint32_t size);
void Radar_Osal_qmssEndMemAccess (void *ptr, uint32_t size);


/* QMSS LLD OSAL Critical section and cache coherency APIs are used without redefinition in TEST application API */
#define Radar_Qmss_osalCsEnter            Radar_Osal_qmssCsEnter
#define Radar_Qmss_osalCsExit             Radar_Osal_qmssCsExit


/**
 * @brief   The macro is used by the QMSS LLD to indicate that a block
 * of memory is about to be accessed. If the memory block is cached then
 * this indicates that the application would need to ensure that the cache
 * is updated with the data from the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssBeginMemAccess (void *ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Radar_Qmss_osalBeginMemAccess     Radar_Osal_qmssBeginMemAccess

/**
 * @brief   The macro is used by the QMSS LLD to indicate that the block of
 * memory has finished being accessed. If the memory block is cached then the
 * application would need to ensure that the contents of the cache are updated
 * immediately to the actual memory.
 *
 * <b> Prototype: </b>
 *  The following is the C prototype for the expected OSAL API.
 *
 *  @verbatim
       void Osal_qmssEndMemAccess (void *ptr, uint32_t size)
    @endverbatim
 *
 *  <b> Parameter </b>
 *  @n  Address of memory block.
 *  @n  Size of memory block.
 *
 *  <b> Return Value </b>
 *  @n  Not applicable.
 */
#define Radar_Qmss_osalEndMemAccess       Radar_Osal_qmssEndMemAccess




/***********************************qmss_mgmt.h*************************************/
void Radar_Qmss_queuePush (Qmss_QueueHnd hnd, void *descAddr, uint32_t packetSize, uint32_t descSize, Qmss_Location location);

void Radar_Qmss_queuePushDescSize (Qmss_QueueHnd hnd, void *descAddr, uint32_t descSize);

void* Radar_Qmss_queuePop (Qmss_QueueHnd hnd);

void Radar_Qmss_queuePopDescSize (Qmss_QueueHnd hnd, void **descAddr, uint32_t *packetSize);

Qmss_QueuePushHnd Radar_Qmss_getQueuePushHandle (Qmss_QueueHnd hnd);

void Radar_Qmss_queueDivert (Qmss_QueueHnd srcQnum, Qmss_QueueHnd dstQnum, Qmss_Location location);

void Radar_Qmss_queueEmpty (Qmss_QueueHnd hnd);

uint32_t Radar_Qmss_getQueueEntryCount (Qmss_QueueHnd hnd);

uint32_t Radar_Qmss_getQueueByteCount (Qmss_QueueHnd hnd);

uint32_t Radar_Qmss_getQueuePacketSize (Qmss_QueueHnd hnd);

/**
@}
*/


/**********************************qmss_qos.h**********************************************/
/* Exported APIs */
Qmss_Result Radar_Qmss_setQosQueueBase (uint32_t queueNum);
Qmss_Result Radar_Qmss_getQosQueueBase (uint32_t *queueNum);
Qmss_Result Radar_Qmss_configureQosTimer (uint32_t timerConstant);
Qmss_Result Radar_Qmss_enableQosCluster (uint32_t clusterIndex);
Qmss_Result Radar_Qmss_disableQosCluster (uint32_t clusterIndex);
Qmss_Result Radar_Qmss_configureQosQueue (uint32_t queIndex, Qmss_QosQueueCfg *cfg);
Qmss_Result Radar_Qmss_configureQosCluster (uint32_t clusterIndex, Qmss_QosClusterCfg *cfg);
Qmss_Result Radar_Qmss_getQosQueueForwardPktStats (uint32_t queueIndex);
Qmss_Result Radar_Qmss_getQosQueueDroppedPktStats (uint32_t queueIndex);
Qmss_Result Radar_Qmss_resetQosQueueStats (uint32_t queueIndex);

/**********************************qmss_qm.h**********************************************/
/* Exported functions */
Qmss_Result Radar_Qmss_init (Qmss_InitCfg *initCfg, Qmss_GlobalConfigParams *lqmssGblCfgParams);
Qmss_Result Radar_Qmss_start (void);
Qmss_Result Radar_Qmss_getMemoryRegionCfg (Qmss_MemRegCfg *memRegInfo);
Qmss_Result Radar_Qmss_insertMemoryRegion (Qmss_MemRegInfo *memRegCfg);
Qmss_QueueHnd Radar_Qmss_initDescriptor (Qmss_DescCfg *descCfg, uint32_t *numAllocated);
Qmss_QueueHnd Radar_Qmss_queueOpen (Qmss_QueueType queType, int32_t queNum, uint8_t *isAllocated);
Qmss_QueueHnd Radar_Qmss_queueOpenInRange (uint32_t startQueNum, uint32_t endQueNum, uint8_t *isAllocated);
Qmss_Result Radar_Qmss_queueClose (Qmss_QueueHnd hnd);
uint32_t Radar_Qmss_getQueueThreshold (Qmss_QueueHnd hnd);
void Radar_Qmss_setQueueThreshold (Qmss_QueueHnd hnd, uint16_t hilo, uint8_t threshold);
uint32_t Radar_Qmss_getStarvationCount (Qmss_QueueHnd hnd);
uint16_t Radar_Qmss_getQueueThresholdStatus (Qmss_QueueHnd hnd);
Qmss_Queue Radar_Qmss_getQueueNumber (Qmss_QueueHnd hnd);
Qmss_QueueHnd Radar_Qmss_getQueueHandle (Qmss_Queue queue);
uint32_t Radar_Qmss_getMemRegDescSize (uint32_t memRegion);
Qmss_Result Radar_Qmss_downloadFirmware (Qmss_PdspId pdspId, void *image, uint32_t size);
Qmss_Result Radar_Qmss_setEoiVector (Qmss_IntdInterruptType type, uint8_t interruptNum);
Qmss_Result Radar_Qmss_ackInterrupt (uint8_t interruptNum, uint8_t value);
uint32_t Radar_Qmss_getVersion (void);
const unsigned char* Radar_Qmss_getVersionStr (void);

/**********************************qmss_pvt.h**********************************************/
int32_t Radar_Qmss_getMemRegQueueHandle(uint32_t memRegion);

/**********************************qmss_acc.h**********************************************/
/* Exported functions */
Qmss_Result Radar_Qmss_programAccumulator (Qmss_PdspId pdspId, Qmss_AccCmdCfg *cfg);
Qmss_Result Radar_Qmss_disableAccumulator (Qmss_PdspId pdspId, uint8_t channel);
Qmss_Result Radar_Qmss_configureAccTimer (Qmss_PdspId pdspId, uint16_t timerConstant);
Qmss_Result Radar_Qmss_programReclaimQueue (Qmss_PdspId pdspId, Qmss_QueueHnd hnd);
Qmss_Result Radar_Qmss_programDiversionQueue (Qmss_PdspId pdspId, Qmss_QueueHnd divQ,
                                               Qmss_QueueHnd divCompletionQ);

void Radar_Qmss_queuePushDesc (Qmss_QueueHnd hnd, void *descAddr);
Uint32 l2_global_address (Uint32 addr);

Ptr Radar_Osal_qmssCsEnter (void);
void Radar_Osal_qmssCsExit (Ptr CsHandle);
void WaitInit(void);

#ifdef __cplusplus
}
#endif

#endif /* QMSS_DRV_H_ */

