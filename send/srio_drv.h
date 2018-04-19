/**
 *   @file  srio_drv.h
 *
 *   @brief
 *      Header file for the SRIO Driver. The file exposes the data structures
 *      and exported API which are available for use by the driver users.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009 Texas Instruments, Inc.
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

/** @defgroup SRIO_LLD_API SRIO LLD
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *  The SRIO Low Level driver provides a well defined standard interface
 *  which allows application developers to send and receive messages via
 *  the Serial RapidIO peripheral. 
 */

#ifndef __SRIO_DRV_H__
#define __SRIO_DRV_H__

#include"c6678.h"

#include <string.h>

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>
#include <ti/csl/csl_srioAuxPhyLayer.h>

/* CSL Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>

/* CSL BootCfg Module */
#include <ti/csl/csl_bootcfg.h>
#include <ti/csl/csl_bootcfgAux.h>

/* CSL PSC Module */
#include <ti/csl/csl_pscAux.h>


/* CPPI/QMSS Include */
#include <qmss_drv.h>
#include <cppi_drv.h>

#include <ti/drv/srio/sriover.h>
/**
@defgroup SRIO_LLD_SYMBOL  SRIO LLD Symbols Defined
@ingroup SRIO_LLD_API
*/
/**
@defgroup SRIO_LLD_FUNCTION  SRIO LLD Functions
@ingroup SRIO_LLD_API
*/
/**
@defgroup SRIO_LLD_DATASTRUCT  SRIO LLD Data Structures
@ingroup SRIO_LLD_API
*/
/**
@defgroup SRIO_OSAL_API  SRIO OSAL Functions
@ingroup SRIO_LLD_API
*/
/**
@defgroup SRIO_DEVICE_API  SRIO Device Functions
@ingroup SRIO_LLD_API
*/

/** @addtogroup SRIO_LLD_SYMBOL
 @{ */

/**
 * @brief   This defines the maximum depth of the SRIO socket receive queues.
 * This is the MAX number of packets which can be enqueued in the SRIO socket
 * receive queue before packets get dropped.
 */
#define DEFAULT_SRIO_MAX_PENDING_PACKETS       5

/**
 * @brief This is the maximum number of Type9 and Type11 sockets that can 
 * be created. This is limited by the hardware and since Type9 and Type11 
 * share the same QID_MAP register both of them are limited to this 
 */
#define NUM_SRIO_TYPE9_TYPE11_SOCKETS          64

/**
 * @brief This is the maximum number of DIO sockets that can be created. This 
 * limit is specified by the number of LSU blocks. 
 */
#define NUM_DIO_SOCKETS                        8

/**
 * @brief This is a macro provided for the application and should be used if a DOORBELL
 * is to be transmitted.
 *
 * @sa
 *  Srio_sockSend 
 */
#define SRIO_SET_DBELL_INFO(DBELL_REG, DBELL_BIT)  (CSL_FMKR(31, 16, (DBELL_REG)) | \
                                                   CSL_FMKR(15,  0, (DBELL_BIT)))

/**
 * @brief This is a macro provided for the application and should be used to get
 * the doorbell register information once data is received on the DIO socket.
 *
 * @sa
 *  Srio_sockRecv
 */
#define SRIO_GET_DBELL_REG(DBELL_INFO)  (CSL_FEXTR((DBELL_INFO), 31, 16))

/**
 * @brief This is a macro provided for the application and should be used to get
 * the doorbell bit information once data is received on the DIO socket.
 *
 * @sa
 *  Srio_sockRecv
 */
#define SRIO_GET_DBELL_BIT(DBELL_INFO)  (CSL_FEXTR((DBELL_INFO), 15,  0))

/**
 * @brief Specifies to use hardware assigned Letter to which the message will be send. 
 * The hardware will check for an unused context starting with letter = 0 (A), 
 * and incrementing to letter = 3 (D). The first unused context with that letter 
 * will be used. If there are no context available with any letters then the 
 * packet is stopped and re-arbitrated in the TXU until one does get available.
 */
#define SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE     4

/**
@}
*/

/** @addtogroup SRIO_LLD_DATASTRUCT
 @{ */

/** 
 * @brief 
 *  This is the handle which is used for sending and receiving data 
 */
typedef void*   Srio_SockHandle;

/** 
 * @brief 
 *  This is the handle which is used accessing the SRIO driver.
 */
typedef void*   Srio_DrvHandle;

/** 
 * @brief 
 *  This is the handle which encapsulates the SRIO driver buffer information.
 */
typedef void*   Srio_DrvBuffer;

/******************************listlib.h**********************************/
/**************************************************************************
 * STRUCTURE -  Srio_ListNode
 **************************************************************************
 *	The structure defines a LIST NODE structure that contains links to the
 *	previous and next element in the list.
 **************************************************************************/
typedef struct Srio_ListNode
{
	void*	p_next;		/* Pointer to the next element in the list. */
    void*   p_prev;     /* Pointer to the prev element in the list. */
}Srio_ListNode;


/** 
 * @brief 
 *  The structure describes the SRIO Driver Managed Receive Configuration
 *
 * @details
 *  This specifies the Receive configuration which is a part of the Driver 
 *  Managed configuration. 
 */
typedef struct Srio_DrvManagedRxCfg
{
    /**
     * @brief   This is the memory region to be used for allocating 
     * the receive buffer descriptors. 
     */
    Qmss_MemRegion      rxMemRegion;

    /**
     * @brief   This is the Number of receive buffers and descriptors 
     * which are to be passed to the SRIO receive queues.
     */
    uint32_t            numRxBuffers;

    /**
     * @brief   For Normal sockets this is the Maximum data size which can be
     * received.
     */
    uint32_t            rxMTU;

    /**
     * @brief   This is the receive completion queue in which the received
     * SRIO packets will be placed. If interrupt support is required then the
     * application would need to ensure that they select a correct high priority
     * queue & accumulator channel. If no interrupts are required then this can
     * be any queue. 
     */
    Qmss_QueueHnd       rxCompletionQueue;

    /**
     * @brief   Boolean flag which indicates if the SRIO driver should configure
     * the accumulator with the provided accumulator configuration or not. If this
     * parameter is set to 0 the accumulator configuration below is ignored. This
     * in turn implies that there is no interrupt support and the application would
     * need to poll.
     */
    uint16_t            bIsAccumlatorCfgValid;

    /**
     * @brief   Accumulator Configuration is exposed to the application which 
     * allows the application to determine the parameters for programming 
     * the accumulator. 
     */
    Qmss_AccCmdCfg      accCfg;
}Srio_DrvManagedRxCfg;

/** 
 * @brief 
 *  The structure describes the SRIO Driver Managed Transmit Configuration
 *
 * @details
 *  This specifies the Transmit configuration which is a part of the Driver 
 *  Managed configuration. 
 */
typedef struct Srio_DrvManagedTxCfg
{
    /**
     * @brief  The number of transmit buffers available to the driver instance
     * which can be used to transmit data.
     */
    uint32_t            numTxBuffers;

    /**
     * @brief   This is the memory region to be used for allocating the transmit 
     * buffer descriptors.
     */
    Qmss_MemRegion      txMemRegion;

    /**
     * @brief   For Normal sockets this is the Maximum data size which can be
     * transmitted.
     */
    uint32_t            txMTU;
}Srio_DrvManagedTxCfg;

/** 
 * @brief 
 *  The structure describes the SRIO Driver Managed Configuration
 *
 * @details
 *  The configuration exposes encapsulates majority of the low level configuration
 *  from the application. The configuration works only with NORMAL sockets. 
 */
typedef struct Srio_DrvManagedCfg
{
    /**
     * @brief   Boolean flag which indicates if the SRIO driver instance being
     * configured should support receive or not? If this flag is set to 0 the
     * Receive configuration below is ignored which implies that the the 
     * receive flow is not configured and thus the driver instance and any 
     * associated sockets opened on this instance are no longer capable of 
     * receiving any data.
     */
    uint16_t                bIsRxCfgValid;

    /**
     * @brief   The receive configuration which determines the location of
     * the buffer descriptors, size, receive MTU etc.
     */
    Srio_DrvManagedRxCfg    rxCfg;

    /**
     * @brief   Boolean flag which indicates if the SRIO driver instance being
     * configured should support transmit or not? If this flag is set to 0 the
     * transmit configuration below is ignored which implies that the driver 
     * instance does not have any transmit buffers available and thus any call 
     * to send out data will fail. 
     */
    uint16_t                bIsTxCfgValid;    

    /**
     * @brief   The transmit configuration which determines the location of
     * the buffer descriptors, size, transmit MTU etc.
     */
    Srio_DrvManagedTxCfg    txCfg;
}Srio_DrvManagedCfg;

/** 
 * @brief
 *  The structure describes the application managed configuration
 *
 * @details
 *  In this configuration the entire low level configuration is exposed to the
 *  application. Applications can specify the CPPI Receive Flows, QMSS Accumulator
 *  configuration. This configuration works only with RAW sockets. 
 */
typedef struct Srio_AppManagedCfg
{
    /**
     * @brief   Boolean flag which indicates if the SRIO driver instance being
     * configured should support receive or not? If this flag is set to 0 the
     * Receive Flow configuration below is ignored which implies that the 
     * the receive flow is not configured and thus the driver instance and any 
     * associated sockets opened on this instance are no longer capable of 
     * receiving any data.
     */
    uint16_t            bIsRxFlowCfgValid;

    /**
     * @brief   The Receive Flow Configuration is exposed to the application. The 
     * Application specifies how flows need to be configured. This allows the 
     * applications complete control over the queues from where the buffer 
     * descriptors are removed when packets are received. 
     */
    Cppi_RxFlowCfg      rxFlowCfg;

    /**
     * @brief   Boolean flag which indicates if the SRIO driver should configure
     * the accumulator with the provided accumulator configuration or not. If this
     * parameter is set to 0 the accumulator configuration below is ignored.
     */
    uint16_t            bIsAccumlatorCfgValid;

    /**
     * @brief   Accumulator Configuration is exposed to the application which 
     * allows the application to determine the parameters for programming 
     * the accumulator. 
     */
    Qmss_AccCmdCfg      accCfg;

    /**
     * @brief   For RAW Sockets this is the receive cleanup API which needs to be 
     * provided by the application. This API is invoked by the driver to cleanup 
     * the buffer descriptor associated with the RAW socket. This parameter can 
     * be set to NULL if the application wishes to only operate on NORMAL sockets
     */
    void                (*rawRxFreeDrvBuffer)(Srio_DrvBuffer hDrvBuffer);

    /**
     * @brief   Indicates the queue number to be used for TX. Using this 
     * parameter same TX queue can be used for multiple SRIO driver instances.
     * This parameter needs to be set to either a valid queue number or 
     * QMSS_PARAM_NOT_SPECIFIED which indicates driver should allocate 
     * the next available queue.
     */
    int16_t             txQueueNum;

    /**
     * @brief   Receive Descriptor Size. This is required to 
     * invalidate cache on the receive side. If this is set to 
     * zero then invalidate will not take place e.g. in case of 
     * buffer descriptors that are allocated from L2 SARAM 
     * which don't need invalidation.
     */
    int32_t             rxDescSize;
}Srio_AppManagedCfg;

/**
 * @brief 
 *  Describes driver configuration.
 *
 * @details
 *  There are 2 types of configuration in the driver. Application Managed
 *  and Driver Managed. 
 */
typedef union Srio_DrvConfigType
{
    /**
     * @brief    This is the driver managed configuration.
     */
    Srio_DrvManagedCfg    drvManagedCfg;

    /**
     * @brief    This is the application managed configuration.
     */    
    Srio_AppManagedCfg    appManagedCfg;
}Srio_DrvConfigType;

/**
 * @brief 
 *  The structure describes the SRIO Driver Configuration 
 *
 * @details
 *  SRIO Driver users are expected to populate the driver configuration
 *  block and pass it to the driver during initialization.
 */
typedef struct Srio_DrvConfig
{
    /**
     * @brief   The SRIO driver can be configured to use either of the
     * following configurations:
     *  - Application Managed configuration
     *  - Driver Managed configuration
     *  This flag can be used to select which configuration is specified.
     */
    uint16_t            bAppManagedConfig;

    /**
     * @brief    Union structure for the driver configuration.
     */
    Srio_DrvConfigType  u;
}Srio_DrvConfig;


/** 
 * @brief 
 *  SRIO Driver Option Commands.
 *
 * @details
 *  These option commands are used for the get/set of various configuration
 *  parameters which exist in the driver.
 */
typedef enum Srio_Opt
{
    /**
     * @brief   This is the command which is used to get/set the MAX Pending
     * Packet limit for each socket. This command when used requires a 
     * 2 byte configuration data.
     */
    Srio_Opt_PENDING_PKT_COUNT       = 0x1,

    /**
     * @brief   This command is applicable only for DIO sockets and is used to
     * get the DIO socket last transfer completion code. If there is a pending 
     * transaction on the socket the function returns 0xFF else the function 
     * returns the last recorded completion code. The command uses a 1 byte 
     * configuration data to return the completion code. A value of 0 indicates 
     * transfer was complete with no errors. All other values indicate an error.
     */
    Srio_Opt_DIO_SOCK_COMP_CODE      = 0x2,

    /**
     * @brief   This command is applicable only for DIO sockets and is used 
     * register a DIO socket with a specific Doorbell register and Doorbell
     * bit. The mappings are maintained *only* on a core specific basis. 
     */
    Srio_Opt_REGISTER_DOORBELL       = 0x3,

    /**
     * @brief   This command is applicable only for DIO sockets and is used to
     * get the DIO socket transfer completion code. Note that this just returns 
     * the last recorded completion code in the socket data structure and doesn't 
     * check if transaction is pending or not. A typical use of this option would
     * be in the case where an ISR fills the completion code and application needs  
     * to know the status of completion code after ISR. The command uses a 1 byte 
     * configuration data to return the completion code. A value of 0 indicates 
     * transfer was complete with no errors. All other values indicate an error.
     */
    Srio_Opt_DIO_READ_SOCK_COMP_CODE = 0x4

}Srio_Opt;



/** 
 * @brief 
 *  RIO Transaction Type for Type2 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief NREAD Transaction
     */
    Srio_Ttype_Request_NREAD        = 4,

    /*
     * @brief ATOMIC Increment Transaction
     */
    Srio_Ttype_Request_ATOMIC_INC   = 12,

    /*
     * @brief ATOMIC Decrement Transaction
     */
    Srio_Ttype_Request_ATOMIC_DEC   = 13,

    /*
     * @brief ATOMIC Set Transaction
     */
    Srio_Ttype_Request_ATOMIC_SET   = 14,

    /*
     * @brief ATOMIC Clear Transaction
     */
    Srio_Ttype_Request_ATOMIC_CLR   = 15
}Srio_Ttype_Request;



/** 
 * @brief 
 *  RIO Transaction Type for Type6 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Streaming Write Transaction there is no Transaction Type.
     */
    Srio_Ttype_Swrite_DEFAULT            = 0
}Srio_Ttype_Swrite;

/** 
 * @brief 
 *  RIO Transaction Type for Type7 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Congestion Transaction there is no Transaction Type.
     */
    Srio_Ttype_Congestion_DEFAULT            = 0
}Srio_Ttype_Congestion;

/** 
 * @brief 
 *  RIO Transaction Type for Type8 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief Maintenance Read
     */
    Srio_Ttype_Maintenance_READ           = 0,

    /*
     * @brief Maintenance Write
     */
    Srio_Ttype_Maintenance_WRITE          = 1,

    /*
     * @brief Maintenance Read Response
     */
    Srio_Ttype_Maintenance_READR          = 2,

    /*
     * @brief Maintenance Write Response
     */
    Srio_Ttype_Maintenance_WRITER         = 3,

    /*
     * @brief Maintenance Port Write Response
     */
    Srio_Ttype_Maintenance_RORT_WRITE     = 4
}Srio_Ttype_Maintenance;

/** 
 * @brief 
 *  RIO Transaction Type for Type9 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Data Streaming there is no Transaction Type.
     */
    Srio_Ttype_Data_Streaming_DEFAULT      = 0
}Srio_Ttype_Data_Streaming;

/** 
 * @brief 
 *  RIO Transaction Type for Type10 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Doorbell there is no Transaction Type.
     */
    Srio_Ttype_Doorbell_DEFAULT            = 0
}Srio_Ttype_Doorbell;

/** 
 * @brief 
 *  RIO Transaction Type for Type11 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Message there is no Transaction Type.
     */
    Srio_Ttype_Message_DEFAULT            = 0
}Srio_Ttype_Message;

/** 
 * @brief 
 *  RIO Transaction Type for Type13 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief Response + Doorbell response
     */
    Srio_Ttype_Response_RESPONSE            = 0,

    /*
     * @brief Message Response
     */
    Srio_Ttype_Response_MSG_RESPONSE        = 1,

    /*
     * @brief Response with payload
     */
    Srio_Ttype_Response_RESPONSE_PAYLOAD    = 8
}Srio_Ttype_Response;

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/
int32_t Srio_init (void);
Srio_DrvHandle Srio_start (Srio_DrvConfig* ptr_cfg);
Srio_SockHandle Srio_sockOpen (Srio_DrvHandle hSrio, Srio_SocketType type,uint16_t isBlocking);

int32_t Srio_sockBind         (Srio_SockHandle srioSock, Srio_SockBindAddrInfo* ptr_addrInfo);
int32_t Srio_sockBind_TYPE11  (Srio_SockHandle srioSock, Srio_SockBindAddrInfo* ptr_addrInfo);
int32_t Srio_sockBind_TYPE9   (Srio_SockHandle srioSock, Srio_SockBindAddrInfo* ptr_addrInfo);
int32_t Srio_sockBind_DIO     (Srio_SockHandle srioSock, Srio_SockBindAddrInfo* ptr_addrInfo);

int32_t Srio_sockSend         (Srio_SockHandle srioSock, Srio_DrvBuffer hBuffer, uint32_t size, Srio_SockAddrInfo* to);
int32_t Srio_sockSend_TYPE11  (Srio_SockHandle srioSock, Srio_DrvBuffer hBuffer, uint32_t size, Srio_SockAddrInfo* to);
int32_t Srio_sockSend_TYPE9   (Srio_SockHandle srioSock, Srio_DrvBuffer hBuffer, uint32_t size, Srio_SockAddrInfo* to);
int32_t Srio_sockSend_DIO     (Srio_SockHandle srioSock, Srio_DrvBuffer hBuffer, uint32_t size, Srio_SockAddrInfo* to);

int32_t Srio_sockRecv         (Srio_SockHandle srioSock, Srio_DrvBuffer* hDrvBuffer,Srio_SockAddrInfo* from);
void Srio_freeRxDrvBuffer     (Srio_SockHandle srioSock, Srio_DrvBuffer hDrvBuffer);
int32_t Srio_setSockOpt       (Srio_SockHandle srioSock, Srio_Opt option,void* optval,int32_t optlen);
int32_t Srio_getSockOpt       (Srio_SockHandle srioSock, Srio_Opt option,void* optval,int32_t optlen);

int32_t Srio_sockClose        (Srio_SockHandle srioSock);
int32_t Srio_sockClose_TYPE11 (Srio_SockHandle srioSock);
int32_t Srio_sockClose_TYPE9  (Srio_SockHandle srioSock);
int32_t Srio_sockClose_DIO    (Srio_SockHandle srioSock);

Srio_DrvBuffer Srio_allocTransmitBuffer (Srio_DrvHandle hSrioDrv, uint8_t** ptrData, uint32_t* bufferLen);
void Srio_freeTransmitBuffer            (Srio_DrvHandle hSrioDrv, Srio_DrvBuffer hDrvBuffer);
void Srio_dioCompletionIsr   (Srio_DrvHandle hSrioDrv, uint8_t intDstDoorbell[]);//uint8_t intDstDoorbell[]
void Srio_dioTxCompletionIsr (Srio_DrvHandle hSrioDrv, CSL_SrioHandle hSrioCSL);
void Srio_rxCompletionIsr    (Srio_DrvHandle hSrioDrv);

uint32_t Srio_getVersion (void);
const unsigned char* Srio_getVersionStr (void);

/*********************************srio_osal.h********************************************/
/* Memory Allocation OSAL Definitions. */
void  biosFree (void* ptr, Uint32 numBytes);
void* Osal_DataBufferMalloc(uint32_t numBytes);
void  Osal_DataBufferFree(void* ptr, uint32_t numBytes);

/* Cache OSAL Definitions. */
void  Osal_CacheInvalidate(void* ptr, uint32_t size);
void  Osal_CacheWriteback(void* ptr, uint32_t size);
void  Osal_srioEndDescriptorAccess (Srio_DrvHandle drvHandle,void* ptr, uint32_t descSize);

/* SRIO OSAL Memory Allocation API for the Data Buffer */
#define Srio_osalDataBufferMalloc                   Osal_DataBufferMalloc
#define Srio_osalDataBufferFree                     Osal_DataBufferFree

/* SRIO OSAL Logging API is mapped directly to an XDC Runtime API */
#define Srio_osalLog                                printf

/* CACHE API: */
#define Srio_osalBeginMemAccess                     Osal_CacheInvalidate
#define Srio_osalEndMemAccess                       Osal_CacheWriteback
#define Srio_osalEndDescriptorAccess                Osal_srioEndDescriptorAccess

/******************************listlib.h**********************************/
/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/
void Srio_listAdd (Srio_ListNode **ptr_list, Srio_ListNode *ptr_node);
Srio_ListNode* Srio_listRemove (Srio_ListNode **ptr_list);
Srio_ListNode* Srio_listGetHead (Srio_ListNode** ptr_list);
Srio_ListNode* Srio_listGetNext (Srio_ListNode* ptr_list);
int32_t Srio_listRemoveNode (Srio_ListNode** ptr_list, Srio_ListNode* ptr_remove);
void Srio_listCat (Srio_ListNode **ptr_dst, Srio_ListNode **ptr_src);

Int32 enable_srio(void);
int32_t SrioDevice_init_Ku (int Rate,int SrioId[4],int MultiId[3],uint8_t infomode,uint8_t DataswapMode,uint8_t PhysicalMode,Bool BoardLinkCheckFlag);
int32_t SrioDevice_init (int Rate,int SrioId,int MultiId[3],uint8_t infomode,uint8_t DataswapMode);
int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize);

Ptr Osal_srioCsEnter (void);

void Osal_srioCsExit (Ptr CsHandle);

Bool Srio_Bind_DIO(void);
Srio_DrvHandle Srio_Navigator_Init(Uint8 VectID,void  interruptISR());
Srio_SockHandle Srio_Bind_9(Srio_SockBindAddrInfo   bindInfo);
Srio_SockHandle Srio_Bind_11(Srio_SockBindAddrInfo   bindInfo);

#endif /* __SRIO_DRV_H__ */

