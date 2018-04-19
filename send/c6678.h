/*
 * c6678.h
 *
 *  Created on: 2012-3-31
 *      Author: wangjie
 */

#ifndef C6678_H_
#define C6678_H_

/********************************************************************
 * 包含的库文件
 *******************************************************************/
#include "types.h"
#include "csl_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "csl_chip.h"
#include "csl_chipAux.h"
#include "csl_semAux.h"
#include "cslr_device.h"
#include "cslr_psc.h"
#include "csl_psc.h"
#include "cslr_emif16.h"
#include "csl_emif4f.h"
#include "csl_emif4fAux.h"
#include "csl_bootcfg.h"

#include "cslr_i2c.h"

#include "csl_cpsw.h"
#include "csl_cpsgmii.h"
#include "csl_cpsgmiiAux.h"
#include "csl_mdio.h"
#include "csl_mdioAux.h"

#include "cslr_uart.h"

#include "csl_gpioAux.h"

#include "csl_pscAux.h"
#include "csl_bootcfg.h"
#include "csl_bootcfgAux.h"

#include "cslr_spi.h"

#include "csl_pllcAux.h"
#include "csl_xmcAux.h"
#include "csl_msmcAux.h"

#include "csl_tsc.h"


/* PCIE LLD include */
#include <ti/drv/pcie/pcie.h>

/*cache*/
#include <csl_cache.h>
#include <csl_cacheAux.h>

/* CSL SRIO Functional Layer */
#include <csl_srio.h>
#include <csl_srioAux.h>
#include <csl_srioAuxPhyLayer.h>


#include <ti/drv/hyplnk/hyplnk.h>

/*intc*/
#include <csl_intc.h>
#include <csl_intcAux.h>
#include <tistdtypes.h>
#include <csl_cpIntcAux.h>
#include <cerrno>
/* CSL CPINTC Include Files. */
#include<csl_cpIntc.h>

/*sem2*/
#include <soc.h>
#include <csl_semAux.h>

/*edma3*/
#include <csl_edma3.h>
#include <csl_edma3Aux.h>

/*timer*/
#include <csl_tmr.h>

/*IPC*/
#include<csl_ipcAux.h>
#include"interrupt_nest.h"

/********************************************************************
 * 可修改的全局变量
 *******************************************************************/
#define DEVICE_IS_DEVICE1
#define DEVICE1 1//主控端
#define DEVICE0 0//被控端

#ifdef DEVICE_IS_DEVICE1
#define DEVICE DEVICE1
#else
#define DEVICE DEVICE0
#endif

/********************************************************************
 * PLL部分
 *******************************************************************/
/* Default PLL PLLM value (100/1*(20/2)) = 1.0GHz) */
#define  PLATFORM_PLL1_PLLM_val (20)//通过该参数可设置核的时钟频率(20:1.0GHz)(25:1.25GHz)

/********************************************************************
 * DDR3部分
 *******************************************************************/
// ddr3 clock = ddr3clockin*(PLL PLLM value+1)/pll PLLD value/2
/*  (100MHz*(9+1)/1/2）*2= 1.0GHz)*/

#define PLLM_DDR3 15		//通过配置该参数可设置DDR3跑的时钟频率
//C6678_4DSP_DDR_V1.1板卡目前支持5（600MHz）和6（700MHz），其他暂不支持，如需要请提需求		//qqb	2017_01_11
//C6678_4DSP_DDR_V2.0板卡目前支持5（600MHz）、6（700MHz）、7（800MHz）和9（1000MHz），其他暂不支持，如需要请提需求		//qqb	2017_01_11
//YingYanH201板卡_1600版目前支持7（800MHz）和9（1000MHz），其他暂不支持，如需要请提需求

#define DDR3_TYPE_1BANK_2Gb  				0   //只有一个BANK，只有正面贴了4片DDR,    ddr是美光1333芯片,每片DDR为2Gb,位宽为16bit，v1.0板卡,，目前芯片和板卡均停产，只有老板卡会用
#define DDR3_TYPE_1BANK_4Gb  				1   //只有一个BANK，只有正面贴了4片DDR,    ddr是美光1333芯片,每片DDR为4Gb,位宽为16bit，v1.0板卡,目前芯片和板卡均停产，只有老板卡会用
#define DDR3_TYPE_2BANK_2Gb  				2	//有两个BANK，     正反面均贴了DDR,共8片，ddr是美光1333芯片,每片2Gb,位宽为16bit，v1.0板卡,目前芯片和板卡均停产，只有老板卡会用
#define DDR3_TYPE_2BANK_4Gb  				3	//有两个BANK，     正反面均贴了DDR,共8片，ddr是美光1333芯片,每片4Gb,位宽为16bit，v1.0板卡,目前芯片和板卡均停产，只有老板卡会用
#define DDR3_TYPE_2BANK_8Gb  				4	//有两个BANK，     正反面均贴了DDR,共8片，ddr是美光1600芯片,每片8Gb,位宽为8bit(x2)，v2.0板卡,目前只应用在FengBaoV303项目中
#define DDR3_TYPE_2BANK_4Gb_1600  			5	//有两个BANK，     正反面均贴了DDR,共8片, ddr是美光1600芯片,每片4Gb,位宽为16bit，v1.0板卡_1600版，通用款
#define DDR3_TYPE_1BANK_2Gb_1600  			6	//只有一个BANK，只有正面贴了4片DDR,    ddr是美光1600芯片,每片2Gb,位宽为16bit，只应用在YingYanH201项目板卡_1600版
#define DDR3_TYPE_2BANK_2Gb_1600  			7   //有两个BANK，     正反面均贴了DDR,共8片, ddr是美光1600芯片,每片2Gb,位宽为16bit，v1.0板卡_1600版  
#define DDR3_TYPE_2BANK_8Gb_1600_PM 		8   //有两个BANK，     正反面均贴了DDR,共8片, ddr是ProMOS的芯片,每片4Gb,位宽为16bit，v1.0板卡_1600版，通用试用版
#define DDR3_TYPE_1BANK_4Gb_2P_1600 		9	//只有一个BANK，只有正面贴了2片DDR,共2片,ddr是美光1600芯片,每片4Gb,位宽为16bit，IO_DZS_Fiber_VPX C6678板卡
#define DDR3_TYPE_2BANK_2Gb_1P_1600 		10	//有两个BANK，     正反面均贴了1片DDR,共2片,ddr是美光1600芯片,每片2Gb,位宽为16bit，IO_DZS_Fiber_VPX C6678板卡
#define DDR3_TYPE_1BANK_4Gb_4P_1600 		11	//只有一个BANK，只有正面贴了4片DDR,共4片,ddr是美光1600芯片,每片4Gb,位宽为16bit，2DSP_3U_VPX C6678板卡
#define DDR3_TYPE_2BANK_4Gb_2P_1600			12  //有两个BANK,正反面均贴了1片DDR,共2片,ddr是美光1600芯片,每片2Gb,位宽为16bit
#define DDR3_TYPE_2BANK_8Gb_8P_1600_LRM		13	//有两个BANK，     正反面均贴了DDR,共8片，ddr是美光1600芯片,每片8Gb,位宽为16bit
#define	DDR3_TYPE_2BANK_8Gb_8P_1600_3U		14	//有两个BANK，     正反面均贴了DDR,共8片，ddr是美光1600芯片,每片8Gb,位宽为16bit，3U处理板1600MTs
#define	DDR3_TYPE_1BANK_8Gb_2P_1600_3U_SW	15	//只有一个BANK， 只有正面贴了DDR,共2片，ddr是美光1600芯片,每片8Gb,位宽为16bit，3U交换板1600MTs
#define	DDR3_TYPE_1BANK_512M16_802_2DSP		16	//只有一个BANK，只有正面贴了4片DDR,共4片,ddr是美光1600芯片,每片8Gb,位宽为16bit
#define	DDR3_TYPE_1BANK_512M16_802_3DSP		17	//只有一个BANK，只有正面贴了4片DDR,共4片,ddr是美光1600芯片,每片8Gb,位宽为16bit


#define DDR3_TYPE   DDR3_TYPE_2BANK_8Gb_8P_1600_LRM //根据自己板子的情况来选择DDR3的类型

/********************************************************************
 * i2c_eeprom部分
 *******************************************************************/

typedef	uint16_t I2C_RET;

// Bus release
enum {
  I2C_RELEASE_BUS,
  I2C_DO_NOT_RELEASE_BUS
};

/********************************************************************
 * spi_norflash部分
 *******************************************************************/
typedef enum {
    RADAR_DEVICE_NAND,
    /**<NAND Flash*/
    RADAR_DEVICE_NOR,
    /**<NOR Flash*/
    RADAR_DEVICE_EEPROM,
    /**<NOR Flash*/
    RADAR_DEVICE_MAX
    /**<End of devices*/
} RADAR_DEVICE_TYPE;

typedef uint32_t RADAR_DEVHANDLE;

typedef struct {
    int32_t manufacturer_id;		/**<manufacturer ID*/
    int32_t device_id;				/**<Manufacturers device ID*/
    RADAR_DEVICE_TYPE  type;		/**<Type of device */
    int32_t width;					/**<Width in bits*/
    int32_t block_count;			/**<Total blocks. First block starts at 0. */
    int32_t page_count;				/**<Page count per block*/
    int32_t page_size;				/**<Number of bytes in a page including spare area*/
    int32_t spare_size;				/**<Spare area size in bytes*/
    RADAR_DEVHANDLE handle;		/**<Handle to the block device as returned by Open. Handle is Opaque, do not interpret or modify */
    int32_t	bboffset;				/**<Offset into spare area to check for a bad block */
	uint32_t column;				/**<Column for a NAND device */
	uint32_t flags;					/**<Flags is a copy of the flags that were used to open the device */
	void	*internal;				/**<Do not use. Used internally by the platform library */
    uint8_t *bblist;				/** <Bad Block list or NULL if device does not support one  */
} RADAR_DEVICE_info;

/* ------------------------------------------------------------------------ *
 *  NOR Error Status                                                        *
 * ------------------------------------------------------------------------ */
#define NOR_STATUS              Uint32          /* NOR status error code */
#define NOR_EFAIL               ((NOR_STATUS)-1)  /* General failure code */
#define NOR_EOK                 0               /* General success code */

static struct platform_mcb_t {
    uint32_t	frequency;
    int32_t  	board_version;
    int32_t		mastercore;
} platform_mcb = {0, 0, 0};

/********************************************************************
 * timecounter部分
 *******************************************************************/
//typedef unsigned long long int Uint64;
/********************************************************************
 * gpio部分
 *******************************************************************/
typedef	enum _GpioDirection
{
	GPIO_OUT = 0,
	GPIO_IN
}GpioDirection;

/********************************************************************
 * emif16_nandflash部分
 *******************************************************************/
typedef	struct _NAND_ADDR
{
    uint32_t uiColumnAddr;
    uint32_t uiPageAddr;
    uint32_t uiBlockAddr;
} NAND_ADDR;

/********************************************************************
 * pcie部分
 *******************************************************************/
/* Global config variable that controls
   the PCIe mode. It is global so it can be poked
   from CCS. It should be set either to EP or RC. */
//注意：RC应该先启动pcie_init()等待连接，否则连不上
#ifdef DEVICE_IS_DEVICE1
#define PCIEMODE   pcie_EP_MODE//pcie_RC_MODE////配置成RC或者EP
#else
#define PCIEMODE   pcie_RC_MODE//pcie_RC_MODE////配置成RC或者EP
#endif

/********************************************************************
 *srio部分
 *******************************************************************/
#define SRIO_SEND 0
#define SRIO_RECV 1
#ifdef DEVICE_IS_DEVICE1
#define SRIOMODE   SRIO_SEND//SRIO_RECV////配置成发送端或者接受端
#define SRIO_SEND_ID
#else
#define SRIOMODE   SRIO_RECV//SRIO_RECV////配置成发送端或者接受端
#endif

#ifdef SRIO_SEND_ID
#define DEVICE_ID1_16BIT    0xBEEF//0xBEEF
#define DEVICE_ID1_8BIT     0x86//0xCD//0xABAB//66//
#define DEVICE_ID2_16BIT    0x4560//0x4560//
#define DEVICE_ID2_8BIT     0x8d//0XF1//0xAB//0xCDCD//99//
#define DEVICE_ID3_16BIT    0x1234
#define DEVICE_ID3_8BIT     0xf1
#define DEVICE_ID4_16BIT    0x5678
#define DEVICE_ID4_8BIT     0x56

#define DEVICE0_TYPE9_COS    	 0
#define DEVICE0_TYPE9_STREAMID    5

#define DEVICE1_TYPE9_COS    	 3
#define DEVICE1_TYPE9_STREAMID    10

#else
/* These are the device identifiers used in the test Application */
#define DEVICE_ID1_16BIT    0x4560
#define DEVICE_ID1_8BIT     0x8d//0xCD
#define DEVICE_ID2_16BIT    0xBEEF
#define DEVICE_ID2_8BIT     0x86//0xAB
#define DEVICE_ID3_16BIT    0x9abc
#define DEVICE_ID3_8BIT     0x12
#define DEVICE_ID4_16BIT    0xdef0
#define DEVICE_ID4_8BIT     0x56

#define DEVICE0_TYPE9_COS    	 3
#define DEVICE0_TYPE9_STREAMID    10

#define DEVICE1_TYPE9_COS    	 0
#define DEVICE1_TYPE9_STREAMID    5
#endif

#define SRIO_2p50	2
#define SRIO_1p25	1
#define SRIO_5p00	4
#define SRIO_3p15	3
#define	SRIO_RATE	SRIO_5p00

/**
 * @brief
 *  Enumeration Type which describes the socket.
 *
 * @details
 *  There can be different kinds of SRIO sockets which can be used to
 *  send and receive data. These enumerations define the supported
 *  types.
 */
typedef enum Srio_SocketType
{
    /**
     * @brief   Type9 Sockets
     */
    Srio_SocketType_TYPE9       = 0x1,

    /**
     * @brief   Type9 RAW Sockets
     */
    Srio_SocketType_RAW_TYPE9   = 0x2,

    /**
     * @brief   Type11 Sockets
     */
    Srio_SocketType_TYPE11      = 0x3,

    /**
     * @brief   Type11 RAW Sockets
     */
    Srio_SocketType_RAW_TYPE11  = 0x4,

    /**
     * @brief   Direct IO Socket.
     */
    Srio_SocketType_DIO         = 0x5,
	Srio_SocketType_NULL         = 0
}Srio_SocketType;
/**
 * @brief
 *  SRIO Socket Type11 Address Information.
 *
 * @details
 *  The structure describes the address information required to send &
 *  receive a Type11 message over a Type11 socket. This is populated to
 *  indicate the remote endpoint where the message has to be sent.
 */
typedef struct sSrio_Type11AddrInfo
{
    /**
     * @brief   Transport Type; 16 bit or 8 bit identifiers.
     */
    uint16_t      tt;

    /**
     * @brief   This is the 8 bit or 16 bit SRIO identifier
     */
    uint16_t      id;

    /**
     * @brief   Letter Identifier
     */
    uint16_t      letter;

    /**
     * @brief   Mailbox number
     */
    uint16_t      mbox;
}Srio_Type11AddrInfo;

/**
 * @brief
 *  SRIO Socket Type9 Information.
 *
 * @details
 *  The structure describes the address information required to send &
 *  receive a Type11 message over a Type11 socket. This is populated to
 *  indicate the remote endpoint where the message has to be sent.
 */
typedef struct sSrio_Type9AddrInfo
{
    /**
     * @brief   Transport Type; 16 bit or 8 bit identifiers.
     */
    uint16_t        tt;

    /**
     * @brief   This is the 8 bit or 16 bit SRIO identifier
     */
    uint16_t        id;

    /**
     * @brief   Class of service
     */
    uint8_t         cos;

    /**
     * @brief   Stream identifier.
     */
    uint16_t        streamId;
}Srio_Type9AddrInfo;

/**
 * @brief
 *  SRIO Socket DIO Information
 *
 * @details
 *  The structure describes the DIO request which has to be sent to the remote
 *  endpoint.
 */
typedef struct sSrio_DioAddrInfo
{
    /**
     * @brief   32b Ext Address Fields 锟絇acket Types 2,5, and 6
     */
    uint32_t  rapidIOMSB;

    /**
     * @brief   32b Address 锟絇acket Types 2,5, and 6
     */
    uint32_t  rapidIOLSB;

    /*
     * @brief RapidIO destinationID field specifying target device
     */
    uint16_t  dstID;

    /*
     * @brief Transaction Type
     */
    uint8_t   ttype;

    /*
     * @brief FType for packets
     */
    uint8_t   ftype;
}Srio_DioAddrInfo;
/**
 * @brief
 *  SRIO Socket Address Information
 *
 * @details
 *  The structure describes the various address socket type address characteristics
 *  which are used while sending & receiving data over the specific SRIO socket type.
 */
typedef union sSrio_SockAddrInfo
{
    Srio_Type11AddrInfo         type11;
    Srio_Type9AddrInfo          type9;
    Srio_DioAddrInfo            dio;
}Srio_SockAddrInfo;
/**
 * @brief
 *  RIO Format Type
 *
 * @details
 *  This enumberation describes the SRIO Packet Ftype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief Type 2 Packet Format (Request Class)
     */
    Srio_Ftype_REQUEST        = 2,

    /*
     * @brief Type 5 Packet Format (Write Class)
     */
    Srio_Ftype_WRITE          = 5,

    /*
     * @brief Type 6 Packet Format (Streaming Write Class)
     */
    Srio_Ftype_SWRITE         = 6,

    /*
     * @brief Type 7 Packet Format (Congestion Class)
     */
    Srio_Ftype_CONGESTION     = 7,

    /*
     * @brief Type 8 Packet Format (Maintenance)
     */
    Srio_Ftype_MAINTENANCE   = 8,

    /*
     * @brief Type 9 Packet Format (Data Streaming)
     */
    Srio_Ftype_DATA_STREAMING = 9,

    /*
     * @brief Type 10 Packet Format (Doorbell)
     */
    Srio_Ftype_DOORBELL       = 10,

    /*
     * @brief Type 11 Packet Format (Doorbell)
     */
    Srio_Ftype_MESSAGE        = 11,

    /*
     * @brief Type 13 Packet Format Response)
     */
    Srio_Ftype_RESPONSE       = 13
} Srio_Ftype;
/**
 * @brief
 *  RIO Transaction Type for Type5 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief NWRITE Transaction
     */
    Srio_Ttype_Write_NWRITE             = 4,

    /*
     * @brief NWRITE_R Transaction
     */
    Srio_Ttype_Write_NWRITE_R           = 5,

    /*
     * @brief Atomic Test and Set Transaction
     */
    Srio_Ttype_Write_ATOMIC_TEST_SET    = 14
}Srio_Ttype_Write;

/**
 * @brief
 *  SRIO Socket Type11 Binding Information.
 *
 * @details
 *  The structure describes the address information required for binding
 *  a Type11 socket. This includes information which describes the Type11
 *  endpoint characteristics and is used to describe the local characteristics
 *  of the endpoint.
 */
typedef struct sSrio_Type11BindAddrInfo
{
    /**
     * @brief   Transport Type; 16 bit or 8 bit identifiers.
     */
    uint16_t      tt;

    /**
     * @brief   This is the 8 bit or 16 bit SRIO identifier
     */
    uint16_t      id;

    /**
     * @brief   Letter Identifier
     */
    uint16_t      letter;

    /**
     * @brief   Mailbox number
     */
    uint16_t      mbox;

    /**
     * @brief   Segmentation Mapping Set to 0 for single segment and 1 for multi segment.
     */
    uint16_t      segMap;
}Srio_Type11BindAddrInfo;


/**
 * @brief
 *  SRIO Socket Type9 Binding Information.
 *
 * @details
 *  The structure describes the address information required for binding
 *  a Type9 socket. This includes information which describes the Type9
 *  endpoint characteristics and is used to describe the local characteristics
 *  of the endpoint.
 */
typedef Srio_Type9AddrInfo  Srio_Type9BindAddrInfo;

/**
 * @brief
 *  SRIO Socket DIO Binding Information.
 *
 * @details
 *  The structure describes the address information required for binding
 *  a DIO socket. This includes information which describes the DIO
 *  endpoint characteristics and is used to describe the local characteristics
 *  of the endpoint.
 */
typedef struct sSrio_DioBindAddrInfo
{
    /**
     * @brief   Indicates if doorbell information needs to be sent out or not.
     */
    uint8_t   doorbellValid;

    /**
     * @brief   CPU controlled request bit used for interrupt generation
     */
    uint8_t   intrRequest;

    /**
     * @brief   Supress good interrupt.
     */
    uint8_t   supInt;

    /**
     * @brief   RapidIO xambs field specifying extended address
     */
    uint8_t   xambs;

    /**
     * @brief   Packet Priority
     */
    uint8_t   priority;

    /*
     * @brief Indicates the output port number for the packet to be transmitted
     */
    uint8_t   outPortID;

    /*
     * @brief RapidIO tt field specifying 8 or 16bit DeviceIDs
     */
    uint8_t   idSize;

    /*
     * @brief Defines which sourceID register to be used for this transaction
     */
    uint8_t   srcIDMap;

    /*
     * @brief RapidIO hop_count field specified for Type 8 Maintenance packets
     */
    uint8_t   hopCount;

    /*
     * @brief RapidIO doorbell info: This is the doorbell register which is to be written
     * There are 4 registers so this should have a value from 0 - 3.
     */
    uint8_t  doorbellReg;

    /*
     * @brief RapidIO doorbell info: This is the doorbell bit which is to be set. There
     * are 16 doorbell bits so this should have a value from 0-15.
     */
    uint8_t  doorbellBit;
}Srio_DioBindAddrInfo;
/**
 * @brief
 *  SRIO Socket Bind Information
 *
 * @details
 *  There are different types of sockets and this union explains the different
 *  types of binding information required.
 */
typedef union sSrio_SockBindAddrInfo
{
    Srio_Type11BindAddrInfo         type11;
    Srio_Type9BindAddrInfo          type9;
    Srio_DioBindAddrInfo            dio;
}Srio_SockBindAddrInfo;

typedef struct srio_enumeration
{
	int C6678_Num;
	int C6678_EnumID[256];
	int FPGA_Num;
	int FPGA_EnumID[256];

}Srio_Enum;
/********************************************************************
 * edma3部分
 *******************************************************************/
typedef struct
{
     Uint32 Addr;                       //传输原/目的地址
     int Acnt;                          //A方向传输的长度（取值范围 为1 - 65535）
     int Bcnt;                          //B方向传输的长度（取值范围 为1 - 65535）
     int Ccnt;                          //C方向的传输的长度（取值范围 为1 - 65535）
     int DAStride;                       //A方向的步进长度（取值范围为-32768 - +32767）
     int DBStride;                       //B方向的步进长度（取值范围为-32768 - +32767）
}DmaTranParam;

typedef struct
{
	Uint32 sAddr;                 //传输的源地址
	Uint32 dAddr;                 //传输的目的地址,目的与源均为RAM；
	Uint32 High;                  //数据矩阵纵向的长度
	Uint32 Width;                 //数据矩阵横向的长度
	Uint8 Unit;                   //数据元素的单位：=0：字; =1：双字; =x:(x-1)字
	Bool Mod;                     //旋转的方向：=0:顺时针;=1:逆时针
	Bool IntrEn;                  // =1：DMA中断使能，不等待DMA传输结束；=0：无DMA中断，等待DMA传输结束再退出;
		                      // =其他值：无DMA中断，不等待DMA传输结束。
}DmaSortTranParam;

typedef struct
{
	Uint32 ChannelCtrlNum;       //DMA的控制器号
	Uint32 ChannelNum;           //DMA的通道号
	Uint8 TCNum;           	 //DMA的通道号对应的TC号
	Int8 ShadowRegion;         //该DMA通道对应的ShadowRegion
	Uint32 FirstParamNum;        //第一个PARAM号
	Uint32 ExternalAddr;         //外部地址
	Uint32 PingAddr;             //Ping buffer 地址
	Uint32 PongAddr;             //Pong buffer 地址
	Uint32 BufferSize_KB;        //buffer 的长度（单位：KB)
	Bool Mod;                    //发送/接收
	Bool DstIsFifo;              //=1：目的为fifo；=0：源为fifo；=其他值：目的与源均为RAM；

}DmaPingPongInitParam;

typedef struct
{
	Uint32 sAddr;               //传输的源地址
	Uint32 dAddr;               //传输的目的地址
	Uint32 High;                //原数据矩阵纵向的长度
	Uint32 Width;               //原数据矩阵横向的长度
	Uint32 Hoffset;             //目标数据矩阵纵向的偏移量
	Uint32 Woffset;             //目标数据矩阵横向的偏移量
	Uint32 Hlength;             //目标数据矩阵纵向的长度
	Uint32 Wlength;             //目标数据矩阵纵向的长度
	Bool Unit;                  //数据元素的单位：=0：字，=1：双字
	Bool DstIsFifo;             //=1：目的为fifo；=0：源为fifo；=其他值：目的与源均为RAM；
	Bool IntrEn;                // =1：DMA中断使能，不等待DMA传输结束；=0：无DMA中断，等待DMA传输结束再退出;
		                    //=其他值：无DMA中断，不等待DMA传输结束。
}DmaSubFrameExtractTranParam;
typedef struct {
	Uint32 infoL;
	Uint32 infoH;
}Uint64edmaTccInfo;

typedef struct
{
	Uint32 ChannelNum;           //DMA的通道号
	Uint32 ParamNum;        	 //PARAM号
	Uint8 TCNum;        	 	 //该DMA通道对应的TC号
	Int8 ShadowRegion;         //该DMA通道对应的ShadowRegion
}DmaChainParam;
/********************************************************************
 * timer部分
 *******************************************************************/
typedef struct {
	Uint32 CntLo;
	Uint32 CntHi;
}Uint64Cnt;
typedef struct {
	Uint32 PrdLo;
	Uint32 PrdHi;
}Uint64Prd;
typedef struct {
	Uint8 PrescaleCnt;
	Uint8 PrescalePrd;
} Uint4Prescale;

/********************************************************************
 * Hyperlink部分
 *******************************************************************/

#define HYPERLINK_ADDR    	 0x40000000//本地hyperlink的地址空间

/*****************************************************************************
 * Select a serial rate
 *****************************************************************************/
#define hyplnk_EXAMPLE_SERRATE_01p250 1//1.25gbps
#define hyplnk_EXAMPLE_SERRATE_03p125 2//3.125gbps
#define hyplnk_EXAMPLE_SERRATE_06p250 3//6.25gbps
#define hyplnk_EXAMPLE_SERRATE_07p500 4//7.5gbps
#define hyplnk_EXAMPLE_SERRATE_10p000 5//10.00gbps
#define hyplnk_EXAMPLE_SERRATE_12p500 6//12.5gbps
#define hyplnk_EXAMPLE_SERRATE_05p000 7//5.00gbps

#define	HYPLNK_RATE	hyplnk_EXAMPLE_SERRATE_07p500//输出速率可改

/*****************************************************************************
 * Match the reference clock on your board
 *
 * The value hyplnk_EXAMPLE_REFCLK_USE_PLATCFG uses the reference clock
 * defined through hyplnk_EXAMPLE_HYPLNK_PLATCFG_REF_CLK_MHZ
 * in hyplnkPlatCfg.h.
 *
 * hyplnk_EXAMPLE_REFCLK_USE_PLATCFG can be commented out and the specific
 * value specified below.
 *****************************************************************************/
//#define hyplnk_EXAMPLE_REFCLK_USE_PLATCFG
#define hyplnk_EXAMPLE_REFCLK_156p25//输入固定，不能改
//#define hyplnk_EXAMPLE_REFCLK_250p00
//#define hyplnk_EXAMPLE_REFCLK_312p50

typedef struct
{
	Uint32 SegSize;       		 //映射的每一个区域的大小
	/************************
	 * 值：区域大小
	 * 0：0x400000（4MB）
	 * 1：0x800000（8MB）
	 * 2：0x1000000（16MB）
	 * 3：0x2000000（32MB）
	 * 4：0x4000000（64MB）
	 * 5：0x8000000（128MB）
	 * 6：0x10000000（256MB）
	 ************************/
	Uint32 SegSel[64];               //每一个区域首地址(必须是区域大小的整倍数)
}HyplnkAddrMapParam;

/********************************************************************
 * navigator部分
 *******************************************************************/
#define NAVIGATOR_SRIO_FLOWID_NUM0	9
#define NAVIGATOR_SRIO_FLOWID_NUM1	10
#define NAVIGATOR_SRIO_FLOWID_NUM2	11

/********************************************************************
 * sem2部分(该部分不能改)
 *******************************************************************/
#define CPPI_HW_SEM             	1
#define QMSS_HW_SEM             	2
#define SRIO_HW_SEM             	3
#define RADAR_SPI_HW_SEM			4
#define SPI_SW_SEM					5
#define I2C_SW_SEM					6
#define EMIF16_SW_SEM				7
#define DDR_SW_SEM					8
#define PCIE_SW_SEM					9
#define HYPERLINK_SW_SEM			10
#define SRIO_SW_SEM					11
#define IPC_SW_SEM                  12

#define SEM_MULTICORE_SYN           20



/********************************************************************
 * intc部分（该部分不能改）
 *******************************************************************/
//中断向量号

//#define  HWINT_SRIO_MSG_RX			7
//#define  HWINT_SRIO_DOORBELL		9
//#define  HWINT_SRIO_OVER			8

//给每个核发送的中断对应的核级事件号
#define  CIC0_OUT2_OR_CIC1_OUT2_EVTID	 62
#define  CIC0_OUT3_OR_CIC1_OUT3_EVTID	 63
#define  CIC0_OUT4_OR_CIC1_OUT4_EVTID	 92

//通道对应核级事件号(所有核都有的)
#define  CIC0_OUT0_OR_CIC1_OUT0_EVTID	 102
#define  CIC0_OUT1_OR_CIC1_OUT1_EVTID	 103
#define  CIC0_OUT8_OR_CIC1_OUT8_EVTID	 104
#define  CIC0_OUT9_OR_CIC1_OUT9_EVTID	 105
#define  CIC0_OUT16_OR_CIC1_OUT16_EVTID	 106
#define  CIC0_OUT17_OR_CIC1_OUT17_EVTID	 107
#define  CIC0_OUT24_OR_CIC1_OUT24_EVTID	 108
#define  CIC0_OUT25_OR_CIC1_OUT25_EVTID	 109

//二级事件通道号
#define  EDMA3_CONTROLLER_NUM0_CHANNEL_EVTID	CIC0_OUT0_OR_CIC1_OUT0_EVTID
#define  EDMA3_CONTROLLER_NUM0_CHANNELID		0
#define  EDMA3_CONTROLLER_NUM1_CHANNEL_EVTID	CIC0_OUT1_OR_CIC1_OUT1_EVTID
#define  EDMA3_CONTROLLER_NUM1_CHANNELID		1
#define  EDMA3_CONTROLLER_NUM2_CHANNEL_EVTID	CIC0_OUT8_OR_CIC1_OUT8_EVTID
#define  EDMA3_CONTROLLER_NUM2_CHANNELID		8
#define  SRIO_OVER_CHANNEL_EVTID				CIC0_OUT9_OR_CIC1_OUT9_EVTID
#define  SRIO_OVER_CHANNELID					9
#define  HYPER_INT_CHANNEL_EVTID				CIC0_OUT16_OR_CIC1_OUT16_EVTID
#define  HYPER_INT_CHANNELID					16

typedef struct {
	unsigned int irp_value;
	unsigned int nrp_value;
	unsigned int pgie_value;
	unsigned int itsr_value;
}InterrputRegInfo;

/********************************************************************
 * other部分
 *******************************************************************/
typedef struct {
	Uint32 Low32bit;
	Uint8  High4bit;
}Phy36bit;

/********************************************************************
	函数声明

	所在文件:   drive_version.c
	作者:		qqb

	函数名:
	  	  char *drive_version()
	函数说明:
	  	  本函数是返回驱动库版本信息
	参数说明:
	  	  无
	返回值:
	 	 无
	备注:
*********************************************************************/
char *drive_version();

/********************************************************************
	文件说明:	以下这部分函数是pll.c中函数的声明
						pll的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   pll.c
	作者:		wj

	函数名:
	  Bool C6678_Pll_Init(unsigned int Pllm);
	函数说明:
	  本函数是初始化pll
	参数说明:
	  Pllm:
	  	  PLATFORM_PLL1_PLLM_val(具体值见c6678.h)
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
Bool C6678_Pll_Init(unsigned int Pll1Pllm);
/********************************************************************
	函数声明

	所在文件:   pll.c
	作者:		wj

	函数名:
	  void C6678_Pll_Delay(uint32_t num);
	函数说明:
	  本函数是delay一段时间
	参数说明:
	  num：delay多少个10ns
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
void C6678_Pll_Delay(uint32_t num);
/********************************************************************
	文件说明:	以下这部分函数是ddr.c中函数的声明
						ddr的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   ddr.c
	作者:		wj

	函数名:
	  Bool C6678_Ddr3_Init(int Ddr3Pllm,int Ddr3Type);
	函数说明:
	  本函数是初始化ddr
	参数说明:
	  Ddr3Pllm:
	  	  PLLM_DDR3(具体值见c6678.h)
	  Ddr3Type:
		  DDR3_TYPE(具体值见c6678.h)
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
Bool C6678_Ddr3_Init(int Ddr3Pllm,int Ddr3Type);
/********************************************************************
	函数声明

	所在文件:   ddr.c
	作者:		wj

	函数名:
	  Bool C6678_Ddr3_MapSelect(Uint32 MapIndex);
	函数说明:
	  本函数是将内存的0x80000000到0xffffffff的2GB内存空间映射到外挂的DDR3的不同的区域
	参数说明:
	  MapIndex:外挂DDR3的区域号(0~3)目前外挂ddr3的大小最大为8GB
	         MapIndex值                  内存地址                                                外挂DDR3的区域地址
	  	        0          0x80000000到0xffffffff      0x000000000到0x07fffffff
	  	        1          0x80000000到0xffffffff      0x080000000到0x0ffffffff
	  	        2          0x80000000到0xffffffff      0x100000000到0x17fffffff
	  	        3          0x80000000到0xffffffff      0x180000000到0x1ffffffff
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
Bool C6678_Ddr3_MapSelect(Uint32 MapIndex);
/********************************************************************
	文件说明:	以下这部分函数是timecounter.c中函数的声明
						timecounter的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   timecounter.c
	作者:		wj

	函数名:
	  void C6678_TimeCounter_Enable(void);
	函数说明:
	  本函数是使时钟计时开始
	参数说明:
	  无
	返回值:
	  无
	备注: 无
*********************************************************************/
void C6678_TimeCounter_Enable(void);

/********************************************************************
	函数声明

	所在文件:   timecounter.c
	作者:		wj

	函数名:
	  Uint32 C6678_TimeCounter_GetHighVal(void);
	函数说明:
	  本函数是得到时钟计时高位
	参数说明:
	  无
	返回值:
	  当时的时钟周期高32bit
	备注: 无
*********************************************************************/
Uint32 C6678_TimeCounter_GetHighVal(void);

/********************************************************************
	函数声明

	所在文件:   timecounter.c
	作者:		wj

	函数名:
	  Uint32 C6678_TimeCounter_GetLowVal(void);
	函数说明:
	  本函数是得到时钟计时低位
	参数说明:
	  无
	返回值:
	  当时的时钟周期低32bit
	备注: 无
*********************************************************************/
Uint32 C6678_TimeCounter_GetLowVal(void);

/********************************************************************
	函数声明

	所在文件:   timecounter.c
	作者:		wj

	函数名:
	  void C6678_TimeCounter_Delaycycles(uint32_t usecs);
	函数说明:
	  本函数是delay几个周期
	参数说明:
	  usecs：周期数
	返回值:
	  无
	备注: 本函数延时范围0~2e32个周期，如需延时更多周期，建议采用定时器延时
*********************************************************************/
void C6678_TimeCounter_Delaycycles(uint32_t usecs);

/********************************************************************
	文件说明:	以下这部分函数是gpio.c中函数的声明
						gpio的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  void C6678_Gpio_Init(void)
	函数说明:
	  本函数是初始化gpio
	参数说明:
	  无
	返回值:
	 无
	备注: 无
*********************************************************************/
void C6678_Gpio_Init(void);
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_SetDirection(Uint8 uiNumber, GpioDirection direction)
	函数说明:
	  本函数是配置GPIO端口的方向
	参数说明:
	  uiNumber：端口号（0-15）
	  direction：GPIO_OUT or GPIO_IN
	返回值:
	 是否配置成功
	 TRUE:成功
	 FALSE：失败
	备注: 无
*********************************************************************/
Bool C6678_Gpio_SetDirection( Uint8 uiNumber, GpioDirection direction );
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_SetOutput(Uint8 uiNumber)
	函数说明:
	  本函数是配置GPIO端口的为1
	参数说明:
	  uiNumber：端口号（0-15）
	返回值:
	 是否配置成功
	 TRUE:成功
	 FALSE：失败
	备注:调用该函数前应先把该端口设为输出端口（GPIO_OUT）
*********************************************************************/
Bool C6678_Gpio_SetOutput( Uint8 uiNumber);
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_ClearOutput(Uint8 uiNumber)
	函数说明:
	  本函数是配置GPIO端口的为0
	参数说明:
	  uiNumber：端口号（0-15）
	返回值:
	 是否配置成功
	 TRUE:成功
	 FALSE：失败
	备注:调用该函数前应先把该端口设为输出端口（GPIO_OUT）
*********************************************************************/
Bool C6678_Gpio_ClearOutput( Uint8 uiNumber);
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  uint32_t C6678_Gpio_ReadInput(Uint8 uiNumber)
	函数说明:
	  本函数是读取GPIO端口的状态
	参数说明:
	  uiNumber：端口号（0-15）
	返回值:
	 1：GPIO端口状态为1
	 0：GPIO端口状态为0
	 2：错误。
	备注:调用该函数前应先把该端口设为输入端口（GPIO_IN）
*********************************************************************/
uint32_t C6678_Gpio_ReadInput( Uint8 uiNumber );
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  void C6678_Gpio_EnableGlobalInterrupt(void)
	函数说明:
	  本函数是使能对CPU的GPIO中断
	参数说明:
	  无
	返回值:
	 无
	备注:无
*********************************************************************/
void C6678_Gpio_EnableGlobalInterrupt( void );
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  void C6678_Gpio_DisableGlobalInterrupt(void)
	函数说明:
	  本函数是不使能对CPU的GPIO中断
	参数说明:
	  无
	返回值:
	  无
	备注:无
*********************************************************************/
void C6678_Gpio_DisableGlobalInterrupt( void );
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_SetRisingEdgeInterrupt( Uint8 uiNumber )
	函数说明:
	  本函数是设置为检测GPIO上升沿触发中断
	参数说明:
	  uiNumber：端口号（0-15）
	返回值:
	  TRUE:成功
	  FALSE：失败
	备注:
		调用该函数前应先把该端口设为输入端口（GPIO_IN）
*********************************************************************/
Bool C6678_Gpio_SetRisingEdgeInterrupt( Uint8 uiNumber );
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_ClearRisingEdgeInterrupt( Uint8 uiNumber )
	函数说明:
	  本函数是清除设置检测GPIO下降沿触发中断
	参数说明:
	  uiNumber：端口号（0-15）
	返回值:
	  TRUE:成功
	  FALSE：失败
	备注:
		调用该函数前应先把该端口设为输入端口（GPIO_IN）
*********************************************************************/
Bool C6678_Gpio_ClearRisingEdgeInterrupt( Uint8 uiNumber );
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_SetFallingEdgeInterrupt( Uint8 uiNumber )
	函数说明:
	  本函数是设置为检测GPIO下降沿触发中断
	参数说明:
	  uiNumber：端口号（0-15）
	返回值:
	  TRUE:成功
	  FALSE：失败
	备注:
		调用该函数前应先把该端口设为输入端口（GPIO_IN）
*********************************************************************/
Bool C6678_Gpio_SetFallingEdgeInterrupt( Uint8 uiNumber );
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_ClearFallingEdgeInterrupt( Uint8 uiNumber )
	函数说明:
	  本函数是清除设置检测GPIO下降沿触发中断
	参数说明:
	  uiNumber：端口号（0-15）
	返回值:
	  TRUE:成功
	  FALSE：失败
	备注:
		调用该函数前应先把该端口设为输入端口（GPIO_IN）
*********************************************************************/
Bool C6678_Gpio_ClearFallingEdgeInterrupt( Uint8 uiNumber );
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_Led_Light (Uint8 LedNum);
	函数说明:
	  本函数是使灯亮
	参数说明:
	  LedNum：灯对应的gpio管脚号（0-15）
	返回值:
	  TRUE:成功
	  FALSE：失败
	备注:需先调用C6678_Gpio_Init
	调用该函数前应先把该端口设为输出端口（GPIO_OUT）
	该函数的实现需要FPGA的配合，需要确定灯对应的gpio的具体管脚
*********************************************************************/
Bool C6678_Gpio_Led_Light (Uint8 LedNum);
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_Led_Dark (Uint8 LedNum);
	函数说明:
	  本函数是使某个灯灭
	参数说明:
	  LedNum：灯对应的gpio管脚号（0-15）
	返回值:
	  TRUE:成功
	  FALSE：失败
	备注:需先调用C6678_Gpio_Init
		调用该函数前应先把该端口设为输出端口（GPIO_OUT）
		该函数的实现需要FPGA的配合，需要确定灯对应的gpio的具体管脚
*********************************************************************/
Bool C6678_Gpio_Led_Dark (Uint8 LedNum);
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_Led_Toggle (Uint8 LedNum);
	函数说明:
	  本函数是使将GPIO端口电平翻转，灯如果亮着熄灭，如果灭着点亮。
	参数说明:
	  LedNum：灯对应的gpio管脚号（0-15）
	返回值:
	  TRUE:成功
	  FALSE：失败
	备注:需先调用C6678_Gpio_Init
		调用该函数前应先把该端口设为输出端口（GPIO_OUT）
		该函数的实现需要FPGA的配合，需要确定灯对应的gpio的具体管脚
*********************************************************************/
Bool C6678_Gpio_Led_Toggle (Uint8 LedNum);
/********************************************************************
	函数声明

	所在文件:   gpio.c
	作者:		wj

	函数名:
	  Bool C6678_Gpio_Led_Blink(Uint8 LedNum, Uint32 delay_n, Uint32 blink_num)
	函数说明:
	  本函数是闪灯。
	参数说明:
	  LedNum：灯对应的gpio管脚号（0-15）
	  delay_n:延时的时间
	  blink_num：闪灯的次数
	返回值:
	  TRUE:成功
	  FALSE：失败
	备注:需先调用C6678_Gpio_Init
		调用该函数前应先把该端口设为输出端口（GPIO_OUT）
		该函数的实现需要FPGA的配合，需要确定灯对应的gpio的具体管脚
*********************************************************************/
Bool C6678_Gpio_Led_Blink(Uint8 LedNum, Uint32 delay_n, Uint32 blink_num);
/********************************************************************
	文件说明:	以下这部分函数是i2c_eeprom.c中函数的声明
						i2c_eeprom的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   i2c_eeprom.c
	作者:		wj

	函数名:
	  void C6678_I2c_Eeprom_Init(void)
	函数说明:
	  本函数是初始化i2c_eeprom
	参数说明:
	  无
	返回值:
	  无
	备注: 无
*********************************************************************/
void C6678_I2c_Eeprom_Init(void);
/********************************************************************
	函数声明

	所在文件:   i2c_eeprom.c
	作者:		wj

	函数名:
	  I2C_RET C6678_I2c_Eeprom_Read ( uint32_t byte_addr, uint32_t uiNumBytes,
                        uint8_t *puiData, uint8_t uchEepromI2cAddress)
	函数说明:
	  本函数是从EEPROM读出一定数量的数据
	参数说明:
	  uchEepromI2cAddress ：EEPROM的i2c地址
	  puiData：读出数据存储的数组的指针
 	  uiNumBytes：读出数据的长度（byte为单位）
	  byte_addr：数据在EEPROM的基地址（byte为单位）
	返回值:
		0：I2C_RET_OK
		1：I2C_RET_LOST_ARB
		2：I2C_RET_NO_ACK
		3：I2C_RET_IDLE_TIMEOUT
		4：I2C_RET_BAD_REQUEST
		5：I2C_RET_CLOCK_STUCK_LOW
		6：I2C_RET_NULL_PTR_ERROR
		99：I2C_RET_GEN_ERROR
	备注:  eeprom总大小为0x10000B,使用中需注意不要超过该大小；The read consists of a master write of 2 bytes (forming a 16 bit address, msb transmitted first), followed by a master read of the
 input number of bytes.The bytes that are read are placed in puiData in big endian format
*********************************************************************/
I2C_RET C6678_I2c_Eeprom_Read ( uint32_t byte_addr, uint32_t uiNumBytes,
                        uint8_t *puiData, uint8_t uchEepromI2cAddress);
/********************************************************************
	函数声明

	所在文件:   i2c_eeprom.c
	作者:		wj

	函数名:
	  I2C_RET C6678_I2c_Eeprom_Write( uint32_t byte_addr, uint8_t uchEepromI2cAddress, uint8_t *puiData,
						uint32_t uiNumBytes, uint32_t uiEndBusState)
	函数说明:
	  本函数是向EEPROM写入一定数量的数据
	参数说明:
	  byte_addr:数据在EEPROM的基地址（byte为单位）
	  uchEepromI2cAddress ：EEPROM的i2c地址
	  puiData：读出数据存储的数组的指针
 	  uiNumBytes：读出数据的长度（byte为单位）
	  uiEndBusState：The state on which bus should be left
	返回值:
		0：I2C_RET_OK
		1：I2C_RET_LOST_ARB
		2：I2C_RET_NO_ACK
		3：I2C_RET_IDLE_TIMEOUT
		4：I2C_RET_BAD_REQUEST
		5：I2C_RET_CLOCK_STUCK_LOW
		6：I2C_RET_NULL_PTR_ERROR
		99：I2C_RET_GEN_ERROR
	备注:  eeprom总大小为0x10000B,使用中需注意不要超过该大小；The write consists of a master write of 2 bytes (forming a 16 bit address, msb transmitted first), followed by a master write of the
input number of bytes.The bytes that are write are placed in puiData in big endian format
*********************************************************************/
I2C_RET C6678_I2c_Eeprom_Write( uint32_t byte_addr, uint8_t uchEepromI2cAddress, uint8_t *puiData,
						uint32_t uiNumBytes, uint32_t uiEndBusState);
/********************************************************************
	文件说明:	以下这部分函数是spi_norflash.c中函数的声明
						spi_norflash的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   spi_norflash.c
	作者:		wj

	函数名:
	  NOR_STATUS C6678_Spi_Norflash_Init(void);
	函数说明:
	  本函数是初始化spi_norflash,配置dsp与NORFLASH之间的数据传输。
	参数说明:
	  无
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
NOR_STATUS C6678_Spi_Norflash_Init(void);
/********************************************************************
	函数声明

	所在文件:   spi_norflash.c
	作者:		wj

	函数名:
	  uint32_t C6678_Spi_Norflash_GetDetails(RADAR_DEVICE_info*   nor_info);
	函数说明:
	  本函数是得到NORFLASH设备的详细信息。
	参数说明:
	  nor_info：详细信息的结构体。
	返回值:
	  true:正常
	  false：失败
	备注: 无
*********************************************************************/
uint32_t C6678_Spi_Norflash_GetDetails(RADAR_DEVICE_info*   nor_info);
/********************************************************************
	函数声明

	所在文件:   spi_norflash.c
	作者:		wj

	函数名:
	  NOR_STATUS C6678_Spi_Norflash_Read(uint32_t addr,uint32_t len,uint8_t* buf)
	函数说明:
	  本函数是从NORFLASH中读出数据。
	参数说明:
	  p_device：设备信息结构体。
	  addr：起始地址（byte为单位）
	  len：数据长度（byte为单位）
	  buf：存储数据的数组的指针
	返回值:错误状态
	  SPI_EFAIL：        (SPI_STATUS)-1   错误
	  SPI_EOK  ：         0              成功
	备注: norflash共16MB,注意不要访问越界;
*********************************************************************/
NOR_STATUS C6678_Spi_Norflash_Read(uint32_t addr,uint32_t len,uint8_t* buf);
/********************************************************************
	函数声明

	所在文件:   spi_norflash.c
	作者:		wj

	函数名:
	  NOR_STATUS C6678_Spi_Norflash_Write(uint32_t addr,uint32_t len,uint8_t* buf)
	函数说明:
	  本函数是将数据写入NORFLASH中。
	参数说明:
	  p_device：设备信息结构体。
	  addr：起始地址（byte为单位）
	  len：数据长度（byte为单位）
	  buf：存储数据的数组的指针
	返回值:错误状态
	  SPI_EFAIL：        (SPI_STATUS)-1   错误
	  SPI_EOK  ：         0              成功
	备注: 写之前要先调用擦函数，否则出错；norflash共16MB,注意不要访问越界;
*********************************************************************/
NOR_STATUS C6678_Spi_Norflash_Write(uint32_t addr,uint32_t len,uint8_t* buf);
/********************************************************************
	函数声明

	所在文件:   spi_norflash.c
	作者:		wj

	函数名:
	  NOR_STATUS C6678_Spi_Norflash_Erase(uint32_t sector_number)
	函数说明:
	  本函数是将NORFLASH中的数据擦除。
	参数说明:
	  p_device：设备信息结构体。
	  sector_number：擦除的sector的number(范围为0到255,共256块)
	返回值:错误状态
	  SPI_EFAIL：        (SPI_STATUS)-1   错误
	  SPI_EOK  ：         0              成功
	备注: 该函数按块将NORFLASH中的数据擦除，总共256块(每块64KB)，共16MB。如果if sector_number = -1, do bulk erase.
*********************************************************************/
NOR_STATUS C6678_Spi_Norflash_Erase(uint32_t sector_number);

/********************************************************************
	文件说明:	以下这部分函数是spi_FPGA.c中函数的声明
						spi_FPGA的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   spi_FPGA.c
	作者:		dyx

	函数名:
	  Bool C6678_Spi_FPGA_Init(void);
	函数说明:
	  本函数是初始化spi_FPGA,配置dsp与FPGA之间的数据传输。
	参数说明:
	  无
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
Bool C6678_Spi_FPGA_Init(void);

/********************************************************************
	函数声明

           所在文件:   spi_FPGA.c
	作者:		dyx

	函数名:
	  Bool C6678_Spi_FPGA_Read(uint32_t addr,uint32_t len,uint16_t* buf)
	函数说明:
	  本函数是从FPGA中读出数据。
	参数说明:
	  addr：起始地址（16bit为单位）
	  len：数据长度（16bit为单位）(最大2048)
	  buf：存储数据的数组的指针
	返回值:错误状态
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
Bool C6678_Spi_FPGA_Read( uint32_t  addr,uint32_t  len,uint16_t* buf);

/********************************************************************
	函数声明

	所在文件:   spi_FPGA.c
	作者:		dyx

	函数名:
	  Bool C6678_Spi_FPGA_Write(uint32_t addr,uint32_t len,uint16_t* buf)
	函数说明:
	  本函数是将数据写入FPGA中。
	参数说明:
	  addr：起始地址（16bit为单位）
	  len：数据长度（16bit为单位）(最大2048)
	  buf：存储数据的数组的指针
	返回值:错误状态
	  true:初始化正常
	  false：初始化失败
	备注: 起始地址addr需要从16（十进制）开始，由于前16个地址被FPGA占用。
*********************************************************************/
Bool C6678_Spi_FPGA_Write(uint32_t addr,uint32_t len,uint16_t* buf);

/********************************************************************
	函数声明

           所在文件:   spi_FPGA.c
	作者:		dyx

	函数名:
	  uint16_t C6678_Spi_FPGA_GetDspId(void)
	函数说明:
	  本函数是获取该DSP的ID号。
	参数说明:
	  无
	返回值:DSP的ID号（0,1,2,3...）
	备注: 无
*********************************************************************/
uint16_t C6678_Spi_FPGA_GetDspId(void);

/********************************************************************
	函数声明

           所在文件:   spi_FPGA.c
	作者:		dyx

	函数名:
	  uint16_t C6678_Spi_FPGA_GetSrioId(void)
	函数说明:
	  本函数是获取该DSP的SRIO的ID。
	参数说明:
	  无
	返回值:SRIO的ID号
	备注: 无
*********************************************************************/
uint16_t C6678_Spi_FPGA_GetSrioId(void);

/********************************************************************
	函数声明

           所在文件:   spi_FPGA.c
	作者:		wyc

	函数名:
	  uint16_t C6678_Spi_FPGA_GetSlotId(void)
	函数说明:
	  本函数是获取该板的SLOTID。
	参数说明:
	  无
	返回值:
	正确：SLOTID号
	错误：0xffff
	备注: 无
*********************************************************************/
uint16_t C6678_Spi_FPGA_GetSlotId(void);
/********************************************************************
	文件说明:	以下这部分函数是emif16_nandflash.c中函数的声明
						emif16_nandflash的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   emif16_nandflash.c
	作者:		wj

	函数名:
	  RADAR_DEVICE_info *C6678_Emif16_Nandflash_Init(void)
	函数说明:
	  本函数是初始化emif16_nandflash
	参数说明:
	  无
	返回值:
	   RADAR_DEVICE_info信息
	备注: 无
*********************************************************************/
RADAR_DEVICE_info *C6678_Emif16_Nandflash_Init(void);
/********************************************************************
	函数声明

	所在文件:   emif16_nandflash.c
	作者:		wj

	函数名:
	  uint32_t C6678_Emif16_Nandflash_ReadPage(NAND_ADDR address, uint8_t* puchBuffer)
	函数说明:
	  本函数是页读nandflash，如果ECC使能，将检测和改正bit错误
	参数说明:
	  address：块或者页地址
	  puchBuffer：数组指针
	返回值:
	   NULL_POINTER_ERROR：指针为0
	   SUCCESS:成功
	   FAIL：失败
	备注: puchBuffer是一个2KB的数组
*********************************************************************/
uint32_t C6678_Emif16_Nandflash_ReadPage(NAND_ADDR address, uint8_t* puchBuffer);
/********************************************************************
	函数声明

	所在文件:   emif16_nandflash.c
	作者:		wj

	函数名:
	  uint32_t C6678_Emif16_Nandflash_BlockErase(uint32_t uiBlockNumber)
	函数说明:
	  本函数是块擦除nandflash
	参数说明:
	  uiBlockNumber：块地址
	返回值:
	   SUCCESS:成功
	   FAIL：失败
	备注:
*********************************************************************/
uint32_t C6678_Emif16_Nandflash_BlockErase(uint32_t uiBlockNumber);
/********************************************************************
	函数声明

	所在文件:   emif16_nandflash.c
	作者:		wj

	函数名:
	  uint32_t C6678_Emif16_Nandflash_WritePage(RADAR_DEVICE_info *p_device, NAND_ADDR address, uint8_t* puchBuffer)
	函数说明:
	  本函数是页写nandflash，如果ECC使能，计算ECC并写入到空白区域
	参数说明:
	  address：块或者页地址
	  puchBuffer：数组指针
	返回值:
	   NULL_POINTER_ERROR：指针为0
	   SUCCESS:成功
	   FAIL：失败
	备注: puchBuffer是一个2KB的数组
	写之前要先调用擦函数，否则出错
*********************************************************************/
uint32_t C6678_Emif16_Nandflash_WritePage(RADAR_DEVICE_info *p_device, NAND_ADDR address, uint8_t* puchBuffer);
/********************************************************************
	函数声明

	所在文件:   emif16_nandflash.c
	作者:		wj

	函数名:
	  uint32_t C6678_Emif16_Nandflash_GetDetails(RADAR_DEVICE_info *pNandInfo)
	函数说明:
	  本函数是得到NANDFLASH的具体信息
	参数说明:
	  pNandInfo：nand信息结构体的指针
	返回值:
	   NULL_POINTER_ERROR：指针为0
	   SUCCESS:成功
	   FAIL：失败
	备注:无
*********************************************************************/
uint32_t C6678_Emif16_Nandflash_GetDetails(RADAR_DEVICE_info *pNandInfo);
/********************************************************************
	文件说明:	以下这部分函数是emif16_fpga.c中函数的声明
						emif16_fpga的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   emif16_fpga.c
	作者:		wj

	函数名:
	  Bool C6678_Emif16_Fpga_Init(void)
	函数说明:
	  本函数是初始化emif16_fpga
	参数说明:
	  无
	返回值:
	   TRUE:成功
	   FALSE：失败
	备注: 无
*********************************************************************/
Bool C6678_Emif16_Fpga_Init(void);
/********************************************************************
	函数声明

	所在文件:   emif16_fpga.c
	作者:		wj

	函数名:
	 	 Bool C6678_Emif16_Fpga_ReadByte(uint32_t byte_addr,uint32_t uiNumBytes,uint16_t *DataBuffer)
	函数说明:
	  本函数是通过emif16读fpga中的数据
	参数说明:
	  DataBuffer：读出数据存储的数组的指针
 	  uiNumBytes：读出数据的长度（16bit为单位）(最大2048)
	  byte_addr：数据在fpga的基地址（16bit为单位）
	返回值:
	   TRUE:成功
	   FALSE：失败
 	备注:默认使用DSP的EMIF CE0(CS2)(映射空间0x70000000)和FPGA通信，如果实际硬件连接其它CE，
		则使用byte_addr进行地址空间的偏移（单个偏移量0x04000000）
		如实际FPGA连接DSP的EMIF CE2（CS4），则byte_addr=0x08000000
*********************************************************************/
Bool C6678_Emif16_Fpga_ReadByte(uint32_t byte_addr,uint32_t uiNumBytes,uint16_t *DataBuffer);
/********************************************************************
	函数声明

	所在文件:   emif16_fpga.c
	作者:		wj

	函数名:
	 	 Bool C6678_Emif16_Fpga_WriteByte(uint32_t byte_addr,uint32_t uiNumBytes,uint16_t *DataBuffer)
	函数说明:
	  本函数是通过emif16写fpga的某段空间
	参数说明:
	  DataBuffer：写入数据存储的数组的指针
 	  uiNumBytes：写入数据的长度（16bit为单位）(最大2048)
	  byte_addr：数据在fpga的基地址（16bit为单位）
	返回值:
	   TRUE:成功
	   FALSE：失败
 	备注:默认使用DSP的EMIF CE0(CS2)(映射空间0x70000000)和FPGA通信，如果实际硬件连接其它CE，
		则使用byte_addr进行地址空间的偏移（单个偏移量0x04000000）
		如实际FPGA连接DSP的EMIF CE2（CS4），则byte_addr=0x08000000
*********************************************************************/
Bool C6678_Emif16_Fpga_WriteByte(uint32_t byte_addr,uint32_t uiNumBytes,uint16_t *DataBuffer);
/********************************************************************
	文件说明:	以下这部分函数是pcie.c中函数的声明
						pcie的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   pcie.c
	作者:		wj

	函数名:
	  Bool C6678_Pcie_Init(int mode)
	函数说明:
	  本函数是初始化pcie
	参数说明:
	 mode:
	 	 PCIEMODE(具体值见c6678.h)
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注:  注意：两个pcie端相连时，两个pcie端调用该函数间隔不能超过几秒，否则连不上。
*********************************************************************/
Bool C6678_Pcie_Init(int mode);
/********************************************************************
	函数声明

	所在文件:   pcie.c
	作者:		wj

	函数名:
	  Bool C6678_Pcie_InboundAdrTrans(unsigned int pcieAdr,unsigned int localAdr)
	函数说明:
	  本函数是配置pcie的inbound地址映射
	参数说明:
	 pcieAdr:
	 	 pcie总线地址
	 localAdr：
	     6678内部总线地址
	返回值:
	  true:映射正常
	  false：映射失败
	备注:
*********************************************************************/
Bool C6678_Pcie_InboundAdrTrans(unsigned int pcieAdr,unsigned int localAdr);
/********************************************************************
	函数声明

	所在文件:   pcie.c
	作者:		wj

	函数名:
	  Bool C6678_Pcie_SetOutboundSize(unsigned int outboundSize)
	函数说明:
	  本函数是配置pcie的outbound窗大小
	参数说明:
	 obSize:
	 	 窗大小
	 	 0:1MB
  	  	 1:2MB
  	  	 2:4MB
  	  	 3:8MB
	返回值:
	  true:设置正常
	  false：设置失败
*********************************************************************/
Bool C6678_Pcie_SetOutboundSize(unsigned int outboundSize);
/********************************************************************
	函数声明

	所在文件:   pcie.c
	作者:		wj

	函数名:
	  Bool C6678_Pcie_OutboundAdrTrans(unsigned int pcieAdrLow,unsigned int pcieAdrHigh,unsigned int regionNum)
	函数说明:
	  本函数是配置pcie的outbound地址映射
	参数说明:
		pcieAdrLow:
			pcie总线低32bit地址
		pcieAdrHigh：
			pcie总线高32bit地址
		regionNum:
			region的编号
	返回值:
	  true:映射正常
	  false：映射失败
	备注:每个region对应的内部地址为0x60000000+obsize*regionNum
*********************************************************************/
Bool C6678_Pcie_OutboundAdrTrans(unsigned int pcieAdrLow,unsigned int pcieAdrHigh,unsigned int regionNum);
/********************************************************************
	文件说明:	以下这部分函数是srio.c中函数的声明
						srio的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Init_Ku(int Rate,int SrioId[4],int MultiId[3],uint8_t InfoMode,uint8_t DataswapMode,uint8_t PhysicalMode,Bool BoardLinkCheckFlag);

	函数说明:
		本函数是初始化srio

	参数说明:
		Rate：
			SRIO_RATE
		SrioId:
			SRIO的ID(4个ID)
			当PhysicalMode==0时:SrioId数组中只有SrioId[0]需要赋值，此时本DSP只有一个ID;
			当PhysicalMode==1时:SrioId数组中只有SrioId[0]和SrioId[2]需要赋值，
			   此时本DSP有两个ID,每个ID对应一个2x，port口分别为port0和port2;
		MultiId:
		    3个多播ID,如果没有使用可以配为0
		InfoMode:
			门铃info的不同模式
			0:8核每个核可以各传递8个info值，平均模式
			1:核0传递16个 info，其他核5个info，非平均模式
		DataswapMode:
			0:mode A
			1:mode B
			2:mode C
			3:mode D
			internal mem                   external SRIO port
			memory address     			mode A    mode B    mode C    mode D
			byte15			first data  byte0     byte1     byte3     byte7
			byte14			 on port    byte1     byte0     byte2     byte6
			byte13                      byte2     byte3     byte1     byte5
			byte12                      byte3     byte2     byte0     byte4
			byte11                      byte4     byte5     byte7     byte3
			byte10                      byte5     byte4     byte6     byte2
			byte9                       byte6     byte7     byte5     byte1
			byte8                       byte7     byte6     byte4     byte0
			byte7                       byte8     byte9     byte11    byte15
			byte6                       byte9     byte8     byte10    byte14
			byte5                       byte10    byte11    byte9     byte13
			byte4                       byte11    byte10    byte8     byte12
			byte3                       byte12    byte13    byte15    byte11
			byte2                       byte13    byte12    byte14    byte10
			byte1            last data  byte14    byte15    byte13    byte9
			byte0             on port   byte15    byte14    byte12    byte8
		PhysicalMode:
		    srio的4x的连接方式
			0:1个4x
			1:2个2x
		BoardLinkCheckFlag:
			是否检查板间link标志,该标志只有在PhysicalMode为1（2个2x）时有意义，目前适用于ku的特殊拓扑。
			TRUE:	检查板间link信号
			FALSE:  不检查板间link信号
	返回值:
		true:初始化正常
		false：初始化失败

	备注: 目前只支持1个4x和2个2x,且目前2个2x不能同时传输，只能分时传输
*********************************************************************/
Bool C6678_Srio_Init_Ku(int Rate,int *SrioId,int *MultiId,uint8_t InfoMode,uint8_t DataswapMode,uint8_t PhysicalMode,Bool BoardLinkCheckFlag);

/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Init(int Rate,int SrioId,int MultiId[3],uint8_t InfoMode,uint8_t DataswapMode);

	函数说明:
		本函数是初始化srio

	参数说明:
		Rate：
			SRIO_RATE
		SrioId:
			SRIO的ID
		MultiId:
		    3个多播ID,如果没有使用可以配为0
		InfoMode:
			门铃info的不同模式
			0:8核每个核可以各传递8个info值，平均模式
			1:核0传递16个 info，其他核5个info，非平均模式
		DataswapMode:
			0:mode A
			1:mode B
			2:mode C
			3:mode D
			internal mem                   external SRIO port
			memory address     			mode A    mode B    mode C    mode D
			byte15			first data  byte0     byte1     byte3     byte7
			byte14			 on port    byte1     byte0     byte2     byte6
			byte13                      byte2     byte3     byte1     byte5
			byte12                      byte3     byte2     byte0     byte4
			byte11                      byte4     byte5     byte7     byte3
			byte10                      byte5     byte4     byte6     byte2
			byte9                       byte6     byte7     byte5     byte1
			byte8                       byte7     byte6     byte4     byte0
			byte7                       byte8     byte9     byte11    byte15
			byte6                       byte9     byte8     byte10    byte14
			byte5                       byte10    byte11    byte9     byte13
			byte4                       byte11    byte10    byte8     byte12
			byte3                       byte12    byte13    byte15    byte11
			byte2                       byte13    byte12    byte14    byte10
			byte1            last data  byte14    byte15    byte13    byte9
			byte0             on port   byte15    byte14    byte12    byte8
	返回值:
		true:初始化正常
		false：初始化失败

	备注: 无
*********************************************************************/
Bool C6678_Srio_Init(int Rate,int SrioId,int MultiId[3],uint8_t InfoMode,uint8_t DataswapMode);

/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		qqb

	函数名:
		Bool srio_linktest(int portnum,int cmd);

	函数说明:
		本函数是向指定的端口发送link request命令

	参数说明:
		portNum : SRIO Port Number
            cmd : Command to be sent in the link-request control symbol.
                  The following values hold good:-
                  - 0x3 i.e. Reset
                  - 0x4 i.e. Input Status

	返回值:
		true:成功
		false：失败

	备注: 无
*********************************************************************/
Bool srio_linktest(int portnum,int cmd);

/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
	   Bool C6678_Srio_Close(void);
	函数说明:
	  本函数是关闭SRIO。
	参数说明:
	  无
	返回值:
	  true:成功
	  false：失败
	备注: 无
*********************************************************************/
Bool C6678_Srio_Close(void);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_Read_Ku(Uint32 PortId,uint32_t Src,uint32_t * Dst, Uint16 SrcID, Uint32 Len,Bool WaitOver,Bool IntrEn,Uint32 priority)

	函数说明:
		本函数是从目标设备读取数据到本地地址

	参数说明:
		PortId:端口号
		    当PhysicalMode==0时:PortId只能为0;
			当PhysicalMode==1时:PortId只能为0和2,分别对应2个2x，port口分别为port0和port2;
			PhysicalMode为C6678_Srio_Init的参数。
		Src:源地址（目标）
		Dst：目的地址（本地）
		SrcID：目标ID
		Len：数据长度（单次传输最大1MB,不得为0）
		WaitOver:是否等待结束
			TRUE:等待结束(之后不需要再调用C6678_Srio_Wait_Over函数)
			FALSE：不等待结束(之后需要再调用C6678_Srio_Wait_Over函数)
		IntrEn：传输结束时是否发送中断
			TRUE:发送中断
			FALSE：不发送中断
		priority：优先级
			该值只能是0,1,2；其他值传输会出错	
	返回值:
		true:成功
	 	false：失败

	备注: 无
*********************************************************************/
Bool C6678_Srio_Dio_Read_Ku(Uint32 PortId,uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_Read(uint32_t Src,uint32_t * Dst, Uint16 SrcID, Uint32 Len,Bool WaitOver,Bool IntrEn,Uint32 priority)

	函数说明:
		本函数是从目标设备读取数据到本地地址

	参数说明:
		Src:源地址（目标）
		Dst：目的地址（本地）
		SrcID：目标ID
		Len：数据长度（单次传输最大1MB,不得为0）
		WaitOver:是否等待结束
			TRUE:等待结束(之后不需要再调用C6678_Srio_Wait_Over函数)
			FALSE：不等待结束(之后需要再调用C6678_Srio_Wait_Over函数)
		IntrEn：传输结束时是否发送中断
			TRUE:发送中断
			FALSE：不发送中断
		priority：优先级
			该值只能是0,1,2；其他值传输会出错
	返回值:
		true:成功
	 	false：失败

	备注: 无
*********************************************************************/
Bool C6678_Srio_Dio_Read(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_Write_Ku(Uint32 PortId,Uint32 *  Src, Uint32 Dst, Uint16 DstID, Uint32 Len,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

	函数说明:
		本函数是向目标设备写数据

	参数说明:
		PortId:端口号
		    当PhysicalMode==0时:PortId只能为0;
			当PhysicalMode==1时:PortId只能为0和2,分别对应2个2x，port口分别为port0和port2;
			PhysicalMode为C6678_Srio_Init的参数。
		Src:源地址（本地）
		Dst：目的地址（目标）
		DstID：目标ID(与fpga通信时，目标ID需与fpga编程人员确定)
		Len：数据长度（单次传输最大1MB,不得为0）
		Writemode：写的模式。
			Srio_Ttype_Write_NWRITE
			Srio_Ttype_Write_NWRITE_R
			Srio_Ftype_SWRITE
		WaitOver:是否等待结束
			TRUE:等待结束(之后不需要再调用C6678_Srio_Wait_Over函数)
			FALSE：不等待结束(之后需要再调用C6678_Srio_Wait_Over函数)
		IntrEn：传输结束时是否发送中断
			TRUE:发送中断
			FALSE：不发送中断
		priority：优先级
			该值只能是0,1,2；其他值传输会出错	
	返回值:
		true:成功
		false：失败

	备注: 无
*********************************************************************/
Bool C6678_Srio_Dio_Write_Ku(Uint32 PortId,Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_Write(Uint32 *  Src, Uint32 Dst, Uint16 DstID, Uint32 Len,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

	函数说明:
		本函数是向目标设备写数据

	参数说明:
		Src:源地址（本地）
		Dst：目的地址（目标）
		DstID：目标ID(与fpga通信时，目标ID需与fpga编程人员确定)
		Len：数据长度（单次传输最大1MB,不得为0）
		Writemode：写的模式。
			Srio_Ttype_Write_NWRITE
			Srio_Ttype_Write_NWRITE_R
			Srio_Ftype_SWRITE
		WaitOver:是否等待结束
			TRUE:等待结束(之后不需要再调用C6678_Srio_Wait_Over函数)
			FALSE：不等待结束(之后需要再调用C6678_Srio_Wait_Over函数)
		IntrEn：传输结束时是否发送中断
			TRUE:发送中断
			FALSE：不发送中断
		priority：优先级
			该值只能是0,1,2；其他值传输会出错
	返回值:
		true:成功
		false：失败

	备注: 无
*********************************************************************/
Bool C6678_Srio_Dio_Write(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_Doorbell_Ku(Uint32 PortId,Uint16 Info,Uint16 DstID,Uint32 priority)

	函数说明:
	  本函数是向目标设备发送门铃
	参数说明:
		PortId:端口号
		    当PhysicalMode==0时:PortId只能为0;
			当PhysicalMode==1时:PortId只能为0和2,分别对应2个2x，port口分别为port0和port2;
			PhysicalMode为C6678_Srio_Init的参数。
		Info:门铃信息
		如果使用C6678_Srio_Init初始化SRIO,info值的对应关系如下：
		info为16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(其中0~7发给了corenum>>1,8~15发给了corenum>>1+1)
		例如：
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		如果使用C6678_Srio_Init_Core0more初始化SRIO,info值的对应关系如下：
		info为16bit
		core0对应16种info值，其他核每个核对应5中info值
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
			 	 	Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		DstID：目标ID(与fpga通信时，目标ID需与fpga编程人员确定)
        priority：优先级
			该值只能是0,1,2；其他值传输会出错	
	返回值:
	  true:成功
	  false：失败
	备注: info应为以上说明中的值，其他值不允许输入
*********************************************************************/
Bool C6678_Srio_Dio_Doorbell_Ku(Uint32 PortId,Uint16 Info,Uint16 DstID,Uint32 priority);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_Doorbell(Uint16 Info,Uint16 DstID,Uint32 priority)

	函数说明:
	  本函数是向目标设备发送门铃
	参数说明:
		Info:门铃信息
		如果使用C6678_Srio_Init初始化SRIO,info值的对应关系如下：
		info为16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(其中0~7发给了corenum>>1,8~15发给了corenum>>1+1)
		例如：
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		如果使用C6678_Srio_Init_Core0more初始化SRIO,info值的对应关系如下：
		info为16bit
		core0对应16种info值，其他核每个核对应5中info值
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		DstID：目标ID(与fpga通信时，目标ID需与fpga编程人员确定)
		priority：优先级
			该值只能是0,1,2；其他值传输会出错
	返回值:
	  true:成功
	  false：失败
	备注: info应为以上说明中的值，其他值不允许输入
*********************************************************************/
Bool C6678_Srio_Dio_Doorbell(Uint16 Info,Uint16 DstID,Uint32 priority);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
	  Bool C6678_Srio_Maitain_Read(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len)
	函数说明:
	  本函数是从目标设备读取数据到本地地址
	参数说明:
	  Src:源地址（目标）
	  Dst：目的地址（本地）
	  SrcID：源ID
	  Len：数据长度
	返回值:
	  true:成功
	  false：失败
	备注: 无
*********************************************************************/
Bool C6678_Srio_Maitain_Read(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		Lin Jiang

	函数名:
		Bool C6678_Srio_Maitain_Read_Hop(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,uint8_t hopcnt)

	函数说明:
		本函数是从目标设备读取数据到本地地址

	参数说明:
		Src:源地址（目标）
		Dst：目的地址（本地）
		SrcID：源ID
		Len：数据长度
		hopcnt：经过的交换芯片数

	返回值:
		true:成功
		false：失败

	备注: 无
*********************************************************************/
Bool C6678_Srio_Maitain_Read_Hop(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,uint8_t hopcnt);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
	  Bool C6678_Srio_Maitain_Write(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len);
	函数说明:
	  本函数是将数据写入目标设备
	参数说明:
	  Src:源地址（本地）
	  Dst：目的地址（目标）
	  SrcID：目标ID
	  Len：数据长度
	返回值:
	  true:成功
	  false：失败
	备注: 无
*********************************************************************/
Bool C6678_Srio_Maitain_Write(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		Lin Jiang

	函数名:
		Bool C6678_Srio_Maitain_Write_Hop(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,uint8_t hopcnt)

	函数说明:
		本函数是将数据写入目标设备

	参数说明:
		Src:源地址（本地）
		Dst：目的地址（目标）
		SrcID：目标ID
		Len：数据长度
        hopcnt：经过的交换芯片数

	返回值:
		true:成功
		false：失败

	备注: 无
*********************************************************************/
Bool C6678_Srio_Maitain_Write_Hop(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,uint8_t hopcnt);
/********************************************************************
	函数声明

	所在文件:   srio_drv.c
	作者:		Lin Jiang

	函数名:
		Bool C6678_Srio_Enumerate(Uint16 HostID,Uint32 srioidnum,Uint16 *srioIDInfo,Srio_Enum *srio_info)

	函数说明:
		本函数是SRIO枚举C6678系统的设备

	参数说明:
     HostID:作主控的C6678的SRIO_ID（本地ID）
     srioidnum:需要枚举的srioid的数量
     srioIDInfo：需要枚举的id的数组指针
     srio_info:枚举结果信息

	返回值:
		true:枚举到设备
		false：没有枚举到设备

	备注: 此模式中6678的srio初始化函数中DataswapMode配成了0;
*********************************************************************/
Bool C6678_Srio_Enumerate(Uint16 HostID,Uint32 srioidnum,Uint16 *srioIDInfo,Srio_Enum *srio_info);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
	  Bool C6678_Srio_Bind(Srio_SocketType mode,Srio_SockBindAddrInfo ptr_bindInfo)
	函数说明:
	  本函数是将一个包绑定
	参数说明:
	  mode:包的种类
	  	  Srio_SocketType_DIO：DIO
	  	  Srio_SocketType_TYPE9：TYPE 9
	  	  Srio_SocketType_TYPE11：TYPE 11
	  ptr_bindInfo：包的参数
	      如果包是type9类型，需要配置
	      ptr_bindInfo->type9.cos
	      ptr_bindInfo->type9.streamId
	      如果包是type11类型，需要配置
	      ptr_bindInfo->type11.letter
	      ptr_bindInfo->type11.mbox
	      如果包是DIO类型，不需要配置
	返回值:
	  true:成功
	  false：失败
	备注: 每种包只需要bind一次，后面不需再调用，再调用会报错
*********************************************************************/
Bool C6678_Srio_Bind(Srio_SocketType mode,Srio_SockBindAddrInfo ptr_bindInfo);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
	  Bool C6678_Srio_Type11_Send(Uint16 DstID,Uint16 mbox,Uint16 letter,uint32_t * Src,Uint32 Len)
	函数说明:
	  本函数是向目标设备发送type11包
	参数说明:
	  DstID：目标ID
	  mbox:mbox的值
	  letter：letter的值
	  Src：源地址（本地）
	  Len：数据长度(应为8B的整数倍)
	返回值:
	  true:成功
	  false：失败
	备注: 无
*********************************************************************/
Bool C6678_Srio_Type11_Send(Uint16 DstID,Uint16 mbox,Uint16 letter,uint32_t * Src,Uint32 Len);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
	  int32_t C6678_Srio_Type11_Recv(uint32_t * Dst,Srio_SockAddrInfo* from,Uint32 Len)
	函数说明:
	  本函数是接收type11的srio包
	参数说明:
	  Dst：目的地址（目标）
	  from：包的参数
	      可以得到参数
	      from.type11.letter
	      from.type11.mbox 
	  Len:数据长度(应为8B的整数倍)
	返回值:
	  接收的数据个数
	备注: 无
*********************************************************************/
int32_t C6678_Srio_Type11_Recv(uint32_t * Dst,Srio_SockAddrInfo* from,Uint32 Len);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
	  Bool C6678_Srio_Type9_Send(Uint16 DstID,Uint16 streamId,Uint16 cos,uint32_t * Src,Uint32 Len)
	函数说明:
	  本函数是发送type9的srio包
	参数说明:
	  cos:cos的值
	  streamId：stream id
	  DstID：目标ID
	  Src：源地址（本地）
	  Len：数据长度
	返回值:
	  true:成功
	  false：失败
	备注: 无
*********************************************************************/
Bool C6678_Srio_Type9_Send(Uint16 DstID,Uint16 streamId,Uint16 cos,uint32_t * Src,Uint32 Len);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
	  int32_t C6678_Srio_Type9_Recv(Uint32 * Dst,Srio_SockAddrInfo from,Uint32 Len)
	函数说明:
	  本函数是接收type9的srio包
	参数说明:
	  Dst：目的地址（目标）
	  from：包的信息
	      可以得到参数
	      from.type9.cos
	      from.type9.streamId 
	  Len:数据长度
	返回值:
	  接收的数据个数
	备注: 无
*********************************************************************/
int32_t C6678_Srio_Type9_Recv(Uint32 * Dst,Srio_SockAddrInfo from,Uint32 Len);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_DoorbellInt_Hookup(Uint8 VectID,void interruptISR())

	函数说明:
		本函数是挂门铃中断

	参数说明:
		interruptISR：中断函数
		VectID:中断向量号（4-15）

	返回值:
		true:成功
		false：失败

	备注: 无
*********************************************************************/
Bool C6678_Srio_DoorbellInt_Hookup(Uint8 VectID,void interruptISR());

/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Uint32 C6678_Srio_DoorbellInt_Info(Uint32 coreNum,uint8_t InfoMode)

	函数说明:
		本函数是获取中断的info值

	参数说明:
		coreNum:核号
		InfoMode:
			门铃info的不同模式
			0:8核每个核可以各传递8个info值，平均模式
			1:核0传递16个 info，其他核5个info，非平均模式
	返回值:
		info值

	备注:在清中断前调用
	info的详细介绍见C6678_Srio_Dio_Doorbell的注释
*********************************************************************/
Uint32 C6678_Srio_DoorbellInt_Info(Uint32 coreNum,uint8_t InfoMode);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_DoorbellInt_Clear(Uint32 coreNum,uint8_t InfoMode)

	函数说明:
		本函数是清中断

	参数说明:
		coreNum:核号
		InfoMode:
			门铃info的不同模式
			0:8核每个核可以各传递8个info值，平均模式
			1:核0传递16个 info，其他核5个info，非平均模式
	返回值:
		true:成功
		false：失败

	备注:无
*********************************************************************/
Bool C6678_Srio_DoorbellInt_Clear(Uint32 coreNum,uint8_t InfoMode);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Over_Hookup(Uint8 VectID,void interruptISR())

	函数说明:
		本函数是传输结束中断

	参数说明:
		interruptISR：中断函数
		VectID:中断向量号（4-15）

	返回值:
		true:成功
		false：失败

	备注: 需先调用
		C6678_CoreInt_Init ();
		C6678_ChipInt_Init (0);
*********************************************************************/
Bool C6678_Srio_Over_Hookup(Uint8 VectID,void interruptISR());
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Wait_Over(void)

	函数说明:
		本函数是等待传输结束函数

	参数说明:
		无

	返回值:
		true:传输成功
		false：传输失败

	备注:本函数是进入传输结束中断后需要后续调用的函数 
*********************************************************************/
Bool C6678_Srio_Wait_Over(void);

/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		void C6678_Srio_Over_Int_Clear(void)

	函数说明:
		本函数是进入传输结束中断后清中断函数

	参数说明:
		无

	返回值:
		无

	备注:无
*********************************************************************/
void C6678_Srio_Over_Int_Clear(void);

/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Navigator_Init(Uint8 VectID,void  interruptISR())

	函数说明:
		本函数是初始化srio所需的多核导航模块

	参数说明:
		VectID:中断向量号（4-15）
		interruptISR：中断服务程序名
	返回值:
		true:初始化正常
		false：初始化失败

	备注: 待改
*********************************************************************/
Bool C6678_Srio_Navigator_Init(Uint8 VectID,void  interruptISR());
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_ReadandDoorbell_Ku(Uint32 PortId,uint32_t Src,uint32_t * Dst, Uint16 SrcID, Uint32 Len,Uint16 Info,Bool WaitOver,Bool IntrEn,Uint32 priority)

	函数说明:
		本函数是从目标设备读取数据到本地地址,读取完成后发送门铃

	参数说明:
		PortId:端口号
			当PhysicalMode==0时:PortId只能为0;
			当PhysicalMode==1时:PortId只能为0和2,分别对应2个2x，port口分别为port0和port2;
			PhysicalMode为C6678_Srio_Init的参数。
		Src:源地址（目标）
		Dst：目的地址（本地）
		SrcID：目标ID
		Len：数据长度（单次传输最大1MB,不得为0）
		Info:门铃信息
		如果使用C6678_Srio_Init初始化SRIO,info值的对应关系如下：
		info为16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(其中0~7发给了corenum>>1,8~15发给了corenum>>1+1)
		例如：
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		如果使用C6678_Srio_Init_Core0more初始化SRIO,info值的对应关系如下：
		info为16bit
		core0对应16种info值，其他核每个核对应5中info值
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
			 	 	Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		WaitOver:是否等待结束
			TRUE:等待结束(之后不需要再调用C6678_Srio_Wait_Over函数)
			FALSE：不等待结束(之后需要再调用C6678_Srio_Wait_Over函数)
		IntrEn：传输结束时是否发送中断
			TRUE:发送中断
			FALSE：不发送中断
		priority：优先级
			该值只能是0,1,2；其他值传输会出错
	返回值:
		true:成功
	 	false：失败

	备注: info应为以上说明中的值，其他值不允许输入
*********************************************************************/
Bool C6678_Srio_Dio_ReadandDoorbell_Ku(Uint32 PortId,uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,Uint16 Info,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_ReadandDoorbell(uint32_t Src,uint32_t * Dst, Uint16 SrcID, Uint32 Len,Uint16 Info,Bool WaitOver,Bool IntrEn,Uint32 priority)

	函数说明:
		本函数是从目标设备读取数据到本地地址,读取完成后发送门铃

	参数说明:
		Src:源地址（目标）
		Dst：目的地址（本地）
		SrcID：目标ID
		Len：数据长度（单次传输最大1MB,不得为0）
		Info:门铃信息
		如果使用C6678_Srio_Init初始化SRIO,info值的对应关系如下：
		info为16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(其中0~7发给了corenum>>1,8~15发给了corenum>>1+1)
		例如：
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		如果使用C6678_Srio_Init_Core0more初始化SRIO,info值的对应关系如下：
		info为16bit
		core0对应16种info值，其他核每个核对应5中info值
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		WaitOver:是否等待结束
			TRUE:等待结束(之后不需要再调用C6678_Srio_Wait_Over函数)
			FALSE：不等待结束(之后需要再调用C6678_Srio_Wait_Over函数)
		IntrEn：传输结束时是否发送中断
			TRUE:发送中断
			FALSE：不发送中断
		priority：优先级
			该值只能是0,1,2；其他值传输会出错
	返回值:
		true:成功
		false：失败

	备注: info应为以上说明中的值，其他值不允许输入
*********************************************************************/
Bool C6678_Srio_Dio_ReadandDoorbell(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,Uint16 Info,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_WriteandDoorbell_Ku(Uint32 PortId,Uint32 * Src, Uint32 Dst, Uint16 DstID, Uint32 Len,Uint16 Info,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

	函数说明:
		本函数是向目标设备写数据,写完数据后发送门铃

	参数说明:
		PortId:端口号
			当PhysicalMode==0时:PortId只能为0;
			当PhysicalMode==1时:PortId只能为0和2,分别对应2个2x，port口分别为port0和port2;
			PhysicalMode为C6678_Srio_Init的参数。
		Src:源地址（本地）
		Dst：目的地址（目标）
		DstID：目标ID(与fpga通信时，目标ID需与fpga编程人员确定)
		Len：数据长度（单次传输最大1MB,不得为0）
		Info:门铃信息
		如果使用C6678_Srio_Init初始化SRIO,info值的对应关系如下：
		info为16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(其中0~7发给了corenum>>1,8~15发给了corenum>>1+1)
		例如：
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		如果使用C6678_Srio_Init_Core0more初始化SRIO,info值的对应关系如下：
		info为16bit
		core0对应16种info值，其他核每个核对应5中info值
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		Writemode：写的模式。
			Srio_Ttype_Write_NWRITE
			Srio_Ttype_Write_NWRITE_R
			Srio_Ftype_SWRITE
		WaitOver:是否等待结束
			TRUE:等待结束(之后不需要再调用C6678_Srio_Wait_Over函数)
			FALSE：不等待结束(之后需要再调用C6678_Srio_Wait_Over函数)
		IntrEn：传输结束时是否发送中断
			TRUE:发送中断
			FALSE：不发送中断
		priority：优先级
			该值只能是0,1,2；其他值传输会出错
	返回值:
		true:成功
		false：失败

	备注: info应为以上说明中的值，其他值不允许输入
*********************************************************************/
Bool C6678_Srio_Dio_WriteandDoorbell_Ku(Uint32 PortId,Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,Uint16 Info,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		Bool C6678_Srio_Dio_WriteandDoorbell(Uint32 * Src, Uint32 Dst, Uint16 DstID, Uint32 Len,Uint16 Info,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

	函数说明:
		本函数是向目标设备写数据,写完数据后发送门铃

	参数说明:
		Src:源地址（本地）
		Dst：目的地址（目标）
		DstID：目标ID(与fpga通信时，目标ID需与fpga编程人员确定)
		Len：数据长度（单次传输最大1MB,不得为0）
		Info:门铃信息
		如果使用C6678_Srio_Init初始化SRIO,info值的对应关系如下：
		info为16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(其中0~7发给了corenum>>1,8~15发给了corenum>>1+1)
		例如：
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		如果使用C6678_Srio_Init_Core0more初始化SRIO,info值的对应关系如下：
		info为16bit
		core0对应16种info值，其他核每个核对应5中info值
		发给对方核0：Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		发给对方核1：Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核2：Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
			 	 	Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核3：Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核4：Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核5：Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核6：Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		发给对方核7：Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		Writemode：写的模式。
			Srio_Ttype_Write_NWRITE
			Srio_Ttype_Write_NWRITE_R
			Srio_Ftype_SWRITE
		WaitOver:是否等待结束
			TRUE:等待结束(之后不需要再调用C6678_Srio_Wait_Over函数)
			FALSE：不等待结束(之后需要再调用C6678_Srio_Wait_Over函数)
		IntrEn：传输结束时是否发送中断
			TRUE:发送中断
			FALSE：不发送中断
		priority：优先级
			该值只能是0,1,2；其他值传输会出错
	返回值:
		true:成功
		false：失败

	备注: info应为以上说明中的值，其他值不允许输入
*********************************************************************/
Bool C6678_Srio_Dio_WriteandDoorbell(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,Uint16 Info,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	函数声明

	所在文件:   srio.c
	作者:		wj

	函数名:
		void C6678_Srio_Type9Type11_rxCompletionIsr(void)

	函数说明:
		本函数是收到TYPE9和TYPE11后中断中的处理函数。

	参数说明:
		无
	返回值:
		无

	备注: 无
*********************************************************************/
void C6678_Srio_Type9Type11_rxCompletionIsr(void);

/********************************************************************
	文件说明:	以下这部分函数是other.c中函数的声明
						XMC,power,ecc的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   other.c
	作者:		wj

	函数名:
	  Bool C6678_XMC_MapSelect(Uint32 Logical32bitAddr,Phy36bit Physical36bitAddr,Uint32 MapLength,Uint32 CpuIndex);
	函数说明:
	  本函数是将32bit的逻辑地址映射到36bit的物理地址。
	参数说明:
	  Logical32bitAddr:32bit逻辑地址
	  Physical36bitAddr:36bit物理地址
	  MapLength:映射的大小
	         0:4KB
	         1:8KB=4KB*2
	         2:16KB=8KB*2
	         3:32KB=16KB*2
	         ........
	         20:4GB
	         (映射的大小理论上可以支持以上值，但是由于目前芯片的内存限制，测试只能测试0到18,19和20无法测试)
	  CpuIndex:cpu一共可以映射16对。0,1,2已被默认映射，所以只有3到15可以用户修改。
			 3~15
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 映射大小决定映射时32bit逻辑地址的取址地址，
	               例如：如果映射大小为4KB，32bit逻辑地址应该为4KB对齐的，小于4KB的地址无效。
*********************************************************************/
Bool C6678_XMC_MapSelect(Uint32 Logical32bitAddr,Phy36bit Physical36bitAddr,Uint32 MapLength,Uint32 CpuIndex);
/********************************************************************
	函数声明

	所在文件:   other.c
	作者:		wj

	函数名:
	  void C6678_XMC_PrefetchBuffer_Inv(void)
	函数说明:
	  本函数失效XMC预取Buffer
	参数说明:
	  无
	返回值:
	 无
	备注: 无
*********************************************************************/
void C6678_XMC_PrefetchBuffer_Inv(void);

/********************************************************************
	函数声明

	所在文件:   other.c
	作者:		wj

	函数名:
	  void C6678_Power_UpDomains(void)
	函数说明:
	  本函数是使能所有电源模块
	参数说明:
	  无
	返回值:
	 无
	备注: this function powers up the PA subsystem domains
*********************************************************************/
void C6678_Power_UpDomains(void);
/********************************************************************
	函数声明

	所在文件:   other.c
	作者:		wj

	函数名:
	  void C6678_Ecc_Enable(void)
	函数说明:
	  本函数是使能ECC
	参数说明:
	  无
	返回值:
	 无
	备注:
*********************************************************************/
void C6678_Ecc_Enable(void);
/********************************************************************
	文件说明:	以下这部分函数是intc.c中函数的声明
						intc的相关操作
*********************************************************************/

/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  Bool C6678_CoreInt_Init (void)
	函数说明:
	  本函数是配置核级中断控制器初始化
	参数说明:
	     无
	返回值:
	  true:初始化设置成功
	  false：初始化设置失败
	备注: 本函数在 Bool C6678_CoreInt_Set ()函数调用之前调用
*********************************************************************/
Bool C6678_CoreInt_Init (void);
/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  Bool C6678_ChipInt_Init (Uint8 ChipIntNum)
	函数说明:
	  本函数是配置特定片级中断控制器的初始化，
	参数说明:
	  ChipIntNum：片级中断控制器号（0-3)

	返回值:
	  true:片级中断初始化成功
	  false：片级中断初始化失败
	备注: 此函数在Bool C6678_ChipInt_Set()之前调用
*********************************************************************/
Bool C6678_ChipInt_Init (Uint8 ChipIntNum);


/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  Bool  C6678_CoreInt_Set (Uint16 EventId,Uint8 VectId,void  interruptISR(),void* Param);
	函数说明:
	  本函数是配置核级中断控制器去响应核级中断事件，包括设置中断事件号，中断向量号，挂中断服务程序
	参数说明:
	  EventId：中断事件号
	  VectId： 中断向量号
	  interruptISR：中断服务程序名
	  Param：传递的参数 ，设为NULL表示不传递参数
	返回值:
	  true:中断设置成功
	  false：中断设置失败
	备注: 无
*********************************************************************/
Bool  C6678_CoreInt_Set (Uint16 EventId,Uint8 VectId,void  interruptISR(),void* Param);
/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  Bool  C6678_ChipInt_Set (Uint8 ChipIntNum, Uint16 EventId,Uint16 ChanId);
	函数说明:
	  本函数是配置片级中断控制器，包括设置中断事件号，通道号
	参数说明:
	  ChipIntNum：片级中断控制器号（0-3)
	  EventId：中断事件号
	  ChanId： 通道号

	返回值:
	  true:片级中断设置成功
	  false：片级中断设置失败
	备注: 片级中断控制器只是将片级中断事件映射到核级中断事件，
	      响应片级中断事件还需配置核级中断控制器，挂中断服务程序等。
*********************************************************************/
Bool  C6678_ChipInt_Set (Uint8 ChipIntNum, Uint16 EventId,Uint16 ChanId);
/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  Bool C6678_ChipInt_Clear (Uint8 ChipIntNum,Uint32 EventId);
	函数说明:
	  本函数是清除特定片级中断控制器的特定事件的中断状态。
	参数说明:
	  ChipIntNum：片级中断控制器号（0-3)
	  EventId：片级中断事件号

	返回值:
	  true:片级中断清除成功
	  false：片级中断清除失败
	备注: 如果不将片级特定中断事件的中断状态清零可能会影响下一次该中断事件的响应。
*********************************************************************/
Bool C6678_ChipInt_Clear (Uint8 ChipIntNum,Uint32 EventId);
/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  void  C6678_CoreInt_Clear (Uint32 EventId);
	函数说明:
	  本函数是清除特定核级中断控制器的特定事件的中断状态。
	参数说明:
	  EventId：核级中断事件号

	返回值:
	 无
	备注: 如果不将核级特定中断事件的中断状态清零可能会影响下一次该中断事件的响应。
*********************************************************************/
void  C6678_CoreInt_Clear (Uint32 EventId);
/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  Bool  C6678_CoreInt_Close (void);
	函数说明:
	  本函数是关闭所创建的核级控制器实例。
	参数说明:
	  无

	返回值:
	  true:关闭成功
	  false：关闭失败
	备注: 该函数调用必须是在C6678_CoreInt_Set（）调用之后
*********************************************************************/
Bool  C6678_CoreInt_Close (void);
/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  void  C6678_CoreInt_Manual_trigger(Uint16 EventId);
	函数说明:
	  本函数是手动触发核级控制器特定中断事件。
	参数说明:
	  EventId：核级中断事件号

	返回值:
	  无
	备注: 该函数调用必须是在C6678_CoreInt_Set（）调用之后。
*********************************************************************/
void  C6678_CoreInt_Manual_trigger(Uint16 EventId);
/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    dyx

	函数名:
	  void  C6678_ChipInt_Manual_trigger(Uint8 ChipIntNum,Uint16 EventId);
	函数说明:
	  本函数是手动触发特定片级中断控制器的特定中断事件。
	参数说明:
	  ChipIntNum：片级中断控制器号（0-3）。
	  EventId：片级中断事件号。

	返回值:
	  无
	备注: 该函数调用必须是在C6678_ChipInt_Set（）调用之后。
*********************************************************************/
void  C6678_ChipInt_Manual_trigger(Uint8 ChipIntNum,Uint16 EventId);
/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    wj

	函数名:InterrputRegInfo  C6678_Int_SaveInterrputRegInfo (void);

	函数说明: 本函数是保存当前中断寄存器的值。

	参数说明:
	  无

	返回值: 包含当前中断寄存器的值的结构体，用于恢复寄存器时使用

	备注: 如需中断嵌套，需要在中断服务函数的开始调用。
*********************************************************************/
InterrputRegInfo  C6678_Int_SaveInterrputRegInfo (void);

/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    wj

	函数名:void  C6678_Int_RestoreInterrputRegInfo (InterrputRegInfo info);

	函数说明:本函数是恢复中断嵌套之前中断寄存器的值。

	参数说明:
	  info：包含嵌套之前中断寄存器的值的结构体

	返回值: 无

	备注: 如需中断嵌套，需要在中断服务函数的最后调用。
*********************************************************************/
void  C6678_Int_RestoreInterrputRegInfo (InterrputRegInfo info);

/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    wj

	函数名:Bool  C6678_Int_GlobalDisable (void);

	函数说明:本函数是失效全局中断。

	参数说明: 无

	返回值:
	  true:失效全局中断成功
	  false：失效全局中断失败

	备注: 无
*********************************************************************/
Bool  C6678_Int_GlobalDisable (void);

/********************************************************************
	函数声明

	所在文件:   intc.c
	作者:	    wj

	函数名:
	  Bool C6678_Int_GlobalEnable (void)
	函数说明:
	  本函数是使能全局中断。
	参数说明:
	     无
	返回值:
	  true:使能全局中断成功
	  false：使能全局中断失败
	备注: 无
*********************************************************************/
Bool C6678_Int_GlobalEnable (void);

/********************************************************************
	文件说明:	以下这部分函数是sem2.c中函数的声明
						sem2的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   sem2.c
	作者:	    dyx

	函数名:
	  Uint8 C6678_Sem2_Acquire(Uint8 Sem2Id,Uint8 Mod);
	函数说明:
	  本函数是获取特定信号量
	参数说明:
	  Sem2Id：信号量号(0-31)
	  Mod： 获取信号量方式：0-直接获取，1-间接获取，2-复合方式获取
	返回值:
		获取成功后返回0xff;
		获取失败时会返回当前拥有该信号量的核ID;
		函数调用错误返回0xaa;
	备注: 无
*********************************************************************/
Uint8 C6678_Sem2_Acquire(Uint8 Sem2Id,Uint8 Mod);
/********************************************************************
	函数声明

	所在文件:   sem2.c
	作者:	    dyx

	函数名:
	  Uint32 C6678_Sem2_GetSatus(Uint8 MasterId);
	函数说明:
	  本函数是获取特定核的信号量状态寄存器信息
	参数说明:
	  MasterId：核ID

	返回值:
	   特定核信号量标志状态寄存器的状态

	备注: 只有间接获取或复合获取时才会影响该寄存器的值，
	      第n位代表第n个信号量的获取状态，如果为1，
	      说明该核收到过该信号量的获取中断，但不代表现在仍拥有该信号量
*********************************************************************/
Uint32 C6678_Sem2_GetSatus(Uint8 MasterId);
/********************************************************************
	函数声明

	所在文件:   sem2.c
	作者:	    dyx

	函数名:
		Bool C6678_Sem2_IntcStatusClear(Uint8 MasterId,Uint8 Sem2Id);
	函数说明:
		本函数是清除sem2的中断信息，以便于下次sem2的中断响应
	参数说明:
		MasterId：核ID
		Sem2Id：信号量号(0-31)

	返回值:
		TRUE:清除成功
		FALSE:清除失败

	备注: 此函数是在核响应完sem2中断后调用（例如，在中断服务程序内）
*********************************************************************/
Bool C6678_Sem2_IntcStatusClear(Uint8 MasterId,Uint8 Sem2Id);
/********************************************************************
	函数声明

	所在文件:   sem2.c
	作者:	    dyx

	函数名:
	  Bool C6678_Sem2_Release(Uint8 Sem2Id);
	函数说明:
	  本函数是释放已获取的信号量，注意：那个核占有那个核释放
	参数说明:

	  Sem2Id：信号量号(0-31)

	返回值:
	   TRUE:释放成功
	   FALSE:释放失败

	备注: 在调用此函数之前，必须有调用 C6678_Sem2_Acquire（）获取过相应的信号量，否则会出错
*********************************************************************/
Bool C6678_Sem2_Release(Uint8 Sem2Id);


/********************************************************************
	函数声明

	所在文件:   sem2.c
	作者:	    dyx

	函数名:
	  void C6678_Sem2_MultiCore_Syn(void);
	函数说明:该函数为基于信号量的多核同步函数。

	参数说明:
            无

	返回值:
	   无

	备注:该函数占用第20号及往后的共10个信号量
*********************************************************************/
void C6678_Sem2_MultiCore_Syn(void);
/********************************************************************
	文件说明:	以下这部分函数是edma3.c中函数的声明
						edma3的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Global_Init(void)

	函数说明: 该函数在所有其他edma3函数调用之前调用

	参数说明:无

	返回值:
	  TRUE:初始化成功
	  FALSE:初始化失败

	备注: 无
*********************************************************************/
Bool C6678_Edma3_Global_Init(void);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Channel_Init(Uint8 ChannelCtrlNum,Uint32 ChannelNum,Uint8 TCNum,Int8 ShadowRegion,Uint32 ParamNum)

	函数说明:
	      该函数在C6678_Edma3_Global_Init()之后，在C6678_Edma3_Transdata_continue/
	      C6678_Edma3_Transdata_interval/C6678_Edma3_Link_Transdata()/C6678_Edma3_Sort_TransData/
	      C6678_Edma3_SubframeExtraction_TransData之前调用，配置用于数据传输的DMA控制器，
	      通道，通道配置参数等

	参数说明:
	  ChannelCtrlNum：DMA控制器号(0~2)
	  ChannelNum：DMA通道号(注意0号控制器只有16个通道，1/2号控制器有64个通道）
	  	  	  	  (DMA控制器为0时：0~15；DMA控制器为1,2时：0~63)
	  TCNum:
	  	  如果ChannelCtrlNum为0，TCNum可选择0,1；
	  	  如果ChannelCtrlNum为1或2，TCNum可选择0,1,2,3；
	  ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
		ParamNum：传输参数PARAM号（0~511）
	返回值:
	  TRUE:初始化成功
	  FALSE:初始化失败

	备注: 多核多通道时，统一由核0做dma初始化
*********************************************************************/
Bool C6678_Edma3_Channel_Init(Uint8 ChannelCtrlNum,Uint32 ChannelNum,Uint8 TCNum,Int8 ShadowRegion,Uint32 ParamNum);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Channel_Close(Uint8 ChannelCtrlNum, Uint8 ChannelNum)

	函数说明:该函数在C6678_Edma3_Transdata_continue/C6678_Edma3_Transdata_interval
	  /C6678_Edma3_Link_Transdata()/C6678_Edma3_Sort_TransData/
	  C6678_Edma3_SubframeExtraction_TransData之后调用，关闭调用的DMA通道

	参数说明:
	ChannelCtrlNum:通道控制器号
	ChanNum：待关闭的EDMA3通道号

	返回值:
	  TRUE:关闭成功
	  FALSE:关闭失败

	备注: 无
*********************************************************************/
Bool C6678_Edma3_Channel_Close(Uint8 ChannelCtrlNum,Uint8 ChannelNum);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Ctrl_Close(Uint8 ChannelCtrlNum)

	函数说明:该函数在C6678_Edma3_Channel_Close()之后调用，关闭指定的DMA控制器

	参数说明:
	CtrlNum：待关闭的EDMA3控制器号

	返回值:
	  TRUE:关闭成功
	  FALSE:关闭失败

	备注: 无
*********************************************************************/
Bool C6678_Edma3_Ctrl_Close(Uint8 ChannelCtrlNum);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Transdata_continue(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,
	            Uint32 ParamNum,Uint32 sAddr, Uint32  dAddr, Uint16 Len_KB, Bool DstIsFifo, Bool IntrEn,Uint8 nTCC)

	函数说明: 本函数是DMA通用的连续地址的数据传输函数
	  
	参数说明:
	   ChannelCtrlNum：DMA控制器号
	   ChannelNum：DMA通道号(注意0号控制器只有16个通道，1/2号控制器有64个通道
	   ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
	   ParamNum：传输参数PARAM号
	   sAddr: 源地址；
	   dAddr: 目的地址；
	   Len_KB: 传输长度(注意以KB为单位!)；传2KB的数据Len_KB=2
	   dstIsFifo:
		  =1：目的为fifo；=0：源为fifo；=其他值：目的与源均为RAM；
	   intrEn:
		  =1：DMA中断使能，不等待DMA传输结束；=0：无DMA中断，等待DMA传输结束再退出;
		  =其他值：无DMA中断，不等待DMA传输结束。
	   nTCC: edma3传输完成info(0-63),需要和ChannelNum一致

	返回值:
	   TRUE:传输正常
	   FALSE：传输失败

	备注: AB mode,global region,最大长度为64MB；该函数在BoolC6678_Edma3_Global_Init();
和 Bool C6678_Edma3_Channel_Init();函数调用之后调用
*********************************************************************/
Bool C6678_Edma3_Transdata_continue(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,Uint32 sAddr, Uint32 dAddr,Uint16 Len_KB,Bool DstIsFifo,Bool IntrEn,Uint8 nTCC);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Transdata_interval(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion, Uint32 ParamNum,
	            DmaTranParam  Srcset , DmaTranParam  Dstset , Bool DstIsFifo, Bool IntrEn ,Uint8 nTCC)

	函数说明:  本函数是DMA通用的跳变地址的数据传输函数

	参数说明:
	   ChannelCtrlNum：DMA控制器号
	   ChannelNum：DMA通道号(注意0号控制器只有16个通道，1/2号控制器有64个通道)
	   ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
	   ParamNum：传输参数PARAM号
	   Srcset: 源地址传输参数的配置，具体参见结构体说明；
	   Dstset: 目的地址传输参数的配置，具体参见结构体说明；
	   DstIsFifo:
		  =1：目的为fifo；=0：源为fifo；=其他值：目的与源均为RAM；
	   IntrEn:
		  =1：DMA中断使能，不等待DMA传输结束；=0：无DMA中断，等待DMA传输结束再退出;
		  =其他值：无DMA中断，不等待DMA传输结束。
       nTCC: edma3传输完成info(0-63),需要和ChannelNum一致
	返回值:
	  TRUE:传输成功
	  FALSE：传输失败
	备注: 支持AB，ABC模式 ,当为 ABC模式 时 （即 Srcset.Ccnt>1）nTCC=ChannelNum；该函数在BoolC6678_Edma3_Global_Init();
Bool C6678_Edma3_Channel_Init();函数调用之后调用 
*********************************************************************/
Bool C6678_Edma3_Transdata_interval(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,DmaTranParam  Srcset,DmaTranParam  Dstset,Bool DstIsFifo,Bool IntrEn,Uint8 nTCC);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Link_Transdata(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,Uint8 ParamCnt,DmaTranParam * pSrcset,
	           DmaTranParam * pDstset,Bool DstIsFifo,Bool IntrEn,Uint8 nTCC)

	函数说明:  本函数是DMA link 数据传输函数

	参数说明:
	  ChannelCtrlNum：DMA控制器号
	  ChannelNum：DMA通道号(注意0号控制器只有16个通道，1/2号控制器有64个通道)
	  ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
	  ParamNum:用于传输的第一个传输参数PARAM号
	  ParamCnt：LINK的PARAM个数，该函数将会占有从ParamNum号开始的ParamCnt个PARAM
	  pSrcset：源地址传输参数的配置参数数组指针，数组维数=ParamCnt
	  pDstset：目的地址传输参数的配置参数数组指针，数组维数=ParamCnt
	  DstIsFifo： =1：目的为fifo；=0：源为fifo；=其他值：目的与源均为RAM；
	  IntrEn：=1：DMA中断使能，不等待DMA传输结束；=0：无DMA中断，等待DMA传输结束再退出;
		  =其他值：无DMA中断，不等待DMA传输结束。
      nTCC:edma3传输完成info,需要和ChannelNum一致

	返回值:
	  TRUE:传输成功
	  FALSE：传输失败

	备注: 该函数调用之前需调用C6678_Edma3_Global_Init（）和C6678_Edma3_Channel_Init（）做初始化
	当为 ABC模式 时 （即 Srcset.Ccnt>1）nTCC=ChannelNum
*********************************************************************/
Bool C6678_Edma3_Link_Transdata(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,Uint8 ParamCnt,DmaTranParam * pSrcset,DmaTranParam * pDstset,Bool DstIsFifo,Bool IntrEn,Uint8 nTCC);
                                 
/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Chain_Init(Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaChainParam * ChainChPa)

	函数说明: 本函数是DMA chaining 传输的初始化函数

	参数说明:
	  ChannelCtrlNum：DMA控制器号
	  ChainCnt：需要chaining 的DMA通道总数(注意0号控制器只有16个通道，1和2号控制器有64个通道）
	  ChainChPa:chain节点使用的dma通道号,param号,TC号,ShadowRegion参数的数组指针，数组维数=ChainCnt

	返回值:
	  TRUE:初始化成功
	  FALSE：初始化失败

	备注: 只有核0调用,注意0号控制器只有16个通道和128个param，16个通道共用这128个param
	      1和2号控制器有64个通道和512个param，64个通道共用这512个param
		注意每一通道使用的param号应该与其他通道不同。
		注意如果N个核调用，每个核调用的通道号应该与其他核不同。
		如果启动N个chain，每个chain调用的通道号应该与其他chain不同。
*********************************************************************/
Bool C6678_Edma3_Chain_Init(Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaChainParam * ChainChPa);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Chain_TransData (Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaTranParam * pSrcset,DmaTranParam * pDstset,DmaChainParam * ChainChPa,Bool DstIsFifo, Bool IntrEn,Uint32 nTCC)

	函数说明:本函数是chaining DMA传输函数

	参数说明:
	 pSrcset：源地址传输参数的配置参数数组指针，数组维数=ChainCnt
	 pDstset：目的地址传输参数的配置参数数组指针，数组维数=ChainCnt
	 DstIsFifo： =1：目的为fifo；=0：源为fifo；=其他值：目的与源均为RAM；
	 IntrEn：=1：DMA中断使能，不等待DMA传输结束；=0：无DMA中断，等待DMA传输结束再退出;
		  =其他值：无DMA中断，不等待DMA传输结束。
	 ChannelCtrlNum：DMA控制器号
	 ChainCnt：需要chaining 的DMA通道总数(注意0号控制器只有16个通道，1和2号控制器有64个通道）
	 ChainChPa:chain节点使用的dma通道号和param号参数的数组指针，数组维数=ChainCnt
     nTCC:  传输完成代码 TCC info 信息,需要和其中一个用过的ChannelNum一致
	返回值:
	  TRUE:传输成功
	  FALSE：传输失败

	备注: 该函数调用前需调用C6678_Edma3_Chain_Init()，不支持ABC传输，即pSrcset[].Ccnt=1，
		该函数的ChainChPa参数数据应该与C6678_Edma3_Chain_Init的ChainChPa参数数据一致。
	 	 注意0号控制器只有16个通道和128个param，16个通道共用这128个param。
	    1和2号控制器有64个通道和512个param，64个通道共用这512个param。
		注意每一通道使用的param号应该与其他通道不同。
		注意如果N个核调用，每个核调用的通道号应该与其他核不同。
		如果启动N个chain，每个chain调用的通道号应该与其他chain不同。
*********************************************************************/
Bool C6678_Edma3_Chain_TransData (Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaTranParam * pSrcset,DmaTranParam * pDstset,DmaChainParam * ChainChPa,Bool DstIsFifo, Bool IntrEn,Uint32 nTCC);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Chain_Close(Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaChainParam * ChainChPa)
	  
	函数说明:关闭调用的DMA资源
	  
	参数说明:
	  ChannelCtrlNum：DMA控制器号
	  ChainCnt：需要chaining 的DMA通道总数(注意0号控制器只有16个通道，1/2号控制器有64个通道
	  ChainChPa:chain节点使用的dma通道号和param号参数的数组指针，数组维数=ChainCnt
	  
	返回值:
	  TRUE:关闭成功
	  FALSE:关闭失败

	备注: 该函数是在C6678_Edma3_Chain_TransData()之后调用
	该函数的ChainChPa参数数据应该与C6678_Edma3_Chain_Init的ChainChPa参数数据一致。
*********************************************************************/
Bool C6678_Edma3_Chain_Close(Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaChainParam * ChainChPa);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_SubframeExtraction_TransData (Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,
	           DmaSubFrameExtractTranParam  SubFraExtrTranParam,Uint8 nTCC)
	  
	函数说明:该函数是采用DMA从大的数据矩阵中提取子矩阵
	  
	参数说明:
	  ChannelCtrlNum：DMA控制器号
      ChannelNum：DMA通道号(注意0号控制器只有16个通道，1/2号控制器有64个通道)
      ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
      ParamNum:用于传输的第一个传输参数PARAM号
	  SubFraExtrTranParam：DMA子帧提取传输参数，具体请参见结构体
	  nTCC:edma 传输完成info,需要和ChannelNum一致

	返回值:
	   TRUE:传输成功
	  FALSE：传输失败

	备注: 该函数调用之前需调用C6678_Edma3_Global_Init（）和C6678_Edma3_Channel_Init（）初始化
*********************************************************************/
Bool C6678_Edma3_SubframeExtraction_TransData (Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,DmaSubFrameExtractTranParam  SubFraExtrTranParam,Uint8 nTCC);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Over_Hookup(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 coreNum,Uint8 EventId,Uint8 VectID,void interruptISR())

	函数说明:
	  本函数是挂edma3传输完成中断

	参数说明:

	  ChannelCtrlNum：DMA传输通道控制器号（0~2）
	  ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
	  coreNum:本地核ID
	  EventId：核级中断事件号
	  	      该核级中断事件号，是指的片级控制器给核级控制器的输入。
	  	      每个核都有17个只给本核发送的片级控制器输入，
	  	    （0~16）
	  	    其中，0~10代表21~31核级事件，11~12代表62~63核级事件,13~16代表92~95核级事件
	  VectID:中断向量号（4-15）
	  interruptISR：中断服务函数
	返回值:
	  true:成功
	  false：失败

	备注: 只由核0来调用，调用此函数前需先调用 C6678_CoreInt_Init ()和C6678_ChipInt_Init (0);
*********************************************************************/
Bool C6678_Edma3_Over_Hookup(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 coreNum,Uint8 EventId,Uint8 VectID,void interruptISR());

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:void C6678_Edma3_Wait_TransData_over(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 nTCC)

	函数说明:
	  本函数是等待DMA传输完成函数，并清除中断事件（TCC）状态

	参数说明:
	  ChannelCtrlNum：DMA控制器号（0~2）
	  ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
	  nTCC:传输完成info。
	返回值:
	  TRUE:初始化成功
	  FALSE：初始化失败

	备注: 如果Edma3传输模式为不等待传输结束，则在下次edma3传输事件之前需调用该函数清除该次传输TCC
*********************************************************************/
void C6678_Edma3_Wait_TransData_over(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 nTCC);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:Bool C6678_Edma3_Over_Intc_Clear(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 EventId)

	函数说明:
	  本函数清除edma3中断事件状态

	参数说明:
	  ChannelCtrlNum：DMA控制器号（0~2）
	  ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
	  EventId：核级中断事件号
	  	      该核级中断事件号，是指的片级控制器给核级控制器的输入。
	  	      每个核都有17个只给本核发送的片级控制器输入，
	  	    （0~16）
	  	    其中，0~10代表21~31核级事件，11~12代表62~63核级事件,13~16代表92~95核级事件
	返回值:
	  无

	备注: 该函数在edma3中断服务程序中，只由核0来调,该函数用到的核级事件，请避免重复使用。
*********************************************************************/
Bool C6678_Edma3_Over_Intc_Clear(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 EventId);

/********************************************************************
	函数声明

	所在文件:   edma3.c
	作者:	    dyx

	函数名:void C6678_Edma3_Get_TCCInfo(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint64edmaTccInfo *edma3IntrInfo)

	函数说明:
	  本函数是获取Edma TCC info 信息，TCC与edma3IntrInfo对应关系：
	  TCC：                                                                  edma3IntrInfo->infoL：
      0                             1
      1                             2
      2                             4
      .                             .
      .                             .
      31                            2^31
      TCC：                                                                  edma3IntrInfo->infoH：
      0                             1
      1                             2
      2                             4
      .                             .
      .                             .
      31                            2^31

            即：若edma3IntrInfo为3 ,则表明有两次edma传输结束，分别为TCC=0,TCC=1的两次传输，以此类推
	参数说明:
	  ChannelCtrlNum：DMA控制器号（0~2）
	  ShadowRegion: SHADOW Regions的区域号
	  	  -1 ： CSL_EDMA3_REGION_GLOBAL
		   0：  CSL_EDMA3_REGION_0
		   1：  CSL_EDMA3_REGION_1
		   2：  CSL_EDMA3_REGION_2
		   3：  CSL_EDMA3_REGION_3
		   4：  CSL_EDMA3_REGION_4
		   5：  CSL_EDMA3_REGION_5
		   6：  CSL_EDMA3_REGION_6
		   7：  CSL_EDMA3_REGION_7
      edma3IntrInfo:TCC info 信息指针
	返回值:
	  无

	备注: 启动下次DMA传输时，需清除相应的TCC info
*********************************************************************/
void C6678_Edma3_Get_TCCInfo(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint64edmaTccInfo *edma3IntrInfo);

/********************************************************************
	文件说明:	以下这部分函数是timer.c中函数的声明
						timer的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
	  void C6678_Timer_Init(void)
	函数说明:
	  本函数是初始化TIMER模块
	参数说明:
	  无
	返回值:
	  无
	备注:
*********************************************************************/
void C6678_Timer_Init(void);

/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
	  Bool C6678_Timer_Set(Uint8 TimerNum,Uint8 TimerMode,Uint64Cnt Counter,Uint64Prd Period,Uint4Prescale Prescale,Uint32 ClkinMode,Uint8 PulseWidth)
	函数说明:
	  本函数是设置一个定时器
	参数说明:
	  TimerNum：定时器号（共16个，0-15，其中0到7可配成任意定时器，而8-15不能配成watch dog定时器）
	  TimerMode:定时器种类。
	  	  	  0：watch dog定时器
	  	  	  1：gp定时器
	  	  	  2: 链式32bit定时器
			  3：低位对应的定时器工作。两个独立的32bit定时器
			  4：高位对应的定时器工作。两个独立的32bit定时器
	  Counter：定时器的初始值。
	  Period：定时器的周期值。
	  Prescale:只有TimerMode==4时，这个参数才有效。pre定时器的初始值和周期值。
	  ClkinMode:输入时钟的种类。
			0：来自内部时钟。
			1：来自外部时钟。
	  PulseWidth：1：脉冲的，1个时钟周期。
				2：脉冲的，2个时钟周期。
				3：脉冲的，3个时钟周期。
				4：脉冲的，4个时钟周期。
	返回值:
	  true:设置成功
	  false：设置失败
	备注: 只能将timer0到7设为watch dog 定时器。
	watch dog定时器是连续模式，时钟必须来自内部时钟。
*********************************************************************/
Bool C6678_Timer_Set(Uint8 TimerNum,Uint8 TimerMode,Uint64Cnt Counter,Uint64Prd Period,Uint4Prescale Prescale,Uint32 ClkinMode,Uint8 PulseWidth);
/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
		Bool C6678_Timer_Start(Uint8 TimerNum,Uint8 TimerOutMode)
	函数说明:
		本函数启动计数器
	参数说明:
		TimerNum:定时器号（只能是0到7）
		TimerOutMode:输出种类
	  	  	0：pulse mode
	  	  	1：continuously mode
	返回值:
		true:启动成功
	  	false:启动失败
	备注:
		无
*********************************************************************/
Bool C6678_Timer_Start(Uint8 TimerNum,Uint8 TimerOutMode);
/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
	  Uint32 C6678_Timer_GetCountHi32(Uint8 TimerNum)
	函数说明:
	  本函数得到运行计数器的高32bit(CNTHI)
	参数说明:
	  TimerNum:定时器号（0到15）
	返回值:
	  运行周期计数器的高32bit
	备注: 无
*********************************************************************/
Uint32 C6678_Timer_GetCountHi32(Uint8 TimerNum);
/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
	  Uint32 C6678_Timer_GetCountLo32(Uint8 TimerNum)
	函数说明:
	  本函数得到运行计数器的低32bit(CNTHI)
	参数说明:
	  TimerNum:定时器号（0到15）
	返回值:
	  运行周期计数器的低32bit
	备注: 无
*********************************************************************/
Uint32 C6678_Timer_GetCountLo32(Uint8 TimerNum);
/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
	  void C6678_Timer_Close(Uint8 TimerNum)
	函数说明:
	  本函数是关闭TIMER模块
	参数说明:
		TimerNum:定时器号（0到15）
	返回值:
	  无
	备注:
*********************************************************************/
void C6678_Timer_Close(Uint8 TimerNum);

/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
	  Bool C6678_Timer_Intc_Hookup(Uint8 TimerNum,Uint8 TimerMode,Uint8 VectID,void interruptISR())
	函数说明:
	  本函数是挂TIMER中断
	参数说明:
		TimerNum：定时器号（共16个，0-15，其中0到7可配成任意定时器，而8-15不能配成watch dog定时器;
						      0号定时器可以被核0响应，
						      1号定时器可以被核1响应,
						      ......
						      7号定时器可以被核7响应，
						      8-15号定时器可以被8个核都响应
						      	  ）
		TimerMode:定时器种类。
	  	  	  0：watch dog定时器
	  	  	  1：gp定时器
	  	  	  2: 链式32bit定时器
			  3：低位对应的定时器工作。两个独立的32bit定时器
			  4：高位对应的定时器工作。两个独立的32bit定时器
		interruptISR()：中断服务程序
		VectID:中断向量号（4-15）
	返回值:
	  无
	备注:无
*********************************************************************/
Bool C6678_Timer_Intc_Hookup(Uint8 TimerNum,Uint8 TimerMode,Uint8 VectID,void interruptISR());
/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
	  Bool C6678_Timer_Intc_Clear(Uint8 TimerNum,Uint8 TimerMode)
	函数说明:
	  本函数是挂TIMER中断
	参数说明:
		TimerNum：定时器号（共16个，0-15，其中0到7可配成任意定时器，而8-15不能配成watch dog定时器）
		TimerMode:定时器种类。
	  	  	  0：watch dog定时器
	  	  	  1：gp定时器
	  	  	  2: 链式32bit定时器
			  3：低位对应的定时器工作。两个独立的32bit定时器
			  4：高位对应的定时器工作。两个独立的32bit定时器
	返回值:
	  无
	备注:无
*********************************************************************/
Bool C6678_Timer_Intc_Clear(Uint8 TimerNum,Uint8 TimerMode);

/********************************************************************
	函数声明

	所在文件:   timer.c
	作者:		wj

	函数名:
	  void C6678_Timer_Watchdog_Feed(Uint8 TimerNum)
	函数说明:
	  本函数是给watchdog定时器喂狗
	参数说明:
		TimerNum：定时器号（共16个，0-15，其中0到7可配成任意定时器，而8-15不能配成watch dog定时器）
	返回值:
	  无
	备注:无
*********************************************************************/
void C6678_Timer_Watchdog_Feed(Uint8 TimerNum);
/********************************************************************
	文件说明:	以下这部分函数是hyperlink.c中函数的声明
						hyperlink的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   hyperlink.c
	作者:		wj

	函数名:
	  Bool C6678_Hypelink_Init(unsigned int SerRate);
	函数说明:
	  本函数是初始化hyperlink
	参数说明:
	  SerRate:Hyperlink输出速率(见c6678.h中的HYPLNK_RATE)
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
Bool C6678_Hypelink_Init(unsigned int SerRate);
/********************************************************************
	函数声明

	所在文件:   hyperlink.c
	作者:		wj

	函数名:
	  Bool C6678_Hypelink_AddrMap(HyplnkAddrMapParam * phyplnkParam);
	函数说明:
	  本函数实现hyperlink的内存映射
	参数说明:
	  phyplnkParam:hyperlink地址转换的参数配置（见c6678.h中的HyplnkAddrMapParam的定义）
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 无
*********************************************************************/
Bool C6678_Hypelink_AddrMap(HyplnkAddrMapParam * phyplnkParam);
/********************************************************************
	函数声明

	所在文件:   hyperlink.c
	作者:		wj

	函数名:
		Bool C6678_Hypelink_Int_Hookup(Uint8 VectID,void interruptISR())

	函数说明:
		本函数是挂hyperlink中断

	参数说明:
		interruptISR：中断函数
		VectID:中断向量号（4-15）
	返回值:
		true:成功
		false：失败

	备注: 需先调用
		C6678_CoreInt_Init ();
		C6678_ChipInt_Init (0);
*********************************************************************/
Bool C6678_Hypelink_Int_Hookup(Uint8 VectID,void interruptISR());
/********************************************************************
	函数声明

	所在文件:   hyperlink.c
	作者:		wj

	函数名:
		Uint32 C6678_Hypelink_Int_Info(void)

	函数说明:
		本函数是获取中断的info值

	参数说明:
		无

	返回值:
		正确返回：info值(32位数据)
		错误返回：0xff

	备注:在清中断前调用。
	C6678_Hypelink_Int_Trigger(info)中的info和此函数返回的info值的对应关系为
	C6678_Hypelink_Int_Trigger(0)对应0x00000001=C6678_Hypelink_Int_Info(void)
	C6678_Hypelink_Int_Trigger(1)对应0x00000002=C6678_Hypelink_Int_Info(void)
	......
	C6678_Hypelink_Int_Trigger(31)对应0x80000000=C6678_Hypelink_Int_Info(void)
*********************************************************************/
Uint32 C6678_Hypelink_Int_Info(void);
/********************************************************************
	函数声明

	所在文件:   hyperlink.c
	作者:		wj

	函数名:
		Bool C6678_Hypelink_Int_Clear(void)

	函数说明:
		本函数是清中断

	参数说明:
		无

	返回值:
		TRUE:正确返回
		FALSE:错误返回

	备注:无
*********************************************************************/
Bool C6678_Hypelink_Int_Clear(void);
/********************************************************************
	函数声明

	所在文件:   hyperlink.c
	作者:		wj

	函数名:
		Bool C6678_Hypelink_Int_Trigger(Uint8 info)

	函数说明:
		本函数是发hyperlink中断

	参数说明:
		info:info值(0到31)

	返回值:
		true:成功
		false：失败

	备注:Hyperlink中断发送速度较快，多次发送Hyperlink中断时，
		应确保上次中断已经响应完毕，否则会出现中断丢失现象
*********************************************************************/
Bool C6678_Hypelink_Int_Trigger(Uint8 info);
/********************************************************************
	文件说明:	以下这部分函数是cache.c中函数的声明						
*********************************************************************/

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:void C6678_Cache_Enable(Uint8 RegionNum)

	函数说明:本函数是使能指定地址段的cache预取功能

	参数说明:
	  RegionNum：地址区域号，具体可参见手册。
	  
	返回值:无

	备注: 无
*********************************************************************/
void C6678_Cache_Enable(Uint8 RegionNum);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:void C6678_Cache_Disable(Uint8 RegionNum)

	函数说明:本函数是失效指定地址段的cache预取功能

	参数说明:
	  RegionNum：地址区域号，具体可参见手册。
	  
	返回值:无

	备注: 无
*********************************************************************/
void C6678_Cache_Disable(Uint8 RegionNum);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:Bool C6678_Cache_Inv(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod)

	函数说明:本函数是将指定地址段的cache预取值失效。

	参数说明:     
		BlockPtr：地址区域的起始地址指针
    ByteCnt：地址区域的大小（单位为字节,不能为0）
	  Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      CACHE_NOWAIT:不等待操作完成
 

	返回值:
		true:正常
	  	false：失败

	备注: 此操作后，核再读该段地址区域数据时cache命失。
*********************************************************************/
Bool C6678_Cache_Inv(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:Bool C6678_Cache_Wb(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod)

	函数说明:该函数是将指定地址区域中 被cache到的数据写回该段地址区域

	参数说明:
	  Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      CACHE_NOWAIT:不等待操作完成
      BlockPtr：地址区域的起始值
      ByteCnt：地址区域的大小（单位为字节,不能为0）

	返回值:
		true:正常
	  	false：失败

	备注: 无
*********************************************************************/
Bool C6678_Cache_Wb(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:Bool C6678_Cache_Freeze(Uint8 MemType)

	函数说明:本函数是cache锁定函数，调用本函数后，当前该段cache里的数据将不会
	                      被驱逐出去。

	参数说明:
      MemType: 0=L1D cache,1=L1P cache ,2=L2 cache

	返回值:
		true:正常
	  	false：失败

	备注: 无
*********************************************************************/
Bool C6678_Cache_Freeze(Uint8 MemType);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:Bool C6678_Cache_Unfreeze(Uint8 MemType)

	函数说明:本函数是cache解锁函数

	参数说明:
     MemType: 0=L1D cache,1=L1P cache ,2=L2 cache

	返回值:
		true:正常
	  	false：失败

	备注: 无
*********************************************************************/
Bool C6678_Cache_Unfreeze(Uint8 MemType);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:Bool C6678_Cache_Set(Uint8 MemType,Uint8 size)

	函数说明:本函数是设置 L1D/L1P/L2 cache的大小

	参数说明:
	MemType: 0=L1D cache,1=L1P cache ,2=L2 cache
	size:L1P/L1D cache 的大小为0~32K
		 L1 cache size:
		 0= L1_0KCACHE ,
		 1= L1_4KCACHE ,
		 2= L1_8KCACHE ,
		 3= L1_16KCACHE,
		 4= L1_32KCACHE,
         L2 cache 的大小为0~512K
		 L2 Cache size:
		 0=L2_0KCACHE  ,
		 1=L2_32KCACHE ,
		 2=L2_64KCACHE ,
		 3=L2_128KCACHE,
		 4=L2_256KCACHE,
		 5=L2_512KCACHE,
		 6=L2_1024KCACHE.
	返回值:无

	备注: 无
*********************************************************************/
Bool C6678_Cache_Set(Uint8 MemType,Uint8 size);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:void C6678_Cache_Wb_Wait(void)

	函数说明:本函数是等待C6678_Cache_Wb(...,...,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_Wb(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_Wb_Wait(void);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    dyx

	函数名:void C6678_Cache_Inv_Wait(void)

	函数说明:本函数是等待C6678_Cache_Inv(...,..,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_Inv(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_Inv_Wait(void);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbInv(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod)

	函数说明:本函数是将指定地址段的cache数据写回并将预取值失效。

	参数说明:
      BlockPtr：地址区域的起始地址指针
      ByteCnt：地址区域的大小（单位为字节,不能为0）
      Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      	  CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      	  CACHE_NOWAIT:不等待操作完成

	返回值:
		true:正常
	  	false：失败

	备注: 此操作后，核再读该段地址区域数据时cache命失。
*********************************************************************/
Bool C6678_Cache_WbInv(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbInv_Wait(void)

	函数说明:本函数是等待C6678_Cache_WbInv(...,..,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_WbInv(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_WbInv_Wait(void);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_InvAllL1d(Uint8 Mod)

	函数说明:本函数是将L1D上所有cache的预取值失效。

	参数说明:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      	  CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      	  CACHE_NOWAIT:不等待操作完成

	返回值:
		true:正常
	  	false：失败

	备注: 此操作后，核再读该段地址区域数据时cache命失。
*********************************************************************/
Bool C6678_Cache_InvAllL1d(Uint8 Mod);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_InvAllL1d_Wait(void)

	函数说明:本函数是等待C6678_Cache_InvAllL1d(...,..,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_InvAllL1d(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_InvAllL1d_Wait(void);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbAllL1d(Uint8 Mod)

	函数说明:本函数是将L1D上所有被cache到的数据写回该段地址区域。

	参数说明:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      	  CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      	  CACHE_NOWAIT:不等待操作完成

	返回值:
		true:正常
	  	false：失败

	备注: 无。
*********************************************************************/
Bool C6678_Cache_WbAllL1d(Uint8 Mod);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbAllL1d_Wait(void)

	函数说明:本函数是等待C6678_Cache_WbAllL1d(...,..,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_WbAllL1d(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_WbAllL1d_Wait(void);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbInvAllL1d(Uint8 Mod)

	函数说明:本函数是将L1D上所有被cache到的数据写回该段地址区域并将cache的预取值失效。。

	参数说明:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      	  CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      	  CACHE_NOWAIT:不等待操作完成

	返回值:
		true:正常
	  	false：失败

	备注: 此操作后，核再读该段地址区域数据时cache命失。
*********************************************************************/
Bool C6678_Cache_WbInvAllL1d(Uint8 Mod);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbInvAllL1d_Wait(void)

	函数说明:本函数是等待C6678_Cache_WbInvAllL1d(...,..,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_WbInvAllL1d(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_WbInvAllL1d_Wait(void);

/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_InvAllL2(Uint8 Mod)

	函数说明:本函数是将L2上所有cache的预取值失效。

	参数说明:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      	  CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      	  CACHE_NOWAIT:不等待操作完成

	返回值:
		true:正常
	  	false：失败

	备注: 此操作后，核再读该段地址区域数据时cache命失。
*********************************************************************/
Bool C6678_Cache_InvAllL2(Uint8 Mod);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_InvAllL2_Wait(void)

	函数说明:本函数是等待C6678_Cache_InvAllL2(...,..,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_InvAllL2(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_InvAllL2_Wait(void);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbAllL2(Uint8 Mod)

	函数说明:本函数是将L2上所有被cache到的数据写回该段地址区域。

	参数说明:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      	  CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      	  CACHE_NOWAIT:不等待操作完成

	返回值:
		true:正常
	  	false：失败

	备注: 无。
*********************************************************************/
Bool C6678_Cache_WbAllL2(Uint8 Mod);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbAllL2_Wait(void)

	函数说明:本函数是等待C6678_Cache_WbAllL2(...,..,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_WbAllL2(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_WbAllL2_Wait(void);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbInvAllL2(Uint8 Mod)

	函数说明:本函数是将L2上所有被cache到的数据写回该段地址区域并将cache的预取值失效。。

	参数说明:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: 调用MFENCE指令等待操作完成。
      	  CACHE_WAIT:等待cache的状态寄存器指示操作完成。
      	  CACHE_NOWAIT:不等待操作完成

	返回值:
		true:正常
	  	false：失败

	备注: 此操作后，核再读该段地址区域数据时cache命失。
*********************************************************************/
Bool C6678_Cache_WbInvAllL2(Uint8 Mod);
/********************************************************************
	函数声明

	所在文件:   cache.c
	作者:	    wj

	函数名:C6678_Cache_WbInvAllL2_Wait(void)

	函数说明:本函数是等待C6678_Cache_WbInvAllL2(...,..,CACHE_NOWAIT)命令结束的函数

	参数说明:无

	返回值:无

	备注: 该函数是在C6678_Cache_WbInvAllL2(...,..,CACHE_NOWAIT)之后调用，等待该
	      cache操作命令完成
*********************************************************************/
void C6678_Cache_WbInvAllL2_Wait(void);
/********************************************************************
	文件说明:	以下这部分函数是qmss.c中函数的声明
						qmss的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   qmss.c
	作者:		wj

	函数名:
	  Bool C6678_Qmss_Init(void);
	函数说明:
	  本函数是初始化qmss
	参数说明:
	  无
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 统一由核0调用
*********************************************************************/
Bool C6678_Qmss_Init(void);

/********************************************************************
	函数声明

	所在文件:   qmss.c
	作者:		wj

	函数名:
	  Bool C6678_QMSS_Core_MemInsert(Uint8 coreNum);
	函数说明:
	  本函数是给QMSS插入内存空间
	参数说明:
	  coreNum:核的序号
	返回值:
	  true:初始化正常
	  false：初始化失败
	备注: 每个核都需调用，调用之前需调用 C6678_QMSS_Init();
*********************************************************************/
Bool C6678_QMSS_Core_MemInsert(Uint8 coreNum);
/********************************************************************
	文件说明:	以下这部分函数是cppi_drv.c中函数的声明
						cppi的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   cppi_drv.c
	作者:		wj

	函数名:
		Bool C6678_Cppi_init(void);

	函数说明:
		本函数是初始化cppi

	参数说明:
		无

	返回值:
		true:初始化正常
		false：初始化失败

	备注: 统一由核0调用
*********************************************************************/
Bool C6678_Cppi_init (void);

/********************************************************************
	文件说明:	以下这部分函数是ipc_navigator.c中函数的声明
						IPC的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   ipc_navigator.c
	作者:	    dyx

	函数名:Bool C6678_IPC_Navigator_coreInit(Uint8 coreNum)

	函数说明: 该函数是IPC（基于多核导航的核间通信）初始化函数

	参数说明:
       coreNum:本地核ID

	返回值:
	  TRUE:初始化成功
	  FALSE:初始化失败
	备注: 该函数 由参与核间通信的各核分别调用,保证各核都初始化结束后再进行多核之间的通信
*********************************************************************/
Bool C6678_IPC_Navigator_coreInit(Uint8 coreNum);

/********************************************************************
	函数声明

	所在文件:   ipc_navigator.c
	作者:	    dyx

	函数名:void C6678_Flag_Multicore_Syn(Uint8 coreNum,Bool coreChoosed[])

	函数说明: 该函数是实现多核同步的函数

	参数说明:
	   coreNum:本地核ID
       coreChoosed[0]:
               TRUE:0核需要同步
               FALSE:0核不需要同步
       .....

 	   coreChoosed[7]:
               TRUE:7核需要同步
               FALSE:7核不需要同步
	返回值:
	  无
	备注: 该函数 由需要同步的核调用,需要同步的核的coreChoosed参数应该一致
	               注意：由于本函数中的flag使用的是一套，所以第一次调用该函数后，
	               需保证所有调用该函数的核都同步结束后，再第二次调用该函数，
	               否则由于第二次也使用相同的flag，可能会和第一次相互影响。
*********************************************************************/
void C6678_Flag_Multicore_Syn(Uint8 coreNum,Bool coreChoosed[]);

/********************************************************************
	函数声明

	所在文件:   ipc_navigator.c
	作者:	    dyx

	函数名:Bool C6678_IPC_Navigator_send(Uint8 destcoreNum,Uint8 *databuf,Uint32 databufLen)

	函数说明: 该函数实现通过IPC向指定核发送数据

	参数说明:
       destcoreNum:目的核ID
       databuf:发送数据缓冲区
       databufLen：发送数据长度

	返回值:
	  TRUE:发送成功
	  FALSE:发送失败
	备注: 该函数在C6678_IPC_Navigator_coreInit（）之后调用
*********************************************************************/
Bool C6678_IPC_Navigator_send(Uint8 destcoreNum,Uint8 *databuf,Uint32 databufLen);
/********************************************************************
	函数声明

	所在文件:  ipc_navigator.c
	作者:	    dyx

	函数名:void C6678_IPC_Intc_send(Uint8 destcoreNum,Uint8 info)

	函数说明: 该函数实现通过IPC向指定核发送通信信息

	参数说明:
       destcoreNum:目的核ID
       info:IPC通信信息，取值为0-27

	返回值:
	  无
	备注:当某个核调用该函数连续给其他多个核发送信息时，可能会出现其他核重复收
	            到该信息的情况：
    	解决方法：在该核给下一个核发送信息前，因确保当前目标核已收到此信息，
    	后再启动该核给下一个核发送信息。
*********************************************************************/
void C6678_IPC_Intc_send(Uint8 destcoreNum,Uint32 info);


/********************************************************************
	函数声明

	所在文件:   ipc_navigator.c
	作者:	    dyx

	函数名:Uint32 C6678_IPC_Intc_Rev(void)

	函数说明: 该函数实现检测IPC通信信息

	参数说明:
      	  无

	返回值:
	   info:检测的IPC附加信息位,取值为2的0次幂---2的27次幂。
	                                    发送                                                  和            接收                           的对应关系
			C6678_IPC_Intc_send(,0)           1=C6678_IPC_Intc_Rev()
			C6678_IPC_Intc_send(,1)           2=C6678_IPC_Intc_Rev()
			C6678_IPC_Intc_send(,2)           4=C6678_IPC_Intc_Rev()
			。。。。。。
			C6678_IPC_Intc_send(,27)          0x8000000=C6678_IPC_Intc_Rev()
	备注:调用该函数后，info会被清掉
*********************************************************************/
Uint32 C6678_IPC_Intc_Rev(void);

/********************************************************************
	函数声明

	所在文件:   ipc_navigator.c
	作者:	    dyx

	函数名:Bool C6678_IPC_Intc_HookUp(void interruptISR(),Uint8 VectID)

	函数说明: 该函数是实现IPC中断挂起

	参数说明:
	  interruptISR()：中断服务程序
      VectID:中断向量号（4-15）

	返回值:
	   TRUE:挂起成功
	   FALSE：挂起失败

	备注:调用此函数前需先调用 C6678_CoreInt_Init ()，各核均可调用
*********************************************************************/
Bool C6678_IPC_Intc_HookUp(void interruptISR(),Uint8 VectID);

/********************************************************************
	函数声明

	所在文件:   ipc_navigator.c
	作者:	    dyx

	函数名:void C6678_IPC_Intc_Clear(void)

	函数说明: 该函数是清除IPC中断状态

	参数说明:
	      无

	返回值:
	   无

	备注:各核均可调用
*********************************************************************/
void C6678_IPC_Intc_Clear(void);
/********************************************************************
	函数声明

	所在文件:   ipc_navigator.c
	作者:	    dyx

	函数名:Uint32 C6678_IPC_Navigator_recv(Uint8 *buffDataPtr,Uint8 mode)

	函数说明: 接收由IPC发来的数据到指定接收缓冲区

	参数说明:
       buffDataPtr：接收数据缓冲区
	   mode:
	       0:wait等待直到收到包，否则一直阻塞
	       other：不等待，只判断以此是否收到，不阻塞
	返回值:
	      返回接收的数据长度

	备注: 该函数在C6678_IPC_Navigator_coreInit（）之后由各核分别调用
*********************************************************************/
Uint32 C6678_IPC_Navigator_recv(Uint8 *buffDataPtr,Uint8 mode);

/********************************************************************
	函数声明

	所在文件:   ipc_navigator.c
	作者:	    dyx

	函数名:Bool C6678_IPC_Navigator_coreClose(void)

	函数说明: 关闭所有的IPC资源

	参数说明:
                  无

	返回值:
	   TRUE:关闭成功
	   FALSE：关闭失败

	备注: 该函数在IPC操作最后由各核分别调用
*********************************************************************/
Bool C6678_IPC_Navigator_coreClose(void);
/********************************************************************
	文件说明:	以下这部分函数是emif16_norflash.c中函数的声明
						EMIF_NORFLASH的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   emif16_norflash.c
	作者:		wj

	函数名:
	  Bool C6678_Emif16_Norflash_Init(unsigned int MainRate)
	函数说明:
	  本函数是初始化emif16_norflash
	参数说明:
	  MainRate:CPU主频
	  	PLATFORM_PLL1_PLLM_val(见c6678.h)
	返回值:
	   TRUE:成功
	   FALSE：失败
	备注: 无
*********************************************************************/
Bool C6678_Emif16_Norflash_Init(unsigned int MainRate);
/********************************************************************
	函数声明

	所在文件:   emif16_norflash.c
	作者:		wj

	函数名:
	 	 Bool C6678_Emif16_Norflash_ReadByte(Uint32 src, Uint32 dst, Uint32 length)
	函数说明:
	  本函数是将数据从nor_flash读出
	参数说明:
	  src：  nor_flash的地址
	  dst： 目的地址
	  length：读出的数据的长度(单位为16bit)
	返回值:
	   TRUE:成功
	   FALSE：失败
	备注: 无
*********************************************************************/
Bool C6678_Emif16_Norflash_ReadByte(Uint32 src, Uint32 dst, Uint32 length);
/********************************************************************
	函数声明

	所在文件:   emif16_norflash.c
	作者:		wj

	函数名:
	 	 Bool C6678_Emif16_Norflash_WriteByte(Uint32 src, Uint32 dst, Uint32 length)

	函数说明:
	  本函数是将数据写入nor_flash
	参数说明:
	  src：  源地址
	  dst： nor_flash的地址
	  length：写入的数据的长度(单位为16bit)
	返回值:
	   TRUE:成功
	   FALSE：失败
	备注: 写之前要先调用擦函数，否则出错
*********************************************************************/
Bool C6678_Emif16_Norflash_WriteByte(Uint32 src, Uint32 dst, Uint32 length);

/********************************************************************
	函数声明

	所在文件:   emif16_norflash.c
	作者:		wj

	函数名:
	  Bool C6678_Emif16_Norflash_BlockErase(Uint32 start, Uint32 length);
	函数说明:
	  本函数是将nor_flash的指定地址空间擦除
	参数说明:
	  start：  擦除的地址空间的起始地址
	  length： 擦除的地址空间的长度
	返回值:
	  true：擦除成功
	  false：擦除失败
	备注: 无
*********************************************************************/
/* Erase a segment of Flash memory */
Bool C6678_Emif16_Norflash_BlockErase(Uint32 start, Uint32 length);

/********************************************************************
	文件说明:	以下这部分函数是i2c_srioswitch.c中函数的声明
						I2C_SRIOSWITCH的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   i2c_srioswitch.c
	作者:		wj

	函数名:
	  void C6678_I2c_SrioSwitch_Init(void)
	函数说明:
	  本函数是初始化i2c_srioswitch
	参数说明:
	  无
	返回值:
	  无
	备注: 无
*********************************************************************/
void C6678_I2c_SrioSwitch_Init(void);
/********************************************************************
	函数声明

	所在文件:   i2c_srioswitch.c
	作者:		wj

	函数名:
	  I2C_RET C6678_I2c_SrioSwitch_Read ( uint32_t byte_addr,uint32_t *puiData, uint8_t uchEepromI2cAddress)
	函数说明:
	  本函数是从srioswitch中读出一定数量的数据
	参数说明:
	  uchEepromI2cAddress ：srioswitch的i2c地址
	  puiData：读出数据存储的数组的指针
	  byte_addr：数据在EEPROM的基地址（byte为单位）
	返回值:
		0：I2C_RET_OK
		1：I2C_RET_LOST_ARB
		2：I2C_RET_NO_ACK
		3：I2C_RET_IDLE_TIMEOUT
		4：I2C_RET_BAD_REQUEST
		5：I2C_RET_CLOCK_STUCK_LOW
		6：I2C_RET_NULL_PTR_ERROR
		99：I2C_RET_GEN_ERROR
	备注:
*********************************************************************/
I2C_RET C6678_I2c_SrioSwitch_Read ( uint32_t byte_addr,uint32_t *puiData, uint8_t uchEepromI2cAddress);
/********************************************************************
	函数声明

	所在文件:   i2c_srioswitch.c
	作者:		wj

	函数名:
	  I2C_RET C6678_I2c_SrioSwitch_Write( uint32_t byte_addr, uint8_t uchEepromI2cAddress, uint32_t puiData)
	函数说明:
	  本函数是向SrioSwitch中写入一定数量的数据
	参数说明:
	  byte_addr:数据在SrioSwitch的基地址（byte为单位）
	  uchEepromI2cAddress ：SrioSwitch的i2c地址
	  puiData：读出数据存储的数组的指针
	返回值:
		0：I2C_RET_OK
		1：I2C_RET_LOST_ARB
		2：I2C_RET_NO_ACK
		3：I2C_RET_IDLE_TIMEOUT
		4：I2C_RET_BAD_REQUEST
		5：I2C_RET_CLOCK_STUCK_LOW
		6：I2C_RET_NULL_PTR_ERROR
		99：I2C_RET_GEN_ERROR
	备注:The write consists of a master write of 2 bytes (forming a 16 bit address, msb transmitted first), followed by a master write of the
input number of bytes.The bytes that are write are placed in puiData in big endian format
*********************************************************************/
I2C_RET C6678_I2c_SrioSwitch_Write( uint32_t byte_addr, uint8_t uchEepromI2cAddress, uint32_t puiData);

/********************************************************************
	函数声明

	所在文件:   i2c_srioswitch.c
	作者:		wj

	函数名:
	  I2C_RET C6678_I2c_SrioSwitch_Config ( uint8_t uchEepromI2cAddress,uint32_t *puiData)
	函数说明:
	  本函数是通过一个包含switch配置数组的文件来对switch进行配置
	参数说明:
	  uchEepromI2cAddress ：srioswitch的i2c地址
	  puiData：配置switch的数组
	返回值:
		0：I2C_RET_OK
		1：I2C_RET_LOST_ARB
		2：I2C_RET_NO_ACK
		3：I2C_RET_IDLE_TIMEOUT
		4：I2C_RET_BAD_REQUEST
		5：I2C_RET_CLOCK_STUCK_LOW
		6：I2C_RET_NULL_PTR_ERROR
		99：I2C_RET_GEN_ERROR
	备注:
*********************************************************************/
I2C_RET C6678_I2c_SrioSwitch_Config ( uint8_t uchEepromI2cAddress,uint32_t *puiData);

/********************************************************************
	文件说明:	以下这部分函数是i2c_temp.c中函数的声明
						I2C_TEMP的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   i2c_Temp.c
	作者:		wj

	函数名:
	  void C6678_I2c_Temp_Init(void)
	函数说明:
	  本函数是初始化i2c_temp
	参数说明:
	  无
	返回值:
	  无
	备注: 无
*********************************************************************/
void C6678_I2c_Temp_Init(void);

/********************************************************************
	函数声明

	所在文件:   i2c_temp.c
	作者:		wj

	函数名:
	  Bool C6678_I2c_Temp_Read (uint8_t TempNum,int32_t *TempValue)
	函数说明:
	  本函数是从温感中读出温度。
	参数说明:
	  TempNum：第几个温感（0~2）
	  	  0：I2C Slave address is 0x48
	  	  1：I2C Slave address is 0x49
	  	  2：I2C Slave address is 0x4A
	  TempValue：返回的温度指针（单位为0.001度）
	返回值:
	   TRUE:成功
	   FALSE：失败
	备注:
*********************************************************************/
Bool C6678_I2c_Temp_Read (uint8_t TempNum,int32_t *TempValue);

/********************************************************************
	文件说明:	以下这部分函数是i2c_voltage.c中函数的声明
						I2C_Voltage的相关操作
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   i2c_voltage.c
	作者:		qqb

	函数名:
	  void C6678_I2c_Voltage_Init(void)
	函数说明:
	  本函数是初始化i2c_voltage
	参数说明:
	  无
	返回值:
	  无
	备注: 无
*********************************************************************/
void C6678_I2c_Voltage_Init(void);

/********************************************************************
	函数声明

	所在文件:   	i2c_voltage.c
	作者:		qqb

	函数名:
		Bool C6678_I2c_Voltage_Read (uint8_t ChanNum,float *VoltValue)
	函数说明:
		本函数是从电压传感器中读出电压值。
	参数说明:
		VoltageRegAdr：电压传感器地址
		VoltValue：返回的电压值指针（单位为:伏）
	返回值:
		TRUE:成功
		FALSE：失败
	备注:   无
*********************************************************************/
Bool C6678_I2c_Voltage_Read (uint8_t VoltageRegAdr,float *VoltValue);

/********************************************************************
	文件说明:	以下这部分函数是ddr.c中函数的声明
				DDR3的相关操作，非用户使用函数
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   ddr.c
	作者:		qqb

	函数名:
	  	  Bool C6678_Ddr3_lib_Init(int Ddr3Pllm,int Ddr3Type);
	函数说明:
	  	  本函数是驱动库内部函数声明，非用户使用函数
	参数说明:
		  Ddr3Pllm:
			  PLLM_DDR3(具体值见c6678.h)
		  Ddr3Type:
			  DDR3_TYPE(具体值见c6678.h)
	返回值:
		  true:正常
		  false：失败
	备注: 此函数为驱动库内部实现需要，非用户使用函数
*********************************************************************/
Bool C6678_Ddr3_lib_Init(int Ddr3Pllm,int Ddr3Type);

extern CSL_Status DDR3Init800m(int Ddr3Type);
extern CSL_Status DDR3Init1000m(int Ddr3Type);
extern CSL_Status DDR3Init600m_C6678_4DSP_DDR3_VPX_v2_0(void);
extern CSL_Status DDR3Init700m_C6678_4DSP_DDR3_VPX_v2_0(void);
extern CSL_Status DDR3Init800m_C6678_4DSP_DDR3_VPX_v2_0(void);
extern CSL_Status DDR3Init1000m_C6678_4DSP_DDR3_VPX_v2_0(void);
extern CSL_Status DDR3Init800m_1600(int Ddr3Type);
extern CSL_Status DDR3Init1000m_1600(int Ddr3Type);
extern CSL_Status DDR3Init800m_C6678_4DSP_DDR3_KU_V2_0(void);
extern CSL_Status DDR3Init1000m_C6678_4DSP_DDR3_KU_V2_0(void);
extern CSL_Status DDR3Init1000m_1600_8G_PM(void);
extern CSL_Status DDR3Init1000m_1600_Fiber(void);
extern CSL_Status DDR3Init800m_1600_2B1P(void);
extern CSL_Status DDR3Init700m_1600_3U_4P(void);
extern CSL_Status DDR3Init1000m_1600_3U_4P(void);
extern CSL_Status DDR3Init1300m_1600_3U_4P(void);
extern CSL_Status DDR3Init1400m_1600_3U_4P(void);
extern CSL_Status DDR3Init1600m_1600_3U_4P(void);
extern CSL_Status DDR3Init1000m_1600_KA_2P(void);
extern CSL_Status DDR3Init1600m_1600_LRM_2B(void);
extern CSL_Status DDR3Init1000m_1600_8G_3U(void);
extern CSL_Status DDR3Init1200m_1600_8G_3U(void);
extern CSL_Status DDR3Init1600m_1600_8G_3U(void);
extern CSL_Status DDR3Init1000m_1600_3U_SW(void);
extern CSL_Status DDR3Init1600m_1600_3U_SW(void);
extern CSL_Status DDR3Init700m_1600_802_2DSP_4P(void);
extern CSL_Status DDR3Init700m_1600_802_3DSP_4P(void);
extern CSL_Status DDR3Init1000m_1600_802_2DSP_4P(void);
extern CSL_Status DDR3Init1000m_1600_802_3DSP_4P(void);
extern CSL_Status DDR3Init1333m_1600_802_2DSP_4P(void);
extern CSL_Status DDR3Init1333m_1600_802_3DSP_4P(void);

inline CSL_Status C6678_Ddr3_Init_inline(int Ddr3Pllm,int Ddr3Type)
{
	extern CSL_Status (*pDDR3_Init_no_para)();
	extern CSL_Status (*pDDR3_Init_a_para)(int Ddr3Type);

//****************		1th		****************//
	#if(DDR3_TYPE_2BANK_8Gb == DDR3_TYPE)
		#if(PLLM_DDR3 == 5)
			pDDR3_Init_no_para = DDR3Init600m_C6678_4DSP_DDR3_VPX_v2_0;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 6)
			pDDR3_Init_no_para = DDR3Init700m_C6678_4DSP_DDR3_VPX_v2_0;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 7)
			pDDR3_Init_no_para = DDR3Init800m_C6678_4DSP_DDR3_VPX_v2_0;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_C6678_4DSP_DDR3_VPX_v2_0;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif
//****************		2th		****************//
	#elif(DDR3_TYPE_2BANK_4Gb_1600 == DDR3_TYPE)
		#if(PLLM_DDR3 == 7)
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = DDR3Init800m_1600;
		#elif(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = DDR3Init1000m_1600;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//	#elif(DDR3_TYPE_2BANK_4Gb_1600 == DDR3_TYPE)
//****************		3th		****************//
	#elif(DDR3_TYPE_2BANK_2Gb_1600 == DDR3_TYPE)
		#if(PLLM_DDR3 == 7)
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = DDR3Init800m_1600;
		#elif(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = DDR3Init1000m_1600;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_2BANK_2Gb_1600 == DDR3_TYPE)
//****************		4th		****************//
	#elif(DDR3_TYPE_1BANK_2Gb_1600 == DDR3_TYPE)
		#if(PLLM_DDR3 == 7)
			pDDR3_Init_no_para = DDR3Init800m_C6678_4DSP_DDR3_KU_V2_0;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_C6678_4DSP_DDR3_KU_V2_0;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_1BANK_2Gb_1600 == DDR3_TYPE)
//****************		5th		****************//
	#elif(DDR3_TYPE_2BANK_8Gb_1600_PM == DDR3_TYPE)
		#if(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_1600_8G_PM;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_2BANK_8Gb_1600_PM == DDR3_TYPE)
//****************		6th		****************//
	#elif(DDR3_TYPE_1BANK_4Gb_2P_1600 == DDR3_TYPE)
		#if(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_1600_Fiber;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_1BANK_4Gb_2P_1600 == DDR3_TYPE)
//****************		7th		****************//
	#elif(DDR3_TYPE_2BANK_2Gb_1P_1600 == DDR3_TYPE)
		#if(PLLM_DDR3 == 7)
			pDDR3_Init_no_para = DDR3Init800m_1600_2B1P;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_2BANK_2Gb_1P_1600 == DDR3_TYPE)
//****************		8th		****************//
	#elif(DDR3_TYPE_1BANK_4Gb_4P_1600 == DDR3_TYPE)
		#if(PLLM_DDR3 == 6)
			pDDR3_Init_no_para = DDR3Init700m_1600_3U_4P;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_1600_3U_4P;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 12)
			pDDR3_Init_no_para = DDR3Init1300m_1600_3U_4P;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 13)
			pDDR3_Init_no_para = DDR3Init1400m_1600_3U_4P;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 15)
			pDDR3_Init_no_para = DDR3Init1600m_1600_3U_4P;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_1BANK_4Gb_4P_1600 == DDR3_TYPE)
//****************		9th		****************//
	#elif(DDR3_TYPE_2BANK_4Gb_2P_1600 == DDR3_TYPE)
		#if(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_1600_KA_2P;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_2BANK_4Gb_2P_1600 == DDR3_TYPE)
//****************		10th		****************//
	#elif(DDR3_TYPE_2BANK_8Gb_8P_1600_LRM == DDR3_TYPE)
		#if(PLLM_DDR3 == 15)
			pDDR3_Init_no_para = DDR3Init1600m_1600_LRM_2B;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_2BANK_8Gb_2P_1600_LPM == DDR3_TYPE)
//****************		11th		****************//
	#elif(DDR3_TYPE_2BANK_8Gb_8P_1600_3U == DDR3_TYPE)		//3U 8Gb处理板卡
		#if(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_1600_8G_3U;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 11)
			pDDR3_Init_no_para = DDR3Init1200m_1600_8G_3U;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 15)
			pDDR3_Init_no_para = DDR3Init1600m_1600_8G_3U;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_2BANK_8Gb_8P_1600 == DDR3_TYPE)
//****************		12th		****************//
	#elif(DDR3_TYPE_1BANK_8Gb_2P_1600_3U_SW == DDR3_TYPE)
		#if(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_1600_3U_SW;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 15)
			pDDR3_Init_no_para = DDR3Init1600m_1600_3U_SW;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif	//#elif(DDR3_TYPE_1BANK_8Gb_2P_1600_3U_SW == DDR3_TYPE)
//****************		13th		****************//
	#elif(DDR3_TYPE_1BANK_512M16_802_2DSP == DDR3_TYPE)
		#if(PLLM_DDR3 == 6)
			pDDR3_Init_no_para = DDR3Init700m_1600_802_2DSP_4P;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_1600_802_2DSP_4P;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 12)
			pDDR3_Init_no_para = DDR3Init1333m_1600_802_2DSP_4P;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif//#elif(DDR3_TYPE_1BANK_512M16_802_2DSP == DDR3_TYPE)
//****************		14th		****************//
	#elif(DDR3_TYPE_1BANK_512M16_802_3DSP == DDR3_TYPE)
		#if(PLLM_DDR3 == 6)
			pDDR3_Init_no_para = DDR3Init700m_1600_802_3DSP_4P;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = DDR3Init1000m_1600_802_3DSP_4P;
			pDDR3_Init_a_para = NULL;
		#elif(PLLM_DDR3 == 12)
			pDDR3_Init_no_para = DDR3Init1333m_1600_802_3DSP_4P;
			pDDR3_Init_a_para = NULL;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif//#elif(DDR3_TYPE_1BANK_512M16_802_2DSP == DDR3_TYPE)
//****************		all else		****************//
	#else
		#if(PLLM_DDR3 == 7)
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = DDR3Init800m;
		#elif(PLLM_DDR3 == 9)
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = DDR3Init1000m;
		#else
			pDDR3_Init_no_para = NULL;
			pDDR3_Init_a_para = NULL;
		#endif
	#endif//*************		all else		****************//

	return C6678_Ddr3_lib_Init(Ddr3Pllm,Ddr3Type);
}

#define C6678_Ddr3_Init  C6678_Ddr3_Init_inline

/********************************************************************
	文件说明:	以下这部分函数是uart.c中函数的声明
				UART的相关操作，用户使用函数声明
*********************************************************************/
/********************************************************************
	函数声明

	所在文件:   uart.c
	作者:		qqb

	函数名:
	  	  int32_t c6678_uart_init(uint32_t baudrate)
	函数说明:
	  	  本函数是初始化UART函数
	参数说明:
		  baudrate:
			  UART的波特率
	返回值:
		  true:正常
		  false：失败
	备注:
*********************************************************************/
int32_t c6678_uart_init(uint32_t baudrate);

/********************************************************************
	函数声明

	所在文件:   uart.c
	作者:		qqb

	函数名:
	  	  int32_t c6678_uart_read(uint8_t *buf, uint32_t delay)
	函数说明:
	  	  本函数是UART读取数据函数
	参数说明:
		  buf:
			  UART的读取数据缓存区
		delay:
			     读取数据超时时间(单位：s)
	返回值:
		  true:正常
		  false：失败
	备注:
*********************************************************************/
int32_t c6678_uart_read(uint8_t *buf, uint32_t delay);

/********************************************************************
	函数声明

	所在文件:   uart.c
	作者:		qqb

	函数名:
	  	  int32_t c6678_uart_write(uint8_t buf)
	函数说明:
	  	  本函数是UART读取数据函数
	参数说明:
		  buf:
			  UART的写数据地址
	返回值:
		  true:正常
		  false：失败
	备注:
*********************************************************************/
int32_t c6678_uart_write(uint8_t buf);

/********************************************************************
	函数声明

	所在文件:   BigLittleSwap.c
	作者:		qqb

	函数名:
		int i_CheckCpuBigEndian(void)

	函数说明:
		本函数是检查CPU的大小端

	参数说明:
     	 	 无

	返回值:
		true:大端CPU
		false：小端CPU

	备注: 无
*********************************************************************/
int i_CheckCpuBigEndian(void);

/********************************************************************
	函数声明

	所在文件:   BigLittleSwap.c
	作者:		qqb

	函数名:
		unsigned int t_LittleToBigl(unsigned int data32)

	函数说明:
		本函数是将小端的32位数据转换为大端

	参数说明:
     	 data32：待转换的数据

	返回值:
		转换为大端的数据

	备注: 无
*********************************************************************/
unsigned int t_LittleToBigl(unsigned int data32);

/********************************************************************
	函数声明

	所在文件:   BigLittleSwap.c
	作者:		qqb

	函数名:
		unsigned int t_BigToLittlel(unsigned int data32)

	函数说明:
		本函数是将大端的32位数据转换为小端

	参数说明:
     	 data32：待转换的数据

	返回值:
		转换为小端的数据

	备注: 无
*********************************************************************/
unsigned int t_BigToLittlel(unsigned int data32);

/********************************************************************
	函数声明

	所在文件:   BigLittleSwap.c
	作者:		qqb

	函数名:
		unsigned short t_LittleToBig(unsigned short data16)

	函数说明:
		本函数是将小端的16位数据转换为大端

	参数说明:
     	 data16：待转换的数据

	返回值:
		转换为大端的数据

	备注: 无
*********************************************************************/
unsigned short t_LittleToBig(unsigned short data16);

/********************************************************************
	函数声明

	所在文件:   BigLittleSwap.c
	作者:		qqb

	函数名:
		unsigned short t_BigToLittle(unsigned short data16)

	函数说明:
		本函数是将大端的16位数据转换为小端

	参数说明:
     	 data16：待转换的数据

	返回值:
		转换为小端的数据

	备注: 无
*********************************************************************/
unsigned short t_BigToLittle(unsigned short data16);

#endif /* C6678_H_ */
