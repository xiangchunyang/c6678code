/*
 * c6678.h
 *
 *  Created on: 2012-3-31
 *      Author: wangjie
 */

#ifndef C6678_H_
#define C6678_H_

/********************************************************************
 * �����Ŀ��ļ�
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
 * ���޸ĵ�ȫ�ֱ���
 *******************************************************************/
#define DEVICE_IS_DEVICE1
#define DEVICE1 1//���ض�
#define DEVICE0 0//���ض�

#ifdef DEVICE_IS_DEVICE1
#define DEVICE DEVICE1
#else
#define DEVICE DEVICE0
#endif

/********************************************************************
 * PLL����
 *******************************************************************/
/* Default PLL PLLM value (100/1*(20/2)) = 1.0GHz) */
#define  PLATFORM_PLL1_PLLM_val (20)//ͨ���ò��������ú˵�ʱ��Ƶ��(20:1.0GHz)(25:1.25GHz)

/********************************************************************
 * DDR3����
 *******************************************************************/
// ddr3 clock = ddr3clockin*(PLL PLLM value+1)/pll PLLD value/2
/*  (100MHz*(9+1)/1/2��*2= 1.0GHz)*/

#define PLLM_DDR3 15		//ͨ�����øò���������DDR3�ܵ�ʱ��Ƶ��
//C6678_4DSP_DDR_V1.1�忨Ŀǰ֧��5��600MHz����6��700MHz���������ݲ�֧�֣�����Ҫ��������		//qqb	2017_01_11
//C6678_4DSP_DDR_V2.0�忨Ŀǰ֧��5��600MHz����6��700MHz����7��800MHz����9��1000MHz���������ݲ�֧�֣�����Ҫ��������		//qqb	2017_01_11
//YingYanH201�忨_1600��Ŀǰ֧��7��800MHz����9��1000MHz���������ݲ�֧�֣�����Ҫ��������

#define DDR3_TYPE_1BANK_2Gb  				0   //ֻ��һ��BANK��ֻ����������4ƬDDR,    ddr������1333оƬ,ÿƬDDRΪ2Gb,λ��Ϊ16bit��v1.0�忨,��ĿǰоƬ�Ͱ忨��ͣ����ֻ���ϰ忨����
#define DDR3_TYPE_1BANK_4Gb  				1   //ֻ��һ��BANK��ֻ����������4ƬDDR,    ddr������1333оƬ,ÿƬDDRΪ4Gb,λ��Ϊ16bit��v1.0�忨,ĿǰоƬ�Ͱ忨��ͣ����ֻ���ϰ忨����
#define DDR3_TYPE_2BANK_2Gb  				2	//������BANK��     �����������DDR,��8Ƭ��ddr������1333оƬ,ÿƬ2Gb,λ��Ϊ16bit��v1.0�忨,ĿǰоƬ�Ͱ忨��ͣ����ֻ���ϰ忨����
#define DDR3_TYPE_2BANK_4Gb  				3	//������BANK��     �����������DDR,��8Ƭ��ddr������1333оƬ,ÿƬ4Gb,λ��Ϊ16bit��v1.0�忨,ĿǰоƬ�Ͱ忨��ͣ����ֻ���ϰ忨����
#define DDR3_TYPE_2BANK_8Gb  				4	//������BANK��     �����������DDR,��8Ƭ��ddr������1600оƬ,ÿƬ8Gb,λ��Ϊ8bit(x2)��v2.0�忨,ĿǰֻӦ����FengBaoV303��Ŀ��
#define DDR3_TYPE_2BANK_4Gb_1600  			5	//������BANK��     �����������DDR,��8Ƭ, ddr������1600оƬ,ÿƬ4Gb,λ��Ϊ16bit��v1.0�忨_1600�棬ͨ�ÿ�
#define DDR3_TYPE_1BANK_2Gb_1600  			6	//ֻ��һ��BANK��ֻ����������4ƬDDR,    ddr������1600оƬ,ÿƬ2Gb,λ��Ϊ16bit��ֻӦ����YingYanH201��Ŀ�忨_1600��
#define DDR3_TYPE_2BANK_2Gb_1600  			7   //������BANK��     �����������DDR,��8Ƭ, ddr������1600оƬ,ÿƬ2Gb,λ��Ϊ16bit��v1.0�忨_1600��  
#define DDR3_TYPE_2BANK_8Gb_1600_PM 		8   //������BANK��     �����������DDR,��8Ƭ, ddr��ProMOS��оƬ,ÿƬ4Gb,λ��Ϊ16bit��v1.0�忨_1600�棬ͨ�����ð�
#define DDR3_TYPE_1BANK_4Gb_2P_1600 		9	//ֻ��һ��BANK��ֻ����������2ƬDDR,��2Ƭ,ddr������1600оƬ,ÿƬ4Gb,λ��Ϊ16bit��IO_DZS_Fiber_VPX C6678�忨
#define DDR3_TYPE_2BANK_2Gb_1P_1600 		10	//������BANK��     �����������1ƬDDR,��2Ƭ,ddr������1600оƬ,ÿƬ2Gb,λ��Ϊ16bit��IO_DZS_Fiber_VPX C6678�忨
#define DDR3_TYPE_1BANK_4Gb_4P_1600 		11	//ֻ��һ��BANK��ֻ����������4ƬDDR,��4Ƭ,ddr������1600оƬ,ÿƬ4Gb,λ��Ϊ16bit��2DSP_3U_VPX C6678�忨
#define DDR3_TYPE_2BANK_4Gb_2P_1600			12  //������BANK,�����������1ƬDDR,��2Ƭ,ddr������1600оƬ,ÿƬ2Gb,λ��Ϊ16bit
#define DDR3_TYPE_2BANK_8Gb_8P_1600_LRM		13	//������BANK��     �����������DDR,��8Ƭ��ddr������1600оƬ,ÿƬ8Gb,λ��Ϊ16bit
#define	DDR3_TYPE_2BANK_8Gb_8P_1600_3U		14	//������BANK��     �����������DDR,��8Ƭ��ddr������1600оƬ,ÿƬ8Gb,λ��Ϊ16bit��3U�����1600MTs
#define	DDR3_TYPE_1BANK_8Gb_2P_1600_3U_SW	15	//ֻ��һ��BANK�� ֻ����������DDR,��2Ƭ��ddr������1600оƬ,ÿƬ8Gb,λ��Ϊ16bit��3U������1600MTs
#define	DDR3_TYPE_1BANK_512M16_802_2DSP		16	//ֻ��һ��BANK��ֻ����������4ƬDDR,��4Ƭ,ddr������1600оƬ,ÿƬ8Gb,λ��Ϊ16bit
#define	DDR3_TYPE_1BANK_512M16_802_3DSP		17	//ֻ��һ��BANK��ֻ����������4ƬDDR,��4Ƭ,ddr������1600оƬ,ÿƬ8Gb,λ��Ϊ16bit


#define DDR3_TYPE   DDR3_TYPE_2BANK_8Gb_8P_1600_LRM //�����Լ����ӵ������ѡ��DDR3������

/********************************************************************
 * i2c_eeprom����
 *******************************************************************/

typedef	uint16_t I2C_RET;

// Bus release
enum {
  I2C_RELEASE_BUS,
  I2C_DO_NOT_RELEASE_BUS
};

/********************************************************************
 * spi_norflash����
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
 * timecounter����
 *******************************************************************/
//typedef unsigned long long int Uint64;
/********************************************************************
 * gpio����
 *******************************************************************/
typedef	enum _GpioDirection
{
	GPIO_OUT = 0,
	GPIO_IN
}GpioDirection;

/********************************************************************
 * emif16_nandflash����
 *******************************************************************/
typedef	struct _NAND_ADDR
{
    uint32_t uiColumnAddr;
    uint32_t uiPageAddr;
    uint32_t uiBlockAddr;
} NAND_ADDR;

/********************************************************************
 * pcie����
 *******************************************************************/
/* Global config variable that controls
   the PCIe mode. It is global so it can be poked
   from CCS. It should be set either to EP or RC. */
//ע�⣺RCӦ��������pcie_init()�ȴ����ӣ�����������
#ifdef DEVICE_IS_DEVICE1
#define PCIEMODE   pcie_EP_MODE//pcie_RC_MODE////���ó�RC����EP
#else
#define PCIEMODE   pcie_RC_MODE//pcie_RC_MODE////���ó�RC����EP
#endif

/********************************************************************
 *srio����
 *******************************************************************/
#define SRIO_SEND 0
#define SRIO_RECV 1
#ifdef DEVICE_IS_DEVICE1
#define SRIOMODE   SRIO_SEND//SRIO_RECV////���óɷ��Ͷ˻��߽��ܶ�
#define SRIO_SEND_ID
#else
#define SRIOMODE   SRIO_RECV//SRIO_RECV////���óɷ��Ͷ˻��߽��ܶ�
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
     * @brief   32b Ext Address Fields �Packet Types 2,5, and 6
     */
    uint32_t  rapidIOMSB;

    /**
     * @brief   32b Address �Packet Types 2,5, and 6
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
 * edma3����
 *******************************************************************/
typedef struct
{
     Uint32 Addr;                       //����ԭ/Ŀ�ĵ�ַ
     int Acnt;                          //A������ĳ��ȣ�ȡֵ��Χ Ϊ1 - 65535��
     int Bcnt;                          //B������ĳ��ȣ�ȡֵ��Χ Ϊ1 - 65535��
     int Ccnt;                          //C����Ĵ���ĳ��ȣ�ȡֵ��Χ Ϊ1 - 65535��
     int DAStride;                       //A����Ĳ������ȣ�ȡֵ��ΧΪ-32768 - +32767��
     int DBStride;                       //B����Ĳ������ȣ�ȡֵ��ΧΪ-32768 - +32767��
}DmaTranParam;

typedef struct
{
	Uint32 sAddr;                 //�����Դ��ַ
	Uint32 dAddr;                 //�����Ŀ�ĵ�ַ,Ŀ����Դ��ΪRAM��
	Uint32 High;                  //���ݾ�������ĳ���
	Uint32 Width;                 //���ݾ������ĳ���
	Uint8 Unit;                   //����Ԫ�صĵ�λ��=0����; =1��˫��; =x:(x-1)��
	Bool Mod;                     //��ת�ķ���=0:˳ʱ��;=1:��ʱ��
	Bool IntrEn;                  // =1��DMA�ж�ʹ�ܣ����ȴ�DMA���������=0����DMA�жϣ��ȴ�DMA����������˳�;
		                      // =����ֵ����DMA�жϣ����ȴ�DMA���������
}DmaSortTranParam;

typedef struct
{
	Uint32 ChannelCtrlNum;       //DMA�Ŀ�������
	Uint32 ChannelNum;           //DMA��ͨ����
	Uint8 TCNum;           	 //DMA��ͨ���Ŷ�Ӧ��TC��
	Int8 ShadowRegion;         //��DMAͨ����Ӧ��ShadowRegion
	Uint32 FirstParamNum;        //��һ��PARAM��
	Uint32 ExternalAddr;         //�ⲿ��ַ
	Uint32 PingAddr;             //Ping buffer ��ַ
	Uint32 PongAddr;             //Pong buffer ��ַ
	Uint32 BufferSize_KB;        //buffer �ĳ��ȣ���λ��KB)
	Bool Mod;                    //����/����
	Bool DstIsFifo;              //=1��Ŀ��Ϊfifo��=0��ԴΪfifo��=����ֵ��Ŀ����Դ��ΪRAM��

}DmaPingPongInitParam;

typedef struct
{
	Uint32 sAddr;               //�����Դ��ַ
	Uint32 dAddr;               //�����Ŀ�ĵ�ַ
	Uint32 High;                //ԭ���ݾ�������ĳ���
	Uint32 Width;               //ԭ���ݾ������ĳ���
	Uint32 Hoffset;             //Ŀ�����ݾ��������ƫ����
	Uint32 Woffset;             //Ŀ�����ݾ�������ƫ����
	Uint32 Hlength;             //Ŀ�����ݾ�������ĳ���
	Uint32 Wlength;             //Ŀ�����ݾ�������ĳ���
	Bool Unit;                  //����Ԫ�صĵ�λ��=0���֣�=1��˫��
	Bool DstIsFifo;             //=1��Ŀ��Ϊfifo��=0��ԴΪfifo��=����ֵ��Ŀ����Դ��ΪRAM��
	Bool IntrEn;                // =1��DMA�ж�ʹ�ܣ����ȴ�DMA���������=0����DMA�жϣ��ȴ�DMA����������˳�;
		                    //=����ֵ����DMA�жϣ����ȴ�DMA���������
}DmaSubFrameExtractTranParam;
typedef struct {
	Uint32 infoL;
	Uint32 infoH;
}Uint64edmaTccInfo;

typedef struct
{
	Uint32 ChannelNum;           //DMA��ͨ����
	Uint32 ParamNum;        	 //PARAM��
	Uint8 TCNum;        	 	 //��DMAͨ����Ӧ��TC��
	Int8 ShadowRegion;         //��DMAͨ����Ӧ��ShadowRegion
}DmaChainParam;
/********************************************************************
 * timer����
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
 * Hyperlink����
 *******************************************************************/

#define HYPERLINK_ADDR    	 0x40000000//����hyperlink�ĵ�ַ�ռ�

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

#define	HYPLNK_RATE	hyplnk_EXAMPLE_SERRATE_07p500//������ʿɸ�

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
#define hyplnk_EXAMPLE_REFCLK_156p25//����̶������ܸ�
//#define hyplnk_EXAMPLE_REFCLK_250p00
//#define hyplnk_EXAMPLE_REFCLK_312p50

typedef struct
{
	Uint32 SegSize;       		 //ӳ���ÿһ������Ĵ�С
	/************************
	 * ֵ�������С
	 * 0��0x400000��4MB��
	 * 1��0x800000��8MB��
	 * 2��0x1000000��16MB��
	 * 3��0x2000000��32MB��
	 * 4��0x4000000��64MB��
	 * 5��0x8000000��128MB��
	 * 6��0x10000000��256MB��
	 ************************/
	Uint32 SegSel[64];               //ÿһ�������׵�ַ(�����������С��������)
}HyplnkAddrMapParam;

/********************************************************************
 * navigator����
 *******************************************************************/
#define NAVIGATOR_SRIO_FLOWID_NUM0	9
#define NAVIGATOR_SRIO_FLOWID_NUM1	10
#define NAVIGATOR_SRIO_FLOWID_NUM2	11

/********************************************************************
 * sem2����(�ò��ֲ��ܸ�)
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
 * intc���֣��ò��ֲ��ܸģ�
 *******************************************************************/
//�ж�������

//#define  HWINT_SRIO_MSG_RX			7
//#define  HWINT_SRIO_DOORBELL		9
//#define  HWINT_SRIO_OVER			8

//��ÿ���˷��͵��ж϶�Ӧ�ĺ˼��¼���
#define  CIC0_OUT2_OR_CIC1_OUT2_EVTID	 62
#define  CIC0_OUT3_OR_CIC1_OUT3_EVTID	 63
#define  CIC0_OUT4_OR_CIC1_OUT4_EVTID	 92

//ͨ����Ӧ�˼��¼���(���к˶��е�)
#define  CIC0_OUT0_OR_CIC1_OUT0_EVTID	 102
#define  CIC0_OUT1_OR_CIC1_OUT1_EVTID	 103
#define  CIC0_OUT8_OR_CIC1_OUT8_EVTID	 104
#define  CIC0_OUT9_OR_CIC1_OUT9_EVTID	 105
#define  CIC0_OUT16_OR_CIC1_OUT16_EVTID	 106
#define  CIC0_OUT17_OR_CIC1_OUT17_EVTID	 107
#define  CIC0_OUT24_OR_CIC1_OUT24_EVTID	 108
#define  CIC0_OUT25_OR_CIC1_OUT25_EVTID	 109

//�����¼�ͨ����
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
 * other����
 *******************************************************************/
typedef struct {
	Uint32 Low32bit;
	Uint8  High4bit;
}Phy36bit;

/********************************************************************
	��������

	�����ļ�:   drive_version.c
	����:		qqb

	������:
	  	  char *drive_version()
	����˵��:
	  	  �������Ƿ���������汾��Ϣ
	����˵��:
	  	  ��
	����ֵ:
	 	 ��
	��ע:
*********************************************************************/
char *drive_version();

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����pll.c�к���������
						pll����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   pll.c
	����:		wj

	������:
	  Bool C6678_Pll_Init(unsigned int Pllm);
	����˵��:
	  �������ǳ�ʼ��pll
	����˵��:
	  Pllm:
	  	  PLATFORM_PLL1_PLLM_val(����ֵ��c6678.h)
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Pll_Init(unsigned int Pll1Pllm);
/********************************************************************
	��������

	�����ļ�:   pll.c
	����:		wj

	������:
	  void C6678_Pll_Delay(uint32_t num);
	����˵��:
	  ��������delayһ��ʱ��
	����˵��:
	  num��delay���ٸ�10ns
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
void C6678_Pll_Delay(uint32_t num);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����ddr.c�к���������
						ddr����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   ddr.c
	����:		wj

	������:
	  Bool C6678_Ddr3_Init(int Ddr3Pllm,int Ddr3Type);
	����˵��:
	  �������ǳ�ʼ��ddr
	����˵��:
	  Ddr3Pllm:
	  	  PLLM_DDR3(����ֵ��c6678.h)
	  Ddr3Type:
		  DDR3_TYPE(����ֵ��c6678.h)
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Ddr3_Init(int Ddr3Pllm,int Ddr3Type);
/********************************************************************
	��������

	�����ļ�:   ddr.c
	����:		wj

	������:
	  Bool C6678_Ddr3_MapSelect(Uint32 MapIndex);
	����˵��:
	  �������ǽ��ڴ��0x80000000��0xffffffff��2GB�ڴ�ռ�ӳ�䵽��ҵ�DDR3�Ĳ�ͬ������
	����˵��:
	  MapIndex:���DDR3�������(0~3)Ŀǰ���ddr3�Ĵ�С���Ϊ8GB
	         MapIndexֵ                  �ڴ��ַ                                                ���DDR3�������ַ
	  	        0          0x80000000��0xffffffff      0x000000000��0x07fffffff
	  	        1          0x80000000��0xffffffff      0x080000000��0x0ffffffff
	  	        2          0x80000000��0xffffffff      0x100000000��0x17fffffff
	  	        3          0x80000000��0xffffffff      0x180000000��0x1ffffffff
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Ddr3_MapSelect(Uint32 MapIndex);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����timecounter.c�к���������
						timecounter����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   timecounter.c
	����:		wj

	������:
	  void C6678_TimeCounter_Enable(void);
	����˵��:
	  ��������ʹʱ�Ӽ�ʱ��ʼ
	����˵��:
	  ��
	����ֵ:
	  ��
	��ע: ��
*********************************************************************/
void C6678_TimeCounter_Enable(void);

/********************************************************************
	��������

	�����ļ�:   timecounter.c
	����:		wj

	������:
	  Uint32 C6678_TimeCounter_GetHighVal(void);
	����˵��:
	  �������ǵõ�ʱ�Ӽ�ʱ��λ
	����˵��:
	  ��
	����ֵ:
	  ��ʱ��ʱ�����ڸ�32bit
	��ע: ��
*********************************************************************/
Uint32 C6678_TimeCounter_GetHighVal(void);

/********************************************************************
	��������

	�����ļ�:   timecounter.c
	����:		wj

	������:
	  Uint32 C6678_TimeCounter_GetLowVal(void);
	����˵��:
	  �������ǵõ�ʱ�Ӽ�ʱ��λ
	����˵��:
	  ��
	����ֵ:
	  ��ʱ��ʱ�����ڵ�32bit
	��ע: ��
*********************************************************************/
Uint32 C6678_TimeCounter_GetLowVal(void);

/********************************************************************
	��������

	�����ļ�:   timecounter.c
	����:		wj

	������:
	  void C6678_TimeCounter_Delaycycles(uint32_t usecs);
	����˵��:
	  ��������delay��������
	����˵��:
	  usecs��������
	����ֵ:
	  ��
	��ע: ��������ʱ��Χ0~2e32�����ڣ�������ʱ�������ڣ�������ö�ʱ����ʱ
*********************************************************************/
void C6678_TimeCounter_Delaycycles(uint32_t usecs);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����gpio.c�к���������
						gpio����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  void C6678_Gpio_Init(void)
	����˵��:
	  �������ǳ�ʼ��gpio
	����˵��:
	  ��
	����ֵ:
	 ��
	��ע: ��
*********************************************************************/
void C6678_Gpio_Init(void);
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_SetDirection(Uint8 uiNumber, GpioDirection direction)
	����˵��:
	  ������������GPIO�˿ڵķ���
	����˵��:
	  uiNumber���˿ںţ�0-15��
	  direction��GPIO_OUT or GPIO_IN
	����ֵ:
	 �Ƿ����óɹ�
	 TRUE:�ɹ�
	 FALSE��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Gpio_SetDirection( Uint8 uiNumber, GpioDirection direction );
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_SetOutput(Uint8 uiNumber)
	����˵��:
	  ������������GPIO�˿ڵ�Ϊ1
	����˵��:
	  uiNumber���˿ںţ�0-15��
	����ֵ:
	 �Ƿ����óɹ�
	 TRUE:�ɹ�
	 FALSE��ʧ��
	��ע:���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_OUT��
*********************************************************************/
Bool C6678_Gpio_SetOutput( Uint8 uiNumber);
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_ClearOutput(Uint8 uiNumber)
	����˵��:
	  ������������GPIO�˿ڵ�Ϊ0
	����˵��:
	  uiNumber���˿ںţ�0-15��
	����ֵ:
	 �Ƿ����óɹ�
	 TRUE:�ɹ�
	 FALSE��ʧ��
	��ע:���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_OUT��
*********************************************************************/
Bool C6678_Gpio_ClearOutput( Uint8 uiNumber);
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  uint32_t C6678_Gpio_ReadInput(Uint8 uiNumber)
	����˵��:
	  �������Ƕ�ȡGPIO�˿ڵ�״̬
	����˵��:
	  uiNumber���˿ںţ�0-15��
	����ֵ:
	 1��GPIO�˿�״̬Ϊ1
	 0��GPIO�˿�״̬Ϊ0
	 2������
	��ע:���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_IN��
*********************************************************************/
uint32_t C6678_Gpio_ReadInput( Uint8 uiNumber );
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  void C6678_Gpio_EnableGlobalInterrupt(void)
	����˵��:
	  ��������ʹ�ܶ�CPU��GPIO�ж�
	����˵��:
	  ��
	����ֵ:
	 ��
	��ע:��
*********************************************************************/
void C6678_Gpio_EnableGlobalInterrupt( void );
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  void C6678_Gpio_DisableGlobalInterrupt(void)
	����˵��:
	  �������ǲ�ʹ�ܶ�CPU��GPIO�ж�
	����˵��:
	  ��
	����ֵ:
	  ��
	��ע:��
*********************************************************************/
void C6678_Gpio_DisableGlobalInterrupt( void );
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_SetRisingEdgeInterrupt( Uint8 uiNumber )
	����˵��:
	  ������������Ϊ���GPIO�����ش����ж�
	����˵��:
	  uiNumber���˿ںţ�0-15��
	����ֵ:
	  TRUE:�ɹ�
	  FALSE��ʧ��
	��ע:
		���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_IN��
*********************************************************************/
Bool C6678_Gpio_SetRisingEdgeInterrupt( Uint8 uiNumber );
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_ClearRisingEdgeInterrupt( Uint8 uiNumber )
	����˵��:
	  ��������������ü��GPIO�½��ش����ж�
	����˵��:
	  uiNumber���˿ںţ�0-15��
	����ֵ:
	  TRUE:�ɹ�
	  FALSE��ʧ��
	��ע:
		���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_IN��
*********************************************************************/
Bool C6678_Gpio_ClearRisingEdgeInterrupt( Uint8 uiNumber );
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_SetFallingEdgeInterrupt( Uint8 uiNumber )
	����˵��:
	  ������������Ϊ���GPIO�½��ش����ж�
	����˵��:
	  uiNumber���˿ںţ�0-15��
	����ֵ:
	  TRUE:�ɹ�
	  FALSE��ʧ��
	��ע:
		���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_IN��
*********************************************************************/
Bool C6678_Gpio_SetFallingEdgeInterrupt( Uint8 uiNumber );
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_ClearFallingEdgeInterrupt( Uint8 uiNumber )
	����˵��:
	  ��������������ü��GPIO�½��ش����ж�
	����˵��:
	  uiNumber���˿ںţ�0-15��
	����ֵ:
	  TRUE:�ɹ�
	  FALSE��ʧ��
	��ע:
		���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_IN��
*********************************************************************/
Bool C6678_Gpio_ClearFallingEdgeInterrupt( Uint8 uiNumber );
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_Led_Light (Uint8 LedNum);
	����˵��:
	  ��������ʹ����
	����˵��:
	  LedNum���ƶ�Ӧ��gpio�ܽźţ�0-15��
	����ֵ:
	  TRUE:�ɹ�
	  FALSE��ʧ��
	��ע:���ȵ���C6678_Gpio_Init
	���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_OUT��
	�ú�����ʵ����ҪFPGA����ϣ���Ҫȷ���ƶ�Ӧ��gpio�ľ���ܽ�
*********************************************************************/
Bool C6678_Gpio_Led_Light (Uint8 LedNum);
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_Led_Dark (Uint8 LedNum);
	����˵��:
	  ��������ʹĳ������
	����˵��:
	  LedNum���ƶ�Ӧ��gpio�ܽźţ�0-15��
	����ֵ:
	  TRUE:�ɹ�
	  FALSE��ʧ��
	��ע:���ȵ���C6678_Gpio_Init
		���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_OUT��
		�ú�����ʵ����ҪFPGA����ϣ���Ҫȷ���ƶ�Ӧ��gpio�ľ���ܽ�
*********************************************************************/
Bool C6678_Gpio_Led_Dark (Uint8 LedNum);
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_Led_Toggle (Uint8 LedNum);
	����˵��:
	  ��������ʹ��GPIO�˿ڵ�ƽ��ת�����������Ϩ��������ŵ�����
	����˵��:
	  LedNum���ƶ�Ӧ��gpio�ܽźţ�0-15��
	����ֵ:
	  TRUE:�ɹ�
	  FALSE��ʧ��
	��ע:���ȵ���C6678_Gpio_Init
		���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_OUT��
		�ú�����ʵ����ҪFPGA����ϣ���Ҫȷ���ƶ�Ӧ��gpio�ľ���ܽ�
*********************************************************************/
Bool C6678_Gpio_Led_Toggle (Uint8 LedNum);
/********************************************************************
	��������

	�����ļ�:   gpio.c
	����:		wj

	������:
	  Bool C6678_Gpio_Led_Blink(Uint8 LedNum, Uint32 delay_n, Uint32 blink_num)
	����˵��:
	  �����������ơ�
	����˵��:
	  LedNum���ƶ�Ӧ��gpio�ܽźţ�0-15��
	  delay_n:��ʱ��ʱ��
	  blink_num�����ƵĴ���
	����ֵ:
	  TRUE:�ɹ�
	  FALSE��ʧ��
	��ע:���ȵ���C6678_Gpio_Init
		���øú���ǰӦ�ȰѸö˿���Ϊ����˿ڣ�GPIO_OUT��
		�ú�����ʵ����ҪFPGA����ϣ���Ҫȷ���ƶ�Ӧ��gpio�ľ���ܽ�
*********************************************************************/
Bool C6678_Gpio_Led_Blink(Uint8 LedNum, Uint32 delay_n, Uint32 blink_num);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����i2c_eeprom.c�к���������
						i2c_eeprom����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   i2c_eeprom.c
	����:		wj

	������:
	  void C6678_I2c_Eeprom_Init(void)
	����˵��:
	  �������ǳ�ʼ��i2c_eeprom
	����˵��:
	  ��
	����ֵ:
	  ��
	��ע: ��
*********************************************************************/
void C6678_I2c_Eeprom_Init(void);
/********************************************************************
	��������

	�����ļ�:   i2c_eeprom.c
	����:		wj

	������:
	  I2C_RET C6678_I2c_Eeprom_Read ( uint32_t byte_addr, uint32_t uiNumBytes,
                        uint8_t *puiData, uint8_t uchEepromI2cAddress)
	����˵��:
	  �������Ǵ�EEPROM����һ������������
	����˵��:
	  uchEepromI2cAddress ��EEPROM��i2c��ַ
	  puiData���������ݴ洢�������ָ��
 	  uiNumBytes���������ݵĳ��ȣ�byteΪ��λ��
	  byte_addr��������EEPROM�Ļ���ַ��byteΪ��λ��
	����ֵ:
		0��I2C_RET_OK
		1��I2C_RET_LOST_ARB
		2��I2C_RET_NO_ACK
		3��I2C_RET_IDLE_TIMEOUT
		4��I2C_RET_BAD_REQUEST
		5��I2C_RET_CLOCK_STUCK_LOW
		6��I2C_RET_NULL_PTR_ERROR
		99��I2C_RET_GEN_ERROR
	��ע:  eeprom�ܴ�СΪ0x10000B,ʹ������ע�ⲻҪ�����ô�С��The read consists of a master write of 2 bytes (forming a 16 bit address, msb transmitted first), followed by a master read of the
 input number of bytes.The bytes that are read are placed in puiData in big endian format
*********************************************************************/
I2C_RET C6678_I2c_Eeprom_Read ( uint32_t byte_addr, uint32_t uiNumBytes,
                        uint8_t *puiData, uint8_t uchEepromI2cAddress);
/********************************************************************
	��������

	�����ļ�:   i2c_eeprom.c
	����:		wj

	������:
	  I2C_RET C6678_I2c_Eeprom_Write( uint32_t byte_addr, uint8_t uchEepromI2cAddress, uint8_t *puiData,
						uint32_t uiNumBytes, uint32_t uiEndBusState)
	����˵��:
	  ����������EEPROMд��һ������������
	����˵��:
	  byte_addr:������EEPROM�Ļ���ַ��byteΪ��λ��
	  uchEepromI2cAddress ��EEPROM��i2c��ַ
	  puiData���������ݴ洢�������ָ��
 	  uiNumBytes���������ݵĳ��ȣ�byteΪ��λ��
	  uiEndBusState��The state on which bus should be left
	����ֵ:
		0��I2C_RET_OK
		1��I2C_RET_LOST_ARB
		2��I2C_RET_NO_ACK
		3��I2C_RET_IDLE_TIMEOUT
		4��I2C_RET_BAD_REQUEST
		5��I2C_RET_CLOCK_STUCK_LOW
		6��I2C_RET_NULL_PTR_ERROR
		99��I2C_RET_GEN_ERROR
	��ע:  eeprom�ܴ�СΪ0x10000B,ʹ������ע�ⲻҪ�����ô�С��The write consists of a master write of 2 bytes (forming a 16 bit address, msb transmitted first), followed by a master write of the
input number of bytes.The bytes that are write are placed in puiData in big endian format
*********************************************************************/
I2C_RET C6678_I2c_Eeprom_Write( uint32_t byte_addr, uint8_t uchEepromI2cAddress, uint8_t *puiData,
						uint32_t uiNumBytes, uint32_t uiEndBusState);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����spi_norflash.c�к���������
						spi_norflash����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   spi_norflash.c
	����:		wj

	������:
	  NOR_STATUS C6678_Spi_Norflash_Init(void);
	����˵��:
	  �������ǳ�ʼ��spi_norflash,����dsp��NORFLASH֮������ݴ��䡣
	����˵��:
	  ��
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
NOR_STATUS C6678_Spi_Norflash_Init(void);
/********************************************************************
	��������

	�����ļ�:   spi_norflash.c
	����:		wj

	������:
	  uint32_t C6678_Spi_Norflash_GetDetails(RADAR_DEVICE_info*   nor_info);
	����˵��:
	  �������ǵõ�NORFLASH�豸����ϸ��Ϣ��
	����˵��:
	  nor_info����ϸ��Ϣ�Ľṹ�塣
	����ֵ:
	  true:����
	  false��ʧ��
	��ע: ��
*********************************************************************/
uint32_t C6678_Spi_Norflash_GetDetails(RADAR_DEVICE_info*   nor_info);
/********************************************************************
	��������

	�����ļ�:   spi_norflash.c
	����:		wj

	������:
	  NOR_STATUS C6678_Spi_Norflash_Read(uint32_t addr,uint32_t len,uint8_t* buf)
	����˵��:
	  �������Ǵ�NORFLASH�ж������ݡ�
	����˵��:
	  p_device���豸��Ϣ�ṹ�塣
	  addr����ʼ��ַ��byteΪ��λ��
	  len�����ݳ��ȣ�byteΪ��λ��
	  buf���洢���ݵ������ָ��
	����ֵ:����״̬
	  SPI_EFAIL��        (SPI_STATUS)-1   ����
	  SPI_EOK  ��         0              �ɹ�
	��ע: norflash��16MB,ע�ⲻҪ����Խ��;
*********************************************************************/
NOR_STATUS C6678_Spi_Norflash_Read(uint32_t addr,uint32_t len,uint8_t* buf);
/********************************************************************
	��������

	�����ļ�:   spi_norflash.c
	����:		wj

	������:
	  NOR_STATUS C6678_Spi_Norflash_Write(uint32_t addr,uint32_t len,uint8_t* buf)
	����˵��:
	  �������ǽ�����д��NORFLASH�С�
	����˵��:
	  p_device���豸��Ϣ�ṹ�塣
	  addr����ʼ��ַ��byteΪ��λ��
	  len�����ݳ��ȣ�byteΪ��λ��
	  buf���洢���ݵ������ָ��
	����ֵ:����״̬
	  SPI_EFAIL��        (SPI_STATUS)-1   ����
	  SPI_EOK  ��         0              �ɹ�
	��ע: д֮ǰҪ�ȵ��ò��������������norflash��16MB,ע�ⲻҪ����Խ��;
*********************************************************************/
NOR_STATUS C6678_Spi_Norflash_Write(uint32_t addr,uint32_t len,uint8_t* buf);
/********************************************************************
	��������

	�����ļ�:   spi_norflash.c
	����:		wj

	������:
	  NOR_STATUS C6678_Spi_Norflash_Erase(uint32_t sector_number)
	����˵��:
	  �������ǽ�NORFLASH�е����ݲ�����
	����˵��:
	  p_device���豸��Ϣ�ṹ�塣
	  sector_number��������sector��number(��ΧΪ0��255,��256��)
	����ֵ:����״̬
	  SPI_EFAIL��        (SPI_STATUS)-1   ����
	  SPI_EOK  ��         0              �ɹ�
	��ע: �ú������齫NORFLASH�е����ݲ������ܹ�256��(ÿ��64KB)����16MB�����if sector_number = -1, do bulk erase.
*********************************************************************/
NOR_STATUS C6678_Spi_Norflash_Erase(uint32_t sector_number);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����spi_FPGA.c�к���������
						spi_FPGA����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   spi_FPGA.c
	����:		dyx

	������:
	  Bool C6678_Spi_FPGA_Init(void);
	����˵��:
	  �������ǳ�ʼ��spi_FPGA,����dsp��FPGA֮������ݴ��䡣
	����˵��:
	  ��
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Spi_FPGA_Init(void);

/********************************************************************
	��������

           �����ļ�:   spi_FPGA.c
	����:		dyx

	������:
	  Bool C6678_Spi_FPGA_Read(uint32_t addr,uint32_t len,uint16_t* buf)
	����˵��:
	  �������Ǵ�FPGA�ж������ݡ�
	����˵��:
	  addr����ʼ��ַ��16bitΪ��λ��
	  len�����ݳ��ȣ�16bitΪ��λ��(���2048)
	  buf���洢���ݵ������ָ��
	����ֵ:����״̬
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Spi_FPGA_Read( uint32_t  addr,uint32_t  len,uint16_t* buf);

/********************************************************************
	��������

	�����ļ�:   spi_FPGA.c
	����:		dyx

	������:
	  Bool C6678_Spi_FPGA_Write(uint32_t addr,uint32_t len,uint16_t* buf)
	����˵��:
	  �������ǽ�����д��FPGA�С�
	����˵��:
	  addr����ʼ��ַ��16bitΪ��λ��
	  len�����ݳ��ȣ�16bitΪ��λ��(���2048)
	  buf���洢���ݵ������ָ��
	����ֵ:����״̬
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��ʼ��ַaddr��Ҫ��16��ʮ���ƣ���ʼ������ǰ16����ַ��FPGAռ�á�
*********************************************************************/
Bool C6678_Spi_FPGA_Write(uint32_t addr,uint32_t len,uint16_t* buf);

/********************************************************************
	��������

           �����ļ�:   spi_FPGA.c
	����:		dyx

	������:
	  uint16_t C6678_Spi_FPGA_GetDspId(void)
	����˵��:
	  �������ǻ�ȡ��DSP��ID�š�
	����˵��:
	  ��
	����ֵ:DSP��ID�ţ�0,1,2,3...��
	��ע: ��
*********************************************************************/
uint16_t C6678_Spi_FPGA_GetDspId(void);

/********************************************************************
	��������

           �����ļ�:   spi_FPGA.c
	����:		dyx

	������:
	  uint16_t C6678_Spi_FPGA_GetSrioId(void)
	����˵��:
	  �������ǻ�ȡ��DSP��SRIO��ID��
	����˵��:
	  ��
	����ֵ:SRIO��ID��
	��ע: ��
*********************************************************************/
uint16_t C6678_Spi_FPGA_GetSrioId(void);

/********************************************************************
	��������

           �����ļ�:   spi_FPGA.c
	����:		wyc

	������:
	  uint16_t C6678_Spi_FPGA_GetSlotId(void)
	����˵��:
	  �������ǻ�ȡ�ð��SLOTID��
	����˵��:
	  ��
	����ֵ:
	��ȷ��SLOTID��
	����0xffff
	��ע: ��
*********************************************************************/
uint16_t C6678_Spi_FPGA_GetSlotId(void);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����emif16_nandflash.c�к���������
						emif16_nandflash����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   emif16_nandflash.c
	����:		wj

	������:
	  RADAR_DEVICE_info *C6678_Emif16_Nandflash_Init(void)
	����˵��:
	  �������ǳ�ʼ��emif16_nandflash
	����˵��:
	  ��
	����ֵ:
	   RADAR_DEVICE_info��Ϣ
	��ע: ��
*********************************************************************/
RADAR_DEVICE_info *C6678_Emif16_Nandflash_Init(void);
/********************************************************************
	��������

	�����ļ�:   emif16_nandflash.c
	����:		wj

	������:
	  uint32_t C6678_Emif16_Nandflash_ReadPage(NAND_ADDR address, uint8_t* puchBuffer)
	����˵��:
	  ��������ҳ��nandflash�����ECCʹ�ܣ������͸���bit����
	����˵��:
	  address�������ҳ��ַ
	  puchBuffer������ָ��
	����ֵ:
	   NULL_POINTER_ERROR��ָ��Ϊ0
	   SUCCESS:�ɹ�
	   FAIL��ʧ��
	��ע: puchBuffer��һ��2KB������
*********************************************************************/
uint32_t C6678_Emif16_Nandflash_ReadPage(NAND_ADDR address, uint8_t* puchBuffer);
/********************************************************************
	��������

	�����ļ�:   emif16_nandflash.c
	����:		wj

	������:
	  uint32_t C6678_Emif16_Nandflash_BlockErase(uint32_t uiBlockNumber)
	����˵��:
	  �������ǿ����nandflash
	����˵��:
	  uiBlockNumber�����ַ
	����ֵ:
	   SUCCESS:�ɹ�
	   FAIL��ʧ��
	��ע:
*********************************************************************/
uint32_t C6678_Emif16_Nandflash_BlockErase(uint32_t uiBlockNumber);
/********************************************************************
	��������

	�����ļ�:   emif16_nandflash.c
	����:		wj

	������:
	  uint32_t C6678_Emif16_Nandflash_WritePage(RADAR_DEVICE_info *p_device, NAND_ADDR address, uint8_t* puchBuffer)
	����˵��:
	  ��������ҳдnandflash�����ECCʹ�ܣ�����ECC��д�뵽�հ�����
	����˵��:
	  address�������ҳ��ַ
	  puchBuffer������ָ��
	����ֵ:
	   NULL_POINTER_ERROR��ָ��Ϊ0
	   SUCCESS:�ɹ�
	   FAIL��ʧ��
	��ע: puchBuffer��һ��2KB������
	д֮ǰҪ�ȵ��ò��������������
*********************************************************************/
uint32_t C6678_Emif16_Nandflash_WritePage(RADAR_DEVICE_info *p_device, NAND_ADDR address, uint8_t* puchBuffer);
/********************************************************************
	��������

	�����ļ�:   emif16_nandflash.c
	����:		wj

	������:
	  uint32_t C6678_Emif16_Nandflash_GetDetails(RADAR_DEVICE_info *pNandInfo)
	����˵��:
	  �������ǵõ�NANDFLASH�ľ�����Ϣ
	����˵��:
	  pNandInfo��nand��Ϣ�ṹ���ָ��
	����ֵ:
	   NULL_POINTER_ERROR��ָ��Ϊ0
	   SUCCESS:�ɹ�
	   FAIL��ʧ��
	��ע:��
*********************************************************************/
uint32_t C6678_Emif16_Nandflash_GetDetails(RADAR_DEVICE_info *pNandInfo);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����emif16_fpga.c�к���������
						emif16_fpga����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   emif16_fpga.c
	����:		wj

	������:
	  Bool C6678_Emif16_Fpga_Init(void)
	����˵��:
	  �������ǳ�ʼ��emif16_fpga
	����˵��:
	  ��
	����ֵ:
	   TRUE:�ɹ�
	   FALSE��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Emif16_Fpga_Init(void);
/********************************************************************
	��������

	�����ļ�:   emif16_fpga.c
	����:		wj

	������:
	 	 Bool C6678_Emif16_Fpga_ReadByte(uint32_t byte_addr,uint32_t uiNumBytes,uint16_t *DataBuffer)
	����˵��:
	  ��������ͨ��emif16��fpga�е�����
	����˵��:
	  DataBuffer���������ݴ洢�������ָ��
 	  uiNumBytes���������ݵĳ��ȣ�16bitΪ��λ��(���2048)
	  byte_addr��������fpga�Ļ���ַ��16bitΪ��λ��
	����ֵ:
	   TRUE:�ɹ�
	   FALSE��ʧ��
 	��ע:Ĭ��ʹ��DSP��EMIF CE0(CS2)(ӳ��ռ�0x70000000)��FPGAͨ�ţ����ʵ��Ӳ����������CE��
		��ʹ��byte_addr���е�ַ�ռ��ƫ�ƣ�����ƫ����0x04000000��
		��ʵ��FPGA����DSP��EMIF CE2��CS4������byte_addr=0x08000000
*********************************************************************/
Bool C6678_Emif16_Fpga_ReadByte(uint32_t byte_addr,uint32_t uiNumBytes,uint16_t *DataBuffer);
/********************************************************************
	��������

	�����ļ�:   emif16_fpga.c
	����:		wj

	������:
	 	 Bool C6678_Emif16_Fpga_WriteByte(uint32_t byte_addr,uint32_t uiNumBytes,uint16_t *DataBuffer)
	����˵��:
	  ��������ͨ��emif16дfpga��ĳ�οռ�
	����˵��:
	  DataBuffer��д�����ݴ洢�������ָ��
 	  uiNumBytes��д�����ݵĳ��ȣ�16bitΪ��λ��(���2048)
	  byte_addr��������fpga�Ļ���ַ��16bitΪ��λ��
	����ֵ:
	   TRUE:�ɹ�
	   FALSE��ʧ��
 	��ע:Ĭ��ʹ��DSP��EMIF CE0(CS2)(ӳ��ռ�0x70000000)��FPGAͨ�ţ����ʵ��Ӳ����������CE��
		��ʹ��byte_addr���е�ַ�ռ��ƫ�ƣ�����ƫ����0x04000000��
		��ʵ��FPGA����DSP��EMIF CE2��CS4������byte_addr=0x08000000
*********************************************************************/
Bool C6678_Emif16_Fpga_WriteByte(uint32_t byte_addr,uint32_t uiNumBytes,uint16_t *DataBuffer);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����pcie.c�к���������
						pcie����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   pcie.c
	����:		wj

	������:
	  Bool C6678_Pcie_Init(int mode)
	����˵��:
	  �������ǳ�ʼ��pcie
	����˵��:
	 mode:
	 	 PCIEMODE(����ֵ��c6678.h)
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע:  ע�⣺����pcie������ʱ������pcie�˵��øú���������ܳ������룬���������ϡ�
*********************************************************************/
Bool C6678_Pcie_Init(int mode);
/********************************************************************
	��������

	�����ļ�:   pcie.c
	����:		wj

	������:
	  Bool C6678_Pcie_InboundAdrTrans(unsigned int pcieAdr,unsigned int localAdr)
	����˵��:
	  ������������pcie��inbound��ַӳ��
	����˵��:
	 pcieAdr:
	 	 pcie���ߵ�ַ
	 localAdr��
	     6678�ڲ����ߵ�ַ
	����ֵ:
	  true:ӳ������
	  false��ӳ��ʧ��
	��ע:
*********************************************************************/
Bool C6678_Pcie_InboundAdrTrans(unsigned int pcieAdr,unsigned int localAdr);
/********************************************************************
	��������

	�����ļ�:   pcie.c
	����:		wj

	������:
	  Bool C6678_Pcie_SetOutboundSize(unsigned int outboundSize)
	����˵��:
	  ������������pcie��outbound����С
	����˵��:
	 obSize:
	 	 ����С
	 	 0:1MB
  	  	 1:2MB
  	  	 2:4MB
  	  	 3:8MB
	����ֵ:
	  true:��������
	  false������ʧ��
*********************************************************************/
Bool C6678_Pcie_SetOutboundSize(unsigned int outboundSize);
/********************************************************************
	��������

	�����ļ�:   pcie.c
	����:		wj

	������:
	  Bool C6678_Pcie_OutboundAdrTrans(unsigned int pcieAdrLow,unsigned int pcieAdrHigh,unsigned int regionNum)
	����˵��:
	  ������������pcie��outbound��ַӳ��
	����˵��:
		pcieAdrLow:
			pcie���ߵ�32bit��ַ
		pcieAdrHigh��
			pcie���߸�32bit��ַ
		regionNum:
			region�ı��
	����ֵ:
	  true:ӳ������
	  false��ӳ��ʧ��
	��ע:ÿ��region��Ӧ���ڲ���ַΪ0x60000000+obsize*regionNum
*********************************************************************/
Bool C6678_Pcie_OutboundAdrTrans(unsigned int pcieAdrLow,unsigned int pcieAdrHigh,unsigned int regionNum);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����srio.c�к���������
						srio����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Init_Ku(int Rate,int SrioId[4],int MultiId[3],uint8_t InfoMode,uint8_t DataswapMode,uint8_t PhysicalMode,Bool BoardLinkCheckFlag);

	����˵��:
		�������ǳ�ʼ��srio

	����˵��:
		Rate��
			SRIO_RATE
		SrioId:
			SRIO��ID(4��ID)
			��PhysicalMode==0ʱ:SrioId������ֻ��SrioId[0]��Ҫ��ֵ����ʱ��DSPֻ��һ��ID;
			��PhysicalMode==1ʱ:SrioId������ֻ��SrioId[0]��SrioId[2]��Ҫ��ֵ��
			   ��ʱ��DSP������ID,ÿ��ID��Ӧһ��2x��port�ڷֱ�Ϊport0��port2;
		MultiId:
		    3���ಥID,���û��ʹ�ÿ�����Ϊ0
		InfoMode:
			����info�Ĳ�ͬģʽ
			0:8��ÿ���˿��Ը�����8��infoֵ��ƽ��ģʽ
			1:��0����16�� info��������5��info����ƽ��ģʽ
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
		    srio��4x�����ӷ�ʽ
			0:1��4x
			1:2��2x
		BoardLinkCheckFlag:
			�Ƿ�����link��־,�ñ�־ֻ����PhysicalModeΪ1��2��2x��ʱ�����壬Ŀǰ������ku���������ˡ�
			TRUE:	�����link�ź�
			FALSE:  �������link�ź�
	����ֵ:
		true:��ʼ������
		false����ʼ��ʧ��

	��ע: Ŀǰֻ֧��1��4x��2��2x,��Ŀǰ2��2x����ͬʱ���䣬ֻ�ܷ�ʱ����
*********************************************************************/
Bool C6678_Srio_Init_Ku(int Rate,int *SrioId,int *MultiId,uint8_t InfoMode,uint8_t DataswapMode,uint8_t PhysicalMode,Bool BoardLinkCheckFlag);

/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Init(int Rate,int SrioId,int MultiId[3],uint8_t InfoMode,uint8_t DataswapMode);

	����˵��:
		�������ǳ�ʼ��srio

	����˵��:
		Rate��
			SRIO_RATE
		SrioId:
			SRIO��ID
		MultiId:
		    3���ಥID,���û��ʹ�ÿ�����Ϊ0
		InfoMode:
			����info�Ĳ�ͬģʽ
			0:8��ÿ���˿��Ը�����8��infoֵ��ƽ��ģʽ
			1:��0����16�� info��������5��info����ƽ��ģʽ
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
	����ֵ:
		true:��ʼ������
		false����ʼ��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Srio_Init(int Rate,int SrioId,int MultiId[3],uint8_t InfoMode,uint8_t DataswapMode);

/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		qqb

	������:
		Bool srio_linktest(int portnum,int cmd);

	����˵��:
		����������ָ���Ķ˿ڷ���link request����

	����˵��:
		portNum : SRIO Port Number
            cmd : Command to be sent in the link-request control symbol.
                  The following values hold good:-
                  - 0x3 i.e. Reset
                  - 0x4 i.e. Input Status

	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: ��
*********************************************************************/
Bool srio_linktest(int portnum,int cmd);

/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
	   Bool C6678_Srio_Close(void);
	����˵��:
	  �������ǹر�SRIO��
	����˵��:
	  ��
	����ֵ:
	  true:�ɹ�
	  false��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Srio_Close(void);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_Read_Ku(Uint32 PortId,uint32_t Src,uint32_t * Dst, Uint16 SrcID, Uint32 Len,Bool WaitOver,Bool IntrEn,Uint32 priority)

	����˵��:
		�������Ǵ�Ŀ���豸��ȡ���ݵ����ص�ַ

	����˵��:
		PortId:�˿ں�
		    ��PhysicalMode==0ʱ:PortIdֻ��Ϊ0;
			��PhysicalMode==1ʱ:PortIdֻ��Ϊ0��2,�ֱ��Ӧ2��2x��port�ڷֱ�Ϊport0��port2;
			PhysicalModeΪC6678_Srio_Init�Ĳ�����
		Src:Դ��ַ��Ŀ�꣩
		Dst��Ŀ�ĵ�ַ�����أ�
		SrcID��Ŀ��ID
		Len�����ݳ��ȣ����δ������1MB,����Ϊ0��
		WaitOver:�Ƿ�ȴ�����
			TRUE:�ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
			FALSE�����ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
		IntrEn���������ʱ�Ƿ����ж�
			TRUE:�����ж�
			FALSE���������ж�
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������	
	����ֵ:
		true:�ɹ�
	 	false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Srio_Dio_Read_Ku(Uint32 PortId,uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_Read(uint32_t Src,uint32_t * Dst, Uint16 SrcID, Uint32 Len,Bool WaitOver,Bool IntrEn,Uint32 priority)

	����˵��:
		�������Ǵ�Ŀ���豸��ȡ���ݵ����ص�ַ

	����˵��:
		Src:Դ��ַ��Ŀ�꣩
		Dst��Ŀ�ĵ�ַ�����أ�
		SrcID��Ŀ��ID
		Len�����ݳ��ȣ����δ������1MB,����Ϊ0��
		WaitOver:�Ƿ�ȴ�����
			TRUE:�ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
			FALSE�����ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
		IntrEn���������ʱ�Ƿ����ж�
			TRUE:�����ж�
			FALSE���������ж�
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������
	����ֵ:
		true:�ɹ�
	 	false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Srio_Dio_Read(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_Write_Ku(Uint32 PortId,Uint32 *  Src, Uint32 Dst, Uint16 DstID, Uint32 Len,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

	����˵��:
		����������Ŀ���豸д����

	����˵��:
		PortId:�˿ں�
		    ��PhysicalMode==0ʱ:PortIdֻ��Ϊ0;
			��PhysicalMode==1ʱ:PortIdֻ��Ϊ0��2,�ֱ��Ӧ2��2x��port�ڷֱ�Ϊport0��port2;
			PhysicalModeΪC6678_Srio_Init�Ĳ�����
		Src:Դ��ַ�����أ�
		Dst��Ŀ�ĵ�ַ��Ŀ�꣩
		DstID��Ŀ��ID(��fpgaͨ��ʱ��Ŀ��ID����fpga�����Աȷ��)
		Len�����ݳ��ȣ����δ������1MB,����Ϊ0��
		Writemode��д��ģʽ��
			Srio_Ttype_Write_NWRITE
			Srio_Ttype_Write_NWRITE_R
			Srio_Ftype_SWRITE
		WaitOver:�Ƿ�ȴ�����
			TRUE:�ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
			FALSE�����ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
		IntrEn���������ʱ�Ƿ����ж�
			TRUE:�����ж�
			FALSE���������ж�
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������	
	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Srio_Dio_Write_Ku(Uint32 PortId,Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_Write(Uint32 *  Src, Uint32 Dst, Uint16 DstID, Uint32 Len,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

	����˵��:
		����������Ŀ���豸д����

	����˵��:
		Src:Դ��ַ�����أ�
		Dst��Ŀ�ĵ�ַ��Ŀ�꣩
		DstID��Ŀ��ID(��fpgaͨ��ʱ��Ŀ��ID����fpga�����Աȷ��)
		Len�����ݳ��ȣ����δ������1MB,����Ϊ0��
		Writemode��д��ģʽ��
			Srio_Ttype_Write_NWRITE
			Srio_Ttype_Write_NWRITE_R
			Srio_Ftype_SWRITE
		WaitOver:�Ƿ�ȴ�����
			TRUE:�ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
			FALSE�����ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
		IntrEn���������ʱ�Ƿ����ж�
			TRUE:�����ж�
			FALSE���������ж�
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������
	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Srio_Dio_Write(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_Doorbell_Ku(Uint32 PortId,Uint16 Info,Uint16 DstID,Uint32 priority)

	����˵��:
	  ����������Ŀ���豸��������
	����˵��:
		PortId:�˿ں�
		    ��PhysicalMode==0ʱ:PortIdֻ��Ϊ0;
			��PhysicalMode==1ʱ:PortIdֻ��Ϊ0��2,�ֱ��Ӧ2��2x��port�ڷֱ�Ϊport0��port2;
			PhysicalModeΪC6678_Srio_Init�Ĳ�����
		Info:������Ϣ
		���ʹ��C6678_Srio_Init��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(����0~7������corenum>>1,8~15������corenum>>1+1)
		���磺
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		���ʹ��C6678_Srio_Init_Core0more��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		core0��Ӧ16��infoֵ��������ÿ���˶�Ӧ5��infoֵ
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
			 	 	Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		DstID��Ŀ��ID(��fpgaͨ��ʱ��Ŀ��ID����fpga�����Աȷ��)
        priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������	
	����ֵ:
	  true:�ɹ�
	  false��ʧ��
	��ע: infoӦΪ����˵���е�ֵ������ֵ����������
*********************************************************************/
Bool C6678_Srio_Dio_Doorbell_Ku(Uint32 PortId,Uint16 Info,Uint16 DstID,Uint32 priority);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_Doorbell(Uint16 Info,Uint16 DstID,Uint32 priority)

	����˵��:
	  ����������Ŀ���豸��������
	����˵��:
		Info:������Ϣ
		���ʹ��C6678_Srio_Init��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(����0~7������corenum>>1,8~15������corenum>>1+1)
		���磺
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		���ʹ��C6678_Srio_Init_Core0more��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		core0��Ӧ16��infoֵ��������ÿ���˶�Ӧ5��infoֵ
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		DstID��Ŀ��ID(��fpgaͨ��ʱ��Ŀ��ID����fpga�����Աȷ��)
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������
	����ֵ:
	  true:�ɹ�
	  false��ʧ��
	��ע: infoӦΪ����˵���е�ֵ������ֵ����������
*********************************************************************/
Bool C6678_Srio_Dio_Doorbell(Uint16 Info,Uint16 DstID,Uint32 priority);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
	  Bool C6678_Srio_Maitain_Read(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len)
	����˵��:
	  �������Ǵ�Ŀ���豸��ȡ���ݵ����ص�ַ
	����˵��:
	  Src:Դ��ַ��Ŀ�꣩
	  Dst��Ŀ�ĵ�ַ�����أ�
	  SrcID��ԴID
	  Len�����ݳ���
	����ֵ:
	  true:�ɹ�
	  false��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Srio_Maitain_Read(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		Lin Jiang

	������:
		Bool C6678_Srio_Maitain_Read_Hop(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,uint8_t hopcnt)

	����˵��:
		�������Ǵ�Ŀ���豸��ȡ���ݵ����ص�ַ

	����˵��:
		Src:Դ��ַ��Ŀ�꣩
		Dst��Ŀ�ĵ�ַ�����أ�
		SrcID��ԴID
		Len�����ݳ���
		hopcnt�������Ľ���оƬ��

	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Srio_Maitain_Read_Hop(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,uint8_t hopcnt);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
	  Bool C6678_Srio_Maitain_Write(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len);
	����˵��:
	  �������ǽ�����д��Ŀ���豸
	����˵��:
	  Src:Դ��ַ�����أ�
	  Dst��Ŀ�ĵ�ַ��Ŀ�꣩
	  SrcID��Ŀ��ID
	  Len�����ݳ���
	����ֵ:
	  true:�ɹ�
	  false��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Srio_Maitain_Write(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		Lin Jiang

	������:
		Bool C6678_Srio_Maitain_Write_Hop(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,uint8_t hopcnt)

	����˵��:
		�������ǽ�����д��Ŀ���豸

	����˵��:
		Src:Դ��ַ�����أ�
		Dst��Ŀ�ĵ�ַ��Ŀ�꣩
		SrcID��Ŀ��ID
		Len�����ݳ���
        hopcnt�������Ľ���оƬ��

	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Srio_Maitain_Write_Hop(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,uint8_t hopcnt);
/********************************************************************
	��������

	�����ļ�:   srio_drv.c
	����:		Lin Jiang

	������:
		Bool C6678_Srio_Enumerate(Uint16 HostID,Uint32 srioidnum,Uint16 *srioIDInfo,Srio_Enum *srio_info)

	����˵��:
		��������SRIOö��C6678ϵͳ���豸

	����˵��:
     HostID:�����ص�C6678��SRIO_ID������ID��
     srioidnum:��Ҫö�ٵ�srioid������
     srioIDInfo����Ҫö�ٵ�id������ָ��
     srio_info:ö�ٽ����Ϣ

	����ֵ:
		true:ö�ٵ��豸
		false��û��ö�ٵ��豸

	��ע: ��ģʽ��6678��srio��ʼ��������DataswapMode�����0;
*********************************************************************/
Bool C6678_Srio_Enumerate(Uint16 HostID,Uint32 srioidnum,Uint16 *srioIDInfo,Srio_Enum *srio_info);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
	  Bool C6678_Srio_Bind(Srio_SocketType mode,Srio_SockBindAddrInfo ptr_bindInfo)
	����˵��:
	  �������ǽ�һ������
	����˵��:
	  mode:��������
	  	  Srio_SocketType_DIO��DIO
	  	  Srio_SocketType_TYPE9��TYPE 9
	  	  Srio_SocketType_TYPE11��TYPE 11
	  ptr_bindInfo�����Ĳ���
	      �������type9���ͣ���Ҫ����
	      ptr_bindInfo->type9.cos
	      ptr_bindInfo->type9.streamId
	      �������type11���ͣ���Ҫ����
	      ptr_bindInfo->type11.letter
	      ptr_bindInfo->type11.mbox
	      �������DIO���ͣ�����Ҫ����
	����ֵ:
	  true:�ɹ�
	  false��ʧ��
	��ע: ÿ�ְ�ֻ��Ҫbindһ�Σ����治���ٵ��ã��ٵ��ûᱨ��
*********************************************************************/
Bool C6678_Srio_Bind(Srio_SocketType mode,Srio_SockBindAddrInfo ptr_bindInfo);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
	  Bool C6678_Srio_Type11_Send(Uint16 DstID,Uint16 mbox,Uint16 letter,uint32_t * Src,Uint32 Len)
	����˵��:
	  ����������Ŀ���豸����type11��
	����˵��:
	  DstID��Ŀ��ID
	  mbox:mbox��ֵ
	  letter��letter��ֵ
	  Src��Դ��ַ�����أ�
	  Len�����ݳ���(ӦΪ8B��������)
	����ֵ:
	  true:�ɹ�
	  false��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Srio_Type11_Send(Uint16 DstID,Uint16 mbox,Uint16 letter,uint32_t * Src,Uint32 Len);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
	  int32_t C6678_Srio_Type11_Recv(uint32_t * Dst,Srio_SockAddrInfo* from,Uint32 Len)
	����˵��:
	  �������ǽ���type11��srio��
	����˵��:
	  Dst��Ŀ�ĵ�ַ��Ŀ�꣩
	  from�����Ĳ���
	      ���Եõ�����
	      from.type11.letter
	      from.type11.mbox 
	  Len:���ݳ���(ӦΪ8B��������)
	����ֵ:
	  ���յ����ݸ���
	��ע: ��
*********************************************************************/
int32_t C6678_Srio_Type11_Recv(uint32_t * Dst,Srio_SockAddrInfo* from,Uint32 Len);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
	  Bool C6678_Srio_Type9_Send(Uint16 DstID,Uint16 streamId,Uint16 cos,uint32_t * Src,Uint32 Len)
	����˵��:
	  �������Ƿ���type9��srio��
	����˵��:
	  cos:cos��ֵ
	  streamId��stream id
	  DstID��Ŀ��ID
	  Src��Դ��ַ�����أ�
	  Len�����ݳ���
	����ֵ:
	  true:�ɹ�
	  false��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Srio_Type9_Send(Uint16 DstID,Uint16 streamId,Uint16 cos,uint32_t * Src,Uint32 Len);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
	  int32_t C6678_Srio_Type9_Recv(Uint32 * Dst,Srio_SockAddrInfo from,Uint32 Len)
	����˵��:
	  �������ǽ���type9��srio��
	����˵��:
	  Dst��Ŀ�ĵ�ַ��Ŀ�꣩
	  from��������Ϣ
	      ���Եõ�����
	      from.type9.cos
	      from.type9.streamId 
	  Len:���ݳ���
	����ֵ:
	  ���յ����ݸ���
	��ע: ��
*********************************************************************/
int32_t C6678_Srio_Type9_Recv(Uint32 * Dst,Srio_SockAddrInfo from,Uint32 Len);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_DoorbellInt_Hookup(Uint8 VectID,void interruptISR())

	����˵��:
		�������ǹ������ж�

	����˵��:
		interruptISR���жϺ���
		VectID:�ж������ţ�4-15��

	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Srio_DoorbellInt_Hookup(Uint8 VectID,void interruptISR());

/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Uint32 C6678_Srio_DoorbellInt_Info(Uint32 coreNum,uint8_t InfoMode)

	����˵��:
		�������ǻ�ȡ�жϵ�infoֵ

	����˵��:
		coreNum:�˺�
		InfoMode:
			����info�Ĳ�ͬģʽ
			0:8��ÿ���˿��Ը�����8��infoֵ��ƽ��ģʽ
			1:��0����16�� info��������5��info����ƽ��ģʽ
	����ֵ:
		infoֵ

	��ע:�����ж�ǰ����
	info����ϸ���ܼ�C6678_Srio_Dio_Doorbell��ע��
*********************************************************************/
Uint32 C6678_Srio_DoorbellInt_Info(Uint32 coreNum,uint8_t InfoMode);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_DoorbellInt_Clear(Uint32 coreNum,uint8_t InfoMode)

	����˵��:
		�����������ж�

	����˵��:
		coreNum:�˺�
		InfoMode:
			����info�Ĳ�ͬģʽ
			0:8��ÿ���˿��Ը�����8��infoֵ��ƽ��ģʽ
			1:��0����16�� info��������5��info����ƽ��ģʽ
	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע:��
*********************************************************************/
Bool C6678_Srio_DoorbellInt_Clear(Uint32 coreNum,uint8_t InfoMode);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Over_Hookup(Uint8 VectID,void interruptISR())

	����˵��:
		�������Ǵ�������ж�

	����˵��:
		interruptISR���жϺ���
		VectID:�ж������ţ�4-15��

	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: ���ȵ���
		C6678_CoreInt_Init ();
		C6678_ChipInt_Init (0);
*********************************************************************/
Bool C6678_Srio_Over_Hookup(Uint8 VectID,void interruptISR());
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Wait_Over(void)

	����˵��:
		�������ǵȴ������������

	����˵��:
		��

	����ֵ:
		true:����ɹ�
		false������ʧ��

	��ע:�������ǽ��봫������жϺ���Ҫ�������õĺ��� 
*********************************************************************/
Bool C6678_Srio_Wait_Over(void);

/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		void C6678_Srio_Over_Int_Clear(void)

	����˵��:
		�������ǽ��봫������жϺ����жϺ���

	����˵��:
		��

	����ֵ:
		��

	��ע:��
*********************************************************************/
void C6678_Srio_Over_Int_Clear(void);

/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Navigator_Init(Uint8 VectID,void  interruptISR())

	����˵��:
		�������ǳ�ʼ��srio����Ķ�˵���ģ��

	����˵��:
		VectID:�ж������ţ�4-15��
		interruptISR���жϷ��������
	����ֵ:
		true:��ʼ������
		false����ʼ��ʧ��

	��ע: ����
*********************************************************************/
Bool C6678_Srio_Navigator_Init(Uint8 VectID,void  interruptISR());
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_ReadandDoorbell_Ku(Uint32 PortId,uint32_t Src,uint32_t * Dst, Uint16 SrcID, Uint32 Len,Uint16 Info,Bool WaitOver,Bool IntrEn,Uint32 priority)

	����˵��:
		�������Ǵ�Ŀ���豸��ȡ���ݵ����ص�ַ,��ȡ��ɺ�������

	����˵��:
		PortId:�˿ں�
			��PhysicalMode==0ʱ:PortIdֻ��Ϊ0;
			��PhysicalMode==1ʱ:PortIdֻ��Ϊ0��2,�ֱ��Ӧ2��2x��port�ڷֱ�Ϊport0��port2;
			PhysicalModeΪC6678_Srio_Init�Ĳ�����
		Src:Դ��ַ��Ŀ�꣩
		Dst��Ŀ�ĵ�ַ�����أ�
		SrcID��Ŀ��ID
		Len�����ݳ��ȣ����δ������1MB,����Ϊ0��
		Info:������Ϣ
		���ʹ��C6678_Srio_Init��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(����0~7������corenum>>1,8~15������corenum>>1+1)
		���磺
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		���ʹ��C6678_Srio_Init_Core0more��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		core0��Ӧ16��infoֵ��������ÿ���˶�Ӧ5��infoֵ
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
			 	 	Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		WaitOver:�Ƿ�ȴ�����
			TRUE:�ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
			FALSE�����ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
		IntrEn���������ʱ�Ƿ����ж�
			TRUE:�����ж�
			FALSE���������ж�
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������
	����ֵ:
		true:�ɹ�
	 	false��ʧ��

	��ע: infoӦΪ����˵���е�ֵ������ֵ����������
*********************************************************************/
Bool C6678_Srio_Dio_ReadandDoorbell_Ku(Uint32 PortId,uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,Uint16 Info,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_ReadandDoorbell(uint32_t Src,uint32_t * Dst, Uint16 SrcID, Uint32 Len,Uint16 Info,Bool WaitOver,Bool IntrEn,Uint32 priority)

	����˵��:
		�������Ǵ�Ŀ���豸��ȡ���ݵ����ص�ַ,��ȡ��ɺ�������

	����˵��:
		Src:Դ��ַ��Ŀ�꣩
		Dst��Ŀ�ĵ�ַ�����أ�
		SrcID��Ŀ��ID
		Len�����ݳ��ȣ����δ������1MB,����Ϊ0��
		Info:������Ϣ
		���ʹ��C6678_Srio_Init��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(����0~7������corenum>>1,8~15������corenum>>1+1)
		���磺
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		���ʹ��C6678_Srio_Init_Core0more��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		core0��Ӧ16��infoֵ��������ÿ���˶�Ӧ5��infoֵ
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		WaitOver:�Ƿ�ȴ�����
			TRUE:�ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
			FALSE�����ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
		IntrEn���������ʱ�Ƿ����ж�
			TRUE:�����ж�
			FALSE���������ж�
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������
	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: infoӦΪ����˵���е�ֵ������ֵ����������
*********************************************************************/
Bool C6678_Srio_Dio_ReadandDoorbell(uint32_t Src,uint32_t * Dst,Uint16 SrcID,Uint32 Len,Uint16 Info,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_WriteandDoorbell_Ku(Uint32 PortId,Uint32 * Src, Uint32 Dst, Uint16 DstID, Uint32 Len,Uint16 Info,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

	����˵��:
		����������Ŀ���豸д����,д�����ݺ�������

	����˵��:
		PortId:�˿ں�
			��PhysicalMode==0ʱ:PortIdֻ��Ϊ0;
			��PhysicalMode==1ʱ:PortIdֻ��Ϊ0��2,�ֱ��Ӧ2��2x��port�ڷֱ�Ϊport0��port2;
			PhysicalModeΪC6678_Srio_Init�Ĳ�����
		Src:Դ��ַ�����أ�
		Dst��Ŀ�ĵ�ַ��Ŀ�꣩
		DstID��Ŀ��ID(��fpgaͨ��ʱ��Ŀ��ID����fpga�����Աȷ��)
		Len�����ݳ��ȣ����δ������1MB,����Ϊ0��
		Info:������Ϣ
		���ʹ��C6678_Srio_Init��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(����0~7������corenum>>1,8~15������corenum>>1+1)
		���磺
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		���ʹ��C6678_Srio_Init_Core0more��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		core0��Ӧ16��infoֵ��������ÿ���˶�Ӧ5��infoֵ
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
					Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
					Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		Writemode��д��ģʽ��
			Srio_Ttype_Write_NWRITE
			Srio_Ttype_Write_NWRITE_R
			Srio_Ftype_SWRITE
		WaitOver:�Ƿ�ȴ�����
			TRUE:�ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
			FALSE�����ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
		IntrEn���������ʱ�Ƿ����ж�
			TRUE:�����ж�
			FALSE���������ж�
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������
	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: infoӦΪ����˵���е�ֵ������ֵ����������
*********************************************************************/
Bool C6678_Srio_Dio_WriteandDoorbell_Ku(Uint32 PortId,Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,Uint16 Info,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		Bool C6678_Srio_Dio_WriteandDoorbell(Uint32 * Src, Uint32 Dst, Uint16 DstID, Uint32 Len,Uint16 Info,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);

	����˵��:
		����������Ŀ���豸д����,д�����ݺ�������

	����˵��:
		Src:Դ��ַ�����أ�
		Dst��Ŀ�ĵ�ַ��Ŀ�꣩
		DstID��Ŀ��ID(��fpgaͨ��ʱ��Ŀ��ID����fpga�����Աȷ��)
		Len�����ݳ��ȣ����δ������1MB,����Ϊ0��
		Info:������Ϣ
		���ʹ��C6678_Srio_Init��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		9bit      2bit         1bit       4bit
		reserved  corenum>>1   reserved   0~15(����0~7������corenum>>1,8~15������corenum>>1+1)
		���磺
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x08    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x27    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x28    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x29    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x47    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4f    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x67    0x00000080=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6f    0x00000080=C6678_Srio_DoorbellInt_Info(void)

		���ʹ��C6678_Srio_Init_Core0more��ʼ��SRIO,infoֵ�Ķ�Ӧ��ϵ���£�
		infoΪ16bit
		core0��Ӧ16��infoֵ��������ÿ���˶�Ӧ5��infoֵ
		�����Է���0��Info=0x00    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x01    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x07    0x00000080=C6678_Srio_DoorbellInt_Info(void)
					Info=0x08    0x00000100=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x09    0x00000200=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x0f    0x00008000=C6678_Srio_DoorbellInt_Info(void)
		�����Է���1��Info=0x20    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x21    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x24    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���2��Info=0x25    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x26    0x00000002=C6678_Srio_DoorbellInt_Info(void)
					...
			 	 	Info=0x29    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���3��Info=0x2a    0x00000001=C6678_Srio_DoorbellInt_Info(void)
					Info=0x2b    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x2e    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���4��Info=0x40    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x41    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x44    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���5��Info=0x48    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x49    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x4c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���6��Info=0x60    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x61    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x64    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		�����Է���7��Info=0x68    0x00000001=C6678_Srio_DoorbellInt_Info(void)
			 	 	Info=0x69    0x00000002=C6678_Srio_DoorbellInt_Info(void)
			 	 	...
			 	 	Info=0x6c    0x00000010=C6678_Srio_DoorbellInt_Info(void)
		Writemode��д��ģʽ��
			Srio_Ttype_Write_NWRITE
			Srio_Ttype_Write_NWRITE_R
			Srio_Ftype_SWRITE
		WaitOver:�Ƿ�ȴ�����
			TRUE:�ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
			FALSE�����ȴ�����(֮����Ҫ�ٵ���C6678_Srio_Wait_Over����)
		IntrEn���������ʱ�Ƿ����ж�
			TRUE:�����ж�
			FALSE���������ж�
		priority�����ȼ�
			��ֵֻ����0,1,2������ֵ��������
	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: infoӦΪ����˵���е�ֵ������ֵ����������
*********************************************************************/
Bool C6678_Srio_Dio_WriteandDoorbell(Uint32 * Src,Uint32 Dst,Uint16 DstID,Uint32 Len,Uint16 Info,Uint32 Writemode,Bool WaitOver,Bool IntrEn,Uint32 priority);
/********************************************************************
	��������

	�����ļ�:   srio.c
	����:		wj

	������:
		void C6678_Srio_Type9Type11_rxCompletionIsr(void)

	����˵��:
		���������յ�TYPE9��TYPE11���ж��еĴ�������

	����˵��:
		��
	����ֵ:
		��

	��ע: ��
*********************************************************************/
void C6678_Srio_Type9Type11_rxCompletionIsr(void);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����other.c�к���������
						XMC,power,ecc����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   other.c
	����:		wj

	������:
	  Bool C6678_XMC_MapSelect(Uint32 Logical32bitAddr,Phy36bit Physical36bitAddr,Uint32 MapLength,Uint32 CpuIndex);
	����˵��:
	  �������ǽ�32bit���߼���ַӳ�䵽36bit�������ַ��
	����˵��:
	  Logical32bitAddr:32bit�߼���ַ
	  Physical36bitAddr:36bit�����ַ
	  MapLength:ӳ��Ĵ�С
	         0:4KB
	         1:8KB=4KB*2
	         2:16KB=8KB*2
	         3:32KB=16KB*2
	         ........
	         20:4GB
	         (ӳ��Ĵ�С�����Ͽ���֧������ֵ����������ĿǰоƬ���ڴ����ƣ�����ֻ�ܲ���0��18,19��20�޷�����)
	  CpuIndex:cpuһ������ӳ��16�ԡ�0,1,2�ѱ�Ĭ��ӳ�䣬����ֻ��3��15�����û��޸ġ�
			 3~15
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ӳ���С����ӳ��ʱ32bit�߼���ַ��ȡַ��ַ��
	               ���磺���ӳ���СΪ4KB��32bit�߼���ַӦ��Ϊ4KB����ģ�С��4KB�ĵ�ַ��Ч��
*********************************************************************/
Bool C6678_XMC_MapSelect(Uint32 Logical32bitAddr,Phy36bit Physical36bitAddr,Uint32 MapLength,Uint32 CpuIndex);
/********************************************************************
	��������

	�����ļ�:   other.c
	����:		wj

	������:
	  void C6678_XMC_PrefetchBuffer_Inv(void)
	����˵��:
	  ������ʧЧXMCԤȡBuffer
	����˵��:
	  ��
	����ֵ:
	 ��
	��ע: ��
*********************************************************************/
void C6678_XMC_PrefetchBuffer_Inv(void);

/********************************************************************
	��������

	�����ļ�:   other.c
	����:		wj

	������:
	  void C6678_Power_UpDomains(void)
	����˵��:
	  ��������ʹ�����е�Դģ��
	����˵��:
	  ��
	����ֵ:
	 ��
	��ע: this function powers up the PA subsystem domains
*********************************************************************/
void C6678_Power_UpDomains(void);
/********************************************************************
	��������

	�����ļ�:   other.c
	����:		wj

	������:
	  void C6678_Ecc_Enable(void)
	����˵��:
	  ��������ʹ��ECC
	����˵��:
	  ��
	����ֵ:
	 ��
	��ע:
*********************************************************************/
void C6678_Ecc_Enable(void);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����intc.c�к���������
						intc����ز���
*********************************************************************/

/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  Bool C6678_CoreInt_Init (void)
	����˵��:
	  �����������ú˼��жϿ�������ʼ��
	����˵��:
	     ��
	����ֵ:
	  true:��ʼ�����óɹ�
	  false����ʼ������ʧ��
	��ע: �������� Bool C6678_CoreInt_Set ()��������֮ǰ����
*********************************************************************/
Bool C6678_CoreInt_Init (void);
/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  Bool C6678_ChipInt_Init (Uint8 ChipIntNum)
	����˵��:
	  �������������ض�Ƭ���жϿ������ĳ�ʼ����
	����˵��:
	  ChipIntNum��Ƭ���жϿ������ţ�0-3)

	����ֵ:
	  true:Ƭ���жϳ�ʼ���ɹ�
	  false��Ƭ���жϳ�ʼ��ʧ��
	��ע: �˺�����Bool C6678_ChipInt_Set()֮ǰ����
*********************************************************************/
Bool C6678_ChipInt_Init (Uint8 ChipIntNum);


/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  Bool  C6678_CoreInt_Set (Uint16 EventId,Uint8 VectId,void  interruptISR(),void* Param);
	����˵��:
	  �����������ú˼��жϿ�����ȥ��Ӧ�˼��ж��¼������������ж��¼��ţ��ж������ţ����жϷ������
	����˵��:
	  EventId���ж��¼���
	  VectId�� �ж�������
	  interruptISR���жϷ��������
	  Param�����ݵĲ��� ����ΪNULL��ʾ�����ݲ���
	����ֵ:
	  true:�ж����óɹ�
	  false���ж�����ʧ��
	��ע: ��
*********************************************************************/
Bool  C6678_CoreInt_Set (Uint16 EventId,Uint8 VectId,void  interruptISR(),void* Param);
/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  Bool  C6678_ChipInt_Set (Uint8 ChipIntNum, Uint16 EventId,Uint16 ChanId);
	����˵��:
	  ������������Ƭ���жϿ����������������ж��¼��ţ�ͨ����
	����˵��:
	  ChipIntNum��Ƭ���жϿ������ţ�0-3)
	  EventId���ж��¼���
	  ChanId�� ͨ����

	����ֵ:
	  true:Ƭ���ж����óɹ�
	  false��Ƭ���ж�����ʧ��
	��ע: Ƭ���жϿ�����ֻ�ǽ�Ƭ���ж��¼�ӳ�䵽�˼��ж��¼���
	      ��ӦƬ���ж��¼��������ú˼��жϿ����������жϷ������ȡ�
*********************************************************************/
Bool  C6678_ChipInt_Set (Uint8 ChipIntNum, Uint16 EventId,Uint16 ChanId);
/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  Bool C6678_ChipInt_Clear (Uint8 ChipIntNum,Uint32 EventId);
	����˵��:
	  ������������ض�Ƭ���жϿ��������ض��¼����ж�״̬��
	����˵��:
	  ChipIntNum��Ƭ���жϿ������ţ�0-3)
	  EventId��Ƭ���ж��¼���

	����ֵ:
	  true:Ƭ���ж�����ɹ�
	  false��Ƭ���ж����ʧ��
	��ע: �������Ƭ���ض��ж��¼����ж�״̬������ܻ�Ӱ����һ�θ��ж��¼�����Ӧ��
*********************************************************************/
Bool C6678_ChipInt_Clear (Uint8 ChipIntNum,Uint32 EventId);
/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  void  C6678_CoreInt_Clear (Uint32 EventId);
	����˵��:
	  ������������ض��˼��жϿ��������ض��¼����ж�״̬��
	����˵��:
	  EventId���˼��ж��¼���

	����ֵ:
	 ��
	��ע: ��������˼��ض��ж��¼����ж�״̬������ܻ�Ӱ����һ�θ��ж��¼�����Ӧ��
*********************************************************************/
void  C6678_CoreInt_Clear (Uint32 EventId);
/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  Bool  C6678_CoreInt_Close (void);
	����˵��:
	  �������ǹر��������ĺ˼�������ʵ����
	����˵��:
	  ��

	����ֵ:
	  true:�رճɹ�
	  false���ر�ʧ��
	��ע: �ú������ñ�������C6678_CoreInt_Set��������֮��
*********************************************************************/
Bool  C6678_CoreInt_Close (void);
/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  void  C6678_CoreInt_Manual_trigger(Uint16 EventId);
	����˵��:
	  ���������ֶ������˼��������ض��ж��¼���
	����˵��:
	  EventId���˼��ж��¼���

	����ֵ:
	  ��
	��ע: �ú������ñ�������C6678_CoreInt_Set��������֮��
*********************************************************************/
void  C6678_CoreInt_Manual_trigger(Uint16 EventId);
/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    dyx

	������:
	  void  C6678_ChipInt_Manual_trigger(Uint8 ChipIntNum,Uint16 EventId);
	����˵��:
	  ���������ֶ������ض�Ƭ���жϿ��������ض��ж��¼���
	����˵��:
	  ChipIntNum��Ƭ���жϿ������ţ�0-3����
	  EventId��Ƭ���ж��¼��š�

	����ֵ:
	  ��
	��ע: �ú������ñ�������C6678_ChipInt_Set��������֮��
*********************************************************************/
void  C6678_ChipInt_Manual_trigger(Uint8 ChipIntNum,Uint16 EventId);
/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    wj

	������:InterrputRegInfo  C6678_Int_SaveInterrputRegInfo (void);

	����˵��: �������Ǳ��浱ǰ�жϼĴ�����ֵ��

	����˵��:
	  ��

	����ֵ: ������ǰ�жϼĴ�����ֵ�Ľṹ�壬���ڻָ��Ĵ���ʱʹ��

	��ע: �����ж�Ƕ�ף���Ҫ���жϷ������Ŀ�ʼ���á�
*********************************************************************/
InterrputRegInfo  C6678_Int_SaveInterrputRegInfo (void);

/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    wj

	������:void  C6678_Int_RestoreInterrputRegInfo (InterrputRegInfo info);

	����˵��:�������ǻָ��ж�Ƕ��֮ǰ�жϼĴ�����ֵ��

	����˵��:
	  info������Ƕ��֮ǰ�жϼĴ�����ֵ�Ľṹ��

	����ֵ: ��

	��ע: �����ж�Ƕ�ף���Ҫ���жϷ������������á�
*********************************************************************/
void  C6678_Int_RestoreInterrputRegInfo (InterrputRegInfo info);

/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    wj

	������:Bool  C6678_Int_GlobalDisable (void);

	����˵��:��������ʧЧȫ���жϡ�

	����˵��: ��

	����ֵ:
	  true:ʧЧȫ���жϳɹ�
	  false��ʧЧȫ���ж�ʧ��

	��ע: ��
*********************************************************************/
Bool  C6678_Int_GlobalDisable (void);

/********************************************************************
	��������

	�����ļ�:   intc.c
	����:	    wj

	������:
	  Bool C6678_Int_GlobalEnable (void)
	����˵��:
	  ��������ʹ��ȫ���жϡ�
	����˵��:
	     ��
	����ֵ:
	  true:ʹ��ȫ���жϳɹ�
	  false��ʹ��ȫ���ж�ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Int_GlobalEnable (void);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����sem2.c�к���������
						sem2����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   sem2.c
	����:	    dyx

	������:
	  Uint8 C6678_Sem2_Acquire(Uint8 Sem2Id,Uint8 Mod);
	����˵��:
	  �������ǻ�ȡ�ض��ź���
	����˵��:
	  Sem2Id���ź�����(0-31)
	  Mod�� ��ȡ�ź�����ʽ��0-ֱ�ӻ�ȡ��1-��ӻ�ȡ��2-���Ϸ�ʽ��ȡ
	����ֵ:
		��ȡ�ɹ��󷵻�0xff;
		��ȡʧ��ʱ�᷵�ص�ǰӵ�и��ź����ĺ�ID;
		�������ô��󷵻�0xaa;
	��ע: ��
*********************************************************************/
Uint8 C6678_Sem2_Acquire(Uint8 Sem2Id,Uint8 Mod);
/********************************************************************
	��������

	�����ļ�:   sem2.c
	����:	    dyx

	������:
	  Uint32 C6678_Sem2_GetSatus(Uint8 MasterId);
	����˵��:
	  �������ǻ�ȡ�ض��˵��ź���״̬�Ĵ�����Ϣ
	����˵��:
	  MasterId����ID

	����ֵ:
	   �ض����ź�����־״̬�Ĵ�����״̬

	��ע: ֻ�м�ӻ�ȡ�򸴺ϻ�ȡʱ�Ż�Ӱ��üĴ�����ֵ��
	      ��nλ�����n���ź����Ļ�ȡ״̬�����Ϊ1��
	      ˵���ú��յ������ź����Ļ�ȡ�жϣ���������������ӵ�и��ź���
*********************************************************************/
Uint32 C6678_Sem2_GetSatus(Uint8 MasterId);
/********************************************************************
	��������

	�����ļ�:   sem2.c
	����:	    dyx

	������:
		Bool C6678_Sem2_IntcStatusClear(Uint8 MasterId,Uint8 Sem2Id);
	����˵��:
		�����������sem2���ж���Ϣ���Ա����´�sem2���ж���Ӧ
	����˵��:
		MasterId����ID
		Sem2Id���ź�����(0-31)

	����ֵ:
		TRUE:����ɹ�
		FALSE:���ʧ��

	��ע: �˺������ں���Ӧ��sem2�жϺ���ã����磬���жϷ�������ڣ�
*********************************************************************/
Bool C6678_Sem2_IntcStatusClear(Uint8 MasterId,Uint8 Sem2Id);
/********************************************************************
	��������

	�����ļ�:   sem2.c
	����:	    dyx

	������:
	  Bool C6678_Sem2_Release(Uint8 Sem2Id);
	����˵��:
	  ���������ͷ��ѻ�ȡ���ź�����ע�⣺�Ǹ���ռ���Ǹ����ͷ�
	����˵��:

	  Sem2Id���ź�����(0-31)

	����ֵ:
	   TRUE:�ͷųɹ�
	   FALSE:�ͷ�ʧ��

	��ע: �ڵ��ô˺���֮ǰ�������е��� C6678_Sem2_Acquire������ȡ����Ӧ���ź�������������
*********************************************************************/
Bool C6678_Sem2_Release(Uint8 Sem2Id);


/********************************************************************
	��������

	�����ļ�:   sem2.c
	����:	    dyx

	������:
	  void C6678_Sem2_MultiCore_Syn(void);
	����˵��:�ú���Ϊ�����ź����Ķ��ͬ��������

	����˵��:
            ��

	����ֵ:
	   ��

	��ע:�ú���ռ�õ�20�ż�����Ĺ�10���ź���
*********************************************************************/
void C6678_Sem2_MultiCore_Syn(void);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����edma3.c�к���������
						edma3����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Global_Init(void)

	����˵��: �ú�������������edma3��������֮ǰ����

	����˵��:��

	����ֵ:
	  TRUE:��ʼ���ɹ�
	  FALSE:��ʼ��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Edma3_Global_Init(void);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Channel_Init(Uint8 ChannelCtrlNum,Uint32 ChannelNum,Uint8 TCNum,Int8 ShadowRegion,Uint32 ParamNum)

	����˵��:
	      �ú�����C6678_Edma3_Global_Init()֮����C6678_Edma3_Transdata_continue/
	      C6678_Edma3_Transdata_interval/C6678_Edma3_Link_Transdata()/C6678_Edma3_Sort_TransData/
	      C6678_Edma3_SubframeExtraction_TransData֮ǰ���ã������������ݴ����DMA��������
	      ͨ����ͨ�����ò�����

	����˵��:
	  ChannelCtrlNum��DMA��������(0~2)
	  ChannelNum��DMAͨ����(ע��0�ſ�����ֻ��16��ͨ����1/2�ſ�������64��ͨ����
	  	  	  	  (DMA������Ϊ0ʱ��0~15��DMA������Ϊ1,2ʱ��0~63)
	  TCNum:
	  	  ���ChannelCtrlNumΪ0��TCNum��ѡ��0,1��
	  	  ���ChannelCtrlNumΪ1��2��TCNum��ѡ��0,1,2,3��
	  ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
		ParamNum���������PARAM�ţ�0~511��
	����ֵ:
	  TRUE:��ʼ���ɹ�
	  FALSE:��ʼ��ʧ��

	��ע: ��˶�ͨ��ʱ��ͳһ�ɺ�0��dma��ʼ��
*********************************************************************/
Bool C6678_Edma3_Channel_Init(Uint8 ChannelCtrlNum,Uint32 ChannelNum,Uint8 TCNum,Int8 ShadowRegion,Uint32 ParamNum);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Channel_Close(Uint8 ChannelCtrlNum, Uint8 ChannelNum)

	����˵��:�ú�����C6678_Edma3_Transdata_continue/C6678_Edma3_Transdata_interval
	  /C6678_Edma3_Link_Transdata()/C6678_Edma3_Sort_TransData/
	  C6678_Edma3_SubframeExtraction_TransData֮����ã��رյ��õ�DMAͨ��

	����˵��:
	ChannelCtrlNum:ͨ����������
	ChanNum�����رյ�EDMA3ͨ����

	����ֵ:
	  TRUE:�رճɹ�
	  FALSE:�ر�ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Edma3_Channel_Close(Uint8 ChannelCtrlNum,Uint8 ChannelNum);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Ctrl_Close(Uint8 ChannelCtrlNum)

	����˵��:�ú�����C6678_Edma3_Channel_Close()֮����ã��ر�ָ����DMA������

	����˵��:
	CtrlNum�����رյ�EDMA3��������

	����ֵ:
	  TRUE:�رճɹ�
	  FALSE:�ر�ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Edma3_Ctrl_Close(Uint8 ChannelCtrlNum);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Transdata_continue(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,
	            Uint32 ParamNum,Uint32 sAddr, Uint32  dAddr, Uint16 Len_KB, Bool DstIsFifo, Bool IntrEn,Uint8 nTCC)

	����˵��: ��������DMAͨ�õ�������ַ�����ݴ��亯��
	  
	����˵��:
	   ChannelCtrlNum��DMA��������
	   ChannelNum��DMAͨ����(ע��0�ſ�����ֻ��16��ͨ����1/2�ſ�������64��ͨ��
	   ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
	   ParamNum���������PARAM��
	   sAddr: Դ��ַ��
	   dAddr: Ŀ�ĵ�ַ��
	   Len_KB: ���䳤��(ע����KBΪ��λ!)����2KB������Len_KB=2
	   dstIsFifo:
		  =1��Ŀ��Ϊfifo��=0��ԴΪfifo��=����ֵ��Ŀ����Դ��ΪRAM��
	   intrEn:
		  =1��DMA�ж�ʹ�ܣ����ȴ�DMA���������=0����DMA�жϣ��ȴ�DMA����������˳�;
		  =����ֵ����DMA�жϣ����ȴ�DMA���������
	   nTCC: edma3�������info(0-63),��Ҫ��ChannelNumһ��

	����ֵ:
	   TRUE:��������
	   FALSE������ʧ��

	��ע: AB mode,global region,��󳤶�Ϊ64MB���ú�����BoolC6678_Edma3_Global_Init();
�� Bool C6678_Edma3_Channel_Init();��������֮�����
*********************************************************************/
Bool C6678_Edma3_Transdata_continue(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,Uint32 sAddr, Uint32 dAddr,Uint16 Len_KB,Bool DstIsFifo,Bool IntrEn,Uint8 nTCC);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Transdata_interval(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion, Uint32 ParamNum,
	            DmaTranParam  Srcset , DmaTranParam  Dstset , Bool DstIsFifo, Bool IntrEn ,Uint8 nTCC)

	����˵��:  ��������DMAͨ�õ������ַ�����ݴ��亯��

	����˵��:
	   ChannelCtrlNum��DMA��������
	   ChannelNum��DMAͨ����(ע��0�ſ�����ֻ��16��ͨ����1/2�ſ�������64��ͨ��)
	   ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
	   ParamNum���������PARAM��
	   Srcset: Դ��ַ������������ã�����μ��ṹ��˵����
	   Dstset: Ŀ�ĵ�ַ������������ã�����μ��ṹ��˵����
	   DstIsFifo:
		  =1��Ŀ��Ϊfifo��=0��ԴΪfifo��=����ֵ��Ŀ����Դ��ΪRAM��
	   IntrEn:
		  =1��DMA�ж�ʹ�ܣ����ȴ�DMA���������=0����DMA�жϣ��ȴ�DMA����������˳�;
		  =����ֵ����DMA�жϣ����ȴ�DMA���������
       nTCC: edma3�������info(0-63),��Ҫ��ChannelNumһ��
	����ֵ:
	  TRUE:����ɹ�
	  FALSE������ʧ��
	��ע: ֧��AB��ABCģʽ ,��Ϊ ABCģʽ ʱ ���� Srcset.Ccnt>1��nTCC=ChannelNum���ú�����BoolC6678_Edma3_Global_Init();
Bool C6678_Edma3_Channel_Init();��������֮����� 
*********************************************************************/
Bool C6678_Edma3_Transdata_interval(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,DmaTranParam  Srcset,DmaTranParam  Dstset,Bool DstIsFifo,Bool IntrEn,Uint8 nTCC);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Link_Transdata(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,Uint8 ParamCnt,DmaTranParam * pSrcset,
	           DmaTranParam * pDstset,Bool DstIsFifo,Bool IntrEn,Uint8 nTCC)

	����˵��:  ��������DMA link ���ݴ��亯��

	����˵��:
	  ChannelCtrlNum��DMA��������
	  ChannelNum��DMAͨ����(ע��0�ſ�����ֻ��16��ͨ����1/2�ſ�������64��ͨ��)
	  ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
	  ParamNum:���ڴ���ĵ�һ���������PARAM��
	  ParamCnt��LINK��PARAM�������ú�������ռ�д�ParamNum�ſ�ʼ��ParamCnt��PARAM
	  pSrcset��Դ��ַ������������ò�������ָ�룬����ά��=ParamCnt
	  pDstset��Ŀ�ĵ�ַ������������ò�������ָ�룬����ά��=ParamCnt
	  DstIsFifo�� =1��Ŀ��Ϊfifo��=0��ԴΪfifo��=����ֵ��Ŀ����Դ��ΪRAM��
	  IntrEn��=1��DMA�ж�ʹ�ܣ����ȴ�DMA���������=0����DMA�жϣ��ȴ�DMA����������˳�;
		  =����ֵ����DMA�жϣ����ȴ�DMA���������
      nTCC:edma3�������info,��Ҫ��ChannelNumһ��

	����ֵ:
	  TRUE:����ɹ�
	  FALSE������ʧ��

	��ע: �ú�������֮ǰ�����C6678_Edma3_Global_Init������C6678_Edma3_Channel_Init��������ʼ��
	��Ϊ ABCģʽ ʱ ���� Srcset.Ccnt>1��nTCC=ChannelNum
*********************************************************************/
Bool C6678_Edma3_Link_Transdata(Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,Uint8 ParamCnt,DmaTranParam * pSrcset,DmaTranParam * pDstset,Bool DstIsFifo,Bool IntrEn,Uint8 nTCC);
                                 
/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Chain_Init(Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaChainParam * ChainChPa)

	����˵��: ��������DMA chaining ����ĳ�ʼ������

	����˵��:
	  ChannelCtrlNum��DMA��������
	  ChainCnt����Ҫchaining ��DMAͨ������(ע��0�ſ�����ֻ��16��ͨ����1��2�ſ�������64��ͨ����
	  ChainChPa:chain�ڵ�ʹ�õ�dmaͨ����,param��,TC��,ShadowRegion����������ָ�룬����ά��=ChainCnt

	����ֵ:
	  TRUE:��ʼ���ɹ�
	  FALSE����ʼ��ʧ��

	��ע: ֻ�к�0����,ע��0�ſ�����ֻ��16��ͨ����128��param��16��ͨ��������128��param
	      1��2�ſ�������64��ͨ����512��param��64��ͨ��������512��param
		ע��ÿһͨ��ʹ�õ�param��Ӧ��������ͨ����ͬ��
		ע�����N���˵��ã�ÿ���˵��õ�ͨ����Ӧ���������˲�ͬ��
		�������N��chain��ÿ��chain���õ�ͨ����Ӧ��������chain��ͬ��
*********************************************************************/
Bool C6678_Edma3_Chain_Init(Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaChainParam * ChainChPa);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Chain_TransData (Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaTranParam * pSrcset,DmaTranParam * pDstset,DmaChainParam * ChainChPa,Bool DstIsFifo, Bool IntrEn,Uint32 nTCC)

	����˵��:��������chaining DMA���亯��

	����˵��:
	 pSrcset��Դ��ַ������������ò�������ָ�룬����ά��=ChainCnt
	 pDstset��Ŀ�ĵ�ַ������������ò�������ָ�룬����ά��=ChainCnt
	 DstIsFifo�� =1��Ŀ��Ϊfifo��=0��ԴΪfifo��=����ֵ��Ŀ����Դ��ΪRAM��
	 IntrEn��=1��DMA�ж�ʹ�ܣ����ȴ�DMA���������=0����DMA�жϣ��ȴ�DMA����������˳�;
		  =����ֵ����DMA�жϣ����ȴ�DMA���������
	 ChannelCtrlNum��DMA��������
	 ChainCnt����Ҫchaining ��DMAͨ������(ע��0�ſ�����ֻ��16��ͨ����1��2�ſ�������64��ͨ����
	 ChainChPa:chain�ڵ�ʹ�õ�dmaͨ���ź�param�Ų���������ָ�룬����ά��=ChainCnt
     nTCC:  ������ɴ��� TCC info ��Ϣ,��Ҫ������һ���ù���ChannelNumһ��
	����ֵ:
	  TRUE:����ɹ�
	  FALSE������ʧ��

	��ע: �ú�������ǰ�����C6678_Edma3_Chain_Init()����֧��ABC���䣬��pSrcset[].Ccnt=1��
		�ú�����ChainChPa��������Ӧ����C6678_Edma3_Chain_Init��ChainChPa��������һ�¡�
	 	 ע��0�ſ�����ֻ��16��ͨ����128��param��16��ͨ��������128��param��
	    1��2�ſ�������64��ͨ����512��param��64��ͨ��������512��param��
		ע��ÿһͨ��ʹ�õ�param��Ӧ��������ͨ����ͬ��
		ע�����N���˵��ã�ÿ���˵��õ�ͨ����Ӧ���������˲�ͬ��
		�������N��chain��ÿ��chain���õ�ͨ����Ӧ��������chain��ͬ��
*********************************************************************/
Bool C6678_Edma3_Chain_TransData (Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaTranParam * pSrcset,DmaTranParam * pDstset,DmaChainParam * ChainChPa,Bool DstIsFifo, Bool IntrEn,Uint32 nTCC);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Chain_Close(Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaChainParam * ChainChPa)
	  
	����˵��:�رյ��õ�DMA��Դ
	  
	����˵��:
	  ChannelCtrlNum��DMA��������
	  ChainCnt����Ҫchaining ��DMAͨ������(ע��0�ſ�����ֻ��16��ͨ����1/2�ſ�������64��ͨ��
	  ChainChPa:chain�ڵ�ʹ�õ�dmaͨ���ź�param�Ų���������ָ�룬����ά��=ChainCnt
	  
	����ֵ:
	  TRUE:�رճɹ�
	  FALSE:�ر�ʧ��

	��ע: �ú�������C6678_Edma3_Chain_TransData()֮�����
	�ú�����ChainChPa��������Ӧ����C6678_Edma3_Chain_Init��ChainChPa��������һ�¡�
*********************************************************************/
Bool C6678_Edma3_Chain_Close(Uint32 ChannelCtrlNum,Uint8 ChainCnt,DmaChainParam * ChainChPa);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_SubframeExtraction_TransData (Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,
	           DmaSubFrameExtractTranParam  SubFraExtrTranParam,Uint8 nTCC)
	  
	����˵��:�ú����ǲ���DMA�Ӵ�����ݾ�������ȡ�Ӿ���
	  
	����˵��:
	  ChannelCtrlNum��DMA��������
      ChannelNum��DMAͨ����(ע��0�ſ�����ֻ��16��ͨ����1/2�ſ�������64��ͨ��)
      ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
      ParamNum:���ڴ���ĵ�һ���������PARAM��
	  SubFraExtrTranParam��DMA��֡��ȡ���������������μ��ṹ��
	  nTCC:edma �������info,��Ҫ��ChannelNumһ��

	����ֵ:
	   TRUE:����ɹ�
	  FALSE������ʧ��

	��ע: �ú�������֮ǰ�����C6678_Edma3_Global_Init������C6678_Edma3_Channel_Init������ʼ��
*********************************************************************/
Bool C6678_Edma3_SubframeExtraction_TransData (Uint8 ChannelCtrlNum,Uint8 ChannelNum,Int8 ShadowRegion,Uint32 ParamNum,DmaSubFrameExtractTranParam  SubFraExtrTranParam,Uint8 nTCC);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Over_Hookup(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 coreNum,Uint8 EventId,Uint8 VectID,void interruptISR())

	����˵��:
	  �������ǹ�edma3��������ж�

	����˵��:

	  ChannelCtrlNum��DMA����ͨ���������ţ�0~2��
	  ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
	  coreNum:���غ�ID
	  EventId���˼��ж��¼���
	  	      �ú˼��ж��¼��ţ���ָ��Ƭ�����������˼������������롣
	  	      ÿ���˶���17��ֻ�����˷��͵�Ƭ�����������룬
	  	    ��0~16��
	  	    ���У�0~10����21~31�˼��¼���11~12����62~63�˼��¼�,13~16����92~95�˼��¼�
	  VectID:�ж������ţ�4-15��
	  interruptISR���жϷ�����
	����ֵ:
	  true:�ɹ�
	  false��ʧ��

	��ע: ֻ�ɺ�0�����ã����ô˺���ǰ���ȵ��� C6678_CoreInt_Init ()��C6678_ChipInt_Init (0);
*********************************************************************/
Bool C6678_Edma3_Over_Hookup(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 coreNum,Uint8 EventId,Uint8 VectID,void interruptISR());

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:void C6678_Edma3_Wait_TransData_over(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 nTCC)

	����˵��:
	  �������ǵȴ�DMA������ɺ�����������ж��¼���TCC��״̬

	����˵��:
	  ChannelCtrlNum��DMA�������ţ�0~2��
	  ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
	  nTCC:�������info��
	����ֵ:
	  TRUE:��ʼ���ɹ�
	  FALSE����ʼ��ʧ��

	��ע: ���Edma3����ģʽΪ���ȴ���������������´�edma3�����¼�֮ǰ����øú�������ôδ���TCC
*********************************************************************/
void C6678_Edma3_Wait_TransData_over(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 nTCC);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:Bool C6678_Edma3_Over_Intc_Clear(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 EventId)

	����˵��:
	  ���������edma3�ж��¼�״̬

	����˵��:
	  ChannelCtrlNum��DMA�������ţ�0~2��
	  ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
	  EventId���˼��ж��¼���
	  	      �ú˼��ж��¼��ţ���ָ��Ƭ�����������˼������������롣
	  	      ÿ���˶���17��ֻ�����˷��͵�Ƭ�����������룬
	  	    ��0~16��
	  	    ���У�0~10����21~31�˼��¼���11~12����62~63�˼��¼�,13~16����92~95�˼��¼�
	����ֵ:
	  ��

	��ע: �ú�����edma3�жϷ�������У�ֻ�ɺ�0����,�ú����õ��ĺ˼��¼���������ظ�ʹ�á�
*********************************************************************/
Bool C6678_Edma3_Over_Intc_Clear(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint8 EventId);

/********************************************************************
	��������

	�����ļ�:   edma3.c
	����:	    dyx

	������:void C6678_Edma3_Get_TCCInfo(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint64edmaTccInfo *edma3IntrInfo)

	����˵��:
	  �������ǻ�ȡEdma TCC info ��Ϣ��TCC��edma3IntrInfo��Ӧ��ϵ��
	  TCC��                                                                  edma3IntrInfo->infoL��
      0                             1
      1                             2
      2                             4
      .                             .
      .                             .
      31                            2^31
      TCC��                                                                  edma3IntrInfo->infoH��
      0                             1
      1                             2
      2                             4
      .                             .
      .                             .
      31                            2^31

            ������edma3IntrInfoΪ3 ,�����������edma����������ֱ�ΪTCC=0,TCC=1�����δ��䣬�Դ�����
	����˵��:
	  ChannelCtrlNum��DMA�������ţ�0~2��
	  ShadowRegion: SHADOW Regions�������
	  	  -1 �� CSL_EDMA3_REGION_GLOBAL
		   0��  CSL_EDMA3_REGION_0
		   1��  CSL_EDMA3_REGION_1
		   2��  CSL_EDMA3_REGION_2
		   3��  CSL_EDMA3_REGION_3
		   4��  CSL_EDMA3_REGION_4
		   5��  CSL_EDMA3_REGION_5
		   6��  CSL_EDMA3_REGION_6
		   7��  CSL_EDMA3_REGION_7
      edma3IntrInfo:TCC info ��Ϣָ��
	����ֵ:
	  ��

	��ע: �����´�DMA����ʱ���������Ӧ��TCC info
*********************************************************************/
void C6678_Edma3_Get_TCCInfo(Uint8 ChannelCtrlNum,Int8 ShadowRegion,Uint64edmaTccInfo *edma3IntrInfo);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����timer.c�к���������
						timer����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
	  void C6678_Timer_Init(void)
	����˵��:
	  �������ǳ�ʼ��TIMERģ��
	����˵��:
	  ��
	����ֵ:
	  ��
	��ע:
*********************************************************************/
void C6678_Timer_Init(void);

/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
	  Bool C6678_Timer_Set(Uint8 TimerNum,Uint8 TimerMode,Uint64Cnt Counter,Uint64Prd Period,Uint4Prescale Prescale,Uint32 ClkinMode,Uint8 PulseWidth)
	����˵��:
	  ������������һ����ʱ��
	����˵��:
	  TimerNum����ʱ���ţ���16����0-15������0��7��������ⶨʱ������8-15�������watch dog��ʱ����
	  TimerMode:��ʱ�����ࡣ
	  	  	  0��watch dog��ʱ��
	  	  	  1��gp��ʱ��
	  	  	  2: ��ʽ32bit��ʱ��
			  3����λ��Ӧ�Ķ�ʱ������������������32bit��ʱ��
			  4����λ��Ӧ�Ķ�ʱ������������������32bit��ʱ��
	  Counter����ʱ���ĳ�ʼֵ��
	  Period����ʱ��������ֵ��
	  Prescale:ֻ��TimerMode==4ʱ�������������Ч��pre��ʱ���ĳ�ʼֵ������ֵ��
	  ClkinMode:����ʱ�ӵ����ࡣ
			0�������ڲ�ʱ�ӡ�
			1�������ⲿʱ�ӡ�
	  PulseWidth��1������ģ�1��ʱ�����ڡ�
				2������ģ�2��ʱ�����ڡ�
				3������ģ�3��ʱ�����ڡ�
				4������ģ�4��ʱ�����ڡ�
	����ֵ:
	  true:���óɹ�
	  false������ʧ��
	��ע: ֻ�ܽ�timer0��7��Ϊwatch dog ��ʱ����
	watch dog��ʱ��������ģʽ��ʱ�ӱ��������ڲ�ʱ�ӡ�
*********************************************************************/
Bool C6678_Timer_Set(Uint8 TimerNum,Uint8 TimerMode,Uint64Cnt Counter,Uint64Prd Period,Uint4Prescale Prescale,Uint32 ClkinMode,Uint8 PulseWidth);
/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
		Bool C6678_Timer_Start(Uint8 TimerNum,Uint8 TimerOutMode)
	����˵��:
		����������������
	����˵��:
		TimerNum:��ʱ���ţ�ֻ����0��7��
		TimerOutMode:�������
	  	  	0��pulse mode
	  	  	1��continuously mode
	����ֵ:
		true:�����ɹ�
	  	false:����ʧ��
	��ע:
		��
*********************************************************************/
Bool C6678_Timer_Start(Uint8 TimerNum,Uint8 TimerOutMode);
/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
	  Uint32 C6678_Timer_GetCountHi32(Uint8 TimerNum)
	����˵��:
	  �������õ����м������ĸ�32bit(CNTHI)
	����˵��:
	  TimerNum:��ʱ���ţ�0��15��
	����ֵ:
	  �������ڼ������ĸ�32bit
	��ע: ��
*********************************************************************/
Uint32 C6678_Timer_GetCountHi32(Uint8 TimerNum);
/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
	  Uint32 C6678_Timer_GetCountLo32(Uint8 TimerNum)
	����˵��:
	  �������õ����м������ĵ�32bit(CNTHI)
	����˵��:
	  TimerNum:��ʱ���ţ�0��15��
	����ֵ:
	  �������ڼ������ĵ�32bit
	��ע: ��
*********************************************************************/
Uint32 C6678_Timer_GetCountLo32(Uint8 TimerNum);
/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
	  void C6678_Timer_Close(Uint8 TimerNum)
	����˵��:
	  �������ǹر�TIMERģ��
	����˵��:
		TimerNum:��ʱ���ţ�0��15��
	����ֵ:
	  ��
	��ע:
*********************************************************************/
void C6678_Timer_Close(Uint8 TimerNum);

/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
	  Bool C6678_Timer_Intc_Hookup(Uint8 TimerNum,Uint8 TimerMode,Uint8 VectID,void interruptISR())
	����˵��:
	  �������ǹ�TIMER�ж�
	����˵��:
		TimerNum����ʱ���ţ���16����0-15������0��7��������ⶨʱ������8-15�������watch dog��ʱ��;
						      0�Ŷ�ʱ�����Ա���0��Ӧ��
						      1�Ŷ�ʱ�����Ա���1��Ӧ,
						      ......
						      7�Ŷ�ʱ�����Ա���7��Ӧ��
						      8-15�Ŷ�ʱ�����Ա�8���˶���Ӧ
						      	  ��
		TimerMode:��ʱ�����ࡣ
	  	  	  0��watch dog��ʱ��
	  	  	  1��gp��ʱ��
	  	  	  2: ��ʽ32bit��ʱ��
			  3����λ��Ӧ�Ķ�ʱ������������������32bit��ʱ��
			  4����λ��Ӧ�Ķ�ʱ������������������32bit��ʱ��
		interruptISR()���жϷ������
		VectID:�ж������ţ�4-15��
	����ֵ:
	  ��
	��ע:��
*********************************************************************/
Bool C6678_Timer_Intc_Hookup(Uint8 TimerNum,Uint8 TimerMode,Uint8 VectID,void interruptISR());
/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
	  Bool C6678_Timer_Intc_Clear(Uint8 TimerNum,Uint8 TimerMode)
	����˵��:
	  �������ǹ�TIMER�ж�
	����˵��:
		TimerNum����ʱ���ţ���16����0-15������0��7��������ⶨʱ������8-15�������watch dog��ʱ����
		TimerMode:��ʱ�����ࡣ
	  	  	  0��watch dog��ʱ��
	  	  	  1��gp��ʱ��
	  	  	  2: ��ʽ32bit��ʱ��
			  3����λ��Ӧ�Ķ�ʱ������������������32bit��ʱ��
			  4����λ��Ӧ�Ķ�ʱ������������������32bit��ʱ��
	����ֵ:
	  ��
	��ע:��
*********************************************************************/
Bool C6678_Timer_Intc_Clear(Uint8 TimerNum,Uint8 TimerMode);

/********************************************************************
	��������

	�����ļ�:   timer.c
	����:		wj

	������:
	  void C6678_Timer_Watchdog_Feed(Uint8 TimerNum)
	����˵��:
	  �������Ǹ�watchdog��ʱ��ι��
	����˵��:
		TimerNum����ʱ���ţ���16����0-15������0��7��������ⶨʱ������8-15�������watch dog��ʱ����
	����ֵ:
	  ��
	��ע:��
*********************************************************************/
void C6678_Timer_Watchdog_Feed(Uint8 TimerNum);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����hyperlink.c�к���������
						hyperlink����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   hyperlink.c
	����:		wj

	������:
	  Bool C6678_Hypelink_Init(unsigned int SerRate);
	����˵��:
	  �������ǳ�ʼ��hyperlink
	����˵��:
	  SerRate:Hyperlink�������(��c6678.h�е�HYPLNK_RATE)
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Hypelink_Init(unsigned int SerRate);
/********************************************************************
	��������

	�����ļ�:   hyperlink.c
	����:		wj

	������:
	  Bool C6678_Hypelink_AddrMap(HyplnkAddrMapParam * phyplnkParam);
	����˵��:
	  ������ʵ��hyperlink���ڴ�ӳ��
	����˵��:
	  phyplnkParam:hyperlink��ַת���Ĳ������ã���c6678.h�е�HyplnkAddrMapParam�Ķ��壩
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Hypelink_AddrMap(HyplnkAddrMapParam * phyplnkParam);
/********************************************************************
	��������

	�����ļ�:   hyperlink.c
	����:		wj

	������:
		Bool C6678_Hypelink_Int_Hookup(Uint8 VectID,void interruptISR())

	����˵��:
		�������ǹ�hyperlink�ж�

	����˵��:
		interruptISR���жϺ���
		VectID:�ж������ţ�4-15��
	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע: ���ȵ���
		C6678_CoreInt_Init ();
		C6678_ChipInt_Init (0);
*********************************************************************/
Bool C6678_Hypelink_Int_Hookup(Uint8 VectID,void interruptISR());
/********************************************************************
	��������

	�����ļ�:   hyperlink.c
	����:		wj

	������:
		Uint32 C6678_Hypelink_Int_Info(void)

	����˵��:
		�������ǻ�ȡ�жϵ�infoֵ

	����˵��:
		��

	����ֵ:
		��ȷ���أ�infoֵ(32λ����)
		���󷵻أ�0xff

	��ע:�����ж�ǰ���á�
	C6678_Hypelink_Int_Trigger(info)�е�info�ʹ˺������ص�infoֵ�Ķ�Ӧ��ϵΪ
	C6678_Hypelink_Int_Trigger(0)��Ӧ0x00000001=C6678_Hypelink_Int_Info(void)
	C6678_Hypelink_Int_Trigger(1)��Ӧ0x00000002=C6678_Hypelink_Int_Info(void)
	......
	C6678_Hypelink_Int_Trigger(31)��Ӧ0x80000000=C6678_Hypelink_Int_Info(void)
*********************************************************************/
Uint32 C6678_Hypelink_Int_Info(void);
/********************************************************************
	��������

	�����ļ�:   hyperlink.c
	����:		wj

	������:
		Bool C6678_Hypelink_Int_Clear(void)

	����˵��:
		�����������ж�

	����˵��:
		��

	����ֵ:
		TRUE:��ȷ����
		FALSE:���󷵻�

	��ע:��
*********************************************************************/
Bool C6678_Hypelink_Int_Clear(void);
/********************************************************************
	��������

	�����ļ�:   hyperlink.c
	����:		wj

	������:
		Bool C6678_Hypelink_Int_Trigger(Uint8 info)

	����˵��:
		�������Ƿ�hyperlink�ж�

	����˵��:
		info:infoֵ(0��31)

	����ֵ:
		true:�ɹ�
		false��ʧ��

	��ע:Hyperlink�жϷ����ٶȽϿ죬��η���Hyperlink�ж�ʱ��
		Ӧȷ���ϴ��ж��Ѿ���Ӧ��ϣ����������ж϶�ʧ����
*********************************************************************/
Bool C6678_Hypelink_Int_Trigger(Uint8 info);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����cache.c�к���������						
*********************************************************************/

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:void C6678_Cache_Enable(Uint8 RegionNum)

	����˵��:��������ʹ��ָ����ַ�ε�cacheԤȡ����

	����˵��:
	  RegionNum����ַ����ţ�����ɲμ��ֲᡣ
	  
	����ֵ:��

	��ע: ��
*********************************************************************/
void C6678_Cache_Enable(Uint8 RegionNum);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:void C6678_Cache_Disable(Uint8 RegionNum)

	����˵��:��������ʧЧָ����ַ�ε�cacheԤȡ����

	����˵��:
	  RegionNum����ַ����ţ�����ɲμ��ֲᡣ
	  
	����ֵ:��

	��ע: ��
*********************************************************************/
void C6678_Cache_Disable(Uint8 RegionNum);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:Bool C6678_Cache_Inv(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod)

	����˵��:�������ǽ�ָ����ַ�ε�cacheԤȡֵʧЧ��

	����˵��:     
		BlockPtr����ַ�������ʼ��ַָ��
    ByteCnt����ַ����Ĵ�С����λΪ�ֽ�,����Ϊ0��
	  Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      CACHE_NOWAIT:���ȴ��������
 

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: �˲����󣬺��ٶ��öε�ַ��������ʱcache��ʧ��
*********************************************************************/
Bool C6678_Cache_Inv(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:Bool C6678_Cache_Wb(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod)

	����˵��:�ú����ǽ�ָ����ַ������ ��cache��������д�ظöε�ַ����

	����˵��:
	  Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      CACHE_NOWAIT:���ȴ��������
      BlockPtr����ַ�������ʼֵ
      ByteCnt����ַ����Ĵ�С����λΪ�ֽ�,����Ϊ0��

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Cache_Wb(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:Bool C6678_Cache_Freeze(Uint8 MemType)

	����˵��:��������cache�������������ñ������󣬵�ǰ�ö�cache������ݽ�����
	                      �������ȥ��

	����˵��:
      MemType: 0=L1D cache,1=L1P cache ,2=L2 cache

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Cache_Freeze(Uint8 MemType);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:Bool C6678_Cache_Unfreeze(Uint8 MemType)

	����˵��:��������cache��������

	����˵��:
     MemType: 0=L1D cache,1=L1P cache ,2=L2 cache

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: ��
*********************************************************************/
Bool C6678_Cache_Unfreeze(Uint8 MemType);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:Bool C6678_Cache_Set(Uint8 MemType,Uint8 size)

	����˵��:������������ L1D/L1P/L2 cache�Ĵ�С

	����˵��:
	MemType: 0=L1D cache,1=L1P cache ,2=L2 cache
	size:L1P/L1D cache �Ĵ�СΪ0~32K
		 L1 cache size:
		 0= L1_0KCACHE ,
		 1= L1_4KCACHE ,
		 2= L1_8KCACHE ,
		 3= L1_16KCACHE,
		 4= L1_32KCACHE,
         L2 cache �Ĵ�СΪ0~512K
		 L2 Cache size:
		 0=L2_0KCACHE  ,
		 1=L2_32KCACHE ,
		 2=L2_64KCACHE ,
		 3=L2_128KCACHE,
		 4=L2_256KCACHE,
		 5=L2_512KCACHE,
		 6=L2_1024KCACHE.
	����ֵ:��

	��ע: ��
*********************************************************************/
Bool C6678_Cache_Set(Uint8 MemType,Uint8 size);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:void C6678_Cache_Wb_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_Wb(...,...,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_Wb(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_Wb_Wait(void);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    dyx

	������:void C6678_Cache_Inv_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_Inv(...,..,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_Inv(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_Inv_Wait(void);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbInv(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod)

	����˵��:�������ǽ�ָ����ַ�ε�cache����д�ز���ԤȡֵʧЧ��

	����˵��:
      BlockPtr����ַ�������ʼ��ַָ��
      ByteCnt����ַ����Ĵ�С����λΪ�ֽ�,����Ϊ0��
      Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      	  CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      	  CACHE_NOWAIT:���ȴ��������

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: �˲����󣬺��ٶ��öε�ַ��������ʱcache��ʧ��
*********************************************************************/
Bool C6678_Cache_WbInv(Uint32 *BlockPtr,Uint32 ByteCnt,Uint8 Mod);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbInv_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_WbInv(...,..,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_WbInv(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_WbInv_Wait(void);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_InvAllL1d(Uint8 Mod)

	����˵��:�������ǽ�L1D������cache��ԤȡֵʧЧ��

	����˵��:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      	  CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      	  CACHE_NOWAIT:���ȴ��������

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: �˲����󣬺��ٶ��öε�ַ��������ʱcache��ʧ��
*********************************************************************/
Bool C6678_Cache_InvAllL1d(Uint8 Mod);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_InvAllL1d_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_InvAllL1d(...,..,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_InvAllL1d(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_InvAllL1d_Wait(void);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbAllL1d(Uint8 Mod)

	����˵��:�������ǽ�L1D�����б�cache��������д�ظöε�ַ����

	����˵��:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      	  CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      	  CACHE_NOWAIT:���ȴ��������

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: �ޡ�
*********************************************************************/
Bool C6678_Cache_WbAllL1d(Uint8 Mod);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbAllL1d_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_WbAllL1d(...,..,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_WbAllL1d(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_WbAllL1d_Wait(void);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbInvAllL1d(Uint8 Mod)

	����˵��:�������ǽ�L1D�����б�cache��������д�ظöε�ַ���򲢽�cache��ԤȡֵʧЧ����

	����˵��:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      	  CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      	  CACHE_NOWAIT:���ȴ��������

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: �˲����󣬺��ٶ��öε�ַ��������ʱcache��ʧ��
*********************************************************************/
Bool C6678_Cache_WbInvAllL1d(Uint8 Mod);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbInvAllL1d_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_WbInvAllL1d(...,..,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_WbInvAllL1d(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_WbInvAllL1d_Wait(void);

/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_InvAllL2(Uint8 Mod)

	����˵��:�������ǽ�L2������cache��ԤȡֵʧЧ��

	����˵��:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      	  CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      	  CACHE_NOWAIT:���ȴ��������

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: �˲����󣬺��ٶ��öε�ַ��������ʱcache��ʧ��
*********************************************************************/
Bool C6678_Cache_InvAllL2(Uint8 Mod);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_InvAllL2_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_InvAllL2(...,..,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_InvAllL2(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_InvAllL2_Wait(void);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbAllL2(Uint8 Mod)

	����˵��:�������ǽ�L2�����б�cache��������д�ظöε�ַ����

	����˵��:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      	  CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      	  CACHE_NOWAIT:���ȴ��������

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: �ޡ�
*********************************************************************/
Bool C6678_Cache_WbAllL2(Uint8 Mod);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbAllL2_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_WbAllL2(...,..,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_WbAllL2(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_WbAllL2_Wait(void);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbInvAllL2(Uint8 Mod)

	����˵��:�������ǽ�L2�����б�cache��������д�ظöε�ַ���򲢽�cache��ԤȡֵʧЧ����

	����˵��:
		Mod: =0 :CACHE_NOWAIT,=1:CACHE_WAIT,=2:CACHE_FENCE_WAIT
	  	  CACHE_FENCE_WAIT: ����MFENCEָ��ȴ�������ɡ�
      	  CACHE_WAIT:�ȴ�cache��״̬�Ĵ���ָʾ������ɡ�
      	  CACHE_NOWAIT:���ȴ��������

	����ֵ:
		true:����
	  	false��ʧ��

	��ע: �˲����󣬺��ٶ��öε�ַ��������ʱcache��ʧ��
*********************************************************************/
Bool C6678_Cache_WbInvAllL2(Uint8 Mod);
/********************************************************************
	��������

	�����ļ�:   cache.c
	����:	    wj

	������:C6678_Cache_WbInvAllL2_Wait(void)

	����˵��:�������ǵȴ�C6678_Cache_WbInvAllL2(...,..,CACHE_NOWAIT)��������ĺ���

	����˵��:��

	����ֵ:��

	��ע: �ú�������C6678_Cache_WbInvAllL2(...,..,CACHE_NOWAIT)֮����ã��ȴ���
	      cache�����������
*********************************************************************/
void C6678_Cache_WbInvAllL2_Wait(void);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����qmss.c�к���������
						qmss����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   qmss.c
	����:		wj

	������:
	  Bool C6678_Qmss_Init(void);
	����˵��:
	  �������ǳ�ʼ��qmss
	����˵��:
	  ��
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ͳһ�ɺ�0����
*********************************************************************/
Bool C6678_Qmss_Init(void);

/********************************************************************
	��������

	�����ļ�:   qmss.c
	����:		wj

	������:
	  Bool C6678_QMSS_Core_MemInsert(Uint8 coreNum);
	����˵��:
	  �������Ǹ�QMSS�����ڴ�ռ�
	����˵��:
	  coreNum:�˵����
	����ֵ:
	  true:��ʼ������
	  false����ʼ��ʧ��
	��ע: ÿ���˶�����ã�����֮ǰ����� C6678_QMSS_Init();
*********************************************************************/
Bool C6678_QMSS_Core_MemInsert(Uint8 coreNum);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����cppi_drv.c�к���������
						cppi����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   cppi_drv.c
	����:		wj

	������:
		Bool C6678_Cppi_init(void);

	����˵��:
		�������ǳ�ʼ��cppi

	����˵��:
		��

	����ֵ:
		true:��ʼ������
		false����ʼ��ʧ��

	��ע: ͳһ�ɺ�0����
*********************************************************************/
Bool C6678_Cppi_init (void);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����ipc_navigator.c�к���������
						IPC����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   ipc_navigator.c
	����:	    dyx

	������:Bool C6678_IPC_Navigator_coreInit(Uint8 coreNum)

	����˵��: �ú�����IPC�����ڶ�˵����ĺ˼�ͨ�ţ���ʼ������

	����˵��:
       coreNum:���غ�ID

	����ֵ:
	  TRUE:��ʼ���ɹ�
	  FALSE:��ʼ��ʧ��
	��ע: �ú��� �ɲ���˼�ͨ�ŵĸ��˷ֱ����,��֤���˶���ʼ���������ٽ��ж��֮���ͨ��
*********************************************************************/
Bool C6678_IPC_Navigator_coreInit(Uint8 coreNum);

/********************************************************************
	��������

	�����ļ�:   ipc_navigator.c
	����:	    dyx

	������:void C6678_Flag_Multicore_Syn(Uint8 coreNum,Bool coreChoosed[])

	����˵��: �ú�����ʵ�ֶ��ͬ���ĺ���

	����˵��:
	   coreNum:���غ�ID
       coreChoosed[0]:
               TRUE:0����Ҫͬ��
               FALSE:0�˲���Ҫͬ��
       .....

 	   coreChoosed[7]:
               TRUE:7����Ҫͬ��
               FALSE:7�˲���Ҫͬ��
	����ֵ:
	  ��
	��ע: �ú��� ����Ҫͬ���ĺ˵���,��Ҫͬ���ĺ˵�coreChoosed����Ӧ��һ��
	               ע�⣺���ڱ������е�flagʹ�õ���һ�ף����Ե�һ�ε��øú�����
	               �豣֤���е��øú����ĺ˶�ͬ���������ٵڶ��ε��øú�����
	               �������ڵڶ���Ҳʹ����ͬ��flag�����ܻ�͵�һ���໥Ӱ�졣
*********************************************************************/
void C6678_Flag_Multicore_Syn(Uint8 coreNum,Bool coreChoosed[]);

/********************************************************************
	��������

	�����ļ�:   ipc_navigator.c
	����:	    dyx

	������:Bool C6678_IPC_Navigator_send(Uint8 destcoreNum,Uint8 *databuf,Uint32 databufLen)

	����˵��: �ú���ʵ��ͨ��IPC��ָ���˷�������

	����˵��:
       destcoreNum:Ŀ�ĺ�ID
       databuf:�������ݻ�����
       databufLen���������ݳ���

	����ֵ:
	  TRUE:���ͳɹ�
	  FALSE:����ʧ��
	��ע: �ú�����C6678_IPC_Navigator_coreInit����֮�����
*********************************************************************/
Bool C6678_IPC_Navigator_send(Uint8 destcoreNum,Uint8 *databuf,Uint32 databufLen);
/********************************************************************
	��������

	�����ļ�:  ipc_navigator.c
	����:	    dyx

	������:void C6678_IPC_Intc_send(Uint8 destcoreNum,Uint8 info)

	����˵��: �ú���ʵ��ͨ��IPC��ָ���˷���ͨ����Ϣ

	����˵��:
       destcoreNum:Ŀ�ĺ�ID
       info:IPCͨ����Ϣ��ȡֵΪ0-27

	����ֵ:
	  ��
	��ע:��ĳ���˵��øú�����������������˷�����Ϣʱ�����ܻ�����������ظ���
	            ������Ϣ�������
    	����������ڸú˸���һ���˷�����Ϣǰ����ȷ����ǰĿ������յ�����Ϣ��
    	���������ú˸���һ���˷�����Ϣ��
*********************************************************************/
void C6678_IPC_Intc_send(Uint8 destcoreNum,Uint32 info);


/********************************************************************
	��������

	�����ļ�:   ipc_navigator.c
	����:	    dyx

	������:Uint32 C6678_IPC_Intc_Rev(void)

	����˵��: �ú���ʵ�ּ��IPCͨ����Ϣ

	����˵��:
      	  ��

	����ֵ:
	   info:����IPC������Ϣλ,ȡֵΪ2��0����---2��27���ݡ�
	                                    ����                                                  ��            ����                           �Ķ�Ӧ��ϵ
			C6678_IPC_Intc_send(,0)           1=C6678_IPC_Intc_Rev()
			C6678_IPC_Intc_send(,1)           2=C6678_IPC_Intc_Rev()
			C6678_IPC_Intc_send(,2)           4=C6678_IPC_Intc_Rev()
			������������
			C6678_IPC_Intc_send(,27)          0x8000000=C6678_IPC_Intc_Rev()
	��ע:���øú�����info�ᱻ���
*********************************************************************/
Uint32 C6678_IPC_Intc_Rev(void);

/********************************************************************
	��������

	�����ļ�:   ipc_navigator.c
	����:	    dyx

	������:Bool C6678_IPC_Intc_HookUp(void interruptISR(),Uint8 VectID)

	����˵��: �ú�����ʵ��IPC�жϹ���

	����˵��:
	  interruptISR()���жϷ������
      VectID:�ж������ţ�4-15��

	����ֵ:
	   TRUE:����ɹ�
	   FALSE������ʧ��

	��ע:���ô˺���ǰ���ȵ��� C6678_CoreInt_Init ()�����˾��ɵ���
*********************************************************************/
Bool C6678_IPC_Intc_HookUp(void interruptISR(),Uint8 VectID);

/********************************************************************
	��������

	�����ļ�:   ipc_navigator.c
	����:	    dyx

	������:void C6678_IPC_Intc_Clear(void)

	����˵��: �ú��������IPC�ж�״̬

	����˵��:
	      ��

	����ֵ:
	   ��

	��ע:���˾��ɵ���
*********************************************************************/
void C6678_IPC_Intc_Clear(void);
/********************************************************************
	��������

	�����ļ�:   ipc_navigator.c
	����:	    dyx

	������:Uint32 C6678_IPC_Navigator_recv(Uint8 *buffDataPtr,Uint8 mode)

	����˵��: ������IPC���������ݵ�ָ�����ջ�����

	����˵��:
       buffDataPtr���������ݻ�����
	   mode:
	       0:wait�ȴ�ֱ���յ���������һֱ����
	       other�����ȴ���ֻ�ж��Դ��Ƿ��յ���������
	����ֵ:
	      ���ؽ��յ����ݳ���

	��ע: �ú�����C6678_IPC_Navigator_coreInit����֮���ɸ��˷ֱ����
*********************************************************************/
Uint32 C6678_IPC_Navigator_recv(Uint8 *buffDataPtr,Uint8 mode);

/********************************************************************
	��������

	�����ļ�:   ipc_navigator.c
	����:	    dyx

	������:Bool C6678_IPC_Navigator_coreClose(void)

	����˵��: �ر����е�IPC��Դ

	����˵��:
                  ��

	����ֵ:
	   TRUE:�رճɹ�
	   FALSE���ر�ʧ��

	��ע: �ú�����IPC��������ɸ��˷ֱ����
*********************************************************************/
Bool C6678_IPC_Navigator_coreClose(void);
/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����emif16_norflash.c�к���������
						EMIF_NORFLASH����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   emif16_norflash.c
	����:		wj

	������:
	  Bool C6678_Emif16_Norflash_Init(unsigned int MainRate)
	����˵��:
	  �������ǳ�ʼ��emif16_norflash
	����˵��:
	  MainRate:CPU��Ƶ
	  	PLATFORM_PLL1_PLLM_val(��c6678.h)
	����ֵ:
	   TRUE:�ɹ�
	   FALSE��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Emif16_Norflash_Init(unsigned int MainRate);
/********************************************************************
	��������

	�����ļ�:   emif16_norflash.c
	����:		wj

	������:
	 	 Bool C6678_Emif16_Norflash_ReadByte(Uint32 src, Uint32 dst, Uint32 length)
	����˵��:
	  �������ǽ����ݴ�nor_flash����
	����˵��:
	  src��  nor_flash�ĵ�ַ
	  dst�� Ŀ�ĵ�ַ
	  length�����������ݵĳ���(��λΪ16bit)
	����ֵ:
	   TRUE:�ɹ�
	   FALSE��ʧ��
	��ע: ��
*********************************************************************/
Bool C6678_Emif16_Norflash_ReadByte(Uint32 src, Uint32 dst, Uint32 length);
/********************************************************************
	��������

	�����ļ�:   emif16_norflash.c
	����:		wj

	������:
	 	 Bool C6678_Emif16_Norflash_WriteByte(Uint32 src, Uint32 dst, Uint32 length)

	����˵��:
	  �������ǽ�����д��nor_flash
	����˵��:
	  src��  Դ��ַ
	  dst�� nor_flash�ĵ�ַ
	  length��д������ݵĳ���(��λΪ16bit)
	����ֵ:
	   TRUE:�ɹ�
	   FALSE��ʧ��
	��ע: д֮ǰҪ�ȵ��ò��������������
*********************************************************************/
Bool C6678_Emif16_Norflash_WriteByte(Uint32 src, Uint32 dst, Uint32 length);

/********************************************************************
	��������

	�����ļ�:   emif16_norflash.c
	����:		wj

	������:
	  Bool C6678_Emif16_Norflash_BlockErase(Uint32 start, Uint32 length);
	����˵��:
	  �������ǽ�nor_flash��ָ����ַ�ռ����
	����˵��:
	  start��  �����ĵ�ַ�ռ����ʼ��ַ
	  length�� �����ĵ�ַ�ռ�ĳ���
	����ֵ:
	  true�������ɹ�
	  false������ʧ��
	��ע: ��
*********************************************************************/
/* Erase a segment of Flash memory */
Bool C6678_Emif16_Norflash_BlockErase(Uint32 start, Uint32 length);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����i2c_srioswitch.c�к���������
						I2C_SRIOSWITCH����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   i2c_srioswitch.c
	����:		wj

	������:
	  void C6678_I2c_SrioSwitch_Init(void)
	����˵��:
	  �������ǳ�ʼ��i2c_srioswitch
	����˵��:
	  ��
	����ֵ:
	  ��
	��ע: ��
*********************************************************************/
void C6678_I2c_SrioSwitch_Init(void);
/********************************************************************
	��������

	�����ļ�:   i2c_srioswitch.c
	����:		wj

	������:
	  I2C_RET C6678_I2c_SrioSwitch_Read ( uint32_t byte_addr,uint32_t *puiData, uint8_t uchEepromI2cAddress)
	����˵��:
	  �������Ǵ�srioswitch�ж���һ������������
	����˵��:
	  uchEepromI2cAddress ��srioswitch��i2c��ַ
	  puiData���������ݴ洢�������ָ��
	  byte_addr��������EEPROM�Ļ���ַ��byteΪ��λ��
	����ֵ:
		0��I2C_RET_OK
		1��I2C_RET_LOST_ARB
		2��I2C_RET_NO_ACK
		3��I2C_RET_IDLE_TIMEOUT
		4��I2C_RET_BAD_REQUEST
		5��I2C_RET_CLOCK_STUCK_LOW
		6��I2C_RET_NULL_PTR_ERROR
		99��I2C_RET_GEN_ERROR
	��ע:
*********************************************************************/
I2C_RET C6678_I2c_SrioSwitch_Read ( uint32_t byte_addr,uint32_t *puiData, uint8_t uchEepromI2cAddress);
/********************************************************************
	��������

	�����ļ�:   i2c_srioswitch.c
	����:		wj

	������:
	  I2C_RET C6678_I2c_SrioSwitch_Write( uint32_t byte_addr, uint8_t uchEepromI2cAddress, uint32_t puiData)
	����˵��:
	  ����������SrioSwitch��д��һ������������
	����˵��:
	  byte_addr:������SrioSwitch�Ļ���ַ��byteΪ��λ��
	  uchEepromI2cAddress ��SrioSwitch��i2c��ַ
	  puiData���������ݴ洢�������ָ��
	����ֵ:
		0��I2C_RET_OK
		1��I2C_RET_LOST_ARB
		2��I2C_RET_NO_ACK
		3��I2C_RET_IDLE_TIMEOUT
		4��I2C_RET_BAD_REQUEST
		5��I2C_RET_CLOCK_STUCK_LOW
		6��I2C_RET_NULL_PTR_ERROR
		99��I2C_RET_GEN_ERROR
	��ע:The write consists of a master write of 2 bytes (forming a 16 bit address, msb transmitted first), followed by a master write of the
input number of bytes.The bytes that are write are placed in puiData in big endian format
*********************************************************************/
I2C_RET C6678_I2c_SrioSwitch_Write( uint32_t byte_addr, uint8_t uchEepromI2cAddress, uint32_t puiData);

/********************************************************************
	��������

	�����ļ�:   i2c_srioswitch.c
	����:		wj

	������:
	  I2C_RET C6678_I2c_SrioSwitch_Config ( uint8_t uchEepromI2cAddress,uint32_t *puiData)
	����˵��:
	  ��������ͨ��һ������switch����������ļ�����switch��������
	����˵��:
	  uchEepromI2cAddress ��srioswitch��i2c��ַ
	  puiData������switch������
	����ֵ:
		0��I2C_RET_OK
		1��I2C_RET_LOST_ARB
		2��I2C_RET_NO_ACK
		3��I2C_RET_IDLE_TIMEOUT
		4��I2C_RET_BAD_REQUEST
		5��I2C_RET_CLOCK_STUCK_LOW
		6��I2C_RET_NULL_PTR_ERROR
		99��I2C_RET_GEN_ERROR
	��ע:
*********************************************************************/
I2C_RET C6678_I2c_SrioSwitch_Config ( uint8_t uchEepromI2cAddress,uint32_t *puiData);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����i2c_temp.c�к���������
						I2C_TEMP����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   i2c_Temp.c
	����:		wj

	������:
	  void C6678_I2c_Temp_Init(void)
	����˵��:
	  �������ǳ�ʼ��i2c_temp
	����˵��:
	  ��
	����ֵ:
	  ��
	��ע: ��
*********************************************************************/
void C6678_I2c_Temp_Init(void);

/********************************************************************
	��������

	�����ļ�:   i2c_temp.c
	����:		wj

	������:
	  Bool C6678_I2c_Temp_Read (uint8_t TempNum,int32_t *TempValue)
	����˵��:
	  �������Ǵ��¸��ж����¶ȡ�
	����˵��:
	  TempNum���ڼ����¸У�0~2��
	  	  0��I2C Slave address is 0x48
	  	  1��I2C Slave address is 0x49
	  	  2��I2C Slave address is 0x4A
	  TempValue�����ص��¶�ָ�루��λΪ0.001�ȣ�
	����ֵ:
	   TRUE:�ɹ�
	   FALSE��ʧ��
	��ע:
*********************************************************************/
Bool C6678_I2c_Temp_Read (uint8_t TempNum,int32_t *TempValue);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����i2c_voltage.c�к���������
						I2C_Voltage����ز���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   i2c_voltage.c
	����:		qqb

	������:
	  void C6678_I2c_Voltage_Init(void)
	����˵��:
	  �������ǳ�ʼ��i2c_voltage
	����˵��:
	  ��
	����ֵ:
	  ��
	��ע: ��
*********************************************************************/
void C6678_I2c_Voltage_Init(void);

/********************************************************************
	��������

	�����ļ�:   	i2c_voltage.c
	����:		qqb

	������:
		Bool C6678_I2c_Voltage_Read (uint8_t ChanNum,float *VoltValue)
	����˵��:
		�������Ǵӵ�ѹ�������ж�����ѹֵ��
	����˵��:
		VoltageRegAdr����ѹ��������ַ
		VoltValue�����صĵ�ѹֵָ�루��λΪ:����
	����ֵ:
		TRUE:�ɹ�
		FALSE��ʧ��
	��ע:   ��
*********************************************************************/
Bool C6678_I2c_Voltage_Read (uint8_t VoltageRegAdr,float *VoltValue);

/********************************************************************
	�ļ�˵��:	�����ⲿ�ֺ�����ddr.c�к���������
				DDR3����ز��������û�ʹ�ú���
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   ddr.c
	����:		qqb

	������:
	  	  Bool C6678_Ddr3_lib_Init(int Ddr3Pllm,int Ddr3Type);
	����˵��:
	  	  ���������������ڲ��������������û�ʹ�ú���
	����˵��:
		  Ddr3Pllm:
			  PLLM_DDR3(����ֵ��c6678.h)
		  Ddr3Type:
			  DDR3_TYPE(����ֵ��c6678.h)
	����ֵ:
		  true:����
		  false��ʧ��
	��ע: �˺���Ϊ�������ڲ�ʵ����Ҫ�����û�ʹ�ú���
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
	#elif(DDR3_TYPE_2BANK_8Gb_8P_1600_3U == DDR3_TYPE)		//3U 8Gb����忨
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
	�ļ�˵��:	�����ⲿ�ֺ�����uart.c�к���������
				UART����ز������û�ʹ�ú�������
*********************************************************************/
/********************************************************************
	��������

	�����ļ�:   uart.c
	����:		qqb

	������:
	  	  int32_t c6678_uart_init(uint32_t baudrate)
	����˵��:
	  	  �������ǳ�ʼ��UART����
	����˵��:
		  baudrate:
			  UART�Ĳ�����
	����ֵ:
		  true:����
		  false��ʧ��
	��ע:
*********************************************************************/
int32_t c6678_uart_init(uint32_t baudrate);

/********************************************************************
	��������

	�����ļ�:   uart.c
	����:		qqb

	������:
	  	  int32_t c6678_uart_read(uint8_t *buf, uint32_t delay)
	����˵��:
	  	  ��������UART��ȡ���ݺ���
	����˵��:
		  buf:
			  UART�Ķ�ȡ���ݻ�����
		delay:
			     ��ȡ���ݳ�ʱʱ��(��λ��s)
	����ֵ:
		  true:����
		  false��ʧ��
	��ע:
*********************************************************************/
int32_t c6678_uart_read(uint8_t *buf, uint32_t delay);

/********************************************************************
	��������

	�����ļ�:   uart.c
	����:		qqb

	������:
	  	  int32_t c6678_uart_write(uint8_t buf)
	����˵��:
	  	  ��������UART��ȡ���ݺ���
	����˵��:
		  buf:
			  UART��д���ݵ�ַ
	����ֵ:
		  true:����
		  false��ʧ��
	��ע:
*********************************************************************/
int32_t c6678_uart_write(uint8_t buf);

/********************************************************************
	��������

	�����ļ�:   BigLittleSwap.c
	����:		qqb

	������:
		int i_CheckCpuBigEndian(void)

	����˵��:
		�������Ǽ��CPU�Ĵ�С��

	����˵��:
     	 	 ��

	����ֵ:
		true:���CPU
		false��С��CPU

	��ע: ��
*********************************************************************/
int i_CheckCpuBigEndian(void);

/********************************************************************
	��������

	�����ļ�:   BigLittleSwap.c
	����:		qqb

	������:
		unsigned int t_LittleToBigl(unsigned int data32)

	����˵��:
		�������ǽ�С�˵�32λ����ת��Ϊ���

	����˵��:
     	 data32����ת��������

	����ֵ:
		ת��Ϊ��˵�����

	��ע: ��
*********************************************************************/
unsigned int t_LittleToBigl(unsigned int data32);

/********************************************************************
	��������

	�����ļ�:   BigLittleSwap.c
	����:		qqb

	������:
		unsigned int t_BigToLittlel(unsigned int data32)

	����˵��:
		�������ǽ���˵�32λ����ת��ΪС��

	����˵��:
     	 data32����ת��������

	����ֵ:
		ת��ΪС�˵�����

	��ע: ��
*********************************************************************/
unsigned int t_BigToLittlel(unsigned int data32);

/********************************************************************
	��������

	�����ļ�:   BigLittleSwap.c
	����:		qqb

	������:
		unsigned short t_LittleToBig(unsigned short data16)

	����˵��:
		�������ǽ�С�˵�16λ����ת��Ϊ���

	����˵��:
     	 data16����ת��������

	����ֵ:
		ת��Ϊ��˵�����

	��ע: ��
*********************************************************************/
unsigned short t_LittleToBig(unsigned short data16);

/********************************************************************
	��������

	�����ļ�:   BigLittleSwap.c
	����:		qqb

	������:
		unsigned short t_BigToLittle(unsigned short data16)

	����˵��:
		�������ǽ���˵�16λ����ת��ΪС��

	����˵��:
     	 data16����ת��������

	����ֵ:
		ת��ΪС�˵�����

	��ע: ��
*********************************************************************/
unsigned short t_BigToLittle(unsigned short data16);

#endif /* C6678_H_ */
