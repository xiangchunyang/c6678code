/*
 * spi_norflash.c
 *
 *  Created on: 2012-4-1
 *      Author: user
 *      Description: This file implements NOR flash driver for Nymonyx N25Q128 NOR flash
 */
#include"c6678.h"
/********************************************************************
 * 内部所需全局变量
 *******************************************************************/
static uint32_t data1_reg_val;
/* ------------------------------------------------------------------------ *
 *  SPI NOR Definitions                                                     *
 * ------------------------------------------------------------------------ */
#define SPI_NOR_MANUFACTURE_ID     0x20     /* Numonyx N25Q128 Manufacture ID assigned by JEDEC */
#define SPI_NOR_SECTOR_COUNT       256      /* Total number of data sectors on the NOR */
#define SPI_NOR_SECTOR_SIZE        65536    /* Number of bytes in a data sector */
#define SPI_NOR_BOOT_SECTOR_COUNT  8        /* Total number of boot data sectors on the NOR */
#define SPI_NOR_PAGE_COUNT         65536    /* Total number of data pages on the NOR */
#define SPI_NOR_PAGE_SIZE          256      /* Number of data pages in a data sector */
#define SPI_NOR_OTP_BYTES          64       /* Number of OTP data bytes */
#define SPI_NOR_MAX_FLASH_SIZE     (SPI_NOR_SECTOR_COUNT*SPI_NOR_SECTOR_SIZE) /* Total device size in Bytes 16Mbytes */

/* ------------------------------------------------------------------------ *
 *  SPI NOR Commands                                                        *
 * ------------------------------------------------------------------------ */
#define SPI_NOR_CMD_RDID           0x9f     /* Read manufacture/device ID */
#define SPI_NOR_CMD_WREN           0x06     /* Write enable */
#define SPI_NOR_CMD_WRDI           0x04     /* Write Disable */
#define SPI_NOR_CMD_RDSR           0x05     /* Read Status Register */
#define SPI_NOR_CMD_WRSR           0x01     /* Write Status Register */
#define SPI_NOR_CMD_READ           0x03     /* Read data */
#define SPI_NOR_CMD_FAST_READ      0x0B     /* Read data bytes at higher speed */
#define SPI_NOR_CMD_PP             0x02     /* Page Program */
#define SPI_NOR_CMD_SSE            0x20     /* Sub Sector Erase */
#define SPI_NOR_CMD_SE             0xd8     /* Sector Erase */
#define SPI_NOR_CMD_BE             0xc7     /* Bulk Erase */

#define SPI_NOR_SR_WIP             (1 << 0)   /* Status Register, Write-in-Progress bit */
#define SPI_NOR_BE_SECTOR_NUM      ((Uint32)-1) /* Sector number set for bulk erase */

/* Read status Write In Progress timeout */
#define SPI_NOR_PROG_TIMEOUT          100000
#define SPI_NOR_PAGE_ERASE_TIMEOUT    500000
#define SPI_NOR_SECTOR_ERASE_TIMEOUT  100000000



/* ------------------------------------------------------------------------ *
 *  SPI Controller                                                          *
 * ------------------------------------------------------------------------ */
#define SPI_BASE                CSL_SPI_REGS
#define SPI_SPIGCR0             *( volatile Uint32* )( SPI_BASE + 0x0 )
#define SPI_SPIGCR1             *( volatile Uint32* )( SPI_BASE + 0x4 )
#define SPI_SPIINT0             *( volatile Uint32* )( SPI_BASE + 0x8 )
#define SPI_SPILVL              *( volatile Uint32* )( SPI_BASE + 0xc )
#define SPI_SPIFLG              *( volatile Uint32* )( SPI_BASE + 0x10 )
#define SPI_SPIPC0              *( volatile Uint32* )( SPI_BASE + 0x14 )
#define SPI_SPIDAT0             *( volatile Uint32* )( SPI_BASE + 0x38 )
#define SPI_SPIDAT1             *( volatile Uint32* )( SPI_BASE + 0x3c )
#define SPI_SPIBUF              *( volatile Uint32* )( SPI_BASE + 0x40 )
#define SPI_SPIEMU              *( volatile Uint32* )( SPI_BASE + 0x44 )
#define SPI_SPIDELAY            *( volatile Uint32* )( SPI_BASE + 0x48 )
#define SPI_SPIDEF              *( volatile Uint32* )( SPI_BASE + 0x4c )
#define SPI_SPIFMT0             *( volatile Uint32* )( SPI_BASE + 0x50 )
#define SPI_SPIFMT1             *( volatile Uint32* )( SPI_BASE + 0x54 )
#define SPI_SPIFMT2             *( volatile Uint32* )( SPI_BASE + 0x58 )
#define SPI_SPIFMT3             *( volatile Uint32* )( SPI_BASE + 0x5c )
#define SPI_INTVEC0             *( volatile Uint32* )( SPI_BASE + 0x60 )
#define SPI_INTVEC1             *( volatile Uint32* )( SPI_BASE + 0x64 )

#define SPI_NOR_CS              0           /* SPI Chip Select number for NOR*/
#define SPI_FPGA_CS             1           /* SPI Chip Select number for FPGA*/
#define SPI_MODULE_CLK          200000000   /* SYSCLK7  = CPU_Clk/6 in HZ */
#define SPI_MAX_FREQ            25000000    /* SPI Max frequency in Hz */
#define SPI_NOR_CHAR_LENTH      8           /* Number of bits per SPI trasfered data element for NOR flash */
#define SPI_FPGA_CHAR_LENTH     16          /* Number of bits per SPI trasfered data element for FPGA */


/* SPI error status */
#define SPI_STATUS        Uint32           /* SPI error status type */
#define SPI_EFAIL         ((SPI_STATUS)-1)   /* General failure code */
#define SPI_EOK           0                /* General success code */

/********************************************************************
 * 内部所需结构体
 *******************************************************************/

/********************************************************************
 * 内部所需函数
 *******************************************************************/
static NOR_STATUS nor_wait_ready(uint32_t timeout);
static void spinorflash_delay(uint32_t delay);
SPI_STATUS spinorflash_cmd_read(uint8_t* cmd,uint32_t cmd_len,uint8_t* data,uint32_t data_len);
SPI_STATUS spinorflash_cmd_write(uint8_t* cmd,uint32_t cmd_len,uint8_t* data,uint32_t data_len);
SPI_STATUS spinorflash_read_word(uint16_t* cmd_buf,uint32_t cmd_len,uint16_t* data_buf,uint32_t data_len);
SPI_STATUS spinorflash_write_word(uint16_t* cmd_buf,uint32_t cmd_len,uint16_t* data_buf,uint32_t data_len);
void Osal_platformSpinorflashCsEnter(void);
void Osal_platformSpinorflashCsExit (void);
SPI_STATUS spinorflash_claim(uint32_t cs,uint32_t freq);
SPI_STATUS spinorflash_xfer(uint32_t nbytes,uint8_t* data_out,uint8_t* data_in,Bool terminate);
SPI_STATUS spinorflash_cmd(uint8_t cmd,uint8_t* response,uint32_t len);
void spinorflash_release(void);
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
NOR_STATUS
C6678_Spi_Norflash_Init
(
    void
)
{
    NOR_STATUS          ret;
    uint8_t               idcode[3]={0};     /* Initialize the SPI interface */

    /* Get the hardware semaphore.
     *
     * Acquire Multi core  synchronization lock
     */
    while ((CSL_semAcquireDirect (SPI_SW_SEM)) == 0)
    {
    }

    /* Claim the SPI controller */
    spinorflash_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /* Read the ID codes */
    ret = spinorflash_cmd(SPI_NOR_CMD_RDID, idcode, sizeof(idcode));
    if (0!=ret)
    {
        //IFPRINT (platform_write( "nor_init: Error in reading the idcode\n"));
        spinorflash_release();
        //platform_errno = PLATFORM_ERRNO_NOR;
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return NOR_EFAIL;
    }

    //IFPRINT (platform_write("SF: Got idcode %02x %02x %02x\n", idcode[0], idcode[1], idcode[2]));

    if (idcode[0] != SPI_NOR_MANUFACTURE_ID) {
        /* Expected Manufacturer ID does not match */
        spinorflash_release();
        //platform_errno = PLATFORM_ERRNO_BADFLASHDEV;
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return NOR_EFAIL;
    }

    spinorflash_release();

    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (SPI_SW_SEM);

    return NOR_EOK;
}

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
uint32_t
C6678_Spi_Norflash_GetDetails
(
		RADAR_DEVICE_info*   nor_info
)
{
    uint32_t uiStatus = SUCCESS, ret;
    uint8_t               idcode[3]={0};     /* Initialize the SPI interface */

    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (SPI_SW_SEM)) == 0)
    {
    }

   /* Claim the SPI controller */
    spinorflash_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /* Read the ID codes */
    ret = spinorflash_cmd(SPI_NOR_CMD_RDID, idcode, sizeof(idcode));
    if (0!=ret) {
        spinorflash_release();
        //platform_errno = PLATFORM_ERRNO_NOR;
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return FAIL;
    }

    /* Get the actuals */
    nor_info->manufacturer_id   = idcode[0];
    nor_info->device_id         = (idcode[1] << SPI_NOR_CHAR_LENTH) | (idcode[2]);

    /* No blocks are bad for NOR.. for now */
    nor_info->bblist = NULL;

    spinorflash_release();

    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (SPI_SW_SEM);

    return uiStatus;
}

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
NOR_STATUS
C6678_Spi_Norflash_Read(
    uint32_t      addr,
    uint32_t      len,
    uint8_t*      buf
)
{
    uint8_t       cmd[4];
    NOR_STATUS  ret_val;

    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (SPI_SW_SEM)) == 0)
    {
    }

    /* Claim the SPI controller */
    spinorflash_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /* Validate address input */
    if((addr + len) > SPI_NOR_MAX_FLASH_SIZE)
    {
    	//platform_errno = PLATFORM_ERRNO_FLASHADDR;
        spinorflash_release();
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return NOR_EFAIL;
    }

    /* Initialize the command to be sent serially */
    cmd[0]              = SPI_NOR_CMD_READ;
    cmd[1]              = (uint8_t)(addr>>16);
    cmd[2]              = (uint8_t)(addr>>8);
    cmd[3]              = (uint8_t)addr;

    ret_val = (spinorflash_cmd_read(cmd, 4, buf, len));

    spinorflash_release();

    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (SPI_SW_SEM);

    return (ret_val);
}

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
	备注: 写之前要先调用擦函数，否则出错;norflash共16MB,注意不要访问越界;
*********************************************************************/
NOR_STATUS
C6678_Spi_Norflash_Write
(
    uint32_t      addr,
    uint32_t      len,
    uint8_t*      buf
)
{
    uint32_t      page_addr;
    uint32_t      byte_addr;
    uint32_t      page_size;
    uint32_t      loopCount;

    uint32_t      chunk_len;
    uint32_t      actual;
    uint32_t      ret;
    uint8_t       cmd[4];

    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (SPI_SW_SEM)) == 0)
    {
    }

    /* Claim the SPI controller */
    spinorflash_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /* Validate address input */
    if((addr + len) > SPI_NOR_MAX_FLASH_SIZE)
    {
    	//platform_errno = PLATFORM_ERRNO_NOFREEBLOCKS;
        spinorflash_release();
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return NOR_EFAIL;
    }

    page_size = SPI_NOR_PAGE_SIZE;
    page_addr = addr / page_size;
    byte_addr = addr & (SPI_NOR_PAGE_SIZE - 1); /* % page_size; */

    ret = NOR_EOK;
    for (actual = 0; actual < len; actual += chunk_len)
    {
        /* Send Write Enable command */
        ret = spinorflash_cmd(SPI_NOR_CMD_WREN, NULL, 0);
        if (0!=ret)
        {
        	//platform_errno = PLATFORM_ERRNO_DEV_FAIL;
            spinorflash_release();
            CSL_semReleaseSemaphore (SPI_SW_SEM);
            return NOR_EFAIL;
        }

        /* Send Page Program command */
        chunk_len = ((len - actual) < (page_size - byte_addr) ?
            (len - actual) : (page_size - byte_addr));

        cmd[0]  = SPI_NOR_CMD_PP;
        cmd[1]  = (uint8_t)(addr>>16);
        cmd[2]  = (uint8_t)(addr>>8);
        cmd[3]  = (uint8_t)addr;

        ret = spinorflash_cmd_write(cmd, (uint32_t)4, (uint8_t *)(buf + actual), chunk_len);
        if (0!=ret)
        {
        	//platform_errno = PLATFORM_ERRNO_DEV_FAIL;
            spinorflash_release();
            CSL_semReleaseSemaphore (SPI_SW_SEM);
            return NOR_EFAIL;
        }

        ret = nor_wait_ready(SPI_NOR_PROG_TIMEOUT);
        if (0!=ret)
        {
        	//platform_errno = PLATFORM_ERRNO_DEV_TIMEOUT;
            spinorflash_release();
            CSL_semReleaseSemaphore (SPI_SW_SEM);
            return NOR_EFAIL;
        }

        page_addr++;
        addr += chunk_len;
        byte_addr = 0;

        loopCount = 4000;
        while (0!=(loopCount--)) {
            asm("   NOP");
        }

    }

    spinorflash_release();

    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (SPI_SW_SEM);

    return ((NOR_STATUS) ret);
}

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
NOR_STATUS
C6678_Spi_Norflash_Erase
(
    uint32_t  sector_number
)
{
    NOR_STATUS  ret;
    uint8_t       cmd[4];
    uint32_t      cmd_len;
    uint32_t      address;

    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (SPI_SW_SEM)) == 0)
    {
    }

    /* Claim the SPI controller */
    spinorflash_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /*
    * This function currently uses sector erase only.
    * probably speed things up by using bulk erase
    * when possible.
    */

    if (sector_number == SPI_NOR_BE_SECTOR_NUM)
    {
        cmd[0]  = SPI_NOR_CMD_BE;
        cmd_len = 1;

    }
    else if (sector_number >= SPI_NOR_SECTOR_COUNT)
    {
    	//platform_errno = PLATFORM_ERRNO_NORADDR;
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return NOR_EFAIL;
    }
    else
    {
        address = sector_number * SPI_NOR_SECTOR_SIZE;
        cmd[0]  = SPI_NOR_CMD_SE;
        cmd[1] = (address >> 16) & 0xff;
        cmd[2] = (address >>  8) & 0xff;
        cmd[3] = (address >>  0) & 0xff;

        cmd_len = 4;
    }

    /* Send Write Enable command */
    ret = spinorflash_cmd(SPI_NOR_CMD_WREN, NULL, 0);
    if (0!=ret)
    {
    	//platform_errno = PLATFORM_ERRNO_DEV_FAIL;
        spinorflash_release();
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return NOR_EFAIL;
    }

    ret = spinorflash_cmd_write(cmd, cmd_len, NULL, 0);
    if (0!=ret)
    {
    	//platform_errno = PLATFORM_ERRNO_DEV_FAIL;
        spinorflash_release();
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return NOR_EFAIL;
    }

    ret = nor_wait_ready(SPI_NOR_SECTOR_ERASE_TIMEOUT);
    if (0!=ret)
    {
    	//platform_errno = PLATFORM_ERRNO_DEV_TIMEOUT;
        spinorflash_release();
        CSL_semReleaseSemaphore (SPI_SW_SEM);
        return NOR_EFAIL;
    }
    spinorflash_release();

    CSL_semReleaseSemaphore (SPI_SW_SEM);

    return ret;
}




static NOR_STATUS
nor_wait_ready
(
    uint32_t  timeout
)
{
    NOR_STATUS  ret;
    uint8_t       status=0;
    uint8_t       cmd = SPI_NOR_CMD_RDSR;

    do
    {

        /* Send Read Status command */
        ret = spinorflash_xfer(1, &cmd, NULL, FALSE);
        if (0!=ret)
        {
            return ret;
        }

        /* Read status value */
        ret = spinorflash_xfer(1, NULL, &status, TRUE);
        if (0!=ret)
        {
            return ret;
        }

        if ((status & SPI_NOR_SR_WIP) == 0)
        {
            break;
        }

        timeout--;
        if (!timeout)
        {
            break;
        }

    } while (TRUE);

    if ((status & SPI_NOR_SR_WIP) == 0)
    {
        return NOR_EOK;
    }

    /* Timed out */
    return NOR_EFAIL;
}





static void
spinorflash_delay
(
    uint32_t delay
)
{
    volatile uint32_t i;

    for ( i = 0 ; i < delay ; i++ ){ };
}


/******************************************************************************
 *
 * Function:    spinorflash_claim
 *
 * Description: This function claims the SPI bus in the SPI controller
 *
 * Parameters:  Uint32 cs       - Chip Select number for the slave SPI device
 *              Uint32 freq     - SPI clock frequency
 *
 * Return Value: error status
 *
 ******************************************************************************/
SPI_STATUS
spinorflash_claim
(
    uint32_t      cs,
    uint32_t      freq
)
{
    uint32_t scalar;

    Osal_platformSpinorflashCsEnter();

    /* Enable the SPI hardware */
    SPI_SPIGCR0 = CSL_SPI_SPIGCR0_RESET_IN_RESET;
    asm(" NOP 9 ");//spinorflash_delay (2000);
    SPI_SPIGCR0 = CSL_SPI_SPIGCR0_RESET_OUT_OF_RESET;

    /* Set master mode, powered up and not activated */
    SPI_SPIGCR1 =   (CSL_SPI_SPIGCR1_MASTER_MASTER << CSL_SPI_SPIGCR1_MASTER_SHIFT)   |
                    (CSL_SPI_SPIGCR1_CLKMOD_INTERNAL << CSL_SPI_SPIGCR1_CLKMOD_SHIFT);


    /* CS0, CS1, CLK, Slave in and Slave out are functional pins */
    if (cs == 0)
    {
        SPI_SPIPC0 =    (CSL_SPI_SPIPC0_SCS0FUN0_SPI << CSL_SPI_SPIPC0_SCS0FUN0_SHIFT) |
                        (CSL_SPI_SPIPC0_CLKFUN_SPI << CSL_SPI_SPIPC0_CLKFUN_SHIFT)     |
                        (CSL_SPI_SPIPC0_SIMOFUN_SPI << CSL_SPI_SPIPC0_SIMOFUN_SHIFT)   |
                        (CSL_SPI_SPIPC0_SOMIFUN_SPI << CSL_SPI_SPIPC0_SOMIFUN_SHIFT);
    }
    else if (cs == 1)
    {
        SPI_SPIPC0 =    ((CSL_SPI_SPIPC0_SCS0FUN1_SPI << CSL_SPI_SPIPC0_SCS0FUN1_SHIFT) |
                        (CSL_SPI_SPIPC0_CLKFUN_SPI << CSL_SPI_SPIPC0_CLKFUN_SHIFT)     |
                        (CSL_SPI_SPIPC0_SIMOFUN_SPI << CSL_SPI_SPIPC0_SIMOFUN_SHIFT)   |
                        (CSL_SPI_SPIPC0_SOMIFUN_SPI << CSL_SPI_SPIPC0_SOMIFUN_SHIFT)) & 0xFFFF;
    }
    else
    {
    	return SPI_EFAIL;
    }

    /* setup format */
    scalar = ((SPI_MODULE_CLK / freq) - 1 ) & 0xFF;

    if ( cs == 0)
    {
        SPI_SPIFMT0 =   (8 << CSL_SPI_SPIFMT_CHARLEN_SHIFT)               |
                        (scalar << CSL_SPI_SPIFMT_PRESCALE_SHIFT)                      |
                        (CSL_SPI_SPIFMT_PHASE_DELAY << CSL_SPI_SPIFMT_PHASE_SHIFT)     |
                        (CSL_SPI_SPIFMT_POLARITY_LOW << CSL_SPI_SPIFMT_POLARITY_SHIFT) |
                        (CSL_SPI_SPIFMT_SHIFTDIR_MSB << CSL_SPI_SPIFMT_SHIFTDIR_SHIFT);
    }
    else if ( cs == 1)
    {
        SPI_SPIFMT0 =   (16 << CSL_SPI_SPIFMT_CHARLEN_SHIFT)               |
                        (scalar << CSL_SPI_SPIFMT_PRESCALE_SHIFT)                      |
                        (CSL_SPI_SPIFMT_PHASE_NO_DELAY << CSL_SPI_SPIFMT_PHASE_SHIFT)     |
                        (CSL_SPI_SPIFMT_POLARITY_LOW << CSL_SPI_SPIFMT_POLARITY_SHIFT) |
                        (CSL_SPI_SPIFMT_SHIFTDIR_MSB << CSL_SPI_SPIFMT_SHIFTDIR_SHIFT);
    }
    else
    {
    	return SPI_EFAIL;
    }

    /* hold cs active at end of transfer until explicitly de-asserted */
    data1_reg_val = (CSL_SPI_SPIDAT1_CSHOLD_ENABLE << CSL_SPI_SPIDAT1_CSHOLD_SHIFT) |
                    (0x02 << CSL_SPI_SPIDAT1_CSNR_SHIFT);
     if (cs == 0)
     {
         SPI_SPIDAT1 =   (CSL_SPI_SPIDAT1_CSHOLD_ENABLE << CSL_SPI_SPIDAT1_CSHOLD_SHIFT) |
                         (0x02 << CSL_SPI_SPIDAT1_CSNR_SHIFT);
     }

    /* including a minor delay. No science here. Should be good even with
    * no delay
    */
    if (cs == 0)
    {
        SPI_SPIDELAY =  (8 << CSL_SPI_SPIDELAY_C2TDELAY_SHIFT) |
                        (8 << CSL_SPI_SPIDELAY_T2CDELAY_SHIFT);
        /* default chip select register */
        SPI_SPIDEF  = CSL_SPI_SPIDEF_RESETVAL;
    }
    else if (cs == 1)
    {
        SPI_SPIDELAY =  (6 << CSL_SPI_SPIDELAY_C2TDELAY_SHIFT) |
                        (3 << CSL_SPI_SPIDELAY_T2CDELAY_SHIFT);
    }
    else
    {
    	return SPI_EFAIL;
    }

    /* no interrupts */
    SPI_SPIINT0 = CSL_SPI_SPIINT0_RESETVAL;
    SPI_SPILVL  = CSL_SPI_SPILVL_RESETVAL;

    /* enable SPI */
    SPI_SPIGCR1 |= ( CSL_SPI_SPIGCR1_ENABLE_ENABLE << CSL_SPI_SPIGCR1_ENABLE_SHIFT );

    if (cs == 1)
    {
        SPI_SPIDAT0 = 1 << 15;
        asm(" NOP 9 ");//spinorflash_delay (10000);
        /* Read SPIFLG, wait untill the RX full interrupt */
        if ( 0!=(SPI_SPIFLG & (CSL_SPI_SPIFLG_RXINTFLG_FULL<<CSL_SPI_SPIFLG_RXINTFLG_SHIFT)) )
        {
            /* Read one byte data */
            scalar = SPI_SPIBUF & 0xFF;
            /* Clear the Data */
            SPI_SPIBUF = 0;
        }
        else
        {
            /* Read one byte data */
            scalar = SPI_SPIBUF & 0xFF;
            return SPI_EFAIL;
        }
    }
    return SPI_EOK;
}

/******************************************************************************
 *
 * Function:    spinorflash_release
 *
 * Description: This function releases the bus in SPI controller
 *
 * Parameters:  None
 *
 * Return Value: None
 *
 ******************************************************************************/
void
spinorflash_release
(
    void
)
{
    /* Disable the SPI hardware */
    SPI_SPIGCR1 = CSL_SPI_SPIGCR1_RESETVAL;

    Osal_platformSpinorflashCsExit ();

}

/******************************************************************************
 *
 * Function:    spinorflash_xfer
 *
 * Description: This function sends and receives 8-bit data serially
 *
 * Parameters:  uint32_t nbytes   - Number of bytes of the TX data
 *              uint8_t* data_out - Pointer to the TX data
 *              uint8_t* data_in  - Pointer to the RX data
 *              Bool terminate  - TRUE: terminate the transfer, release the CS
 *                                FALSE: hold the CS
 *
 * Return Value: error status
 *
 ******************************************************************************/
SPI_STATUS
spinorflash_xfer
(
    uint32_t              nbytes,
    uint8_t*              data_out,
    uint8_t*              data_in,
    Bool                terminate
)
{
    uint32_t          i, buf_reg;
    uint8_t*          tx_ptr = data_out;
    uint8_t*          rx_ptr = data_in;


    /* Clear out any pending read data */
    SPI_SPIBUF;

    for (i = 0; i < nbytes; i++)
    {
        /* Wait untill TX buffer is not full */
        while( 0!=(SPI_SPIBUF & CSL_SPI_SPIBUF_TXFULL_MASK) )
        {
        }

        /* Set the TX data to SPIDAT1 */
        data1_reg_val &= ~0xFFFF;
        if(0!=tx_ptr)
        {
            data1_reg_val |= *tx_ptr & 0xFF;
            tx_ptr++;
        }

        /* Write to SPIDAT1 */
        if((i == (nbytes -1)) && (terminate))
        {
            /* Release the CS at the end of the transfer when terminate flag is TRUE */
            SPI_SPIDAT1 = (data1_reg_val & (~(CSL_SPI_SPIDAT1_CSHOLD_ENABLE << CSL_SPI_SPIDAT1_CSHOLD_SHIFT)));
        }
        else
        {
            SPI_SPIDAT1 = data1_reg_val;
        }

        /* Read SPIBUF, wait untill the RX buffer is not empty */
        while ( 0!=( SPI_SPIBUF & CSL_SPI_SPIBUF_RXEMPTY_MASK ) )
        {
        }

        /* Read one byte data */
        buf_reg = SPI_SPIBUF;
        if(0!=rx_ptr)
        {
            *rx_ptr = buf_reg & 0xFF;
            rx_ptr++;
        }
    }
    return SPI_EOK;
}


/******************************************************************************
 *
 * Function:    spinorflash_cmd
 *
 * Description: This function sends a single byte command and receives response data
 *
 * Parameters:  uint8_t  cmd      - Command sent to the NOR flash
 *              uint8_t* response - Pointer to the RX response data
 *              uint32_t len      - Lenght of the response in bytes
 *
 * Return Value: error status
 *
 ******************************************************************************/
SPI_STATUS
spinorflash_cmd
(
    uint8_t               cmd,
    uint8_t*              response,
    uint32_t              len
)
{
    Bool        flags = FALSE;
    uint32_t      ret;

    if (len == 0)
    {
        flags = TRUE;
    }

    /* Send the command byte */
    ret = spinorflash_xfer(1, &cmd, NULL, flags);
    if (0!=ret)
    {
    	//IFPRINT (platform_write("SF: Failed to send command %02x: %d\n", cmd, ret));
        return ret;
    }

    /* Receive the response */
    if (len)
    {
        ret = spinorflash_xfer(len, NULL, response, TRUE);
        if (0!=ret)
        {
        	//IFPRINT (platform_write("SF: Failed to read response (%zu bytes): %d\n",  len, ret));
        }
    }

    return ret;
}

/******************************************************************************
 *
 * Function:    spinorflash_cmd_read
 *
 * Description: This function sends a read command and reads data from the flash
 *
 * Parameters:  uint8_t  cmd      - Command sent to the NOR flash
 *              uint32_t cmd_len  - Length of the command in bytes
 *              uint8_t* dat      - Pointer to the data read
 *              uint32_t data_len - Lenght of the data read in bytes
 *
 * Return Value: error status
 *
 ******************************************************************************/
SPI_STATUS
spinorflash_cmd_read
(
    uint8_t*              cmd,
    uint32_t              cmd_len,
    uint8_t*              data,
    uint32_t              data_len
)
{
    Bool        flags = FALSE;
    uint32_t      ret;

    if (data_len == 0)
    {
        flags = TRUE;
    }

    /* Send read command */
    ret = spinorflash_xfer(cmd_len, cmd, NULL, flags);
    if (0!=ret)
    {
    	//IFPRINT (platform_write("SF: Failed to send read command (%zu bytes): %d\n",
               //cmd_len, ret));
    }
    else
    {
    	if (data_len != 0)
    	{
			/* Read data */
			ret = spinorflash_xfer(data_len, NULL, data, TRUE);
			if (0!=ret)
			{
				//IFPRINT (platform_write("SF: Failed to read %zu bytes of data: %d\n",
					   //data_len, ret));
			}
    	}
    }

    return ret;
}

/******************************************************************************
 *
 * Function:    spinorflash_cmd_write
 *
 * Description: This function sends a write command and writes data to the flash
 *
 * Parameters:  uint8_t  cmd      - Command sent to the NOR flash
 *              uint32_t cmd_len  - Length of the command in bytes
 *              uint8_t* dat      - Pointer to the data to be written
 *              uint32_t data_len - Lenght of the data in bytes
 *
 * Return Value: error status
 *
 ******************************************************************************/
SPI_STATUS
spinorflash_cmd_write
(
    uint8_t*        cmd,
    uint32_t        cmd_len,
    uint8_t*        data,
    uint32_t        data_len
)
{
    Bool           flags = FALSE;
    uint32_t         ret;

    if (data_len == 0)
    {
        flags = TRUE;
    }

    /* Send write command */
    ret = spinorflash_xfer(cmd_len, cmd, NULL, flags);
    if (0!=ret)
    {
    	//IFPRINT (platform_write("SF: Failed to send write command (%zu bytes): %d\n",
            //cmd_len, ret));
    }
    else
    {
    	if (data_len != 0)
    	{
			/* Write data */
			ret = spinorflash_xfer(data_len, data, NULL, TRUE);
			if (0!=ret)
			{
				//IFPRINT (platform_write("SF: Failed to write %zu bytes of data: %d\n",
				//data_len, ret));
			}
    	}
    }

    return ret;
}


/******************************************************************************
 *
 * Function:    spinorflash_read_word
 *
 * Description: This function sends a read command and reads data in 16-bit data format
 *
 * Parameters:  uint16_t* cmd_buf  - Pointer to the command sent
 *              uint32_t cmd_len   - Length of the command in words
 *              uint16_t* data_buf - Pointer to the data read
 *              uint32_t data_len  - Lenght of the data read in words
 *
 * Return Value: error status
 *
 ******************************************************************************/
SPI_STATUS
spinorflash_read_word
(
    uint16_t*             cmd_buf,
    uint32_t              cmd_len,
    uint16_t*             data_buf,
    uint32_t              data_len
)
{
    uint32_t          data1_reg;
    uint16_t*         tx_ptr = cmd_buf;
    uint16_t*         rx_ptr = data_buf;

	/* disable the SPI communication by setting
	 * the SPIGCR1.ENABLE to 0
	 *
	 *
	 * SPIGCR1
	 * ============================================
	 * Bit 	Field 	Value 		Description
	 * 24 	ENABLE 	0			SPI disable
	 * ============================================
	 */
	data1_reg = 0x1 << 24;
	data1_reg = ~data1_reg;
    SPI_SPIGCR1 &= data1_reg;

	/*
	 * clean TX data into SPIDAT0
	 *
	 * SPIDAT0
	 * ============================================
	 * Bit 	Field 	Value 		Description
	 * 15-0 TXDATA 	0-FFFFh 	SPI transmit data
	 * ============================================
	 *
	 */

    SPI_SPIDAT0 = 0;

	/* 8.
	 * Enable the SPI communication by setting
	 * the SPIGCR1.ENABLE to 1
	 *
	 *
	 * SPIGCR1
	 * ============================================
	 * Bit 	Field 	Value 		Description
	 * 24 	ENABLE 	1			SPI enable
	 * ============================================
	 */

	data1_reg = 0x1 << 24;
    SPI_SPIGCR1 = (SPI_SPIGCR1 | data1_reg);

    {
        {
            SPI_SPIDAT0 = *tx_ptr;
            spinorflash_delay(10000);
        }

        /* Read SPIFLG, wait untill the RX full interrupt */
        if ( 0!=(SPI_SPIFLG & (CSL_SPI_SPIFLG_RXINTFLG_FULL<<CSL_SPI_SPIFLG_RXINTFLG_SHIFT)) )
        {
            /* Read one byte data */
            *rx_ptr = SPI_SPIBUF & 0xFF;
            SPI_SPIBUF = 0;
        }
        else
        {
            return SPI_EFAIL;
        }
    }
    return SPI_EOK;
}

/******************************************************************************
 *
 * Function:    spinorflash_write_word
 *
 * Description: This function sends a write command and writes data in 16-bit data format
 *
 * Parameters:  uint16_t* cmd_buf  - Pointer to the command sent
 *              uint32_t cmd_len   - Length of the command in bytes
 *              uint16_t* data_buf - Pointer to the data read
 *              uint32_t data_len  - Lenght of the data read in bytes
 *
 * Return Value: error status
 *
 ******************************************************************************/
SPI_STATUS
spinorflash_write_word
(
    uint16_t*             cmd_buf,
    uint32_t              cmd_len,
    uint16_t*             data_buf,
    uint32_t              data_len
)
{
    uint32_t          data1_reg;
    uint16_t*         tx_ptr = cmd_buf;

	/* disable the SPI communication by setting
	 * the SPIGCR1.ENABLE to 0
	 *
	 *
	 * SPIGCR1
	 * ============================================
	 * Bit 	Field 	Value 		Description
	 * 24 	ENABLE 	0			SPI disable
	 * ============================================
	 */
	data1_reg = 0x1 << 24;
	data1_reg = ~data1_reg;
    SPI_SPIGCR1 &= data1_reg;

	/*
	 * clean TX data into SPIDAT0
	 *
	 * SPIDAT0
	 * ============================================
	 * Bit 	Field 	Value 		Description
	 * 15-0 TXDATA 	0-FFFFh 	SPI transmit data
	 * ============================================
	 *
	 */

    SPI_SPIDAT0 = 0;

	/* 8.
	 * Enable the SPI communication by setting
	 * the SPIGCR1.ENABLE to 1
	 *
	 *
	 * SPIGCR1
	 * ============================================
	 * Bit 	Field 	Value 		Description
	 * 24 	ENABLE 	1			SPI enable
	 * ============================================
	 */

	data1_reg = 0x1 << 24;
    SPI_SPIGCR1 = (SPI_SPIGCR1 | data1_reg);

    {
        {
            SPI_SPIDAT0 = *tx_ptr;
            spinorflash_delay(10000);
        }

        /* Read SPIFLG, wait untill the RX full interrupt */
        if ( 0!=(SPI_SPIFLG & (CSL_SPI_SPIFLG_TXINTFLG_EMPTY<<CSL_SPI_SPIFLG_TXINTFLG_SHIFT)) )
        {
            /* Clear the SPIBUF */
            SPI_SPIBUF = 0;
            return SPI_EOK;
        }
        else
        {
            return SPI_EFAIL;
        }
    }
}


void Osal_platformSpinorflashCsEnter(void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (RADAR_SPI_HW_SEM)) == 0)
    {
    }

    return;
}

void Osal_platformSpinorflashCsExit (void)
{
    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (RADAR_SPI_HW_SEM);

    return;
}


