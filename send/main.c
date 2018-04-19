/*
 * main.c
 */

#include"c6678.h"
/********************************************************************
 * 可修改的全局变量
 *******************************************************************/

/********************************************************************
 * spi_norflash部分
 *******************************************************************/
#define SPI_NOR_TEST_SIZE (64*1024)
#define NOR_SEND_BUF_ADDR  0x0C170000
#define NOR_RECV_BUF_ADDR  0x0C180000

#define SPI_NOR_TOTAL_SIZE (16*1024*1024)
#define TEST_LOOP_COUNT  			1
#define SPI_NOR_TOTAL_SECTORS  		256

//记录速度
CSL_Uint64 average_write_speed=0;
CSL_Uint64 average_read_speed=0;

//DMA测试宏变量
#define DEVICE_HZ   1000000000//800MHZ:800000000//1GHZ:1000000000;1.25GHZ:1250000000
#define MB   		0x100000
#define KB   		1024

Bool test_nor(void);



void main(void)
{
	int i;
	NOR_STATUS      nor_status;
	int coreNum;
	/* Get the core number. */
	coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
	if(coreNum==0)
	{
		//初始化PLL
	    if (C6678_Pll_Init(PLATFORM_PLL1_PLLM_val)!= TRUE)
	    {
	    	printf("PLL failed to initialize!!!!!!!!!!!!!!!!!!!!!!!!! \n" );
	    }
	    else
	    {
	    	printf("PLL successd to initialize\n" );
	    }
	    /**/

	    C6678_Power_UpDomains();
	    C6678_Ecc_Enable();
	}
	//使能TIMER
	C6678_TimeCounter_Enable();
	
	if(coreNum==0)
	{
		//初始化SPI_NORFLASH
		nor_status=C6678_Spi_Norflash_Init();
	    if (nor_status != NOR_EOK)
	    {
	    	printf("SPI_NORFLASH failed to initialize!!!!!!!!!!!!!!!!!!!!!!!!! \n" );
	    }
	    else
	    {
	    	printf("SPI_NORFLASH successd to initialize! \n" );
	    }
	}
	    
	//开始测试
	if(DEVICE==DEVICE1)
	{
		for(i=0;i<1;i++)
		{
			if (coreNum ==0)
			{
				
				//测试NOR
				if (test_nor()!= TRUE)
				{
					printf("NOR test failed!!!!!!!!!!!!!!!!!!!!!!!!! \n" );
				}
				else
				{
					printf("NOR test success! \n" );
				}
			}

		}
	}


    while(1)
	{
    	C6678_TimeCounter_Delaycycles(5000);
	}
}


Bool test_nor(void)
{
	uint8_t 		* buf1_nor,* buf2_nor;
	Uint32 index;
	Uint32 index_times;
	Uint32 index_total;
	CSL_Uint64 val,val_start,val_end;
	Uint32 val_high,val_low;

   //开始测试SPI_NORFLASH
	buf2_nor=(uint8_t *)NOR_RECV_BUF_ADDR;
	buf1_nor=(uint8_t *)NOR_SEND_BUF_ADDR;

	for(index_times=0;index_times<TEST_LOOP_COUNT;index_times++)
	{
		for(index=0;index<SPI_NOR_TOTAL_SECTORS;index++)
		{
			//擦出SPI_NORFLASH（起始地址：0 setors；长度：1 setors）
			C6678_Spi_Norflash_Erase(index);
		}
		average_write_speed = 0;
		average_read_speed = 0;

		for(index_total=0;index_total<(SPI_NOR_TOTAL_SIZE/SPI_NOR_TEST_SIZE);index_total++)
		{
			/* Fill in a test pattern */
			for (index = 0; index < SPI_NOR_TEST_SIZE; index++)
			{
				buf2_nor[index]=index+index_total;
				buf1_nor[index]=0;
			}
			//cache操作
			C6678_Cache_Wb((Uint32 *)buf1_nor,SPI_NOR_TEST_SIZE,CACHE_WAIT);
			C6678_Cache_Wb((Uint32 *)buf2_nor,SPI_NOR_TEST_SIZE,CACHE_WAIT);

			val_high = C6678_TimeCounter_GetHighVal();
			val_low = C6678_TimeCounter_GetLowVal();
			val_start = (CSL_Uint64)(val_low+(val_high*0x100000000));

			//写入SPI_NORFLASH（起始地址：0 setors；长度：1 setors）
			C6678_Spi_Norflash_Write(index_total*64*1024, SPI_NOR_TEST_SIZE, buf2_nor);

			val_high = C6678_TimeCounter_GetHighVal();
			val_low = C6678_TimeCounter_GetLowVal();
			val_end = (CSL_Uint64)(val_low+(val_high*0x100000000));

			val=val_end-val_start;
			printf("time counter is %lld! ",val );
			printf("the speed of spinorflash_write is %lldKB/s!\n",((SPI_NOR_TEST_SIZE/KB)*(DEVICE_HZ/val)));

			average_write_speed+=((SPI_NOR_TEST_SIZE/KB)*(DEVICE_HZ/val));

			val_high = C6678_TimeCounter_GetHighVal();
			val_low = C6678_TimeCounter_GetLowVal();
			val_start = (CSL_Uint64)(val_low+(val_high*0x100000000));

			//再读出NANDFLASH（起始地址：0 setors；长度：1 setors）
			C6678_Spi_Norflash_Read(index_total*64*1024, SPI_NOR_TEST_SIZE, buf1_nor);

			val_high = C6678_TimeCounter_GetHighVal();
			val_low = C6678_TimeCounter_GetLowVal();
			val_end = (CSL_Uint64)(val_low+(val_high*0x100000000));

			val=val_end-val_start;
			printf("time counter is %lld! ",val );
			printf("the speed of spinorflash_read is %lldKB/s!\n",((SPI_NOR_TEST_SIZE/KB)*(DEVICE_HZ/val)));

			average_read_speed+=((SPI_NOR_TEST_SIZE/KB)*(DEVICE_HZ/val));

			//cache操作
			C6678_Cache_Inv((Uint32 *)buf2_nor,SPI_NOR_TEST_SIZE,CACHE_WAIT);

			//比较
			for (index = 0; index < SPI_NOR_TEST_SIZE; index++)
			{
				if (buf2_nor[index] != buf1_nor[index])
				{
					printf ("Received data = %d\nTransmited data = %d\nIndex = %d.\n\nTest failed!!!!!!!!!!!!!!!!!!!\n",
										buf2_nor[index], buf1_nor[index], index);
					return FALSE;
				}
			}
		}
		printf("the average speed of spinorflash_write is %lldKB/s!\n",average_write_speed/(SPI_NOR_TOTAL_SIZE/SPI_NOR_TEST_SIZE));
		printf("the average speed of spinorflash_read is %lldKB/s!\n",average_read_speed/(SPI_NOR_TOTAL_SIZE/SPI_NOR_TEST_SIZE));
	}
	return TRUE;
}



