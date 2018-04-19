/******************************************************************************
 * Copyright (c) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/
/*
 *  Linker command file
 *
 */

-c
-heap  0x41000
-stack 0xa000

/* Memory Map 1 - the default */
MEMORY
{
    L1PSRAM (RWX)  : org = 0x0E00000, len = 0x7FFF
    L1DSRAM (RWX)  : org = 0x0F00000, len = 0x7FFF 

    L2SRAM (RWX)   : org = 0x00800000,len = 0x80000
    MSMCSRAM (RWX) : org = 0x0c000000,len = 0x400000
    DDR3 (RWX)     : org = 0x80000000,len = 0x10000000
}

SECTIONS
{
    .csl_vect   >       L2SRAM//srio测试可以放在L2或者L3上
    .text       >       L2SRAM//srio测试可以放在L2或者L3上
    GROUP (NEAR_DP)
    {
    .neardata
    .rodata
    .bss
    } load > L2SRAM//srio测试可以放在L2
    .stack      >       L2SRAM//srio测试可以放在L2
    .cinit      >       L2SRAM//srio测试可以放在L2或者L3上
    .cio        >       L2SRAM//srio测试可以放在L2
    .const      >       L2SRAM//srio测试可以放在L2或者L3上
    .data       >       L2SRAM//srio测试可以放在L2或者L3上
    .switch     >       L2SRAM//srio测试可以放在L2或者L3上
    .sysmem     >       MSMCSRAM//srio测试可以放在L2或者L3上
    .far        >       L2SRAM//srio测试必须放在L2或者L3上
    .testMem    >       L2SRAM//srio测试可以放在L2或者L3上
    .fardata    >       L2SRAM//srio测试可以放在L2或者L3上
    platform_lib > 		L2SRAM//srio测试可以放在L2或者L3上

    //必须放在MSMCSRAM
    .qmssSharedMem:          load >> MSMCSRAM
    .cppiSharedMem:          load >> MSMCSRAM
    .i2ceeprom:     load >> MSMCSRAM//i2ceeprom段必须放在MSMCSRAM
    .emif16nandflash 	load >> MSMCSRAM//emif16nandflash段必须放在MSMCSRAM
    .srioSharedMem >   	MSMCSRAM//srio放在msmc上的代码
    .Sharemem   >       MSMCSRAM //edma3多核测试main.c程序中的段
    .timerSharedMem   >       MSMCSRAM //timer多核测试main.c程序中的段

    //必须放在L2上
    .srioL2Mem	>   	L2SRAM//srio放在L2上的代码
    .qmssL2Mem:          load >> L2SRAM//qmss放在L2上的代码
    .cppiL2Mem:          load >> L2SRAM//cppi放在L2上的代码
    .CoreIntcL2Mem:		load >> L2SRAM//coreintc放在L2上的代码
    .edma3				load >> L2SRAM//edma3放在L2上的代码
}
