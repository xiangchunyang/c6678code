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
    .csl_vect   >       L2SRAM//srio���Կ��Է���L2����L3��
    .text       >       L2SRAM//srio���Կ��Է���L2����L3��
    GROUP (NEAR_DP)
    {
    .neardata
    .rodata
    .bss
    } load > L2SRAM//srio���Կ��Է���L2
    .stack      >       L2SRAM//srio���Կ��Է���L2
    .cinit      >       L2SRAM//srio���Կ��Է���L2����L3��
    .cio        >       L2SRAM//srio���Կ��Է���L2
    .const      >       L2SRAM//srio���Կ��Է���L2����L3��
    .data       >       L2SRAM//srio���Կ��Է���L2����L3��
    .switch     >       L2SRAM//srio���Կ��Է���L2����L3��
    .sysmem     >       MSMCSRAM//srio���Կ��Է���L2����L3��
    .far        >       L2SRAM//srio���Ա������L2����L3��
    .testMem    >       L2SRAM//srio���Կ��Է���L2����L3��
    .fardata    >       L2SRAM//srio���Կ��Է���L2����L3��
    platform_lib > 		L2SRAM//srio���Կ��Է���L2����L3��

    //�������MSMCSRAM
    .qmssSharedMem:          load >> MSMCSRAM
    .cppiSharedMem:          load >> MSMCSRAM
    .i2ceeprom:     load >> MSMCSRAM//i2ceeprom�α������MSMCSRAM
    .emif16nandflash 	load >> MSMCSRAM//emif16nandflash�α������MSMCSRAM
    .srioSharedMem >   	MSMCSRAM//srio����msmc�ϵĴ���
    .Sharemem   >       MSMCSRAM //edma3��˲���main.c�����еĶ�
    .timerSharedMem   >       MSMCSRAM //timer��˲���main.c�����еĶ�

    //�������L2��
    .srioL2Mem	>   	L2SRAM//srio����L2�ϵĴ���
    .qmssL2Mem:          load >> L2SRAM//qmss����L2�ϵĴ���
    .cppiL2Mem:          load >> L2SRAM//cppi����L2�ϵĴ���
    .CoreIntcL2Mem:		load >> L2SRAM//coreintc����L2�ϵĴ���
    .edma3				load >> L2SRAM//edma3����L2�ϵĴ���
}
