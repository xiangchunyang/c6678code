#include "c6678.h"
/*
 * test.h
 *
 *  Created on: 2013-8-13
 *      Author: Administrator
 */

#ifndef TEST_H_
#define TEST_H_


/* By wangjie,2012-8-13
* 函数功能：save The contents of IRP
* A4 = OutPut:输出的数值(IRP的值)*/
extern unsigned int save_irp(void);

/* By wangjie,2012-8-13
* 函数功能：save The contents of NRP
* A4 = OutPut:输出的数值(NRP的值)
*/
extern unsigned int save_nrp(void);

/* By wangjie,2012-8-13
* 函数功能：save The contents of PGIE
* A4 = OutPut:输出的数值(PGIE的值)
*/
extern unsigned int save_pgie(void);

/* By wangjie,2012-8-13
* 函数功能：save The contents of ITSR
* A4 = OutPut:输出的数值(ITSR的值)
*/
extern unsigned int save_itsr(void);

/* By wangjie,2012-8-13
* 函数功能：set GIE bit
*
*/
extern void set_gie(void);

/* By wangjie,2012-8-13
* 函数功能：clear GIE bit
*
*/
extern void clear_gie(void);

/* By wangjie,2012-8-13
* 函数功能：restore The contents of PGIE bit
* A4 = InPut:输出的数值(PGIE的值)
*/
extern void restore_pgie(unsigned int pgie_value);

/* By wangjie,2012-8-13
* 函数功能：restore The contents of ITSR
* A4 = InPut:输入的数值(ITSR的值)
*/
extern void restore_itsr(unsigned int itsr_value);

/* By wangjie,2012-8-13
* 函数功能：restore The contents of IRP
* A4 = InPut:输入的数值(IRP的值)
*/
extern void restore_irp(unsigned int irp_value);

/* By wangjie,2012-8-13
* 函数功能：restore The contents of NRP
* A4 = InPut:输入的数值(NRP的值)
*/
extern void restore_nrp(unsigned int nrp_value);

#endif /* TEST_H_ */
