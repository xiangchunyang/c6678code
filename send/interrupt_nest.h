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
* �������ܣ�save The contents of IRP
* A4 = OutPut:�������ֵ(IRP��ֵ)*/
extern unsigned int save_irp(void);

/* By wangjie,2012-8-13
* �������ܣ�save The contents of NRP
* A4 = OutPut:�������ֵ(NRP��ֵ)
*/
extern unsigned int save_nrp(void);

/* By wangjie,2012-8-13
* �������ܣ�save The contents of PGIE
* A4 = OutPut:�������ֵ(PGIE��ֵ)
*/
extern unsigned int save_pgie(void);

/* By wangjie,2012-8-13
* �������ܣ�save The contents of ITSR
* A4 = OutPut:�������ֵ(ITSR��ֵ)
*/
extern unsigned int save_itsr(void);

/* By wangjie,2012-8-13
* �������ܣ�set GIE bit
*
*/
extern void set_gie(void);

/* By wangjie,2012-8-13
* �������ܣ�clear GIE bit
*
*/
extern void clear_gie(void);

/* By wangjie,2012-8-13
* �������ܣ�restore The contents of PGIE bit
* A4 = InPut:�������ֵ(PGIE��ֵ)
*/
extern void restore_pgie(unsigned int pgie_value);

/* By wangjie,2012-8-13
* �������ܣ�restore The contents of ITSR
* A4 = InPut:�������ֵ(ITSR��ֵ)
*/
extern void restore_itsr(unsigned int itsr_value);

/* By wangjie,2012-8-13
* �������ܣ�restore The contents of IRP
* A4 = InPut:�������ֵ(IRP��ֵ)
*/
extern void restore_irp(unsigned int irp_value);

/* By wangjie,2012-8-13
* �������ܣ�restore The contents of NRP
* A4 = InPut:�������ֵ(NRP��ֵ)
*/
extern void restore_nrp(unsigned int nrp_value);

#endif /* TEST_H_ */
