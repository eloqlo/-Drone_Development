/*
 * FS-iA6B.h
 *
 *  Created on: Jul 30, 2025
 *      Author: jh
 */

#ifndef INC_FS_IA6B_H_
#define INC_FS_IA6B_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct _FSiA6B_iBus{
	unsigned short RH;
	unsigned short RV;
	unsigned short LV;
	unsigned short LH;
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned char FailSafe;		// Fail-safe 상태를 저장할 변수
} FSiA6B_iBus;

extern FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parsing(unsigned char* data, FSiA6B_iBus* ibus);
void FSiA6B_UART5_Init(void);
unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus);

#ifdef __cplusplus
}
#endif
#endif	/* __FSIA6B_H */
