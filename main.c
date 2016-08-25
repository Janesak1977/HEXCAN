#include "config.h"
#include "main.h"
#include "CAN.h"
#include "timerfunc.h"
#include "comm.h"


#define	EE_Name			0x0001
#define EE_ChksumHI		0x0011
#define EE_ChksumLO		0x0012
#define EE_0014			0x0014
#define EE_0015			0x0015
#define EE_0016			0x0016
#define EE_VersionMinor	0x0018
#define EE_VersionMajor	0x0019
#define EE_001B			0x001B
#define EE_001C			0x001C
#define EE_FOSC			0x0020



typedef union {
    unsigned char myByte[4];
    unsigned long mylong;
} unionLong_t;

#define DivideS(Divisor,Divider) \
	asm volatile ( \
		"lds r22, %A0" "\n\t" \
		"lds r23, %B0" "\n\t" \
		"lds r24, %C0" "\n\t" \
		"lds r25, %D0" "\n\t" \
		"mov r20,%A1" \
		: \
		: "m" (Divisor), "=r" (Divider) \
		: \
		)

#define loop_until_var_is_nz(var) \
	asm volatile (	\
		"1: " "lds r24,%0" "\n\t" \
		"tst r24" "\n\t" \
		"brne 1b" \
		: \
		: "m" (var)\
		: "r24" \
		)



#define delay_loop(count) \
	asm volatile (		\
		"1: " "nop" "\n\t"	\
		"subi %0, 1" "\n\t"	\
		"brne 1b"			\
		: /* no outputs */	\
		: "r" ((uint8_t)(count))	\
		)

#define bit_set(p,m) ((p) |= _BV(m))
#define bit_clear(p,m) ((p) &= ~_BV(m))

#define mem_bit_is_set(var,bit)  (var & _BV(bit))
#define mem_bit_is_clear(var,bit)  (!(var & _BV(bit)))


/*
struct CANMessage {

	unsigned char SIDH;
	unsigned char SIDL;
	unsigned char EID8;
	unsigned char EID0;
	unsigned char DLC;
	unsigned char Data[7];
};


struct CANBuffer {
	unsigned char BuffID;
	struct CANMessage CANMsg;
};
*/


typedef struct RXPacket {
	unsigned char MAGIC;
	unsigned char PacketLen;
	unsigned char PacketCMD;
	unsigned char PacketData[125];

} RXPacket_t;


struct sQLONG { 
	unsigned long First;
	unsigned long Second;
	unsigned long Third;
	unsigned long Fourth;

};

struct sDLONG { 
	unsigned long First;
	unsigned long Second;
};


unsigned int tmpEE0016 = 0xFFFF;
unsigned int tempOCR1A = 0x0000;
unsigned char KLines_OUT_MASK = 0x00;
unsigned char unk_100105 = 0x00;
unsigned char CMD_0C_Executed = 0x00;
unsigned char unk_100107 = 0x00;
unsigned char CANRXBuffer[16] = {0x00};
unsigned char CANMsgBuffID = 0x00;
unsigned char CANMsgBuff[13] = {0x00};
unsigned char CANMsgXORTable[16] = {0x00};
unsigned char QLONGBuff[16] = {0x00};
unsigned char DLONGBuff[8] = {0x00};
unsigned char byte_10014E[8] = {0x00};
unsigned char byte_100156[42] = {0x00};
unsigned char byte_100180[16] =  {0x00};
unsigned char byte_100190[70] =  {0x00};
unsigned char byte_1001D6[128] = {0x00};
unsigned char byte_100256[32] = {0x00};
unsigned char TMR0_OFCounter = 0x00;
unsigned char PacketSended = 0x00;
unsigned char CharReceived_flag = 0x00;
unsigned char UART_ReceivedChar = 0x00;
unsigned long CurrentBaudrate = 0x00000000;
unsigned char BytesInECUPCBuff = 0x00;
unsigned char byteFromECU = 0x00;
unsigned char ECUPCBuffPtr = 0x00;
unsigned char BitCounter = 0x00;
unsigned char ECUBuffer[32] = {0x00};
unsigned char KLines_IN_MASK = 0x00;
unsigned char ByteFromECUCompleted = 0x00;
unsigned int TempTCNT1 = 0x0000;
unsigned char tempByteFromECU = 0x00;
unsigned char ErrCodeToPC = 0x00;
struct Timerstruct RXCharDelay = {0x00, 0x00};
unsigned char TXPacketLength = 0x00;
unsigned char PacketReceived = 0x00;
unsigned char RXBuffer[128] = {0x00};
unsigned char RXBuffChksum = 0x00;
		 char TXBuffClearToWrite = 0x00;
unsigned char PacketRXError = 0x00;
unsigned char TXBuffPtr = 0x00;
unsigned char TXBuffer[128] = {0x00};
unsigned char RXBuffPtr = 0x00;
unsigned long dw_FOSC = 0x00;
unsigned long HASH = 0x00;


//***********************************************************************
//  FLASH constants
//***********************************************************************
unsigned char rawData[256] PROGMEM =
{
    0x00, 0xEA, 0x28, 0x3A, 0x50, 0x1D, 0xB7, 0x58, 0xDB, 0x11, 0x60, 0x0E, 0xE2, 0xC1, 0x41, 0xD8, 
    0x80, 0x2C, 0x70, 0x3F, 0xB0, 0xB3, 0xD7, 0x58, 0xBE, 0xC2, 0x42, 0x8D, 0xFB, 0xB2, 0xB6, 0x9F, 
    0xB5, 0xEE, 0x05, 0xEC, 0x66, 0x55, 0x9F, 0xEE, 0x2C, 0xE0, 0xAB, 0x05, 0x79, 0x85, 0x56, 0x76, 
    0x0C, 0x5B, 0x6B, 0xB3, 0x20, 0xFE, 0x25, 0xDD, 0xE3, 0x35, 0x41, 0x87, 0x29, 0xD2, 0x56, 0xBD, 
    0x62, 0xDA, 0x69, 0x44, 0xC9, 0xE6, 0xBC, 0x24, 0xDF, 0xC9, 0xE8, 0x64, 0x18, 0x73, 0x2B, 0x15, 
    0xD4, 0x16, 0x03, 0x92, 0x90, 0x87, 0xFC, 0x05, 0x5E, 0xE6, 0xC6, 0x2D, 0x92, 0x81, 0x8A, 0x5F, 
    0xBF, 0xF6, 0x7E, 0xCB, 0xE2, 0x98, 0xB8, 0x01, 0xDC, 0x14, 0x40, 0xB4, 0x26, 0x55, 0x68, 0xBD, 
    0xC0, 0xA4, 0x60, 0x62, 0x6A, 0x14, 0x05, 0xD9, 0x17, 0x1D, 0xFA, 0x08, 0x9F, 0x87, 0xFA, 0x8F, 
    0xB4, 0x89, 0x6C, 0x08, 0x17, 0x33, 0x38, 0x8D, 0x0B, 0x09, 0xDA, 0x7B, 0x0B, 0xF1, 0xB5, 0x76, 
    0xB8, 0x4F, 0xA9, 0xAE, 0x15, 0x6E, 0xE7, 0x60, 0xF6, 0x22, 0x04, 0x9F, 0xB6, 0xAB, 0x4D, 0x54, 
    0x29, 0xDD, 0x5B, 0x84, 0xD1, 0x7E, 0xE7, 0xD1, 0x55, 0xF0, 0xDF, 0x44, 0x2E, 0x0F, 0xB8, 0x49, 
    0xA4, 0x5D, 0x07, 0xFB, 0xF9, 0x5D, 0x4B, 0xA2, 0xE4, 0x3C, 0x0D, 0x7A, 0x41, 0xB6, 0x2C, 0xB7, 
    0x06, 0x38, 0x72, 0xC5, 0x79, 0x42, 0x6A, 0xD4, 0xA0, 0x10, 0x76, 0x94, 0xF9, 0x79, 0x1C, 0x3D, 
    0x6C, 0x17, 0xA1, 0xD3, 0x7E, 0xA8, 0xD9, 0xA8, 0xC7, 0xB4, 0x3D, 0x22, 0xA6, 0x71, 0x3E, 0xBE, 
    0x33, 0xE3, 0xD9, 0x55, 0x75, 0x47, 0x6C, 0x9F, 0xD6, 0xB2, 0xC8, 0xF5, 0xD3, 0xF6, 0x87, 0x5B, 
    0xF8, 0xC5, 0xA0, 0xBD, 0x0C, 0x19, 0x38, 0x79, 0x89, 0xD2, 0xBB, 0x1E, 0x4F, 0xA1, 0x2C, 0x73
};


unsigned char QLONG00B8[16] PROGMEM = 
{
    0xBD, 0xA1, 0xC6, 0xB9, 0x2D, 0xCB, 0xD2, 0x81, 0x7A, 0x33, 0x29, 0x0C, 0x79, 0x2A, 0x41, 0x5F
};

unsigned char QLONG00C0[16] PROGMEM =
{
    0x43, 0xAC, 0x71, 0xC1, 0xAF, 0x1F, 0x33, 0xC4, 0x46, 0xE1, 0x14, 0x28, 0x90, 0x54, 0x82, 0x31 
};

unsigned char sRossTech[] PROGMEM = "Copyright (c) 2011 by Hex Microsystems Pty & Ross-Tech, LLC.\r\n";





//***********************************************************************
//  EEPROM Content
//***********************************************************************
/*uint8_t EEMEM EE_Start = 0xFF;
uint8_t EEMEM EE_Name[8] = "ROSSTECH";
uint8_t EEMEM EE_Serial[8] = {00,00,00,0xA8,0xB0,0xE7,0x92,0x24};
uint8_t EEMEM EE_ChksumHI = 0x00;
uint8_t EEMEM EE_ChksumLO = 0x00;
uint8_t EEMEM EE_Dummy1 = 0xFF;
uint8_t EEMEM EE_0014 = 0xFF;
uint8_t EEMEM EE_0015 = 0xAB;
uint16_t EEMEM EE_0016 = 0xFFFF;
uint8_t EEMEM EE_VersionMinor = 0xFF;
uint8_t EEMEM EE_VersionMajor = 0xFF;
uint8_t EEMEM EE_Dummy2 = 0xFF;
uint8_t EEMEM EE_001B = 0x02;
uint32_t EEMEM EE_001C= 0x00000002; 
uint32_t EEMEM EE_FOSC = 0xFFFFFFFF;
uint8_t EEMEM EE_Dummy3[28] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
							   0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
							   0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t EEMEM EE_KEY[32]={0xDC,0x49,0x4E,0xC0,0xFA,0xEF,0xEB,0x0E,0x8F,0xAC,0x26,0xA1,0x04,0x1C,0x64,0x8E,
						  0x86,0xA9,0x43,0x6E,0x20,0x4C,0x2F,0x0F,0x1F,0x44,0x37,0x04,0x8D,0x20,0x5C,0x3B};

*/

void sub_1A55(unsigned char*);

unsigned int CalcBaudrateDivisor(unsigned long);

ISR(TIMER1_COMPA_vect, ISR_NAKED)
{
	asm volatile
	(
		/* prologue */
		"__reg16__ = 16" "\n"
		"__reg17__ = 17" "\n"
		"	in __zero_reg__, 0x10" "\n"
		"	push __tmp_reg__" "\n"
		"	in __tmp_reg__, __SREG__" "\n"
		"	push __reg16__" "\n"
		"	push __reg17__" "\n"
		/* prologue end  */
		"	lds  __reg16__, KLines_IN_MASK" "\n"
		"	lds  __reg17__, BitCounter" "\n"
		"	cpi __reg17__, 0x09" "\n"
		"	brcc L10" "\n"
		"	cpi __reg17__, 0x01" "\n"
		"	brcc L11" "\n"
		"	and __zero_reg__, __reg16__" "\n"
		"	brne L12" "\n"
		"	rjmp L13" "\n"
		"L11:				" "\n"
		"	lds __reg17__, tempByteFromECU" "\n"
		"	lsr __reg17__" "\n"
		"	and __zero_reg__, __reg16__" "\n"
		"	breq L14" "\n"
		"	ori __reg17__, 0x80" "\n"
		"L14:				" "\n"
		"	sts tempByteFromECU, __reg17__" "\n"
		"	rjmp L13" "\n"
		"L10:				" "\n"
		"	and __zero_reg__, __reg16__" "\n"
		"	breq L12" "\n"
		"	lds __reg17__, tempByteFromECU" "\n"
		"	sts byteFromECU, __reg17__" "\n"
		"	ldi __reg17__, 0x01" "\n"
		"	sts ByteFromECUCompleted, __reg17__" "\n"
		"L12:				" "\n"
		"	ldi __reg17__, 0x02" "\n"
		"	out 0x39, __reg17__" "\n"	//TIMSK
		"	clr __reg17__" "\n"
		"	sbrc __reg16__, 2" "\n"
		"	ori __reg17__, 0x40" "\n"
		"	sbrc __reg16__, 3" "\n"
		"	ori __reg17__, 0x42" "\n"
		"	out 0x3B, __reg17__" "\n"	//GICR
		"	out 0x3A, __reg17__" "\n"	//GIFR
		"L13:					" "\n"
		"	clr __zero_reg__" "\n"
		"	lds __reg17__, BitCounter" "\n"
		"	inc __reg17__" "\n"
		"	sts BitCounter, __reg17__" "\n"
		/* epilogue */
		"	pop	 __reg17__" "\n"
		"	pop	 __reg16__" "\n"
		"	out __SREG__, __tmp_reg__" "\n"
		"	pop	 __tmp_reg__" "\n"
		"	reti" "\n"	
	);

/*
	unsigned char tmpPIND = PIND;
	
	if (BitCounter>=9)
	{
		if ((tmpPIND & KLines_IN_MASK) != 0)
		{
			byteFromECU = tempByteFromECU;
			ByteFromECUCompleted = 1;
		}
	}
	else
	if (BitCounter>=1)
	{
		tempByteFromECU = tempByteFromECU >> 1;
		if ((tmpPIND & KLines_IN_MASK) != 0)
			tempByteFromECU |= 0x80;
		BitCounter++;
		reti();		
	}
	else
	if ((tmpPIND & KLines_IN_MASK) == 0)
	{
		BitCounter++;
		reti();
	}
	
    
	TIMSK = 0x02;
	if (mem_bit_is_set(KLines_IN_MASK,2))
	{
		GICR = 0x40;
		GIFR = 0x40;
	}
	
	if (mem_bit_is_set(KLines_IN_MASK,3))
	{
		GICR = 0x80;
		GIFR = 0x80;
	}
	BitCounter++;	
*/

}


ISR(INT0_vect, ISR_NAKED)
{
	asm volatile
	(
		"__my_tmp_reg__ = 16" "\n"
		/* prologue */
		"	push __my_tmp_reg__" "\n"
		/* prologue end  */ 

		"   lds __my_tmp_reg__, %B0" "\n"
		"   out 0x2D,__my_tmp_reg__" "\n"
		"   lds __my_tmp_reg__, %A0" "\n"
		"   out 0x2C,__my_tmp_reg__" "\n"
		"   ldi __my_tmp_reg__, 0x40" "\n"
		"   out 0x38,__my_tmp_reg__" "\n"
		"   sbic 0x10, 2" "\n"
		"	rjmp L2" "\n"
		"	ldi	__my_tmp_reg__, 0x42" "\n"
		"   out 0x39, __my_tmp_reg__" "\n"
		"   out 0x3B, __zero_reg__" "\n"
		"	sts tempByteFromECU, __zero_reg__" "\n"
		"	sts BitCounter, __zero_reg__" "\n"
		/* epilogue */
		"L2:				   " "\n"
		"	pop __my_tmp_reg__" "\n"
		"	reti" "\n"
		/* epilogue end */
		:: "m" (TempTCNT1)
	);

/*
	TCNT1 = TempTCNT1;
	TIFR = 0x40;
	if (bit_is_clear(PIND,INT0))
	{
		TIMSK = 0x42;
		GICR = 0x00;
		tempByteFromECU = 0x00;
		BitCounter = 0x00;
	}
*/
}




ISR(INT1_vect, ISR_NAKED)
{	
	asm volatile
	(
		"__my_tmp_reg__ = 16" "\n"
		/* prologue */
		"	push __my_tmp_reg__" "\n"
		/* prologue end  */ 

		"   lds __my_tmp_reg__, %B0" "\n"
		"   out 0x2D,__my_tmp_reg__" "\n"
		"   lds __my_tmp_reg__, %A0" "\n"
		"   out 0x2C,__my_tmp_reg__" "\n"
		"   ldi __my_tmp_reg__, 0x40" "\n"
		"   out 0x38,__my_tmp_reg__" "\n"
		"   sbic 0x10, 3" "\n"
		"	rjmp L1" "\n"
		"	ldi	__my_tmp_reg__, 0x42" "\n"
		"   out 0x39, __my_tmp_reg__" "\n"
		"   out 0x3B, __zero_reg__" "\n"
		"	sts tempByteFromECU, __zero_reg__" "\n"
		"	sts BitCounter, __zero_reg__" "\n"
		/* epilogue */
		"L1:				   " "\n"
		"	pop __my_tmp_reg__" "\n"
		"	reti" "\n"
		/* epilogue end */
		:: "m" (TempTCNT1)
	);

/*    TCNT1 = TempTCNT1;
	TIFR = 0x40;
	if (bit_is_clear(PIND,INT1))
	{
		TIMSK = 0x42;
		GICR = 0x00;
		tempByteFromECU = 0x00;
		BitCounter = 0x00;
	}
*/
}

unsigned long LoadDWORDFromPTRToTemp(unsigned long *Ptr)
{
	return ((unsigned long)*Ptr);
}

unsigned long MulTempBy16(unsigned long tmp)
{
	return (tmp * 16);
}

unsigned long DIVdwTempBY32(unsigned long tmp)
{
	return (tmp / 32);
}

unsigned long ADD_TMPBuff_TO_dwTemp(unsigned long tmp, unsigned long Src)
{
	return (tmp + Src);
}

unsigned long ADDdwConstTOdwTemp(unsigned long Arg, unsigned long CONSTANT)
{
	return (Arg + CONSTANT);
}

unsigned long Load_DWORD_FromPacketDataTodwTemp(struct sDLONG *Ptr)
{
	return(LoadDWORDFromPTRToTemp(&Ptr->First));
}

unsigned long Load_DWORD_FromPacketData4TodwTemp(struct sDLONG *Ptr)
{
	return(LoadDWORDFromPTRToTemp(&Ptr->Second));
}

unsigned long COPYdwTempTOdwTemp1(unsigned long src)
{
	return src;
}

unsigned long EORdwTempTOdwTemp1(unsigned long First, unsigned long Second)
{
	return  (First | Second);
}

void ADDdwTemp1TOPacketDataANDStoreToPacketData(unsigned long *dest, unsigned long *src )
{
	*dest = *dest + *src;	
}


void CalcHASH1(struct sDLONG* Ptr1,struct sQLONG* Ptr2)
{
	unsigned long temp1,temp2;
	unsigned long constant = 0;
	unsigned char i=32;

	while (i!=0)
	{
		constant += 0x9E3779B9;

	/*	temp1 = MulTempBy16(Load_DWORD_FromPacketData4TodwTemp(Ptr1));
		temp1 = ADD_TMPBuff_TO_dwTemp(temp1, Ptr2->First );
		temp2 = COPYdwTempTOdwTemp1(temp1);
		temp1 = Load_DWORD_FromPacketData4TodwTemp(Ptr1);
		temp1 = ADDdwConstTOdwTemp(temp1,constant);
		temp2 = EORdwTempTOdwTemp1(temp2,temp1);

		temp1 = DIVdwTempBY32(Load_DWORD_FromPacketData4TodwTemp(Ptr1));
		temp1 = ADD_TMPBuff_TO_dwTemp(temp1,Ptr2->Second);
		temp2 = EORdwTempTOdwTemp1(temp2,temp1);
		ADDdwTemp1TOPacketDataANDStoreToPacketData(&Ptr1->First, &temp2);
		
		temp1 = MulTempBy16(Load_DWORD_FromPacketDataTodwTemp(Ptr1));
		temp1 = ADD_TMPBuff_TO_dwTemp(temp1, Ptr2->Third);
		temp2 = COPYdwTempTOdwTemp1(temp1);

		temp1 = Load_DWORD_FromPacketDataTodwTemp(Ptr1);
		temp1 = ADDdwConstTOdwTemp(temp1,constant);
		temp2 = EORdwTempTOdwTemp1(temp2,temp1);

		temp1 = DIVdwTempBY32(Load_DWORD_FromPacketDataTodwTemp(Ptr1));
		temp1 = ADD_TMPBuff_TO_dwTemp(temp1, Ptr2->Fourth);
		temp2 = EORdwTempTOdwTemp1(temp2,temp1);
		ADDdwTemp1TOPacketDataANDStoreToPacketData(&Ptr1->Second, &temp2);
	*/
		
		temp1 = Ptr1->Second;
		temp1 = temp1 * 16;
		temp1 = temp1 + Ptr2->First;
		temp2 = temp1;
		temp1 = Ptr1->Second;
		temp1 = temp1 + constant;
		temp2 = temp2 ^ temp1;

		temp1 = Ptr1->Second;
		temp1 = temp1 / 32;
		temp1 = temp1 + Ptr2->Second;
		temp2 = temp2 ^ temp1;
		Ptr1->First = Ptr1->First + temp2;


		temp1 = Ptr1->First;
		temp1 = temp1 * 16;
		temp1 = temp1 + Ptr2->Third;
		temp2 = temp1;
		temp1 = Ptr1->First;
		temp1 = temp1 + constant;
		temp2 = temp2 ^ temp1;

		temp1 = Ptr1->First;
		temp1 = temp1 / 32;
		temp1 = temp1 + Ptr2->Fourth;
		temp2 = temp2 ^ temp1;
		Ptr1->Second = Ptr1->Second + temp2;

		i--;
	}
}

void CalcHASH2(struct sDLONG* Ptr1,struct sQLONG* Ptr2)
{
	unsigned long temp1,temp2;
	unsigned long constant = 0xC6EF3720;
	unsigned char i=32;

	while (i!=0)
	{	
/*		temp1 = MulTempBy16(Load_DWORD_FromPacketDataTodwTemp(Ptr1));
		temp1 = ADD_TMPBuff_TO_dwTemp(temp1,Ptr2->Third);
		temp2 = COPYdwTempTOdwTemp1(temp1);
		temp1 = Load_DWORD_FromPacketDataTodwTemp(Ptr1);
		temp1 = ADDdwConstTOdwTemp(temp1,constant);
		temp2 = EORdwTempTOdwTemp1(temp2,temp1);

		temp1 = DIVdwTempBY32(Load_DWORD_FromPacketDataTodwTemp(Ptr1));
		temp1 = ADD_TMPBuff_TO_dwTemp(temp1,Ptr2->Fourth);
		temp2 = EORdwTempTOdwTemp1(temp2,temp1);
		SUB_dwTemp1FROMPacketData(&Ptr1->Second, &temp2);

		temp1 = MulTempBy16(Load_DWORD_FromPacketData4TodwTemp(Ptr1));
		temp1 = ADD_TMPBuff_TO_dwTemp(temp1, Ptr2->First );
		temp2 = COPYdwTempTOdwTemp1(temp1);
		temp1 = Load_DWORD_FromPacketData4TodwTemp(Ptr1);
		temp1 = ADDdwConstTOdwTemp(temp1,constant);
		temp2 = EORdwTempTOdwTemp1(temp2,temp1);

		temp1 = DIVdwTempBY32(Load_DWORD_FromPacketData4TodwTemp(Ptr1));
		temp1 = ADD_TMPBuff_TO_dwTemp(temp1,Ptr2->Second);
		temp2 = EORdwTempTOdwTemp1(temp2,temp1);
		SUB_dwTemp1FROMPacketData(&Ptr1->First, &temp2);
*/
		temp1 = Ptr1->First;
		temp1 = temp1 * 16;
		temp1 = temp1 + Ptr2->Third;
		temp2 = temp1;
		temp1 = Ptr1->First;
		temp1 = temp1 + constant;
		temp2 = temp2 ^ temp1;

		temp1 = Ptr1->First;
		temp1 = temp1 / 32;
		temp1 = temp1 + Ptr2->Fourth;
		temp2 = temp1 ^ temp2;
		Ptr1->Second = Ptr1->Second - temp2;
		
		temp1 = Ptr1->Second;
		temp1 = temp1 * 16;
		temp1 = temp1 + Ptr2->First;
		temp2 = temp1;
		temp1 = Ptr1->Second;
		temp1 = temp1 + constant;
		temp2 = temp2 ^ temp1;

		temp1 = Ptr1->Second;
		temp1 = temp1 / 32;
		temp1 = temp1 + Ptr2->Second;
		temp2 = temp1 ^ temp2;
		Ptr1->First = Ptr1->First - temp2;
	
		constant -= 0x9E3779B9;
		i--;
	}
}

void SetupPORTs()
{
	TIMSK &= 0x17;
	TCCR1B = 0x00;
	GICR = 0x00;
	TIFR |= 0x40;
	GIFR = 0xC0;
	sei();
	unsigned char i = PORTA;
    i &= 0x80;
	i |= 0x6F;
	PORTA = i;
	DDRA = 0xFF;
	PORTB &= 0x10;
	DDRB = 0xB8;
	PORTC = 0xE0;
	DDRC = 0xE0;
	PORTD = 0x33;
	DDRD = 0x32;
	LED_PORT = 0x03;
	DDRE = 0x03;
	UCSR1B = 0x00;	
}


void SetK1K2MaskRegs(unsigned char Mask)
{
	KLines_OUT_MASK = 0;
	KLines_IN_MASK = 0;
	
	if ((Mask & 0x01)==1)
	{
		KLines_OUT_MASK = _BV(5);		
		KLines_IN_MASK  = _BV(2);
	};
	
	if ((Mask & 0x02)==2)
	{
		KLines_OUT_MASK |= _BV(6);
		KLines_IN_MASK |= _BV(3);
	};
}


void Set_USB_K_Communication()
{
	if (mem_bit_is_set(KLines_OUT_MASK,5))
		bit_clear(PORTA,PA2);
	else
		bit_set(PORTA,PA2);

	if (mem_bit_is_set(KLines_OUT_MASK,6))
		bit_clear(PORTA,PA3);
	else
		bit_set(PORTA,PA3);

	bit_clear(PORTA,PA4);

	if (mem_bit_is_set(KLines_IN_MASK,2))
		bit_clear(PORTA,PA0);
	else
		bit_set(PORTA,PA0);

	if (mem_bit_is_set(KLines_IN_MASK,3))
		bit_clear(PORTA,PA1);
	else
		bit_set(PORTA,PA1);
}


void Set_Interface_K_Communication()
{
	bit_set(PORTA,PA2);
	bit_set(PORTA,PA3);

	if (mem_bit_is_set(KLines_OUT_MASK,5))
		bit_clear(PORTA,PA5);
	else
		bit_set(PORTA,PA6);
	
	if (mem_bit_is_set(KLines_OUT_MASK,6))
		bit_clear(PORTA,PA6);
	else
		bit_set(PORTA,PA6);

	if (mem_bit_is_set(KLines_IN_MASK,2))
		bit_clear(PORTA,PA0);
	else
		bit_set(PORTA,PA0);
	
	if (mem_bit_is_set(KLines_IN_MASK,3))
		bit_clear(PORTA,PA1);
	else
		bit_set(PORTA,PA1);

	bit_set(PORTA,PA4);
}


unsigned int EstimateBaudrate(unsigned char mask)
{
	ErrCodeToPC = 3;
	unsigned char tmpTIMSK = TIMSK;
	TCCR1B = 0x03;						// DIV:64, Mode:0(Normal)
	TCNT1 = 0;
	TIFR |= _BV(TOV1);
	unsigned int wMask1 = mask & 0x0001;
	unsigned int wMask2 = mask & 0x0002;
	unsigned char KINMask = KLines_IN_MASK;
	unsigned char KOUTMask = KLines_OUT_MASK;
	unsigned char tmpPIND;
	unsigned int volatile TNCTBuff[9];

	do
	{	
		do
		{
			tmpPIND = PIND;
			if ( (wMask1!=0) && ((tmpPIND&0x02)==0) )
			{
				KOUTMask= _BV(5);
				KINMask = _BV(2);
				break;
			};

			if ( (wMask2!=0) && ((tmpPIND&0x04)==0)  )
			{
				KOUTMask= _BV(6);
				KINMask = _BV(3);
				break;
			};

			if (bit_is_set(TIFR,TOV1))
			{
				KLines_IN_MASK = KINMask;
				KLines_OUT_MASK = KOUTMask;
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
		}while (1);
	} while( (PIND&KINMask)!=0 );

	KLines_IN_MASK = KINMask;
	KLines_OUT_MASK = KOUTMask;
	cli();
	
	TCCR1B = 0x02;					// DIV:8, Mode:0(Normal)
	TCNT1 = 0;
	SFIOR |= _BV(PSR310);
	
	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)==0 );

		TNCTBuff[0] = TCNT1;

	} while((PIND&KINMask)==0);

	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)!=0 );

		TNCTBuff[1] = TCNT1;

	} while((PIND&KINMask)!=0);

	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)==0 );

		TNCTBuff[2] = TCNT1;

	} while((PIND&KINMask)==0);

	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)!=0 );

		TNCTBuff[3] = TCNT1;

	} while((PIND&KINMask)!=0);

	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;				
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)==0 );

		TNCTBuff[4] = TCNT1;

	} while((PIND&KINMask)==0);


	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)!=0 );

		TNCTBuff[5] = TCNT1;

	} while((PIND&KINMask)!=0);

	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)==0 );

		TNCTBuff[6] = TCNT1;

	} while((PIND&KINMask)==0);

	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)!=0 );

		TNCTBuff[7] = TCNT1;

	} while((PIND&KINMask)!=0);

	do
	{	
		do
		{
			tmpPIND = PIND;
			if (bit_is_set(TIFR,TOV1))
			{
				TempTCNT1 = 0;
				tempOCR1A = 0;
				KLines_IN_MASK = 0;
				KLines_OUT_MASK = 0;
				TCCR1B = 0;
				TIMSK = tmpTIMSK;
				return 0;
			};
				
		} while( (tmpPIND&KINMask)==0 );

		TNCTBuff[8] = TCNT1;

	} while((PIND&KINMask)==0);

	TCCR1B = 0;
	TIMSK = tmpTIMSK;
	unsigned int TCNT1st = TNCTBuff[0];

	unsigned int tmp = TNCTBuff[8] - TCNT1st;
	
	tempOCR1A = tmp;

	unsigned int AvgBitTime = tmp / 8;
	unsigned int AvgHalfBitTime = tmp / 16;

	unsigned int *BufPtr = (unsigned int*)TNCTBuff + 1;			//from 2nd TCNT sample
	unsigned int Val;
	unsigned int ValSub;
	

	do
	{
		Val = *BufPtr;
		ValSub = Val - TCNT1st;
		if ( (ValSub<(AvgBitTime-AvgHalfBitTime)) && (Val>(AvgBitTime+AvgHalfBitTime)))
		{
			ErrCodeToPC = 9;
			TempTCNT1 = 0;
			tempOCR1A = 0;
			KLines_IN_MASK = 0;
			KLines_OUT_MASK = 0;
			TCCR1B = 0;
			TIMSK = tmpTIMSK;
			return 0;
		};
		BufPtr += 2;
		TCNT1st = Val;
	}while(BufPtr!=&TNCTBuff[9]);

	tempOCR1A = tmp - 1;
	return 0;
}


unsigned long CalcBaudFromTMR1Tick()
{
	if (tempOCR1A==0)
		return 0;
	
	return (dw_FOSC/(tempOCR1A-1));
}


void SetTMR1BaudrateDivisor()
{
	TIMSK &= ~_BV(TOIE1) & ~_BV(OCIE1A) & ~_BV(OCIE1B) & ~_BV(TICIE1);
	TCCR1B = 0x09;					//CLK/1, Mode:4(CTC)

	if (tempOCR1A<59)
		tempOCR1A = 59;
	
	 OCR1A = tempOCR1A;
	 TempTCNT1 = (tempOCR1A / 2) + 26;

	if (mem_bit_is_set(KLines_IN_MASK, 2))
	{
		MCUCR |= _BV(ISC01);
		MCUCR &= ~_BV(ISC00);
		GICR |= _BV(INT0);
	};

	if (mem_bit_is_set(KLines_IN_MASK, 3))
	{
		MCUCR |= _BV(ISC11);
		MCUCR &= ~_BV(ISC10);
		GICR |= _BV(INT1);
	};

	GIFR = 0xC0;
}


void SendByteToLINE(unsigned char c)
{
	TIMSK &= ~_BV(TOIE1) & ~_BV(OCIE1A) & ~_BV(OCIE1B) & ~_BV(TICIE1);
	GIFR = 0xC0;

	unsigned int ByteToSend = c | 0x0100;
	unsigned char tmpPORTA = PORTA;

	tmpPORTA &= ~KLines_OUT_MASK;
	TCNT1 = 0;
	PORTA = tmpPORTA;
	TIFR |= _BV(OCF1A);

	do
	{
		if ((ByteToSend&0x01)==1)
			tmpPORTA |= KLines_OUT_MASK;
		else
			tmpPORTA &= ~KLines_OUT_MASK;
	
		ByteToSend = ByteToSend >> 1;

		loop_until_bit_is_clear(TIFR,OCF1A);
		PORTA = tmpPORTA;
		TIFR |= _BV(OCF1A);
	} while(ByteToSend!=0);

	loop_until_bit_is_clear(TIFR,OCF1A);

	if (mem_bit_is_set(KLines_IN_MASK, 2))
	{
		MCUCR |= _BV(ISC01);
		MCUCR &= ~_BV(ISC00);
		GICR |= _BV(INT0);
	};

	if (mem_bit_is_set(KLines_IN_MASK, 3))
	{
		MCUCR |= _BV(ISC11);
		MCUCR &= ~_BV(ISC10);
		GICR |= _BV(INT1);
	};

	GIFR = 0xC0;
}


void SendToECUReadFromUART0(unsigned char c)
{
	TIMSK &= ~_BV(TOIE1) & ~_BV(OCIE1A) & ~_BV(OCIE1B) & ~_BV(TICIE1);
	GICR = 0;
	GIFR = 0xC0;

	unsigned int ByteToSend = c | 0x0100;
	unsigned char tmpPORTA = PORTA;

	tmpPORTA &= ~KLines_OUT_MASK;
	TCNT1 = 0;
	PORTA = tmpPORTA;
	TIFR |= _BV(OCF1A);
	unsigned char BuffPtr = BytesInECUPCBuff; 

	do
	{
		if ((ByteToSend&0x01)==1)
			tmpPORTA |= KLines_OUT_MASK;
		else
			tmpPORTA &= ~KLines_OUT_MASK;
	
		ByteToSend = ByteToSend >> 1;

		if (bit_is_set(UCSR0A, RXC0))
		{
			BuffPtr++;
			unsigned char cntr = BuffPtr & 0x1F; 
			BuffPtr -= 2;
			if (cntr!=ECUPCBuffPtr)
			{
				ECUBuffer[BuffPtr++] = UCSR0A;
				ECUBuffer[BuffPtr] = UDR0;
				BuffPtr = cntr;
			}
			else
			{
				unsigned char tmp = UDR0;
			};
		};

		loop_until_bit_is_clear(TIFR,OCF1A);
		PORTA = tmpPORTA;
		TIFR |= _BV(OCF1A);
	} while(ByteToSend!=0);

	BytesInECUPCBuff = BuffPtr;

	loop_until_bit_is_clear(TIFR,OCF1A);

	if (mem_bit_is_set(KLines_IN_MASK, 2))
	{
		MCUCR |= _BV(ISC01);
		MCUCR &= ~_BV(ISC00);
		GICR |= _BV(INT0);
	};

	if (mem_bit_is_set(KLines_IN_MASK, 3))
	{
		MCUCR |= _BV(ISC11);
		MCUCR &= ~_BV(ISC10);
		GICR |= _BV(INT1);
	};

	GIFR = 0xC0;
}


unsigned char CheckIfRcvdByteFromECU(unsigned char *byte)
{
	if (ByteFromECUCompleted==0)
		return 0;

	*byte = byteFromECU;
	ByteFromECUCompleted = 0;
	return 1;
}

void Set5BPSBaudrate()
{
	TIMSK &= ~_BV(TOIE1) & ~_BV(OCIE1A) & ~_BV(OCIE1B) & ~_BV(TICIE1);
	TCCR1B = 0x0B;							//CLK/64, Mode:4(CTC)

	tempOCR1A = CalcBaudrateDivisor(320) - 1;
	OCR1A = tempOCR1A;
	TCNT1 = tempOCR1A / 2;

	if (mem_bit_is_set(KLines_IN_MASK, 2))
	{
		MCUCR |= _BV(ISC01);
		MCUCR &= ~_BV(ISC00);
		GICR |= _BV(INT0);
	};

	if (mem_bit_is_set(KLines_IN_MASK, 3))
	{
		MCUCR |= _BV(ISC11);
		MCUCR &= ~_BV(ISC10);
		GICR |= _BV(INT1);
	};

	GIFR = 0xC0;
}


void SetECUBaudrate(unsigned long baudrate)
{
	tempOCR1A = CalcBaudrateDivisor(baudrate) - 1;
}


void Send5BPSaddress(unsigned char addr)
{
	unsigned int ECUAddr = addr | 0x0100;
	unsigned int BaudDivisor;
	
	TCCR1B = 0x0B;

	BaudDivisor = CalcBaudrateDivisor(InitECUBaudrate * TCCR1_CKDiv);
	
	OCR1A = BaudDivisor - 1;

	unsigned char tmpTIMSK = TIMSK;
	unsigned char tmpPORTA = PORTA;
	tmpPORTA &= ~_BV(PA5) & ~_BV(PA6);
	TIMSK = 0;
	TCNT1 = 0;
	PORTA = tmpPORTA;
	TIFR |= _BV(OCF1A);
	do
	{
		if ((ECUAddr&0x01)==1)
			tmpPORTA |= (_BV(PA5) | _BV(PA6));
		else
			tmpPORTA &= (~_BV(PA5) & ~_BV(PA6));
	
		ECUAddr = ECUAddr >> 1;

		loop_until_bit_is_clear(TIFR,OCF1A);
		PORTA = tmpPORTA;
		TIFR |= _BV(OCF1A);
	} while(ECUAddr!=0);

	loop_until_bit_is_clear(TIFR,OCF1A);
	TCCR1B = 0;
	TIMSK = tmpTIMSK;
}


unsigned int WaitForByteFromECU(unsigned char timeout, unsigned char* c)
{
	struct Timerstruct volatile Timer1;
	ErrCodeToPC = 4;
	SetTimeDelay(&Timer1, timeout);

	do
	{
		if (ByteFromECUCompleted!=0)
		{
			*c = byteFromECU;
			ByteFromECUCompleted = 0;
			return 1;	
		}
	}while(CheckTimerDelay(&Timer1)==0);
	return 0;
}


void MPC2515WriteReg(unsigned char RegAddr, unsigned char RegVal)
{
	PORTD &= ~_BV(PD5);
	asm("nop");
	SPDR = 0x02;			//MPC2515 Instruction 0x02 - Write
	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = RegAddr;
	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = RegVal;
	loop_until_bit_is_clear(SPSR, SPIF);
	PORTD |= _BV(PD5);
	asm("nop");
}

unsigned char MPC2515ReadReg(unsigned char RegAddr)
{
	PORTD &= ~_BV(PD5);
	asm("nop");
	SPDR = 0x03;			//MPC2515 Instruction 0x03 - Read
	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = RegAddr;
	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = 0x00;
	loop_until_bit_is_clear(SPSR, SPIF);
	unsigned char regval = SPDR;
	PORTD |= _BV(PD5);
	asm("nop");
	return(regval);
}


unsigned int MPC2515ReadStatusReg()
{
	PORTD &= ~_BV(PD5);
	asm("nop");
	SPDR = 0xA0;			//MPC2515 Instruction 0xA0 - ReadStatus
	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = 0x00;
	loop_until_bit_is_clear(SPSR, SPIF);
	unsigned char status = SPDR;
	PORTD |= _BV(PD5);
	asm("nop");
	return(status);
}

void SoftResetMPC()
{
	SPCR &= ~_BV(SPE);
	SPCR = 0x5C;
	SPSR = 1;
	PORTD &= ~_BV(PD5);
	__asm__ __volatile__ ("nop");
	SPDR = 0xC0;
	loop_until_bit_is_set(SPSR,SPIF);
	PORTD |= _BV(PD5);
	delay_loop(0x80);

}


unsigned int CANMode_SetMPC2515ToConfigMode()
{
	MPC2515WriteReg(0x0F, 0x80);
	if ( (MPC2515ReadReg(0x0E) & 0xE0)!=0x80);
		return 0;
	
	return 1;
}

void CANMode_WriteCNFRegs(unsigned char CNF1, unsigned char CNF2, unsigned char CNF3 )
{
	MPC2515WriteReg(0x28, CNF3);
	MPC2515WriteReg(0x29, CNF2);
	MPC2515WriteReg(0x2A, CNF1);
}


unsigned int CANMode_WriteRXMaskRegs(unsigned char RegNo, unsigned char *MaskValuesPtr)
{
	unsigned char MaskRegAddr;
	
	switch (RegNo)
	{
		case 0:
			MaskRegAddr=0x20;
			break;
		case 1:
			MaskRegAddr=0x24;
			break;
		default:
			return 0;
	}
	
	MPC2515WriteReg(MaskRegAddr++, *MaskValuesPtr++);
	MPC2515WriteReg(MaskRegAddr++, *MaskValuesPtr++);
	MPC2515WriteReg(MaskRegAddr++, *MaskValuesPtr++);
	MPC2515WriteReg(MaskRegAddr++, *MaskValuesPtr++);
	return 1;
}


unsigned int CANMode_WriteRXFilterRegs(unsigned char RegNo, unsigned char *FilterValuesPtr)
{
	unsigned char FltrRegAddr;

	switch (RegNo)
	{
		case 0:
			FltrRegAddr=0x00;
			break;
		case 1:
			FltrRegAddr=0x04;
			break;
 		case 2:
			FltrRegAddr=0x08;
			break;
		case 3:
			FltrRegAddr=0x10;
			break;
		case 4:
			FltrRegAddr=0x14;
			break;
		case 5:
			FltrRegAddr=0x18;
			break;
		default:
			return 0;
	}

	MPC2515WriteReg(FltrRegAddr++, *FilterValuesPtr++);
	MPC2515WriteReg(FltrRegAddr++, *FilterValuesPtr++);
	MPC2515WriteReg(FltrRegAddr++, *FilterValuesPtr++);
	MPC2515WriteReg(FltrRegAddr++, *FilterValuesPtr++);
	return 1;
}


unsigned int CANMode_WriteRXBCTRLRegs(unsigned char RegNo, unsigned char Value)
{
	unsigned char CtrlRegAddr;
	
	switch (RegNo)
	{
		case 0:
			CtrlRegAddr=0x60;
			break;
		case 1:
			CtrlRegAddr=0x70;
			break;
		default:
			return 0;
	}

	MPC2515WriteReg(CtrlRegAddr, Value);
	return 1;
}

unsigned int CANMode_SetMPC2515NormalMode()
{
	MPC2515WriteReg(0x0D, 0x07);
	MPC2515WriteReg(0x0C, 0x0F);
	MPC2515WriteReg(0x2B, 0x20);
	MPC2515WriteReg(0x0F, 0x00);
	
	if ( (MPC2515ReadReg(0x0E)&0xE0)==0)
		return 1;

	return 0;
}

unsigned int ReadCANRXBuffers(unsigned char* BuffIDPtr, unsigned char* CANMsgPtr)
{
	unsigned char val;

	unsigned char status = MPC2515ReadStatusReg();

	if ((status&0x01) == 1)
	{
		*BuffIDPtr = 0;
		val = 0x90;					//MPC2515 Instruction - Read RX Buffer 0
	}
	else if ((status&0x02) == 2)
	{
		*BuffIDPtr = 1;
		val = 0x94;					//MPC2515 Instruction - Read RX Buffer 1
	}
	else
		return 0;

	
	PORTD &= ~_BV(PD5);
	asm("nop");

	SPDR = val;								//send Instruction
	loop_until_bit_is_clear(SPSR, SPIF);

	SPDR = 0;
	loop_until_bit_is_clear(SPSR, SPIF);
	*CANMsgPtr++ = SPDR;					//read SIDH

	SPDR = 0;
	loop_until_bit_is_clear(SPSR, SPIF);
	*CANMsgPtr++ = SPDR;					//read SIDL

	SPDR = 0;
	loop_until_bit_is_clear(SPSR, SPIF);
	*CANMsgPtr++ = SPDR;					//read EID8

	SPDR = 0;
	loop_until_bit_is_clear(SPSR, SPIF);
	*CANMsgPtr++ = SPDR;					//read EID0

	SPDR = 0;
	loop_until_bit_is_clear(SPSR, SPIF);
	*CANMsgPtr = SPDR;						//read DLC

	unsigned char cntr = (*CANMsgPtr++) & 0x0F;
	do
	{
		SPDR = 0;
		loop_until_bit_is_clear(SPSR, SPIF);
		*CANMsgPtr++ = SPDR;					//read out data bytes
		cntr--;
	}while(cntr!=0);

	return 1;
}

unsigned int CANMode_SendCANMessage(unsigned char buffID, unsigned char* (CANMsgPtr))
{
	unsigned char mask,val;	

	switch (buffID)
	{
		case 0:
			mask=0x04;		//TXB0CTRL.TXREQ
			val =0x81; 		//MP2515 Instruction 0x81 - Request-To-Send TXB0
			break;
		case 1:
			mask=0x10;		//TXB1CTRL.TXREQ
			val =0x82;		//MP2515 Instruction 0x81 - Request-To-Send TXB1
			break;
 		case 2:
			mask=0x40;		//TXB2CTRL.TXREQ
			val =0x84;		//MP2515 Instruction 0x81 - Request-To-Send TXB2
			break;
		default:
			return 0;
	}

	if ( (MPC2515ReadStatusReg()&mask)!=0)
		return 0;

	PORTD &= ~_BV(PD5);
	asm("nop");

	SPDR = 0x40 | (buffID << 1);

	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = *CANMsgPtr++;						//write SIDH
	loop_until_bit_is_clear(SPSR, SPIF);

	SPDR = *CANMsgPtr++;						//write SIDL
	loop_until_bit_is_clear(SPSR, SPIF);

	SPDR = *CANMsgPtr++;						//write EID8
	loop_until_bit_is_clear(SPSR, SPIF);

	SPDR = *CANMsgPtr++;						//write EID0
	loop_until_bit_is_clear(SPSR, SPIF);

	SPDR = *CANMsgPtr;							//write DLC
	loop_until_bit_is_clear(SPSR, SPIF);

	unsigned char cntr = *CANMsgPtr++ & 0x0F;

	do
	{
		SPDR = *CANMsgPtr++;					//write out data bytes
		loop_until_bit_is_clear(SPSR, SPIF);		
		cntr--;
	}while(cntr!=0);

	PORTD |= _BV(PD5);
	asm("nop");

	PORTD &= ~_BV(PD5);
	asm("nop");
	SPDR = val;
	loop_until_bit_is_clear(SPSR, SPIF);
	PORTD |= _BV(PD5);
	asm("nop");

	return 1;
}


unsigned int CheckCANError(unsigned char volatile *ErrPtr)
{
	if (bit_is_set(PINC,PC2))
		return 0;
	
	if ( (MPC2515ReadReg(0x2C)&_BV(5))==0)			//CANINTF.ERRIF
		return 0;

	*ErrPtr = MPC2515ReadReg(0x2D);					//EFLG Reg
	MPC2515WriteReg(0x2D,0x00);

	PORTD &= ~_BV(PD5);
	asm("nop");
	SPDR = 0x05;								//MPC2515 Instruction 0x05 - Bit manipulation
	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = 0x2C;								//CANINTF
	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = 0x20;								//Mask  00100000b (bit ERRIF)
	loop_until_bit_is_clear(SPSR, SPIF);
	SPDR = 0x00;								//Value 00000000b
	loop_until_bit_is_clear(SPSR, SPIF);
	PORTD |= _BV(PD5);
	asm("nop");
	return 1;
}


void InitTXRXBuffers()
{
	RXBuffPtr = 0;
	TXBuffPtr = 0;
	PacketReceived = 0;
	TXBuffClearToWrite = 0;
	PacketRXError = 0;
	RXBuffChksum = 0;
}

unsigned int sub_8AC(unsigned char* CharToSend)
{
	if (TXBuffClearToWrite==0)
		return 0;
	
	if (TXBuffPtr>=TXPacketLength)
	{
		TXBuffClearToWrite = 0;
		return 0;
	}

	*(CharToSend) = TXBuffer[TXBuffPtr];
	TXBuffPtr++;
	return 1;
}

unsigned int CheckIfPacketIsSended()
{
	if (TXBuffClearToWrite==0)
		return 1;
	else
		return 0;
}

unsigned char* CreatePacket(unsigned char ReplyCMD)
{
	
	loop_until_var_is_nz(TXBuffClearToWrite);

	TXBuffer[0] = 0x4D;			//Reply MAGIC
	TXBuffer[1] = 0x03;
	TXBuffer[2] = ReplyCMD; 

	return &TXBuffer[3];
}


unsigned char CheckPacketRcvdFlag()
{
	return(PacketReceived); 
}


unsigned char* GetRXBuffPtr()
{
	if (PacketReceived==0)
		return 0;
	else
		return RXBuffer;
}


void ClearPacketRcvdFlag()
{
	PacketReceived = 0;
}

void SendPacket(unsigned char* ChksumPtr)
{
	TXPacketLength = (ChksumPtr - TXBuffer) + 1;
	TXBuffer[1] = TXPacketLength;
	unsigned char chksum = 0;
	unsigned char* Ptr = TXBuffer;
	do
	{
		chksum = chksum ^ *(Ptr);
		Ptr++;
	}while(Ptr!=ChksumPtr);

	*(ChksumPtr) = chksum;
	TXBuffClearToWrite = 1;
	TXBuffPtr = 1;
	SendONEChar(TXBuffer[0]);
}


void SendPacketToPC(unsigned char* buffer, unsigned char length, unsigned char TXCMD)
{
	unsigned char chksum;

	loop_until_var_is_nz(TXBuffClearToWrite);

	TXBuffer[0] = 0x4D;			//Reply MAGIC
	chksum = 0x4D;
	TXBuffer[1] = length + 4;
	chksum ^= length + 4;
	TXBuffer[2] = TXCMD;
	chksum ^= TXCMD;

	unsigned char* Dest = TXBuffer + 3;
	unsigned char* Src = buffer;
	while(length!=0)
	{
		chksum = chksum ^ *Src;
		*Dest++ = *Src++;
		length--;
	};

	*Dest = chksum;
	TXBuffClearToWrite = 1;
	TXPacketLength = (Dest - TXBuffer);
	TXBuffPtr = 1;
	SendONEChar(TXBuffer[0]);
}

void ProcessRXInt(unsigned char RXChar)
{

	switch(PacketRXError) 
	{
		case 0:
			if (PacketReceived!=0)
			{
				PacketRXError = 1;
				SetTimeDelay(&RXCharDelay, 7);
 				return;
			}
			break;

		case 1:
			if (CheckTimerDelay(&RXCharDelay)==0)
			{
				SetTimeDelay(&RXCharDelay, 7);
				return;
			}
			
			PacketRXError = 0;

			if (PacketReceived!=0)
			{
				PacketRXError = 1;
				SetTimeDelay(&RXCharDelay, 7);
				return;
			}
			break;
	}

	
	switch(RXBuffPtr)
	{
		case 0:
			if (RXChar!=0x53)
			{
				RXBuffChksum = 0;
				RXBuffPtr = 0;
				PacketRXError = 1;
				SetTimeDelay(&RXCharDelay, 7);
				return;
			};
			break;

		case 1:
			if (CheckTimerDelay(&RXCharDelay)!=0)		//Check timeout between characters
			{
				RXBuffChksum = 0;
				RXBuffPtr = 0;
				return;
			};
			break;
	}

	SetTimeDelay(&RXCharDelay, 3);		//Set Timeout for next received character
	RXBuffer[RXBuffPtr] = RXChar;
	RXBuffChksum = RXBuffChksum ^ RXChar;
	RXBuffPtr++;
	if (RXBuffPtr==128)
	{
		PacketRXError = 1;
		RXBuffChksum = 0;
		RXBuffPtr = 0;
		SetTimeDelay(&RXCharDelay, 7);
		return;
	}

	if ( RXBuffPtr<4)
		return;

	if (RXBuffer[1]!=RXBuffPtr)
		return;

	if (RXBuffChksum!=0)
	{
		PacketRXError = 1;
		SetTimeDelay(&RXCharDelay, 7);
		RXBuffChksum = 0;
		RXBuffPtr = 0;
		return;
	}

	PacketReceived = 1;
	StopTimer(&RXCharDelay);
	RXBuffChksum = 0;
	RXBuffPtr = 0;
	return;
}

void CalcChksum(unsigned int* chksum, unsigned char byte)
{
	unsigned int seed = 0x1021;
	unsigned int temp;
	unsigned char Cnt = 0;

	temp = byte << 8;
	temp = temp ^ *chksum;
		
	do
	{
		if ((temp&0x8000)!=0)
			temp = (temp<<1) ^ seed;
		else
			temp = (temp<<1);

		Cnt++;
	}while(Cnt!=8);
	*chksum = temp;
}


unsigned int CheckEEPROMChksum()
{
	unsigned int chksum = 0x0000;
	unsigned char EEChksumHI, EEChksumLO;
	const uint8_t *EE_Ptr = (const uint8_t *)EE_Name;

	
	do
	{
		CalcChksum(&chksum, eeprom_read_byte((const uint8_t *)EE_Ptr++));
			
	}while(EE_Ptr!=(const uint8_t *)EE_Name+0x10);

	EEChksumHI = eeprom_read_byte((const uint8_t *)EE_ChksumHI);

	if (EEChksumHI!= (chksum>>8)) 
		return 0x0000;
	
	EEChksumLO = eeprom_read_byte((const uint8_t *)EE_ChksumLO);

	if (EEChksumLO!=(unsigned char)chksum)
		return 0x0000;
	else
		return 0x0001;

}


void ResetMPC2515()
{
	unsigned char tmp=0;
	PORTD &= ~_BV(PD4);
	DDRD |= _BV(DD4);
	
	delay_loop(0xFF);

	PORTD |= _BV(PD4);

	do
	{
		if (bit_is_set(PIND,PD4)) break;
		tmp++;
	}while(tmp!=0xFF);
	unk_100107 = tmp;
}

void GenerateQLONGNumber(unsigned char seed)
{
	unsigned char * BufPtr = QLONGBuff;
	unsigned long result;

	result = seed - 0x17D20969;

	do
	{	if (result==0)
			result = 1;
		result = result * 0x00010DCD;
		*BufPtr++ = result;
	}while (BufPtr!=&QLONGBuff[16]);

}

void CheckK1K2Lines(unsigned char* K1state, unsigned char* K2state)
{
	unsigned char K1,K2;

	if (bit_is_clear(PIND,K1_RX))
		K1 = 1;
	else
	{
		PORTA &= ~_BV(PA5);
		Delay_ms(10);
		if (bit_is_set(PIND,K1_RX))
			K1=2;
		else
			K1=0;

		PORTA |= _BV(PA5);
	}

	if (bit_is_clear(PIND,K2_RX))
		K2 = 1;
	else
	{
		PORTA &= ~_BV(PA6);
		Delay_ms(10);
		if (bit_is_set(PIND,K2_RX))
			K2=2;
		else
			K2=0;

		PORTA |= _BV(PA6);
	}

	*K1state = K1;
	*K2state = K2;
}

void sub_A4B(unsigned char seed, unsigned char* InpBuffPtr)
{
	unsigned char *SrcPtr;
	unsigned char *DestPtr, *DLONFGBufPtr;	

	GenerateQLONGNumber(seed);
	unsigned char i = 4;
	do
	{
		DestPtr = InpBuffPtr;
		DLONFGBufPtr = DLONGBuff;
		do
		{
			*DestPtr++ ^= *DLONFGBufPtr++;
		}while(DLONFGBufPtr!=&DLONGBuff[8]);

		CalcHASH1( (struct sDLONG*) InpBuffPtr, (struct sQLONG*) QLONGBuff);

		DestPtr = DLONFGBufPtr - 8;
		SrcPtr = InpBuffPtr;
		do
		{
			*DestPtr++ =  *SrcPtr++;
		}while(DestPtr!=&DLONGBuff[8]);

		i--;
		InpBuffPtr += 8;
	}while(i!=0);

}


void sub_A7A(unsigned char seed, unsigned char* BuffPtr)
{
	unsigned char *SrcPtr;
	unsigned char *DestPtr, *DLONFGBufPtr;	

	GenerateQLONGNumber(seed);
	
	
	unsigned char i = 4;
	do
	{
		DestPtr = byte_10014E;
		SrcPtr = BuffPtr;
		do
		{
			*DestPtr++ = *SrcPtr++;
	
		} while(DestPtr!=&byte_10014E[8]);
	
		CalcHASH2( (struct sDLONG*) BuffPtr, (struct sQLONG*) QLONGBuff);
	
		DestPtr = BuffPtr;
		DLONFGBufPtr = DLONGBuff;
		do
		{
			*DestPtr++ ^= *DLONFGBufPtr++;
		} while(DLONFGBufPtr!=&DLONGBuff[8]);

		DLONFGBufPtr -= 8;
		SrcPtr = byte_10014E;
		do
		{
			*DLONFGBufPtr++ =  *SrcPtr++;
		}while(DestPtr!=&DLONGBuff[8]);

		i--;
		BuffPtr += 8;

	} while(i!=0);
}

unsigned char InitVAGController(unsigned char addr, unsigned char delay, unsigned char* KWLo, unsigned char* KWHi, unsigned char mask)
{
	Send5BPSaddress(addr);
	if (EstimateBaudrate(mask) == 0)
		return 0;

	SetTMR1BaudrateDivisor();
	if (WaitForByteFromECU(7, KWLo)==0)
	{	
		SetupPORTs();
		return 0;
	};

	SetTMR1BaudrateDivisor();
	if (WaitForByteFromECU(7, KWHi)==0)
	{	
		SetupPORTs();
		return 0;
	};

	if (delay==0)
	{
		SendByteToLINE(~(*KWHi));
		Delay(delay);
		SendByteToLINE(~(*KWHi));
		return 1;
	}

	if (delay!=0xFF)
	{
		Delay(delay);
		SendByteToLINE(~(*KWHi));
	}
	
	return 1;
}


void DoECUComm()
{
	unsigned char volatile ECUbyte;
	struct Timerstruct volatile Timer2;
	struct Timerstruct volatile Timer4;
	struct Timerstruct volatile Timer6;
	struct Timerstruct16 volatile Timer8;
	

	StopTimer(&Timer4);
	SetTimeDelay(&Timer2, 3);

	ECUPCBuffPtr = 0;
	BytesInECUPCBuff = 0;
	unsigned long tmpBaud = GETCurrentBaudrate();
	if (unk_100105==0)
	{
		GRN_OFF;
		RED_ON;
	}	
	else
		TURNOFF_LEDS;
	
	InitECUComm();
	SetTimeDelay(&Timer6, 28);

	do
	{
		if (CheckTimerDelay(&Timer4)!=0)
		{
			StopTimer(&Timer4);
			if (unk_100105==0)
			{
				GRN_OFF;
				RED_ON;
			};

			SetTimeDelay(&Timer2, 3);
		};

		if (CheckIfRcvdByteFromECU((unsigned char*)&ECUbyte)==0)
		{
			EncodeByteAndSendToPC(ECUbyte);
			if (unk_100105!=0)
				TURNOFF_LEDS;
			SetTimeDelay(&Timer6, 42);
			if (CheckTimerDelay(&Timer2)!=0)
			{
				StopTimer(&Timer2);
				if (unk_100105==0)
				{
					RED_OFF;
					GRN_ON;
				};
				SetTimeDelay(&Timer4, 3);
			}
		}
	
		if (bit_is_set(UCSR0A, RXC0))
		{
			if (bit_is_set(UCSR0A, FE0))
			{
				unsigned char tmp = UDR0;
ProcessCMDs:	SetupPORTs();
				InitUSART();
				SetupUBRR0Value(tmpBaud);
				SetTimeDelay16(&Timer8, 100);
				do
				{
					if (CheckPacketRcvdFlag()==0)
					{
						RXPacket_t* RXPacketPtr = (RXPacket_t*)GetRXBuffPtr();
						unsigned char PacketDataLen = (RXPacketPtr->PacketLen) - 4;
						unsigned char PacketCMD = RXPacketPtr->PacketCMD;

						if (PacketCMD==0x85)
						{
							if (PacketDataLen!=4)
							{
								ClearPacketRcvdFlag();
								break;
							};
							
							unsigned long baudrate;
							baudrate = (unsigned long) *(&RXPacketPtr->PacketData[0]);
							SetECUBaudrate(baudrate);
							SendPacketToPC(0x0000,0,0xFE);
							do {} while(CheckIfPacketIsSended()==0);
							ClearPacketRcvdFlag();
							break;
						}
						else if (PacketCMD==0x08)
						{
							if (PacketDataLen!=0)
							{
								ClearPacketRcvdFlag();
								break;
							};
							SendPacketToPC(0x0000,0,0xFE);
							do {} while(CheckIfPacketIsSended()==0);
							Delay(3);
							cli();
							asm("jmp 0x0000");						//RESET processor
						}
						else
						{
							ClearPacketRcvdFlag();
							break;
						};
					};
				}while(CheckTimerDelay16(&Timer8)==0);

				ECUPCBuffPtr = 0;
				BytesInECUPCBuff = 0;
				InitECUComm();
				SetTMR1BaudrateDivisor();
			}
			else
			{
				unsigned char tmpUDR0 = UDR0;
				if (unk_100105!=0)
				{
					RED_ON;
					GRN_ON;
				};

				SendToECUReadFromUART0(tmpUDR0);
				EncodeByteAndSendToPC(tmpUDR0);

				do
				{
					tmpUDR0 = ECUBuffer[ECUPCBuffPtr++];
					if ( (tmpUDR0&0x10)==0x10)
						goto ProcessCMDs;
					tmpUDR0 = ECUBuffer[ECUPCBuffPtr++];
					SendToECUReadFromUART0(tmpUDR0);
					EncodeByteAndSendToPC(tmpUDR0);					

				} while(ECUPCBuffPtr!=BytesInECUPCBuff);
				
				if (unk_100105!=0)
					TURNOFF_LEDS;
				SetTimeDelay(&Timer6, 42);
				
				if (CheckTimerDelay(&Timer2)!=0)
				{
					StopTimer(&Timer2);
					if (unk_100105==0)
					{
						RED_OFF;
						GRN_ON;
					};
					
					SetTimeDelay(&Timer4, 3);
				}
			};
		};
	}while(CheckTimerDelay(&Timer6)==0);

	SetupPORTs();
	InitUSART();
	SetupUBRR0Value(tmpBaud);
}


unsigned int CalcBaudrateDivisor(unsigned long baudrate)
{
	unsigned long volatile tmpUBRR, tmpUBRR_rem;

	tmpUBRR = dw_FOSC / baudrate;
	tmpUBRR_rem = dw_FOSC % baudrate;

	if ( ( baudrate / 2) >= tmpUBRR_rem )
		tmpUBRR -= 1;

	return(tmpUBRR);
}

unsigned int sub_CBC()
{

}

void DoCANMode()
{
	struct Timerstruct volatile Timer2, Timer4;
	unsigned char volatile CANError;
	

	StopTimer(&Timer4);
	SetTimeDelay(&Timer2, 3);
	unsigned char CANCfgMode = 1;

	do
	{
		if (CheckTimerDelay(&Timer4)!=0)
		{
			StopTimer(&Timer4);	
			GRN_OFF;
			RED_ON;
			SetTimeDelay(&Timer4, 3);
		};

		if (CANCfgMode==0)
		{
			if (ReadCANRXBuffers(&CANMsgBuffID, CANMsgBuff)!= 0)
			{
				unsigned char *XORTblPtr = CANMsgXORTable;
				unsigned char *CANBuffPtr = CANMsgBuff;
				unsigned char Cntr = 0;
				unsigned char CANMsgDLC;
				CANMsgDLC = (CANMsgBuff[DLC]&0x0F) + 6;
				while(Cntr < CANMsgDLC)
				{
					*CANBuffPtr++ ^= XORTblPtr[Cntr];
					Cntr++;
				}

				CANError = Cntr;
				SendPacketToPC(CANMsgBuff, sizeof(CANMsgBuff+1), CANMode_ReceivedMsg);		// 0xB7

				if (CheckTimerDelay(&Timer2)!=0)
				{
					StopTimer(&Timer2);	
					RED_OFF;
					GRN_ON;					
					SetTimeDelay(&Timer4, 3);
				}
			}

			if (CheckCANError(&CANError)!=0)
				SendPacketToPC((unsigned char *)&CANError, sizeof(unsigned char), CANMode_BusError);		// 0xB9
		}

		if (CheckPacketRcvdFlag()!=0)
		{
				RXPacket_t* RXPacketPtr = (RXPacket_t*)GetRXBuffPtr();
				unsigned char PacketDataLen = (RXPacketPtr->PacketLen) - 4;
				unsigned char PacketCMD = RXPacketPtr->PacketCMD;

				unsigned char Ptr = 0;
				unsigned char *PcktDataPtr = (unsigned char *) &RXPacketPtr->PacketData;
				while(Ptr<PacketDataLen)
				{	
					CANRXBuffer[Ptr++] = *PcktDataPtr++;
				}
				ClearPacketRcvdFlag();
				
				switch(PacketCMD)
				{
					case CANMode_SendMsg:		// CMD:0xB8 - send CAN message
					{
						if ( (PacketDataLen<6) || (PacketDataLen>15) || (CANCfgMode==1))
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};
						
						unsigned char Cntr = 0 ;
						while(Cntr < PacketDataLen)
						{
							CANRXBuffer[Cntr] ^= CANMsgXORTable[Cntr];
							Cntr++;
						}

						CANError = Cntr;
						
						if (CANMode_SendCANMessage(CANRXBuffer[0], (CANRXBuffer+1))==0)
						{
							ErrCodeToPC = 0x20;
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}

						if (CheckTimerDelay(&Timer2)!=0)
						{
							StopTimer(&Timer2);
							RED_OFF;
							GRN_ON;
							SetTimeDelay(&Timer4, 3);
							SendPacketToPC(0x0000, 0,0xFE);
							break;
						}

						SendPacketToPC(0x0000, 0,0xFE);
						break;
					}


					case CANMode_ConfigCAN:			// CMD: 0xB1 - switch to CONFIG mode
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						if (CANMode_SetMPC2515ToConfigMode()==0)
						{
							ErrCodeToPC = 0x20;
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;	
						}
						else
						{
							CANCfgMode = 1;
							SendPacketToPC(0x0000, 0,0xFE);
							break;	
						}
					}


					case CANMode_WriteConfig:		// CMD: 0xB2 - write CAN config register
					{
						if (PacketDataLen!=3)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						CANMode_WriteCNFRegs(CANRXBuffer[0], CANRXBuffer[1], CANRXBuffer[2]);
						SendPacketToPC(0x0000, 0,0xFE);
						break;	
					}


					case CANMode_WriteMASK:			// CMD: 0xB3 - write CAN mask register
					{
						if (PacketDataLen!=5)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						if (CANMode_WriteRXMaskRegs(CANRXBuffer[0],(CANRXBuffer+1))==0)
						{
							ErrCodeToPC = 0x20;
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}
						else
						{
							SendPacketToPC(0x0000, 0,0xFE);
							break;
						}
					}


					case CANMode_WriteFILTER:			// CMD: 0xB4 - write CAN filter register
					{
						if (PacketDataLen!=5)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						if (CANMode_WriteRXFilterRegs(CANRXBuffer[0],(CANRXBuffer+1))==0)
						{
							ErrCodeToPC = 0x20;
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}
						else
						{
							SendPacketToPC(0x0000, 0,0xFE);
							break;
						}
					}


					case CANMode_WriteRXCTRL:			// CMD: 0xB5 - write CAN RX Buffer Control register
					{
						if (PacketDataLen!=2)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						if (CANMode_WriteRXBCTRLRegs(CANRXBuffer[0], CANRXBuffer[1])==0)
						{
							ErrCodeToPC = 0x20;
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}
						else
						{
							SendPacketToPC(0x0000, 0,0xFE);
							break;
						}
					}


					case CANMode_NormalMode:			// CMD:0xB6 - switch to NORMAL mode (from CONFIG mode)
					{
						if (PacketDataLen!=16)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unsigned char *BuffPtr = CANRXBuffer;
						unsigned char *XORTblPtr = CANMsgXORTable; 

						do
						{
							*XORTblPtr++ = pgm_read_byte(&rawData[(*BuffPtr++)]);
								
						}while(BuffPtr!=(unsigned char*)&CANRXBuffer[16]);

						if (CANMode_SetMPC2515NormalMode()!=0)
						{
							CANCfgMode = 0;
							SendPacketToPC(0x0000, 0,0xFE);
							break;
						}
						else
						{
							ErrCodeToPC = 0x20;
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}
					}


					case CANMode_StopCANMode:			// CMD: 0xA0 - Stop CANMode and return
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						return;
					}


					default:
					{
						ErrCodeToPC = 1;
						ClearPacketRcvdFlag();
						SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
						break;
					}
				}
		}
	}while(1);
}


unsigned int sub_E63()
{

}

unsigned int sub_FCC()
{

}

unsigned int sub_105F()
{

}























































































































































































//*************************************************************************
// MAIN Function
//*************************************************************************

int main()
{
	unsigned long templong;
	unsigned char PowerOnReset;
	unsigned char PacketLen,PacketDataLen, PacketCMD;
	RXPacket_t *RXPacketPtr;
	unsigned char volatile K1State, K2State;
	unsigned long volatile tmpEE001C;
	unsigned char volatile tmpEE0014;
	struct Timerstruct volatile Timer4;
	struct Timerstruct volatile Timer6;


	if (bit_is_clear(MCUCSR,PORF))
		PowerOnReset=0;
	else
	{
		MCUCSR &= (~_BV(PORF));
		PowerOnReset=1;
	}


	eeprom_read_block(&dw_FOSC,(const uint8_t *)EE_FOSC,sizeof(long));
	if ((dw_FOSC<6635520) || (dw_FOSC>8110080))
		dw_FOSC = 7372800;
		
	ResetMPC2515();
	SetupPORTs();
	InitTXRXBuffers();
	InitUSART();
	InitTimer0();
	SoftResetMPC();

	eeprom_read_block((void*)&tmpEE001C, (const void*)EE_001C,sizeof(long));
	tmpEE001C++;
	eeprom_write_block((const void*)&tmpEE001C,(void*)EE_001C,sizeof(long));
	
	tmpEE0014 = eeprom_read_byte((const uint8_t *)EE_0014);	

	sei();

	RXBuffer[0] = 0x53;
	RXBuffer[1] = 0x06;
	RXBuffer[2] = 0x0B;
	RXBuffer[3] = 0x00;
	RXBuffer[4] = 0x00;
	RXBuffer[5] = 0x5E;

 	goto tet;

//	Delay(7);
	unsigned char tmp = eeprom_read_byte((const uint8_t *)EE_001B);
	switch(tmp)	
	{
	case 1:
	case 2:	
	{
		if (tmp==2)
			SetupUBRR0Value(115200);
		
		if (PowerOnReset!=0)									// tmp = 1
		{
			SetTimeDelay(&Timer4, 4);
			do {} while(CheckTimerDelay(&Timer4)==0);

			unsigned char K1State, K2State;
			CheckK1K2Lines(&K1State,&K2State);
			if ( (K1State!=0) || (K2State!=0) )
			{
				do
				{
					if (CheckTimerDelay(&Timer4)!=0)
					{
						SetTimeDelay(&Timer4, 7);
						if (bit_is_set(LED_PORT,GRN_LED)==1)
							TURNOFF_LEDS;
						else
						{
							GRN_ON;
							RED_OFF;
						}		
					}	
				}while( CheckPacketRcvdFlag()==0 );
			}
		}
	}
	default:
	{
		SetTimeDelay(&Timer4, 4);
		unsigned char flag = 0;
		do
		{
			if (flag!=0)
			{
				if (bit_is_clear(PIND,K1_RX))
				{
					RED_ON;
					GRN_ON;
				}
				else
				{
					if (bit_is_set(PIND,K2_RX))
					{
						GRN_OFF;
						RED_ON;
					}
					else
					{
						RED_ON;
						GRN_ON;
					}
				}
			}
			else
			{
				if (CheckTimerDelay(&Timer4)!=0)
				{
					SetTimeDelay(&Timer4, 7);
					if (bit_is_set(PORTE,GRN_LED))
						TURNOFF_LEDS;
					else
					{
						RED_OFF;
						GRN_ON;
					}

					CheckK1K2Lines(&K1State,&K2State);
					if ( (K1State==0) && (K2State==0) )
					{
						SetK1K2MaskRegs(3);
						Set_USB_K_Communication();
						flag = 1;
					}
				}
			}
		
			if (CheckPacketRcvdFlag()!=0)
			{
				RXPacketPtr = (RXPacket_t*)GetRXBuffPtr();
				PacketLen = RXPacketPtr->PacketLen;
				PacketCMD = RXPacketPtr->PacketCMD;
				ClearPacketRcvdFlag();
				if ((PacketCMD==0xA0) && (PacketLen==4))
					break;
			}
		} while(1);

		SetupPORTs();
		Delay_ms(100);
		SendPacketToPC(0x0000,0,0xFE);
		StopTimer(&Timer4);
	}

	}

	if (eeprom_read_byte((const uint8_t *) EE_0015)==0xAB)
	{
		tmpEE0016 = eeprom_read_word((const void *)EE_0016);
		if (PowerOnReset!=0)
		{
			if (tmpEE0016>0x8000 || tmpEE0016!=0xFFFF)
			{
				if (tmpEE0016>=65450)
					tmpEE0016 = tmpEE0016 - 8;
				else
					tmpEE0016 = tmpEE0016 - 72;
				eeprom_write_word( (uint16_t *)EE_0016, (uint16_t)tmpEE0016);
			}
		}
	}

	unsigned int  FWVersion = eeprom_read_byte((const uint8_t *)EE_VersionMinor);
	unsigned char VerMaj = eeprom_read_byte((const uint8_t *)EE_VersionMajor);
	FWVersion = FWVersion << 8;
	FWVersion = FWVersion + VerMaj;
	if (FWVersion<0x0137)
		unk_100105 = 1;


	//******************************************************
	// Main program loop
	//******************************************************
tet:do
	{
		while(CheckPacketRcvdFlag()==0)
		{
			if (bit_is_clear(PIND,K1_RX))
			{
				RED_ON;
				GRN_ON;
			}
			else
			{
				if (bit_is_set(PIND,K2_RX))
				{
					GRN_OFF;
					RED_ON;
				}
				else
				{
					RED_ON;
					GRN_ON;
				}
			}
		}

		GRN_OFF;
		RED_ON;
		
		RXPacketPtr = (RXPacket_t*)GetRXBuffPtr();
		PacketDataLen = (RXPacketPtr->PacketLen) - 4;

		if (tmpEE0014==0xAB)
		{
			if (RXPacketPtr->PacketCMD>=0x0B)
			{
				ErrCodeToPC = 1;
				ClearPacketRcvdFlag();
				SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);

			}
		}
		else
		{
			switch(RXPacketPtr->PacketCMD)
			{

				case UpdateFW:
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}
						TURNOFF_LEDS;
						SendPacketToPC(0x0000, 0, 0xFE);
						Delay(14);
						asm volatile ("jmp 0x3800");			//Jump to Bootloader at 0x1C00
						break;
					}
			

				case GetVersion:
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}
						
						unsigned char VerMin = eeprom_read_byte((const uint8_t *)EE_VersionMinor);
						VerMaj = eeprom_read_byte((const uint8_t *)EE_VersionMajor);
						unsigned char *TXPacktDataPtr = CreatePacket(GetVersion);
						FWVersion = VerMin << 8;
						FWVersion = FWVersion + VerMaj;
						if (FWVersion<0x015C)
						{
							TXPacktDataPtr[0] = VerMaj;
							TXPacktDataPtr[1] = VerMin;
						}
						else
						{
							TXPacktDataPtr[0] = VERSION_MAJOR;
							TXPacktDataPtr[1] = VERSION_MINOR;
						}
						
						TXPacktDataPtr[2] = 0x44;
						SendPacket(&TXPacktDataPtr[3]);
						ClearPacketRcvdFlag();
						break;
					}


				case SetBaudrate:
					{
						if (PacketDataLen!=4)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}

								templong = GETCurrentBaudrate();
								unsigned long tmpBaud = (unsigned long) *(&RXPacketPtr->PacketData[0]);
						ClearPacketRcvdFlag();
						if (CheckBaudrateUBRRValue(tmpBaud)==0)
						{
							ErrCodeToPC = 7;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}
						SendPacketToPC(0x0000, 0, 0xFE);
						do {} while(CheckIfPacketIsSended()==0);
						Delay(7);
						SetupUBRR0Value(tmpBaud);
						SendPacketToPC(0x0000, 0, 0xFD);
						SetTimeDelay(&Timer4,14);
						while(1)
						{
							if (CheckPacketRcvdFlag()!=0)
							{
								unsigned char* Ptr = GetRXBuffPtr();
								ClearPacketRcvdFlag();
								if (*(Ptr+2)==0xFE)
								{
									StopTimer(&Timer4);
									ClearPacketRcvdFlag();
									break;
								}

							};

							if (CheckTimerDelay(&Timer4)!=0)
							{
								SetupUBRR0Value(templong);
								ErrCodeToPC = 2;
								ClearPacketRcvdFlag();
								SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
								break;
							};
						}

						break;
					}


				case GetInterfaceID:		// CMD: 0x04 - send EE_InterfaceID and EE_Serial  (Interface ID?)
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						if (CheckEEPROMChksum()==0)
						{
							ErrCodeToPC = 5;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unsigned char *TXPacktDataPtr = CreatePacket(0x04);
						unsigned char *EEAdr = (unsigned char*)EE_Name;

						do
						{
							*(TXPacktDataPtr++) = eeprom_read_byte((const uint8_t *)EEAdr++);
						} while(EEAdr!=(unsigned char *)EE_ChksumHI);
						SendPacket(TXPacktDataPtr);
						
						ClearPacketRcvdFlag();
						break;
					}

				
				case 0x05:
					{
						if (PacketDataLen!=16)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						if (CheckEEPROMChksum()!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						
						unsigned char *EEAdr = (unsigned char *)EE_Name;
						unsigned int chksum = 0;
						unsigned char *Ptr = RXPacketPtr->PacketData;

						do
						{
							eeprom_write_byte((uint8_t *)EEAdr++, (uint8_t)*(Ptr));
							CalcChksum(&chksum, *(Ptr++));

						} while(EEAdr!=(unsigned char*)EE_ChksumHI);

						eeprom_write_byte((uint8_t *)EE_ChksumHI, (uint8_t) (chksum>>8));
						eeprom_write_byte((uint8_t *)EE_ChksumLO, (uint8_t) chksum);
						SendPacketToPC(0x0000, 0, 0xFE);
						ClearPacketRcvdFlag();
						break;
					}


				case SoftReset:				// CMD: 0x08
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						SendPacketToPC(0x0000, 0, 0xFE);

						do {} while(CheckIfPacketIsSended()==0);
						
						Delay(3);
						asm("jmp 0x0000");	
						break;
					}


				case 0x09:
					{
						if (PacketDataLen!=9)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unsigned char SubCMD = RXPacketPtr->PacketData[0];
		
						memcpy_P(QLONGBuff,(PGM_VOID_P) QLONG00B8, 16);
						
						switch(SubCMD)
						{
							case 0x01:
								{
									unsigned char *Ptr = (unsigned char *) &RXPacketPtr->PacketData+1;
									unsigned long *Ptrlong = Ptr;
									CalcHASH1( (struct sDLONG*) Ptr, (struct sQLONG*) QLONGBuff );
									HASH = *Ptrlong;
									SendPacketToPC((unsigned char*)Ptr, 8, 0x09);
									ClearPacketRcvdFlag();
									break;
								}
							case 0x02:
								{
									unsigned char *Ptr = (unsigned char *) &RXPacketPtr->PacketData+1;
									unsigned int *PtrInt = Ptr;
									CalcHASH2( (struct sDLONG*) Ptr, (struct sQLONG*) QLONGBuff );
									*PtrInt ^= tmpEE0016;									
									*(Ptr+2) ^= unk_100107;
									SendPacketToPC((unsigned char*)Ptr, 8, 0x09);
									ClearPacketRcvdFlag();
									break;
								}

							case 0x03:
								{
									unsigned char *Ptr = (unsigned char *) &RXPacketPtr->PacketData+1;
									unsigned int *PtrInt = Ptr;
									CalcHASH2( (struct sDLONG*) Ptr, (struct sQLONG*) QLONGBuff );
									tmpEE0016 = *PtrInt;
									eeprom_write_byte((uint8_t *)EE_0015, 0xAB);
									eeprom_write_word((uint16_t *)EE_0016, tmpEE0016);
									*PtrInt ^= *(PtrInt+1);
									*PtrInt ^= *(PtrInt+2);
									SendPacketToPC((unsigned char*)Ptr,8,0x09);
									ClearPacketRcvdFlag();
									break;
								}

							case 0x04:
								{
									unsigned char *Ptr = (unsigned char*) &RXPacketPtr->PacketData+1;
									CalcHASH2( (struct sDLONG*) Ptr, (struct sQLONG*) QLONGBuff );
									*(Ptr) ^= 1;
									*(Ptr+1) ^= 0x5C;
									*(Ptr+2) ^= eeprom_read_byte((uint8_t *)EE_VersionMinor);
									*(Ptr+3) ^= eeprom_read_byte((uint8_t *)EE_VersionMajor);
									SendPacketToPC((unsigned char*)Ptr,8,0x09);
									ClearPacketRcvdFlag();
									break;
								}

							case 0x05:
								{
									unsigned char *Ptr = (unsigned char*) &RXPacketPtr->PacketData+1;
									CalcHASH2( (struct sDLONG*) Ptr, (struct sQLONG*) QLONGBuff );
									eeprom_write_byte((uint8_t *) EE_VersionMinor, *(Ptr));
									eeprom_write_byte((uint8_t *) EE_VersionMajor, *(Ptr+1));
									*(Ptr) ^= *(Ptr+2);
									*(Ptr+1) ^= *(Ptr+3);
									SendPacketToPC((unsigned char*)Ptr,8,0x09);
									ClearPacketRcvdFlag();
									break;
								}
						}

						break;						
					}


/*				case 0x0A:
					{
						if (PacketDataLen!=8)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};
		
						memcpy_P(QLONGBuff,(PGM_VOID_P) QLONG00C0, 16);

						unsigned char *Ptr = RXPacketPtr->PacketData+1;
						CalcHASH1( (struct sDLONG*) Ptr, (struct sQLONG*) QLONGBuff );
						
						//TODO
					}
*/

				case 0x0B:			// CMD: 0x0B - Get EEPROM KEY
					{
						if (PacketDataLen!=2)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unsigned char SubCMD = RXPacketPtr->PacketData[0];
						unsigned char CMDData1 = RXPacketPtr->PacketData[1];

						if (SubCMD > 9)
						{
							ErrCodeToPC = 6;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}

						unsigned char *EEAdr;
						if (SubCMD != 8)
							EEAdr = (unsigned char*) (0x040 + (SubCMD * 32));
						else
							EEAdr = (unsigned char*) 0x0180;
						
						unsigned char *TXPacktDataPtr = CreatePacket(0x0B);
						unsigned int temp = GetTMR0_OFCounter() << 8;
						unsigned long tmplong = temp | TCNT0;
						unsigned char *Ptr = TXPacktDataPtr;
						unsigned char *Ptr1 = DLONGBuff;

						do
						{
							tmplong *= 0x00010DCD;	
							if (tmplong==0) tmplong=0x00000001;
							*Ptr++ = *Ptr1++ = tmplong;
						}while(Ptr1!=&DLONGBuff[8]);

/*						*(Ptr++) = *(Ptr1++) = 0XB6;
						*(Ptr++) = *(Ptr1++) = 0XBE;
						*(Ptr++) = *(Ptr1++) = 0X26;
						*(Ptr++) = *(Ptr1++) = 0X6E;
						*(Ptr++) = *(Ptr1++) = 0X16;
						*(Ptr++) = *(Ptr1++) = 0X9E;
						*(Ptr++) = *(Ptr1++) = 0X86;
						*(Ptr++) = *(Ptr1++) = 0X4E;*/

						Ptr = TXPacktDataPtr + 8;

						if (SubCMD==9)
						{
							Ptr1 = byte_100256;
							do
							{
								*(Ptr++) = *(Ptr1++);	
							} while(Ptr1!=&byte_100256[32]);
						}
						else
						{
							for (unsigned char i=0;i<32;i++)
								*(Ptr++) = eeprom_read_byte((const uint8_t *)EEAdr++);
						}

						sub_A4B(CMDData1,TXPacktDataPtr + 8);
						SendPacket((TXPacktDataPtr+8)+32);
						ClearPacketRcvdFlag();
						break;						
					}


				case 0x0C:
					{
						if (PacketDataLen!=42)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unsigned char SubCMD = RXPacketPtr->PacketData[0];
						unsigned char CMDData1 = RXPacketPtr->PacketData[1];

						unsigned char* Dest = DLONGBuff;
						unsigned char* Src = &RXPacketPtr->PacketData[2];
						do
						{
							*(Dest) = *(Src);
						} while(Dest!=&DLONGBuff[8]);

						Src = (&RXPacketPtr->PacketData[2]) + 8;
						if (SubCMD>=10)
						{
							ErrCodeToPC = 6;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						}

						sub_A7A(CMDData1,Src);
						
						if(SubCMD==9)
						{
							Dest = byte_100256;
							do
							{
								*Dest++ = *Src++;	
							} while(Dest!=&byte_100256[32]);
							CMD_0C_Executed = 1;
							SendPacketToPC(0x0000, 0, 0xFE);
							ClearPacketRcvdFlag();
							break;
						}
						
						unsigned char *EEAdr;

						if (SubCMD==8)
							EEAdr = (unsigned char*) 0x0180;
						else
							EEAdr = (unsigned char*) 0x0040 + (SubCMD * 32);

						for (unsigned char i=0;i<32;i++)
							eeprom_write_byte((uint8_t *)EEAdr++, (uint8_t) *(Src++));
						
						SendPacketToPC(0x0000, 0, 0xFE);
						ClearPacketRcvdFlag();
						break;
					}					

				case 0x0D:
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};
						
						unsigned char volatile tmpEE001B =  eeprom_read_byte((const uint8_t *)EE_001B);
						SendPacketToPC((uint8_t *)&tmpEE001B, sizeof(unsigned char), 0x0D);
						ClearPacketRcvdFlag();
						break;	
					}

				case 0x0E:
					{
						if (PacketDataLen!=1)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						eeprom_write_byte((uint8_t *)EE_001B, (uint8_t) RXPacketPtr->PacketData[0]);
						loop_until_bit_is_set(EECR, EEWE);
						SendPacketToPC(0x0000, 0, 0xFE);
						ClearPacketRcvdFlag();
						break;
					}


				case 0x16:		//CMD: 0x16 - get EEPROM location 0x001C
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						eeprom_read_block((void *) &tmpEE001C,(void *)EE_001C,sizeof(long));
						SendPacketToPC((uint8_t *)&tmpEE001C, sizeof(unsigned long), 0x16);
						ClearPacketRcvdFlag();
						break;
					}

				
				case GetFOscValue:				// CMD: 0x17
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						SendPacketToPC((uint8_t *)&dw_FOSC, sizeof(unsigned long), 0x17);
						

					}

				case SetFOscValue:				// CMD: 0x18
					{
						if (PacketDataLen!=4)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unsigned long *Ptr = (unsigned long *)RXPacketPtr->PacketData;
						dw_FOSC = *Ptr;

						if ((dw_FOSC<6635520) || (dw_FOSC>8110080))
							dw_FOSC = 7372800;
						
						eeprom_write_block((void*)&dw_FOSC, (void*)EE_FOSC, sizeof(unsigned long));
						SendPacketToPC(0x0000, 0, 0xFE);
						ClearPacketRcvdFlag();
						break;	
					}

	
				case 0x80:
					{
						if (PacketDataLen!=2)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};
						unsigned char SelKL = RXPacketPtr->PacketData[0];
						tmp = RXPacketPtr->PacketData[1];
						SendPacketToPC(0x0000, 0, 0xFE);
						do {}while(CheckIfPacketIsSended()==0);
						unsigned long tmpBaud = GETCurrentBaudrate();
						DisableUSART0();
						SetK1K2MaskRegs(SelKL);
						Set_USB_K_Communication();

						do {}while(tmp!=0);

						Timerstruct_t Timer6;
						SetTimeDelay(&Timer6, 28);

						while(CheckTimerDelay(&Timer6)==0)
						{
							if (bit_is_clear(PIND,K1_RX))
							{
								RED_ON;
								GRN_ON;
							}
							else
							{
								if (bit_is_clear(PIND,K2_RX))
								{
									RED_ON;
									GRN_ON;
								}
								else
									TURNOFF_LEDS;
							}
							
							PORTD |= _BV(PD0);
						};

						SetupPORTs();
						InitUSART();
						SetupUBRR0Value(tmpBaud);
						ClearPacketRcvdFlag();
						break;
					}


				case 0x81:
					{
						if (PacketDataLen!=2)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unsigned char ECUAddress = RXPacketPtr->PacketData[0];
						unsigned char ReplyDelayAfterInit = RXPacketPtr->PacketData[1];
						unsigned char KEYWORDLo, KEYWORDHi;

						if (InitVAGController(ECUAddress, ReplyDelayAfterInit, &KEYWORDLo, &KEYWORDHi, 3)==0)
						{
							ClearPacketRcvdFlag();
							ErrCodeToPC = 1;
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unsigned long EstBaudrate= CalcBaudFromTMR1Tick();
						unsigned char *TXPacktDataPtr = CreatePacket(GetVersion);
						*(TXPacktDataPtr) = EstBaudrate;
						TXPacktDataPtr[4] = KEYWORDLo;
						TXPacktDataPtr[5] = KEYWORDHi;
						SendPacket(&TXPacktDataPtr[6]);

						do {}while(CheckIfPacketIsSended()==0);

						unsigned long tmpBaud = GETCurrentBaudrate();
						DisableUSART0();
						Set_USB_K_Communication();

						SetTimeDelay(&Timer6, 28);

						while(CheckTimerDelay(&Timer6)==0)
						{
							if (bit_is_clear(PIND,K1_RX))
							{
								RED_ON;
								GRN_ON;
							}
							else
							{
								if (bit_is_clear(PIND,K2_RX))
								{
									RED_ON;
									GRN_ON;
								}
								else
									TURNOFF_LEDS;
							}
							
							PORTD |= _BV(PD0);
						};

						SetupPORTs();
						InitUSART();
						SetupUBRR0Value(tmpBaud);
						ClearPacketRcvdFlag();
						break;
					}


				case KLineTest:			// CMD: 0x82
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};
						
						unsigned char K1LineState, K2LineState;
						CheckK1K2Lines(&K1LineState, &K2LineState);
						unsigned char *TXPacktDataPtr = CreatePacket(KLineTest);
						*(TXPacktDataPtr++) = K1LineState;
						*(TXPacktDataPtr++) = K2LineState;
						SendPacket(TXPacktDataPtr);

						do {}while(CheckIfPacketIsSended()==0);
						ClearPacketRcvdFlag();
						break;
					}

				case VAGInit:			// CMD: 0x84
					{
						if (PacketDataLen >= 4)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						
						
						unsigned char ECUAddress = RXPacketPtr->PacketData[0];
						unsigned char ReplyDelayAfterInit = RXPacketPtr->PacketData[1];
						unsigned char KEYWORDLo, KEYWORDHi;
						unsigned char mask;
						if (PacketDataLen==3)
						{
							mask = RXPacketPtr->PacketData[2];
							if (mask>=4)
								mask = 3;
						}
						else
							mask = 3;

						ClearPacketRcvdFlag();
						if (InitVAGController(ECUAddress, ReplyDelayAfterInit, &KEYWORDLo, &KEYWORDHi, mask) == 0)
						{
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						unionLong_t ECUBaudrate;
						
						ECUBaudrate.mylong = CalcBaudFromTMR1Tick();
						SetupPORTs();
						unsigned char *TXPacktDataPtr = CreatePacket(0x84);
						*TXPacktDataPtr++ = ECUBaudrate.myByte[0];
						*TXPacktDataPtr++ = ECUBaudrate.myByte[1];
						*TXPacktDataPtr++ = ECUBaudrate.myByte[2];
						*TXPacktDataPtr++ = ECUBaudrate.myByte[3];
						*TXPacktDataPtr++ = KEYWORDLo;
						*TXPacktDataPtr++ = KEYWORDHi;
						SendPacket(TXPacktDataPtr);
						do {}while(CheckIfPacketIsSended()==0);
						DoECUComm();
						ClearPacketRcvdFlag();
						break;
					}

				case RevInit:			// CMD: 0x86
				{
					if (PacketDataLen!=0)
					{
						ErrCodeToPC = 1;
						ClearPacketRcvdFlag();
						SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
						break;
					};

					SendPacketToPC(0x0000, 0, 0xFF);
					SetK1K2MaskRegs(1);		//Select K
					Set5BPSBaudrate();
					unsigned char volatile ECUbyte;
					do
					{
						if(CheckPacketRcvdFlag()!=0)
						{
						RXPacketPtr = (RXPacket_t*)GetRXBuffPtr();
						PacketLen = RXPacketPtr->PacketLen;
						PacketCMD = RXPacketPtr->PacketCMD;
						//TODO
						}


					}while(CheckIfRcvdByteFromECU((unsigned char*)&ECUbyte)==0);
					break;
				}


				case 0x83:				// CMD: 0x83
				{
					if (PacketDataLen!=1)
					{
						ErrCodeToPC = 1;
						ClearPacketRcvdFlag();
						SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
						break;
					};
					unsigned char mask = RXPacketPtr->PacketData[0];

					SendPacketToPC(0x0000, 0, 0xFF);
					do {}while(CheckIfPacketIsSended()==0);
					unsigned long tmpBaud = GETCurrentBaudrate();
					DisableUSART0();
					SetK1K2MaskRegs(1);		//Select K
					Set_USB_K_Communication();
					if (EstimateBaudrate(mask)==0)
					{
						SetupPORTs();
						InitUSART();
						SetupUBRR0Value(tmpBaud);
						ClearPacketRcvdFlag();
						SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
						break;
					}

					SetTMR1BaudrateDivisor();
					unsigned char KWLo;
					if (WaitForByteFromECU(7, &KWLo)==0)
					{
						SetupPORTs();
						InitUSART();
						SetupUBRR0Value(tmpBaud);
						ClearPacketRcvdFlag();
						SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
						break;
					}
					
					unsigned char KWHi;	
					if (WaitForByteFromECU(7, &KWHi)==0)
					{
						SetupPORTs();
						InitUSART();
						SetupUBRR0Value(tmpBaud);
						ClearPacketRcvdFlag();
						SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
						break;
					}

					SendByteToLINE(~KWHi);
					SetupPORTs();
					InitUSART();
					SetupUBRR0Value(tmpBaud);
					unionLong_t ECUBaudrate;
					ECUBaudrate.mylong = CalcBaudFromTMR1Tick();
					Delay(14);
					unsigned char *TXPacktDataPtr = CreatePacket(0x83);
					*TXPacktDataPtr++ = ECUBaudrate.myByte[0];
					*TXPacktDataPtr++ = ECUBaudrate.myByte[1];
					*TXPacktDataPtr++ = ECUBaudrate.myByte[2];
					*TXPacktDataPtr++ = ECUBaudrate.myByte[3];
					*TXPacktDataPtr++ = KWLo;
					*TXPacktDataPtr++ = KWHi;
					SendPacket(TXPacktDataPtr);
					ClearPacketRcvdFlag();
					break;
				}	

				
				case FastInit:				//CMD: 0x98
					{
						if (PacketDataLen!=6)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};	

						//TODO
					}



				case 0xA0:
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};

						SendPacketToPC(0x0000, 0, 0xFE);
						ClearPacketRcvdFlag();
						break;	
					}


				case CANMode:			// CMD: 0xB0 - Switch to CAN mode
					{
						if (PacketDataLen!=0)
						{
							ErrCodeToPC = 1;
							ClearPacketRcvdFlag();
							SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
							break;
						};
						
						SendPacketToPC(0x0000, 0, 0xFE);
						ClearPacketRcvdFlag();
						DoCANMode();
						break;
					}


				default:
					{
						ErrCodeToPC = 1;
						ClearPacketRcvdFlag();
						SendPacketToPC(&ErrCodeToPC, sizeof(unsigned char),0xFF);
						break;
					}
			}
		}



	}while(1);
			
	//******************************************************
	// END of Main program loop
	//******************************************************		
}




