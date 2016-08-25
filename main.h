//**********************************************
//HEX+CAN Interface commands
//***********************************************

#pragma once

#ifndef MAIN_H
#define MAIN_H


#define VERSION_MAJOR	 1
#define VERSION_MINOR	92

#define	TCCR1_CKDiv		64
#define	InitECUBaudrate	5


#define	UpdateFW			0x01
#define	GetVersion			0x02
#define	SetBaudrate			0x03
#define GetInterfaceID		0x04
#define SoftReset			0x08
#define GetFOscValue		0x17
#define SetFOscValue		0x18
#define	KLineTest			0x82
#define	VAGInit				0x84
#define	RevInit				0x86
#define	FastInit			0x98
#define CANMode				0xB0
#define CANMode_ConfigCAN	0xB1
#define CANMode_WriteConfig	0xB2
#define CANMode_WriteMASK	0xB3
#define CANMode_WriteFILTER	0xB4
#define CANMode_WriteRXCTRL	0xB5
#define CANMode_NormalMode	0xB6
#define CANMode_ReceivedMsg	0xB7
#define CANMode_SendMsg		0xB8
#define CANMode_BusError	0xB9
#define CANMode_StopCANMode	0xA0


#define LED_PORT	PORTE

#define GRN_LED		PE0
#define RED_LED		PE1

#define GRN_ON		LED_PORT |= _BV(GRN_LED)
#define GRN_OFF		LED_PORT &= ~_BV(GRN_LED)

#define RED_ON		LED_PORT |= _BV(RED_LED)
#define RED_OFF		LED_PORT &= ~_BV(RED_LED) 

#define TURNOFF_LEDS	PORTE &= 0xFC

#define	K1_RX		PD2
#define K2_RX		PD3

// CAN Message Buffer offset definitions
#define	SIDH	0
#define SIDL	1
#define EID8	2
#define EID0	3
#define DLC		4
#define Data	5


typedef struct Timerstruct {
	unsigned char status;
	unsigned char value;
}Timerstruct_t;


typedef struct Timerstruct16 {
	unsigned char status;
	unsigned int value;
} Timerstruct16_t;

void ProcessRXInt(unsigned char);
unsigned int sub_8AC(unsigned char*);

#endif /* MAIN_H */
