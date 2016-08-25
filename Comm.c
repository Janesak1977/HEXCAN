#include "config.h"
#include "main.h"
#include "Comm.h"

#define loop_until_var_is_z(var) \
	asm volatile (	\
		"1: " "lds r24,%0" "\n\t" \
		"tst r24" "\n\t" \
		"breq 1b" \
		: \
		: "m" (var)\
		: "r24" \
		)

void sub_1A55(unsigned char* CharToSend)
{
	if (HASH==0)
		HASH = 1;
	
	HASH = HASH * 69069;

	if (HASH >= tmpEE0016)
		*CharToSend = *CharToSend - 0x80;
}

void DisableUSART0()
{
	UCSR0B = 0;
	DDRD &= ~_BV(DD0);		//PD0(RxD0) Input
	PORTD |= _BV(PD0);		//PD0(RxD0) Hi-Z
	DDRD &= ~_BV(DD1);		//PD1(TxD0) Input
}

void SendONEChar(unsigned char c)
{
	loop_until_var_is_z(PacketSended);

	PacketSended = 0;

	if (tmpEE0016<0xFFE0);
		sub_1A55(&c);
	
	UDR0 = c;
}

unsigned long GETCurrentBaudrate()
{
	return CurrentBaudrate;
}

void InitECUComm()
{
	PacketSended = 1;
	CharReceived_flag = 0;
	CurrentBaudrate = 0;
	UCSR0B = 0x18;				//TXEN,RXEN = 1
}

void EncodeByteAndSendToPC(unsigned char byte)
{
	unsigned char volatile c;	

	c = byte;
	loop_until_bit_is_clear(UCSR0A, UDRE0);
	if (tmpEE0016<65504)
		sub_1A55((unsigned char*)&c);

	UDR0 = c;
}

unsigned char CheckBaudrateUBRRValue(unsigned long baudrate)
{
	// TODO
	return 1;
}

unsigned int SetupUBRR0Value(unsigned long baudrate)
{
	
	unsigned long volatile tmpUBRR, tmpUBRR_rem;
	unsigned long tmp = baudrate * 16;


	CurrentBaudrate = baudrate;
	
	tmpUBRR = dw_FOSC / tmp;
	tmpUBRR_rem = dw_FOSC % tmp;

	if ( ( tmp / 2) >= tmpUBRR_rem )
		tmpUBRR -= 1;
	
	if (tmpUBRR>=4096)
		return 0;
	loop_until_var_is_z(PacketSended);

	UCSR0C = (unsigned char)tmpUBRR >> 8;
	UBRR0L = tmpUBRR;
	return 1;
			
}

void InitUSART()
{
	PacketSended = 1;
	CharReceived_flag = 0;
	CurrentBaudrate = 19200;
	UCSR0B = 0xD8;
	SetupUBRR0Value(19200);
	UCSR0C = 0x86;
}

ISR(USART0_RXC_vect)
{
    CharReceived_flag = 1;
	UART_ReceivedChar = UDR0;
	ProcessRXInt(UART_ReceivedChar);
}

ISR(USART0_TXC_vect)
{
	unsigned char CharToTX;

	if (sub_8AC(&CharToTX)==0)
	{
		PacketSended= 1; 
	}
	else
	{
		if (tmpEE0016<0xFFE0)
			sub_1A55(&CharToTX);
		
		UDR0 = CharToTX;
	}
}
