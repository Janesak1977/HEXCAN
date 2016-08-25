

extern unsigned long HASH;
extern unsigned int tmpEE0016;
extern unsigned char PacketSended;
extern unsigned long dw_FOSC;
extern unsigned char CharReceived_flag;
extern unsigned char UART_ReceivedChar;
extern unsigned long CurrentBaudrate;

void sub_1A55(unsigned char*);
void DisableUSART0();
void SendONEChar(unsigned char);
unsigned long GETCurrentBaudrate();
void InitECUComm();
void EncodeByteAndSendToPC(unsigned char);
unsigned char CheckBaudrateUBRRValue(unsigned long);
unsigned int SetupUBRR0Value(unsigned long);
void InitUSART();


