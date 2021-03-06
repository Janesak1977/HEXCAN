
#define EEPROM __attribute__((section(".eeprom")))

uint8_t EEPROM EE_Reserved1 = 0xFF; // do not use EEPROM address 0
uint8_t EEPROM EE_InterfaceID[8] = "ROSSTECH";
uint64_t EEPROM EE_Serial = 0x000000A8B0E79224;
uint8_t EEPROM EE_ChksumHI = 0x00;		// addr 0x11
uint8_t EEPROM EE_ChksumLO = 0x00;		// addr 0x12
uint8_t EEPROM EE_Reserved2 = 0xFF;		// addr 0x13
uint8_t EEPROM EE_0014 = 0xFF;			// addr 0x14
uint8_t EEPROM EE_0015 = 0xAB;			// addr 0x15
uint16_t EEPROM EE_0016 = 0xFFFF;		// addr 0x16
uint8_t EEPROM EE_VersionMinor = 0xFF;	// addr 0x18
uint8_t EEPROM EE_VersionMajor = 0xFF;	// addr 0x19
uint8_t EEPROM EE_Reserved2 = 0xFF;		// addr 0x1A
uint8_t EEPROM EE_001B = 0xFF;			// addr 0x1B
uint32_t EEPROM EE_001C = 0x02000000;	// addr 0x1C
uint32_t EEPROM EE_FOsc = 0xFFFFFFFF;	// addr 0x20
uint8_t EEPROM EE_Reserved3[28]			// addr 0x24
uint8_t EEPROM EE_KEY[32] = { 0xDC,0x49,0x4E,0xC0,0xFA,0xEF,0xEB,0x0E,		// addr 0x40
							  0x8F,0xAC,0x26,0xA1,0x04,0x1C,0x64,0x8E,
							  0x86,0xA9,0x43,0x6E,0x20,0x4C,0x2F,0x0F,
							  0x1F,0x44,0x37,0x04,0x8D,0x20,0x5C,0x3B };
