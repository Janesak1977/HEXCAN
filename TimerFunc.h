

extern unsigned char TMR0_OFCounter;

void InitTimer0();

void SetTimeDelay(volatile Timerstruct_t*, unsigned char);
unsigned int CheckTimerDelay(volatile Timerstruct_t*);
void StopTimer(volatile Timerstruct_t*);

void SetTimeDelay16(volatile Timerstruct16_t*, unsigned int);
unsigned int CheckTimerDelay16(volatile Timerstruct16_t*);
void StopTimer16();

void Delay_ms(unsigned char);
int GetTMR0_OFCounter();
void Delay(unsigned char);

