#include "config.h"
#include "main.h"
#include "TimerFunc.h"


ISR(TIMER0_OVF_vect, ISR_NAKED)
{
	asm volatile
	(
		"	sei			" "\n"
		"__my_tmp_reg__ = 16" "\n"
		"__my_tmp_reg1__ = 17" "\n"
		/* prologue */
		"	push __my_tmp_reg__" "\n"
		"	push __my_tmp_reg1__" "\n"
		"	in __my_tmp_reg1__, __SREG__" "\n"
		/* prologue end  */ 
		"	lds  __my_tmp_reg__, TMR0_OFCounter" "\n"
		"	inc	r16			" "\n"
		"	sts TMR0_OFCounter, __my_tmp_reg__" "\n" 
		/* epilogue */
		"	out __SREG__, __my_tmp_reg1__" "\n"
		"	pop	 __my_tmp_reg1__" "\n"
		"	pop	 __my_tmp_reg__" "\n"
		"	reti"
	::
	);

/*	
	TMR0_OFCounter++;
*/
}


void InitTimer0()
{
	TMR0_OFCounter = 0;
	TCCR0 = 5;
	TCNT0 = 0;
	TIFR |= 0x02;
	TIMSK |= 0x02;
}

void SetTimeDelay(volatile Timerstruct_t* Timer, unsigned char time)
{
	Timer->value = TMR0_OFCounter + time;
	Timer->status = 0;
}

unsigned int CheckTimerDelay(volatile Timerstruct_t* Timer)
{
	unsigned char tmp,tmpOFCnt;

	tmpOFCnt = TMR0_OFCounter;
	if (Timer->status==2)
		return 1;
	if (Timer->status==1)
		return 0;
	tmp = Timer->value;
	if (tmpOFCnt < tmp)
	{
		tmp = tmp - tmpOFCnt;
		if ( (tmp&0x80)==0)
			return 0;
		else
		{
			Timer->status=2;
			return 1;
		}
	}
	else
	{
		tmpOFCnt = tmpOFCnt - tmp;
		if ( (tmpOFCnt&0x80)==0)
		{
			Timer->status=2;
			return 1;
		}
		else
			return 0;
	}
}


void StopTimer(volatile Timerstruct_t* Timer)
{
	Timer->status = 1;	
}

void SetTimeDelay16(volatile Timerstruct16_t* Timer, unsigned int time)
{
	unsigned int timervalue;
	unsigned char TCNTOF, tmp;
	
	asm ("lds %0, TMR0_OFCounter" "\n\t"
		  "in %1, 0x32" "\n\t"
		  "lds r18, TMR0_OFCounter" "\n\t"
		  "sbrc %1, 7" "\n\t"
		  "mov	%0, r18"
		   :"=r" (TCNTOF), "=r" (tmp));


	timervalue = ((7372800 * time) + 512000) / 1024000;

	timervalue += tmp;
	timervalue += 	TCNTOF << 8;
	Timer->value = timervalue;
	Timer->status = 0;	
}


unsigned int CheckTimerDelay16(volatile Timerstruct16_t* Timer)
{
	unsigned int tmpOFCnt;
	unsigned int tmp;
	unsigned char tmpTCNT,tmpTCNTOF;

	asm ("lds %0, TMR0_OFCounter" "\n\t"
		  "in %1, 0x32" "\n\t"
		  "lds r18, TMR0_OFCounter" "\n\t"
		  "sbrc %1, 7" "\n\t"
		  "mov	%0, r18"
		   :"=r" (tmpTCNTOF), "=r" (tmpTCNT));
	
	if (Timer->status==2)
		return 1;
	if (Timer->status==1)
		return 0;
	
	tmpOFCnt = tmpTCNT + (tmpTCNTOF << 8);
	tmp = Timer->value;
	if (tmpOFCnt >= tmp)
	{
		if ( (int)(tmpOFCnt-tmp)>=0)
		{
			Timer->status=2;
			return 1;
		}
		else
			return 0;
	}
	else
	{
		if ( (int)(tmp-tmpOFCnt)<0)
		{	
			Timer->status=2;
			return 1;
		}
		else
			return 0;

	}
}

void StopTimer16()
{

}


void Delay_ms(unsigned char delay)
{
	volatile Timerstruct16_t Timer;

	SetTimeDelay16(&Timer, delay);
	do {}while(CheckTimerDelay16(&Timer)==0);
}

int GetTMR0_OFCounter()
{
	return TMR0_OFCounter;
}

void Delay(unsigned char delay)
{
	Timerstruct_t Timer;

	Timer.value = TMR0_OFCounter + delay;
	Timer.status = 0;
	do { } while((unsigned char)CheckTimerDelay(&Timer)==0);
}
