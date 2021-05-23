/*
 * Segway_Controller.c
 *
 * Created: 2020-10-11 오전 12:17:06
 * Author : 민규
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int cnt=0;
int adc[3];
char adcx[5];
char adcy[5];
int ts=50;
int large_x;
int largest_x;
int small_x;
int smallest_x;
int large_y;
int largest_y;
int small_y;
int smallest_y;
int overlap_flag=0;

void TIMER_INIT()
{
	TCCR0 = (0<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(1<<CS02)|(1<<CS01)|(0<<CS00);
	TIMSK = (1<<TOIE0);
	TCNT0 = 131;
	sei(); // global interrupt activate
}

void ADC_INIT()
{
	DDRF = 0x00;
	ADMUX = 0x40;
	ADCSRA = 0x87;
}

void UART1_INIT()
{//HC-05 init
	DDRD = 0b00001000;
	UCSR1A = 0x00;
	UCSR1B = 0b00011000;
	UCSR1C = 0b00000110;
	
	UBRR1H = 0;
	UBRR1L = 103;
}// bps 9600, Non Parity, Stop bit 1, 8bit data

void UART0_INIT()
{//serial uart init
	DDRE = 0b00000010;
	UCSR0A = 0x00;
	UCSR0B = 0b00011000;
	UCSR0C = 0b00000110;
	
	UBRR0H = 0;
	UBRR0L = 103;
}// bps 9600, Non Parity, Stop bit 1, 8bit data

void UART1_TX(unsigned char cData)
{
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1 = cData;
}

unsigned char UART1_RX()
{
	while(!(UCSR1A & (1<<RXC1)));
	return UDR1;
}

void UART0_TX(unsigned char cData)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = cData;
}

unsigned char UART0_RX()
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

ISR(TIMER0_OVF_vect){
	cnt++;
	TCNT0 = 131; // overflow 인터럽트이기 때문에 인터럽트 발생 시 TCNT0값 설정
	
	if(cnt == ts){ // 2ms*50cnt=100ms
		ADMUX = 0x40;
		ADCSRA = 0x87;
		for(int i=0; i<3; i++)
		{
			ADCSRA |= (1<<ADSC); // ADC Conversion Start
			ADMUX |= i;
			while(!(ADCSRA & (1<<ADIF))); // ADC 변환 종료시 인터럽트 발생
			adc[i] = (int)ADCL+((int)ADCH<<8);
		}
		itoa(adc[0],adcx,10);
		itoa(adc[1],adcy,10);
		if(adc[0]>767)
		{
			large_x=1;
			if((large_x==1) & (adc[0]>1000))
			{
				largest_x=1;
			}
		}
		else if(adc[0]<255)
		{
			small_x=1;
			if((small_x==1) & (adc[0]<23))
			{
				smallest_x=1;
			}
		}
		
		if(adc[1]>767)
		{
			large_y=1;
			if((large_y==1) & (adc[1]>1000))
			{
				largest_y=1;
			}
		}
		else if(adc[1]<255)
		{
			small_y=1;
			if((small_y==1) & (adc[1]<23))
			{
				smallest_y=1;
			}
		}
		if ((large_x+small_x+large_y+small_y)>1)
		{
			overlap_flag=1;
		}
		if((large_x||large_y||small_x||small_y||largest_x||largest_y||smallest_x||smallest_y)&(!overlap_flag))
		{
			//Move Back instruction
			if (largest_x)
			{
				UART1_TX('M');
				UART1_TX('B');
			}
			else if (large_x)
			{
				UART1_TX('B');
			}
			//Move Right instruction
			if (largest_y)
			{
				UART1_TX('M');
				UART1_TX('R');
			}
			else if (large_y)
			{
				UART1_TX('R');
			}
			//Move Front instruction
			if (smallest_x)
			{
				UART1_TX('M');
				UART1_TX('F');
			}
			else if (small_x)
			{
				UART1_TX('F');
			}
			//Move Left instruction
			if (smallest_y)
			{
				UART1_TX('M');
				UART1_TX('L');
			}
			else if (small_y)
			{
				UART1_TX('L');
			}
		}
		else
		{
			if (overlap_flag)
			{
				UART1_TX('X');
			}
			else
			{
				UART1_TX('S');	
			}
		}
		
		//UART ADC Voltage값 전송 코드
		/*for(int j=0; j<5; j++)
		{
		if(adcx[j]!=0)
		{
		UART0_TX(adcx[j]);
		}
		}
		UART0_TX(44);
		for(int j=0; j<5; j++)
		{
		if(adcy[j]!=0)
		{
		UART0_TX(adcy[j]);
		}
		}*/
		UART1_TX(10);
		large_x=0;
		large_y=0;
		small_x=0;
		small_y=0;
		largest_x=0;
		largest_y=0;
		smallest_x=0;
		smallest_y=0;
		overlap_flag=0;
		cnt = 0;
	}
}

int main(void)
{
	TIMER_INIT();
	ADC_INIT();
    UART0_INIT();
	UART1_INIT();
    while (1) 
    {
    }
}

