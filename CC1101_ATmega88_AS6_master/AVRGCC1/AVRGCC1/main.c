/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
//#include <asf.h>



#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdint.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#define F_CPU 8000000UL
#include <util/delay.h>


#include "macro.h"


#define nop() asm volatile ("nop")

#define SUCCESS     1       /* 成功 */
#define FAILURE     0       /* 失敗 */

//#define QUEUE_SIZE 10			/* 待ち行列に入るデータの最大数 */
#define QUEUE_SIZE 20			/* 待ち行列に入るデータの最大数 */

typedef int data_t;				/* データ型 */



//data_t queue_data[QUEUE_SIZE];	/* 待ち行列データ本体 */
//int queue_head;					/* データ先頭 */
//int queue_num;					/* データ個数 */





int	gUartRcvData=-1;
u8 rx_char[40];
u8 uartData;
volatile int testtest=0;
volatile int count32MSec=0;
volatile int count1Sec=0;
volatile int countBusyCancel=-1;
int uart_rx_index=0;
int uart_rx_length=0;
int ledTest=0;
int waitCtrSound=-1;
u8 gSoundPlay = false;
u8 busySigOut=false;
u8 oldBusySigOut=false;


void myDelay_us(int usec)
{
	int i;
	
	for(i=0;i<usec;i++)
	{
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
	}
}



void myDelay_ms(int usec)
{
	int i;
	
	for(i=0;i<usec;i++)
	{
		myDelay_us(250);
		myDelay_us(250);
		myDelay_us(250);
		myDelay_us(250);
	}
}




void inituart(void)
{
	UCSR0A = 0x02;	//U2X = 1
	UCSR0B = 0x00; // Rx/Tx enable
	UCSR0C = 0x06; // 8bit, 1 stop bit, no parity
	UBRR0H = 0x00; 
	//UBRR0L = 0x33;	// Baudrate 9600 for master clk=4MHz
	//UBRR0L = 0x67;		// Baudrate 9600 for master clk=8MHz
	UBRR0L = 0xcf;		// Baudrate 4800 for master clk=8MHz
	UCSR0B = 0x98; // rx / tx enable , rx int enable 0x98
}


void hw_setup(void)
{

	//CLKPR = _BV(CLKPCE);	// CLKPCEビットを1にする
	//CLKPR = 0b0001;			// 2分周にする



    DDRC=0x30;
	DDRD=0x04;
	DDRB=0x00;

	/*pull_up*/
	//PORTC = (1<<PC4) |(1<<PC4);
	//PORTD = 0xf7; 

	PORTD = (1<<PD0) | (1<<PD2); 


	//PORTB = (1<<PB0) | (1<<PB1);	//for yellow LED



	//EICRA &= ~(1<<ISC00);	//INT0の割り込み発生条件を立下り検知でトリガで固定
	//EICRA &= ~(1<<ISC01);

	//EICRA &= ~(1<<ISC10);	//INT1の割り込み発生条件を立下り検知でトリガで固定
	//EICRA &= ~(1<<ISC11);
}




ISR(USART_RX_vect)
{

	rx_char[uart_rx_index++] = UDR0;

	//if(rx_char[uart_rx_index]==0x0d)
	if(rx_char[uart_rx_index - 1]==0xec)
	{
		uart_rx_length = uart_rx_index - 1;
		//gUartRcvData = 0;	
		uart_rx_index = 0;
		gSoundPlay=true;
	}
}










#if 0


void sleep_set(u8 alarm,u8 bo)
{
	WDTCSR &= ~(1 << WDIE);
	wdt_disable();
	if(bo == 0)
		rtc_set_next_alarm(alarm);

	cc1100_cmd_idle();
	cc1100_cmd_pwd();

	spi_signalpin_opendrain_active();
	EIMSK |= (1<<INT0);
	sleep_mode();
	wdt_enable(WDTO_4S);
	WDTCSR |= (1 << WDIE);
}

#endif


ISR(TIMER0_COMPA_vect)	//タイマ割り込み
{
	if(count32MSec > -1)
	{
		//PORTC |= (1 << 4);
		count32MSec=-1;
	}
	else
	{
		//PORTC &= ~(1 << 4);
		count32MSec=0;
	}
	
	
	
	if(count1Sec++ > 31)	
	{
		if(ledTest==0)
		{
			//PORTC |= (1 << 4);
			ledTest++;
		}
		else
		{
			//PORTC &= ~(1 << 4);
			ledTest=0;
		}
		
		
		if(waitCtrSound > -1)
		{	
			waitCtrSound++;
			
			if(waitCtrSound > 3)
			{
				waitCtrSound = -1;
			}
		}
		
		if(countBusyCancel>-1)
		{
			countBusyCancel++;
		}

		
		count1Sec=0;
	}
	
	
}


void timer_init(void)
{
	
	//タイマ0,CTC,割り込み用、比較A一致で割り込み
	TCCR0A = 0b00000010; 
	//TCCR0B = 0b00000011; // N=64
	TCCR0B = 0b00000101; // N=1024
	//OCR0A = 78;  // 5msごとに割り込み
	OCR0A = 255;  // 5msごとに割り込み
	TIMSK0 = 0b0000010;	//比較A一致割り込み有効
}


void SoundPlay(u8 data)
{
	int i=0;
	u8 outData = 0;
	
	hw_setup();
		
	
	PORTC |= _BV(PC4);
	//_delay_us(10);
	_delay_ms(1);
	PORTC |= _BV(PC5);
	//_delay_us(10);
	_delay_ms(1);

	PORTC &= ~_BV(PC4);
	//_delay_us(10);
	_delay_ms(1);
	
	PORTC &= ~_BV(PC5);
	//_delay_us(10);
	_delay_ms(1);

	
	for(i=0;i<8;i++)
	{
		outData = (data >> i);
		
		if(outData & 0x01)
		{
			PORTC |= _BV(PC4);
		}
		else
		{
			PORTC &= ~_BV(PC4);
		}
		//_delay_us(10);
		_delay_ms(1);
		
		PORTC |= _BV(PC5);
		//_delay_us(10);
		_delay_ms(1);
		
		PORTC &= ~_BV(PC5);
		//_delay_us(10);
		_delay_ms(1);
	}
	
	
	//_delay_us(10);
	_delay_ms(1);
	PORTC |= _BV(PC5);
	//_delay_us(10);
	_delay_ms(1);
	
	PORTC |= _BV(PC4);
	//_delay_us(10);
	_delay_ms(1);
	
	

}



u8 get_R_busy(void)
{
	return (PINB & 0x01) ? true : false;
}



u8 get_L_busy(void)
{
	return (PINB & 0x02) ? true : false;
}





int main(void)
{


	hw_setup();
	inituart();
	timer_init();
	//i2c_init();
	//check_env();

	//device_status_setup();

	wdt_enable(WDTO_4S);
	WDTCSR |= (1 << WDIE);

	sei();
	
	PORTC |= _BV(PC4);
	PORTC |= _BV(PC5);
	
	PORTB |= _BV(PB0);
	PORTB |= _BV(PB1);

	waitCtrSound=0;
	_delay_ms(100);
	
    while(1)
    {
		#if 1
		wdt_reset();


		if(gSoundPlay == true)
		{
			countBusyCancel=0;
			#if 1
			while(1)
			{
				wdt_reset();

				
				if(get_R_busy()==false && get_L_busy()==false)			
				{
					countBusyCancel=-1;
					break;
				}
				
				if(countBusyCancel > 1)
				{
					countBusyCancel=-1;
					break;
				}
			}
			#endif

			waitCtrSound=0;
			//_delay_ms(500);
			//SoundPlay(soundNumber++);
			SoundPlay(rx_char[uart_rx_length - 1]);
			
			//_delay_ms(500);
				
			gSoundPlay=false;
			uart_rx_length=0;

			
		}
		
		busySigOut = get_R_busy() | get_L_busy();
		
		if(busySigOut != oldBusySigOut)
		{
			if(busySigOut == true)
			{
				PORTD &= ~_BV(PD2);
			}
			else
			{
				PORTD |= _BV(PD2);
			}
			
			oldBusySigOut = busySigOut;
		}
		

		testtest++;
		
		#endif
		
		
	
	}
}





