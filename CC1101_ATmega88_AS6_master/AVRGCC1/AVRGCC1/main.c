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
#include "cc1100.h"
#include "cc1100_globals.h"

#include "spi1.h"
#include "i2c.h"



#define SUCCESS     1       /* 成功 */
#define FAILURE     0       /* 失敗 */

//#define QUEUE_SIZE 10			/* 待ち行列に入るデータの最大数 */
#define QUEUE_SIZE 20			/* 待ち行列に入るデータの最大数 */

typedef int data_t;				/* データ型 */



//data_t queue_data[QUEUE_SIZE];	/* 待ち行列データ本体 */
//int queue_head;					/* データ先頭 */
//int queue_num;					/* データ個数 */








#define EEPROM __attribute__((section(".eeprom")))
save_eeprom_data_t EEPROM save_eeprom_data;




int	gUartRcvData=-1;
u8 rx_char[40];
u8 uartData;
volatile int testtest=0;
int uart_rx_index=0;
int uart_rx_length=0;



void putc_(u8 data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void putb(u8 data)
{
	if(data < 0x0a)
		putc_('0' + data);
	else 
		putc_('a' + (data-10));
}

void puthex(u8 data)
{
	putb(data/16);
	putb(data%16);
}


u8 putchr2hex(u8 data)
{
	u8 tmp;

	if(data&0x60)
		tmp = data - 0x60;
	else
		tmp = data - 0x30;

	return tmp;
}


u8 chr2hex(u8 data_h,u8 data_l)
{
	u8 test;

	data_h = putchr2hex(data_h);
	data_l = putchr2hex(data_l);
	
	test=(data_l&0x0F)|((data_h<<4)&0xF0);
	
	return test;	
}


void putstr(u8 * data)
{
	int i =0 ;

	for( i = 0 ;i < 50 ; i++)
	{
		putc_(data[i]);
		if(data[i] == 0x0d || data[i] == 0x00) break;
	}
}


char asc_to_hex(u8 asc)
{
	if((asc >= '0') && (asc <= '9')) return (asc - '0');
	else if((asc >= 'A') && (asc <= 'F')) return (asc - 'A' + 0x0a);
	else if((asc >= 'a') && (asc <= 'f')) return (asc - 'a' + 0x0a);
	else return 0xff;
}

u8 AscToHex(char * val)
{
	u8 tmp;
	
	tmp = asc_to_hex(val[0]) << 4 | asc_to_hex(val[1]);	
	
	return tmp;
}

char asc_to_int(u8 asc)
{
	if((asc >= '0') && (asc <= '9')) return (asc - '0');
	else return 0xff;
}

int AscToInt(char * val)
{
	u8 tmp;
	
	tmp = (asc_to_int(val[0]) * 10) +  asc_to_int(val[1]);	
	
	return tmp;
}


char hex_to_asc(u8 hex)
{
	char da;
	da = hex & 0x0f;
	if((da >= 0) && (da <= 9)) return ('0' + da);
	else return ('a' + da - 0x0a);
}

void hexToAsc(u8 hex,u8 * dest)
{
	dest[0] = hex_to_asc((hex >> 4 ) & 0x0f);
	dest[1] = hex_to_asc((hex >> 0 ) & 0x0f);
}



u8 my_eeprom_read_byte(u16 addr)
{
	eeprom_busy_wait();
	return eeprom_read_byte((u8 *)addr);
}

void my_eeprom_read_block(u8 * data,u8 *addr,u16 length)
{
	eeprom_busy_wait();
	return eeprom_read_block(data,addr,length);
}


void inituart(void)
{
	UCSR0A = 0x02;	//U2X = 1
	UCSR0B = 0x00; // Rx/Tx enable
	UCSR0C = 0x06; // 8bit, 1 stop bit, no parity
	UBRR0H = 0x00; 
	//UBRR0L = 0x33;	// Baudrate 9600 for master clk=4MHz
	UBRR0L = 0x67;		// Baudrate 9600 for master clk=8MHz
	UCSR0B = 0x98; // rx / tx enable , rx int enable 0x98
}


void hw_setup(void)
{

	//CLKPR = _BV(CLKPCE);	// CLKPCEビットを1にする
	//CLKPR = 0b0001;			// 2分周にする



    DDRC=0x30;      
	DDRD=0x00;
	DDRB=0x00;

	/*pull_up*/
	//PORTC = (1<<PC6) |(1<<PC0) | (1<<PC1);
	//PORTD = 0xf7; 




	//PORTB = 0x01;	//for yellow LED



	//EICRA &= ~(1<<ISC00);	//INT0の割り込み発生条件を立下り検知でトリガで固定
	//EICRA &= ~(1<<ISC01);

	//EICRA &= ~(1<<ISC10);	//INT1の割り込み発生条件を立下り検知でトリガで固定
	//EICRA &= ~(1<<ISC11);
}




ISR(USART_RX_vect)
{

	rx_char[uart_rx_index++] = UDR0;

	if(rx_char[uart_rx_index]==0x0d)
	{
		uart_rx_length = uart_rx_index;
		gUartRcvData = 0;	
		uart_rx_index = 0;
	}
}


ISR(INT1_vect)
{
	


}


ISR(INT0_vect)
{

}














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





ISR(TIMER0_COMPA_vect)	//タイマ割り込み
{
	
	
	
	

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


int main(void)
{


	hw_setup();
	inituart();
	timer_init();
	//i2c_init();
	//check_env();

	//device_status_setup();


	sei();
	
	

	
    while(1)
    {
		
		if(gUartRcvData > 0)
		{
			testtest++;
		}
		
		
		
	
	}
}





