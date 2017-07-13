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


typedef struct {
	u8 slave_id0;
	u8 slave_id1;
	u8 slave_id2;
	u8 user_data0;
	u8 user_data1;
	u8 bat_val;
	u8 RecvRssi;
	u8 rssi;
} master2pc_data_t;


typedef struct {
	int head;
	int num;
	master2pc_data_t master2pc_data[QUEUE_SIZE];
} queue_t;


//data_t queue_data[QUEUE_SIZE];	/* 待ち行列データ本体 */
//int queue_head;					/* データ先頭 */
//int queue_num;					/* データ個数 */








#define EEPROM __attribute__((section(".eeprom")))
save_eeprom_data_t EEPROM save_eeprom_data;

void insert_sr_buf(void);
u8	get_alarm_time(void);
void master_oper(cc1101_client * Client);
void master_main(void);
u8 slave_oper(cc1101_client * Client);
u8 setup_oper(void);
int AscToInt(char * val);
u16 ad_get_val(u8 ad_ch,u8 adlar);
char asc_to_hex(u8 asc);
void add_batt_low(void);
char asc_to_int(u8 asc);
u8 AscToHex(char * val);
void carrier_no_sense(void);
void carrier_sense(void);
void cc1101_carrier_wave_setup(void);
u8 cc1101_rx(u8 * data,u8 loop);
u8 cc1101_tx(u8 * data , u8 length,u8 th);
void cc1101_tx_carrier(cc1101_client * client);
void check_env(void);
void device_status_setup(void);
u8 chr2hex(u8 data_h,u8 data_l);
u8	get_frq(void);
u8 get_input_data(void);
u8 get_sr_status(void);
void hex_change_level(u8 * data);
void hexToAsc(u8 hex,u8 * dest);
char hex_to_asc(u8 hex);
void hw_setup(void);
void init_voltcomparator(void);
void led_flasher(int count,int m10sec);
void inituart(void);
void level_change_hex(u8 * data);
void led_onoff(u8 onoff);
void my_eeprom_read_block(u8 * data,u8 *addr,u16 length);
u8 my_eeprom_read_byte(u16 addr);
u8 my_memcmp(u8 * src,u8 * dest,u8 len);
void putb(u8 data);
void putc_(u8 data);
u8 putchr2hex(u8 data);
void puthex(u8 data);
void putstr(u8 * data);
void random_tx_test(void);
int reg_slave_num_serch(u8 * id);
void rtc_alarm_disenable(void);
void rtc_set_next_alarm(u8 alarm);
u8 rx_fifo_read(u8 * data);
void sig_set(void);
u8 	sig_status(void);
void slave_fifo_preset(fifo_t * fifo);
void slave_main(void);
void sleep_set(u8 alarm,u8 bo);
void temperature_carrier_test(void);
u16 temperature_get(void);
void temperature_test_disenable(void);
void temperature_test_enable(void);
void test_wave_mode(void);
void tx_fifo_write(u8 * data,u8 length);
void setup_main(void);
void timer_init(void);
void master2pc_out(master2pc_data_t *d);
int enqueue(queue_t *que, fifo_t *enq_data);
int dequeue(queue_t *que, master2pc_data_t *deq_data);



queue_t	que;



const time_set_t time_set[4]={
							{0x09,0x59,0x57}, //ALARM_2SEC
							{0x09,0x59,0x00}, //ALARM_1MIN
							{0x09,0x58,0x00}, //ALARM_2MIN
							{0x09,0x55,0x00}, //ALARM_5MIN

							};

reg_t  reg_net_alarm[7]=
							{
								{0x0E,0x00},
								{0x0C,0x10},
								{0x0B,0x00},
								{0x02,0x00},
								{0x01,0x00},
								{0x00,0x00},
								{0x0E,0x40}
							};

reg_t reg_next_sch_alarm[9]=
{
	{0x0e,0x00},
	{0x0C,0x00},
	{0x0B,0x00},
	{0x0a,0x00},
	{0x03,0x00},
	{0x02,0x00},
	{0x01,0x00},
	{0x00,0x01},
	{0x0e,0x60}
};

//u8 day_cnt[4]={0x40,0x20,0x08,0x04};
//u8 day_cnt[4]={0x02,0x04,0x08,0x10};
u8 day_cnt[4]={0x03,0x05,0x09,0x11};


cc1101_client 	gClient;
char			gWCmd[6][16];

u8				wakeup_condition=0;
u8 				get_user0_data=0;
char 			rx_char[60];
u8 				uart_data;
u8 				uart_rx_index=0;
u8 				uart_rx_length;
reg_slave 		tSlave,tSlave2;
ctl_data  		tCtl;
u16				gTemperature;
char * 			pC;

u8 				sr_status=0x00,sr_status_old=0x00;
u8 				emg_status = 0x00;
u8 				tampa_status = 0x00,tampa_status_old = 0x00;
u8				setup_mode = SETUP_NONE;
u8				gslave_master_ctl = 0x00;
u8 				fix_id_mel[3]={0x4d,0x45,0x4c};
u16				gTemperature;
u8				gLevel;
u8				gRecvRssi=0x00, gRecvLqi= 0x00;
f_option_t		gOpt;
int	 			level_offset[14]={-3,-4,-4,-4,-3,-3,-1,0,1,2,3,4,5,5};
const int	 	default_offset[14]={-3,-4,-4,-4,-3,-3,-1,0,1,2,3,4,5,5};
int				SRBuf[2];
u8				SRBufIndex;
char			gTestMode;
u8				throw=OFF;
u8				sig_sch_send=OFF;
u8				input_data_old;
int				test20140825=0;
new_ctl_data	tnew_ctl_data;
int				test20140901=0;
//char			gWCmd2[6][16];

master2pc_data_t tmp_d;
int				data_send_wait_time = 0;
int				que_check_time = 0;
int				send2pc_retry_cnt=0;
//int				send2pc_ready = 0;


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

void led_onoff(u8 onoff)
{
	if(onoff == 1)
		B_CTL_HIGH;
	else
		B_CTL_LOW;
}


void led_flasher(int count,int m10sec)
{
	u8 i;
	u8 j=0;
	for(i=0;i<count;i++)
	{
		B_CTL_HIGH;
		//mdelay(msec);
		for(j=0;j<m10sec;j++)
		{
			_delay_ms(10);
		}

		B_CTL_LOW;
		//mdelay(msec);
		for(j=0;j<m10sec;j++)
		{
			_delay_ms(10);
		}
	}
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



    DDRC=0x00;      
	DDRD=0x00;
	DDRB=0x0D;

	/*pull_up*/
	PORTC = (1<<PC6) |(1<<PC0) | (1<<PC1);
	PORTD = 0xf7; 




	PORTB = 0x01;	//for yellow LED



	EICRA &= ~(1<<ISC00);	//INT0の割り込み発生条件を立下り検知でトリガで固定
	EICRA &= ~(1<<ISC01);

	EICRA &= ~(1<<ISC10);	//INT1の割り込み発生条件を立下り検知でトリガで固定
	EICRA &= ~(1<<ISC11);
}



u16 ad_get_val(u8 ad_ch,u8 adlar)
{
	u8 i=0;
	u8 ad_data_l=0;
	u8 ad_data_h=0;
	u16 ad=0;

	//u8 temp=0xc0;

	ADCSRA = 0b10000100; // AD許可:1 AD開始:0 AD自動起動:0 AD割込:0 AD完了割込:0 ck/16

	// reg set
	ADMUX = 0xc0 | adlar | (ad_ch & 0x0f);

	// dummy
	for(i = 0 ; i  < 4 ; i++)
	{
		ADCSRA |= _BV(ADSC);						//変換開始 要ﾙｰﾌﾟ内
		while((ADCSRA&0x10)==0x00);
		ADCSRA |= (1<<ADIF);						//ADIF clear
	}
	
	// read
	ADCSRA |= _BV(ADSC);						//変換開始 要ﾙｰﾌﾟ内
	while((ADCSRA&0x10)==0x00);
	ADCSRA |= (1<<ADIF);						//ADIF clear

	ad_data_l = ADCL;
	ad_data_h = ADCH;

	ad = ad_data_h;
	ad = (ad << 2);
	ad = (ad & 0x03FC) | ((ad_data_l >> 6) & 0x03);

	ADCSRA = 0b00000100; // AD許可:0 AD開始:0 AD自動起動:0 AD割込:0 AD完了割込:0 ck/16
	return ad;
}



void temperature_test_enable(void)
{
	cc1100_cmd_idle();
	PORTC &= ~(1<<PC2);	//pull-up disenable
	cc1100_write_reg(CC1100_REG_PTEST,0xBF);
	cc1100_cfg_gdo0(0x80);
}


void temperature_test_disenable(void)
{
	cc1100_cfg_gdo0(0x3F);
	cc1100_write_reg(CC1100_REG_PTEST,0x7F);
}


u16 temperature_get(void)
{
	PORTC &= ~(1<<PC2);	//pull-up disenable
	temperature_test_enable();

	DIDR0 |= (1<<ADC2D);		//ADC2 Digital disable

	gTemperature = ad_get_val(AD_PIN_TEMPERATURE,AD_VAL_FROM_L);

	DIDR0 &= ~(1<<ADC2D);		//ADC2 Digital enable

	temperature_test_disenable();
	PORTC |= (1<<PC2);		//pull-up enable

	return gTemperature;
}


int reg_slave_num_serch(u8 * id)
{
	u8 i=0;
	u8 Serial[6];

	memcpy(Serial,fix_id_mel,3);

	for(i=0;i<100;i++)
	{
		my_eeprom_read_block((u8 *)&tSlave,(u8 *)&save_eeprom_data.greg_slave[i],sizeof(reg_slave));

		memcpy((u8 *)&Serial[3],(u8 *)tSlave.serial,3);
		if(memcmp((u8 *)Serial,(u8 *)id,6) == 0)
		{
			return i;
		}
	}
	return -1;
}
	 



u8 get_input_data(void)
{
	u8 reg = 0;

	reg = (reg & 0xFF) | (((PIND) << 2) & 0x80);	//F_SEL2
	reg = (reg & 0xFF) | (((PIND) << 2) & 0x40);	//F_SEL1

	reg = (reg & 0xFF) | (((PIND) >> 2) & 0x20);	//D_SEL2
	reg = (reg & 0xFF) | (((PIND) >> 2) & 0x10);	//D_SEL1

	reg = (reg & 0xFF) | (((PINC) << 2) & 0x08);	//M_SEL2
	reg = (reg & 0xFF) | (((PINC) << 2) & 0x04);	//M_SEL1


	return reg;
}

void level_change_hex(u8 * data)
{
	int i;

	for(i = 0 ;i < 14 ; i+=2)
	{
		 data[i/2] = (((level_offset[i] + 7) << 4) & 0xf0) | ((level_offset[i+1] + 7) & 0x0f);
	}
}

void hex_change_level(u8 * data)
{
	int i;

	for(i = 0 ;i < 7 ; i++)
	{
		level_offset[i * 2] = (int)((data[i] >> 4 ) & 0x0f) - 7;
		level_offset[i* 2 + 1] = (int)((data[i]  ) & 0x0f) - 7;
	}

	eeprom_busy_wait();		
	eeprom_write_block(level_offset,&save_eeprom_data.eprom_level,sizeof(level_offset));
}

void slave_fifo_preset(fifo_t * fifo)
{

	memcpy(fifo->slave_id,gClient.status.serial,6);
	memset(fifo->master_id,0xff,3);
	fifo->user_data0 = gClient.status.user_data0;
	if(gClient.status.bat_val < 0xbf)
		fifo->user_data0 |= (1<<0);	// BATT_LOW active

	//if(gClient.status.status != SL_SCH_SEND)
	if(sig_sch_send == ON)
	{
		if(tampa_status == 1)
			fifo->user_data0 |= (0x01  << 1);		// Tampa
		else
			fifo->user_data0 &= ~(0x01 << 1);	// Tampa
			
		sig_sch_send = OFF;
	}


	fifo->user_data1 = gslave_master_ctl;
	fifo->bat_val = gClient.status.bat_val;
	fifo->setup_mode = (setup_mode == SETUP_NONE ? 0 : 1) ;
	fifo->master_ctl = 0x00;
	fifo->version_l = PRG_VERSION_SUB;


	if(setup_mode != SETUP_NONE)
	{
		fifo->length = sizeof(fifo_t);
		fifo->opt.env_data[0] = (gTemperature >> 8) & 0xff;
		fifo->opt.env_data[1] = (gTemperature >> 0) & 0xff;
		fifo->opt.env_data[2] = my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.adj_freq);	//serial no 01;
		fifo->opt.env_data[3] = my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.freq_add_sub);	//serial no 01;
		fifo->opt.env_data[4] = my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.adj_temper);	//serial no 01;
		fifo->opt.env_data[5] = my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.temper_add_sub);	//serial no 01;
		level_change_hex(fifo->opt.level);
	}
	else
	{
		fifo->length = sizeof(fifo_t) - sizeof(f_option_t);
	}
}



void tx_fifo_write(u8 * data,u8 length)
{
	data[0] = length + 2;
	cc1100_fifo_put(data,1);
	cc1100_fifo_put(data,length+2);
}



u8 rx_fifo_read(u8 * data)
{
	u8 length;

	cc1100_fifo_get(&length,1); 

	cc1100_fifo_get(data,length);

	if(gClient.status.type != MASTER)
	{
		cc1100_fifo_get(&gClient.fifo.rssi,1);
		cc1100_fifo_get(&gClient.fifo.lqi,1);
	}
	else
	{
		cc1100_fifo_get(&gRecvRssi,1);
		cc1100_fifo_get(&gRecvLqi,1);
	}

	return length - 2;
}



#define GET_FR() (gClient.status.input_data & 0xC0) >> 6

enum {
	CA_WAVE_CH5=3,
	CA_WAVE_CH9=2,
	CA_WAVE_CH13=1,
	CA_WAVE_CH17=0
};

u8	get_frq(void)
{
	return (((~get_input_data()) >> 6) & 0x03);
}


void cc1101_carrier_wave_setup(void)
{
	
	volatile u16	frq;
	volatile int 	fr;
	volatile u16 	offset;
	volatile u16 	level;
	volatile u16 	temperature;
	

	fr = get_frq();//GET_FR();
	
	//frq = 0x68FC + (fr * 0x0040);		//25kHz next	Ver0xAD
	//frq = 0x68FC + (fr * 0x003E);		//25kHz next	Ver0xAE
	frq = 0x68FC + (fr * 0x003F);		//25kHz next



	if(my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.freq_add_sub) == 0x80)
		frq -= my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.adj_freq);
	else
		frq += my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.adj_freq);


	temperature = temperature_get();


	if(my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.temper_add_sub) == 0x80)
		temperature -= my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.adj_temper);
	else
		temperature += my_eeprom_read_byte((u16)&save_eeprom_data.gctl_data.adj_temper);


	offset = 0x0339 - temperature;
	
	level = 13 - (offset / 0x0c);
	

	if(offset & 0x8000)
		level = 13;
	else if((offset / 0x0c)>13)
		level = 0;

	gLevel = level;

	frq += level_offset[level];


	cc1100_write_reg(CC1100_REG_FREQ2, 0x10);
	cc1100_write_reg(CC1100_REG_FREQ1, (frq >> 8)  & 0xff);
	cc1100_write_reg(CC1100_REG_FREQ0, frq & 0xff);
}



void init_voltcomparator(void)
{
	ACSR = 0x4b;
	DIDR1 = 0x02;
}



	 


void add_batt_low(void)
{
	u16 bat_val;

	B_CTL_HIGH;
	//mdelay(1);
	_delay_ms(1);


	bat_val = ad_get_val(AD_PIN_BATT_LOW,AD_VAL_FROM_L);

	gClient.status.bat_val = (bat_val>>2);


	B_CTL_LOW;
}


u8 get_sr_status(void)
{
	if((PINB >> 7) & 0x01)
		return 1;

	return 0;

}


#if 0

void sig_set(void)
{
	//mdelay(10);
	_delay_ms(10);
	
	gClient.status.make_alert = 0x00;


	//emg_status_old = emg_status;
	if(!(PINB  & 0x02))
		emg_status = 0x01;
	else
		emg_status = 0x00;

	//sr_status_old = sr_status;
	sr_status = get_sr_status();


	tampa_status_old = tampa_status;

//	tampa_status = ad_get_val(AD_PIN_TAMPA,AD_VAL_FROM_L);

	if(ad_get_val(AD_PIN_TAMPA,AD_VAL_FROM_L) > 0x80)
		tampa_status = 0;
	else
		tampa_status = 1;

	//tampa_status = ad_get_val(AD_PIN_TAMPA,AD_VAL_FROM_L);


	add_batt_low(); 


	//if(emg_status == 0x01 && emg_status == 0x01 && gClient.status.status != SL_ALARM_WAIT )
	if(emg_status == 0x01 && emg_status == 0x01 && gClient.status.status != SL_SLEEP )
		gClient.status.make_alert |= 0x08;

	if(sr_status_old ^ sr_status)
	{
		gClient.status.make_alert |= 0x04;
		sr_status_old = sr_status;
		
		if(sr_status==0x01)
		{
			gClient.status.make_alert |= 0x04;		//set
		}
		else
		{
			gClient.status.make_alert &= 0xFB;		//reset
		}
	}
	else
	{
		if(sr_status_old==0x01)
		{
			gClient.status.make_alert |= 0x04;		//set
		}
		else
		{
			gClient.status.make_alert &= 0xFB;		//reset
		}
		
	}

	if((tampa_status_old ^ tampa_status))
		gClient.status.make_alert |= 0x02;
	
	if(gClient.status.bat_val < 0xbf)
		gClient.status.make_alert |= (1<<0);	// BATT_LOW active
		
		
	get_user0_data = gClient.status.make_alert;
}


#endif

u8 	sig_status(void)
{
	
	u8 status = 0x00;
	gClient.status.make_alert = 0x00;


	//emg_status_old = emg_status;
	if(!(PINB  & 0x02))
		status|= 0x08;


	if(get_sr_status())
		status|= 0x04;


	tampa_status = ad_get_val(AD_PIN_TAMPA,AD_VAL_FROM_L);

	//if(tampa_status == 0x00)
	if(tampa_status < 0x80)
		status|= 0x02;


	add_batt_low(); 


	if(gClient.status.bat_val < 0xbf)
		status |= 0x01;

	return status;
}

u8 cmd[40];






void carrier_sense(void)
{
	while(PINC&0x04);		//Carrier sense
}

void carrier_no_sense(void)
{
	while(!(PINC&0x04));		//Carrier sense
}

void cc1101_tx_carrier(cc1101_client * client)
{

	cc1101_carrier_wave_setup();

	cc1100_cfg_txoff_mode(CC1100_TXOFF_MODE_STAY_TX);
	cc1100_cfg_manchester_en(CC1100_MANCHESTER_DISABLE);
	cc1100_write_reg(CC1100_REG_MDMCFG3,0x33);
	cc1100_cfg_mod_format(CC1100_MODULATION_ASK);
	cc1100_write_reg(CC1100_REG_FREND0,0x10);
	cc1100_cfg_gdo0(CC1100_GDOx_CLK_XOSC_1);

	/* IDLE */
	cc1100_cmd_idle();
	/* MANCAL*/
	cc1100_cmd_calibrate();
	/* FS WAKEUP */
	cc1100_cmd_flush_tx();

	cc1100_cfg_gdo0(0x0e);
	carrier_sense();

	cc1100_cmd_tx();
}


u8 gStatus=0x00;

u8 cc1101_tx(u8 * data , u8 length,u8 th)
{
	//int i;

	cc1101_rtx_reg_set(1); // rx
	//led_flasher(10,500);
	cc1101_carrier_wave_setup();
	cc1100_cfg_gdo0(0x0e);

	cc1100_cmd_idle();
	cc1100_cmd_flush_rx();
	cc1100_cmd_calibrate();
	cc1100_cmd_rx();
	
	//mdelay(5);
	_delay_ms(5);

	if(PINC & 0x04)
	{
		cc1100_cmd_idle();
		//mdelay(195);
		_delay_ms(195);
		
		if(th == 0)
			return 0;

	}


	cc1101_rtx_reg_set(0); // tx
	cc1101_carrier_wave_setup();
	cc1100_cmd_idle();
	cc1100_cmd_calibrate();
	tx_fifo_write(data,length);
	
	cc1100_write_reg(CC1100_REG_MCSM1,0x00);
	cc1100_cfg_gdo0(0x09);



	cc1100_cfg_gdo0(CC1100_GDOx_SYNC_WORD);
	cc1100_cmd_tx();
	while(!(PINC&0x04));	//data send start
	while(PINC&0x04);		//data send end

	return 1;
}



u8 cc1101_rx(u8 * data,u8 loop)
{
	u8 cnt_wait_syn_ack=0;
	u8 length;

	cc1101_rtx_reg_set(1); // rx
	cc1101_carrier_wave_setup();
	cc1100_cfg_gdo0(CC1100_GDOx_SYNC_WORD);

	cc1100_cmd_idle();
	cc1100_cmd_flush_rx();
	cc1100_cmd_calibrate();
	cc1100_cmd_rx();
	
	while(!(PINC & 0x04))
	{

		_delay_ms(1);
		if(++cnt_wait_syn_ack > 50)
			return 0;
	}

	while(1)
	{
		if(!( PINC & 0x04) )
		{	
			if((cc1100_status_crc_lqi() & 0x80))
			{

				length = rx_fifo_read(data);
				return length;
			}
			break;
		}
	}

	return 0;
}




u8 my_memcmp(u8 * src,u8 * dest,u8 len)
{
	u8 i;

	for(i= 0 ; i < len ; i++)
	{
		if(src[i] != dest[i]) return i;
	}

	return 0;
}



void rtc_alarm_disenable(void)
{
	//u8 ret;

	EIMSK &= ~(1<<INT0);
	//ret = rtc_1byte_data_write(0x0E,0x00);
	rtc_1byte_data_write(0x0E,0x00);
	//if(ret == 0)
		//ret = rtc_1byte_data_write(0x0E,0x00);
	//if(ret == 0)
		//ret = rtc_1byte_data_write(0x0E,0x00);

	//rtc_1byte_data_write(0x0E,0x00);		//Alarm enable disenable
}



ISR(USART_RX_vect)
{
	uart_data=UDR0;

	rx_char[uart_rx_index++]=uart_data;

	if(uart_data==0x0d)
	{
		uart_rx_length = uart_rx_index;
		//if(gClient.status.status!=MS_SEND2PC_DATA)
		if(send2pc_retry_cnt==0)
		{
			gClient.status.status=MS_UART_RECV;			
		}

		uart_rx_index = 0;
	}


}


void insert_sr_buf(void)
{
	if(SRBuf[0] == -1)
	{
		SRBuf[0] = (get_user0_data >> 2) & 0x01;
	}
	else
	{
		if(SRBuf[0] == 1 && !(get_user0_data & 0x40) && SRBuf[1] == -1)
			SRBuf[1] =  0;
		else if(SRBuf[0] == 0 && (get_user0_data & 0x40) && SRBuf[1] == -1)
			SRBuf[1] =  1;
	}
}

ISR(INT1_vect)
{
	
	#if 0
	//_delay_ms(10);
	_delay_ms(50);
	spi_signalpin_opendrain_nonactive();



	//led_flasher(2,100);
	sr_status = get_sr_status();



	if(gClient.status.status == SL_SEND_FAILE || gClient.status.status == SL_ALARM_WAIT || gClient.status.status == SL_SLEEP)
	{
		
		if(sr_status_old ^ sr_status)
		{
			gClient.status.status = SL_SEND;
			//sig_set();
			//throw = OFF;
			sr_status_old = sr_status;
			gClient.status.signal_type = SIG_EMG;
		}
		else
		{
			if(gClient.status.status == SL_SLEEP)
			{
				rtc_alarm_disenable();
				gClient.status.status = SL_SEND;
				gClient.status.signal_type = SIG_EMG;
				//gClient.status.send_err_cnt = 0;
				//sig_set();
			}
		}
	}
	#endif

}


ISR(INT0_vect)
{
	//_delay_ms(5);
	//_delay_ms(10);
	
	#if 0
	
	_delay_ms(50);
	spi_signalpin_opendrain_nonactive();
	
	//led_flasher(2,100);

	rtc_alarm_disenable();



	if(gClient.status.status == SL_SEND_RETRY_WAIT)
	{
		gClient.status.status = SL_SEND;
		gClient.status.signal_type = SIG_EMG;
	}
	else if(gClient.status.status == SL_SEND_FAILE || gClient.status.status == SL_ALARM_WAIT)
	{
		gClient.status.status = SL_SLEEP;
		throw = ON;
	}
	else if(gClient.status.status == SL_SLEEP)	
	{
		gClient.status.status = SL_SEND;
		gClient.status.signal_type = SIG_SCH;
	}

	#endif

}


/*
 	input_data
 	b0 b1 		type 00=master,01=slave,10=carrier,11=setup
*/

void device_status_setup(void)
{
	memcpy((u8 *)&gClient.status.serial,fix_id_mel,3);
	my_eeprom_read_block((u8 *)&gClient.status.serial[3],(u8 *)&save_eeprom_data.gctl_data.serial,3);
	//gClient.status.send_err_cnt=0;
	
	gClient.status.input_data = ~get_input_data();

	gClient.status.type = (gClient.status.input_data >> 2) & 0x3;
}


u8	get_alarm_time(void)
{
	return (((~get_input_data()) >> 4) & 0x03);
}




void rtc_set_next_alarm(u8 alarm)
{
	volatile int i;
	//u8 ret;

	if(alarm > 7)
	{
		led_flasher(20,500);
	}

	if(alarm < 4)
	{
		reg_net_alarm[3].data = time_set[alarm].hour;
		reg_net_alarm[4].data = time_set[alarm].min;
		reg_net_alarm[5].data = time_set[alarm].sec;
	
		for(i = 0 ;i < 7 ; i++)
		{
			//ret = rtc_1byte_data_write(reg_net_alarm[i].addr,reg_net_alarm[i].data);
			rtc_1byte_data_write(reg_net_alarm[i].addr,reg_net_alarm[i].data);
			//if(ret == 0)
				//ret = rtc_1byte_data_write(reg_net_alarm[i].addr,reg_net_alarm[i].data);
			//if(ret == 0)
				//ret = rtc_1byte_data_write(reg_net_alarm[i].addr,reg_net_alarm[i].data);
		}
	}
	else
	{
		reg_next_sch_alarm[3].data = day_cnt[alarm-4];

		for(i = 0 ;i < 9 ; i++)
		{
			//ret = rtc_1byte_data_write(reg_next_sch_alarm[i].addr,reg_next_sch_alarm[i].data);
			rtc_1byte_data_write(reg_next_sch_alarm[i].addr,reg_next_sch_alarm[i].data);
			//if(ret == 0)
				//ret = rtc_1byte_data_write(reg_next_sch_alarm[i].addr,reg_next_sch_alarm[i].data);
			//if(ret == 0)
				//ret = rtc_1byte_data_write(reg_next_sch_alarm[i].addr,reg_next_sch_alarm[i].data);
		}
	}
}


void temperature_carrier_test(void)
{
	cc1100_cfg_gdo2(0x30);		//26MHz XOSC/1  output
	while(1)
	{
		led_flasher(5,50);
		add_batt_low();
		cc1101_tx_carrier((cc1101_client *)&gClient);

		_delay_ms(5000);
		temperature_test_enable();
		
		_delay_ms(1000);
		temperature_test_disenable();
	}
}



void random_tx_test(void)
{

	while(1)
	{
		cc1100_cmd_idle();
		//sdelay(3);
		_delay_ms(3000);

		cc1101_rtx_reg_set(0); // tx
		cc1100_write_reg(CC1100_REG_PKTCTRL0,0x22);	//random TX mode
		cc1101_carrier_wave_setup();
		cc1100_cmd_idle();
		cc1100_cmd_calibrate();
		cc1100_cmd_tx();

		//cc1100_cmd_tx();
		wdt_disable();
		//sdelay(3);
		_delay_ms(3000);
	}
}






int enqueue(queue_t *que, fifo_t *enq_data)
{
	if(que->num < QUEUE_SIZE)
	{
		que->master2pc_data[(que->head + que->num) % QUEUE_SIZE].slave_id0 = enq_data->slave_id[3];
		que->master2pc_data[(que->head + que->num) % QUEUE_SIZE].slave_id1 = enq_data->slave_id[4];
		que->master2pc_data[(que->head + que->num) % QUEUE_SIZE].slave_id2 = enq_data->slave_id[5];
		que->master2pc_data[(que->head + que->num) % QUEUE_SIZE].user_data0 = enq_data->user_data0;
		que->master2pc_data[(que->head + que->num) % QUEUE_SIZE].user_data1 = enq_data->user_data1;
		que->master2pc_data[(que->head + que->num) % QUEUE_SIZE].bat_val = enq_data->bat_val;
		que->master2pc_data[(que->head + que->num) % QUEUE_SIZE].RecvRssi = gRecvRssi;
		que->master2pc_data[(que->head + que->num) % QUEUE_SIZE].rssi = enq_data->rssi;
		
		que->num ++;
		return SUCCESS;
	}
	else
	{
		return FAILURE;
	}
}




int dequeue(queue_t *que, master2pc_data_t *deq_data)
{
	//data_t d;
	if (que->num > 0)
	{
		*deq_data = que->master2pc_data[que->head];
		que->head = (que->head + 1) % QUEUE_SIZE;
		que->num --;
		return SUCCESS;
	}
	else
	{
		return FAILURE;
	}
}










u8 gggLength;


#if 1
void master_oper(cc1101_client * Client)
{
	int i=0;
	int j=0;
	int length=0;
	int id=0;
	fifo_t * fifo;
	u8 ack_serial[3];

	fifo = &gClient.fifo;

	while(1)
	{
		wdt_reset();

		switch(gClient.status.status)
		{
			case MS_DATA_RACV:
	

				length = cc1101_rx((u8 *)fifo,1);
				if(length > 0 )
				{
					fifo->master_ctl = my_eeprom_read_byte(&save_eeprom_data.greg_slave[id].slave_ctl_set);
					memcpy(fifo->master_id,&gClient.status.serial[3],3);
	
					if(setup_mode == SETUP_NONE)
					{
						id = reg_slave_num_serch(fifo->slave_id);
						if( id >= 0 )
						{
							//cc1101_tx((u8 *)fifo, sizeof(fifo_t) - sizeof(f_option_t),0);
			
							if(length < sizeof(fifo_t))
							{
							
								enqueue(&que,&gClient.fifo);
							
								//led_flasher(4,5);
							
							}
							
							cc1101_tx((u8 *)fifo, sizeof(fifo_t) - sizeof(f_option_t),0);
							
							led_flasher(4,5);
	
							//return;
						}
						//sdelay(2);		//ACK limit delay
					}
					else
					{
						u8 * pD;
	
						if(setup_mode == SETUP_WRITE)
						{
							fifo->setup_mode = SETUP_WRITE;
	
							memcpy((u8 *)&fifo->opt,(u8 *)&gOpt,sizeof(f_option_t));
							if(cc1101_tx((u8 *)fifo,sizeof(fifo_t),0) == true)
							{
								setup_mode = SETUP_READY;
								putstr((u8 *)"wr ok\r");
							}
						}
						else
						{
							putc_('w');
							putc_('r');
							putc_(' ');
							putc_('d');
							putc_('u');
							putc_('m');
							putc_('p');
							putc_(' ');
		
							pD = (u8 *) fifo;
							for(i =0 ;i < length ; i++)
							{
								hexToAsc(pD[i],(u8 *)&cmd[0]);
								putc_(cmd[0]);
								putc_(cmd[1]);
							}
							hexToAsc(gRecvRssi,(u8 *)&cmd[0]);
							putc_(cmd[0]);
							putc_(cmd[1]);
							hexToAsc(gRecvLqi,(u8 *)&cmd[0]);
							putc_(cmd[0]);
							putc_(cmd[1]);
							putc_(0x0d);
							//return;
							break;
						}
					}
				}
				
				
				
				if(gClient.status.status==MS_UART_RECV)
				{
					j=0;
					break;
				}
				
				//if(send2pc_retry_cnt!=0)
				//{
					//gClient.status.status=MS_SEND2PC_DATA;
					//j=0;
				//}
				//else
				{
					if(j>5)
					{
						gClient.status.status=MS_WAIT;
						j=0;
					}
					else
					{
						j++;
					}
				}
			
				break;
			
			case MS_WAIT:

				//_delay_ms(500);
				//if((dequeue(&que,&tmp_d) != FAILURE)&&(send2pc_retry_cnt == 0))
				//if((que.head != que.num)&&(send2pc_retry_cnt==0))
				if(send2pc_retry_cnt==0)
				{
					if(dequeue(&que,&tmp_d) != FAILURE)
					{
						gClient.status.status=MS_SEND2PC_DATA;
						send2pc_retry_cnt++;						
						break;
					}
					gClient.status.status=MS_DATA_RACV;
					break;
				}
				else if(send2pc_retry_cnt != 0)
				{
					gClient.status.status=MS_SEND2PC_DATA;
					break;
				}
				else
				{
					if(gClient.status.status==MS_UART_RECV)
					{
						break;
					}
					
					gClient.status.status=MS_DATA_RACV;					
				}
				//_delay_ms(500);
				//_delay_ms(500);			

	
				break;

			case MS_SEND2PC_DATA:

				if(que_check_time!=0)
				{
					gClient.status.status=MS_DATA_RACV;
					break;
				}
				else
				{
					//que_check_time = 33;		//que check time is every about 1sec
					que_check_time = 66;		//que check time is every about 1sec
				}
				
				//wdt_reset();
				master2pc_out(&tmp_d);
				//_delay_ms(500);
				//_delay_ms(500);
				//_delay_ms(500);
				//_delay_ms(500);
				//_delay_ms(500);
				memset(gWCmd,0,sizeof(gWCmd));
						
				pC = strtok(rx_char," ");
			
				j=0;
				while(pC != NULL)
				{
					strcpy(&gWCmd[j][0],pC);
					pC = strtok(NULL," ");
					j++;
				}
			
				if(j != 0)
				{
					if(strstr(&gWCmd[0][0],"ok") != NULL)
					{
						if(gWCmd[1][0] == NULL)				//for under Version 1.9-0.2   without new ack signal retry function.
						{
							gClient.status.status=MS_WAIT;
							send2pc_retry_cnt=0;
							break;
						}
						
						
						
						ack_serial[0] = AscToHex(&gWCmd[2][0]);
						ack_serial[1] = AscToHex(&gWCmd[2][2]);
						ack_serial[2] = AscToHex(&gWCmd[2][4]);
						
						if((tmp_d.slave_id0 != ack_serial[0]) || (tmp_d.slave_id1 != ack_serial[1]) || (tmp_d.slave_id2 != ack_serial[2]))
						{	
							
							if(send2pc_retry_cnt > 10)
							{
								gClient.status.status=MS_WAIT;
								send2pc_retry_cnt=0;			
								break;				
							}
							
							gClient.status.status=MS_DATA_RACV;
							send2pc_retry_cnt++;
							break;
						}

			
						//ack_id = AscToInt(&gWCmd[1][0]);
						//if(id == ack_id)
						//gClient.status.status=MS_BOOT;
						gClient.status.status=MS_WAIT;
						send2pc_retry_cnt=0;
						//send2pc_ready = 0;
						break;
						
					}
				}

	
				if(send2pc_retry_cnt > 10)
				{
					//gClient.status.status=MS_BOOT;
					gClient.status.status=MS_WAIT;
					send2pc_retry_cnt=0;
					//send2pc_ready = 0;
					break;
				}		
				send2pc_retry_cnt++;
				
				if(gClient.status.status==MS_UART_RECV)
				{
					break;
				}
				gClient.status.status=MS_DATA_RACV;
				break;
			default:
				break;
		
		
		}	
		
		if(gClient.status.status==MS_UART_RECV)
			return;
			
	}
}

#endif










#if 1

void master_main(void)
{
	//u8  ret=0;
	int i;
	u8 tex_test[3];
	
	while(1)
	{
		master_oper((cc1101_client *)&gClient);
	
		if(gClient.status.status==MS_UART_RECV)
		{
			//ret = 0;
			memset(gWCmd,0,sizeof(gWCmd));
			
			pC = strtok(rx_char," ");
			
			i=0;
			while(pC != NULL)
			{
				strcpy(&gWCmd[i][0],pC);
				pC = strtok(NULL," ");
				i++;
			}

			if(i != 0)
			{
				if(strstr(&gWCmd[0][0],"wr") != NULL)
				{
					if(strstr(&gWCmd[1][0],"env") != NULL)
					{
						int id=0;

						id = AscToInt(&gWCmd[2][0]);
						if(id >= 0 && id < 100)
						{
							tSlave.serial[0] = AscToHex(&gWCmd[3][0]);
							tSlave.serial[1] = AscToHex(&gWCmd[3][2]);
							tSlave.serial[2] = AscToHex(&gWCmd[3][4]);
							tSlave.slave_ctl_set = AscToHex(&gWCmd[4][0]);


							my_eeprom_read_block((u8 *)&tSlave2,(u8 *)&save_eeprom_data.greg_slave[id],sizeof(reg_slave));

							if(memcmp((u8 *)&tSlave,&tSlave2,sizeof(reg_slave)) != 0)
							{
								eeprom_busy_wait();		//EEPROM使用可能まで待機
								eeprom_write_block(&tSlave, &save_eeprom_data.greg_slave[id], sizeof(reg_slave)); //write 8byte 
							}

							//ret = 1;
							putstr((u8 * )"ok\r");
						}
					}
				}
				else if(strstr(&gWCmd[0][0],"rd") != NULL)
				{
					if(strstr(&gWCmd[1][0],"env") != NULL)
					{
						int id=0;

						id = AscToInt(&gWCmd[2][0]);
						if(id >= 0 && id < 100)
						{
							my_eeprom_read_block((u8 *)&tSlave2,(u8 *)&save_eeprom_data.greg_slave[id],sizeof(reg_slave));

							i=0;
							cmd[i++] = 'r';
							cmd[i++] = 'd';
							cmd[i++] = ' ';
							cmd[i++] = 'e';
							cmd[i++] = 'n';
							cmd[i++] = 'v';
							cmd[i++] = ' ';
							hexToAsc(tSlave2.serial[0],(u8 *)&cmd[i]);
							i+=2;
							hexToAsc(tSlave2.serial[1],(u8 *)&cmd[i]);
							i+=2;
							hexToAsc(tSlave2.serial[2],&cmd[i]);
							i+=2;
							cmd[i++] = ' ';
							hexToAsc(tSlave2.slave_ctl_set,&cmd[i]);
							i+=2;
							cmd[i++] = 0x0d;
							cmd[i]=0;
							putstr(cmd);
						}
					}
				}
				else if(strstr(&gWCmd[0][0],"setenv") != NULL)
				{
					if(strstr(&gWCmd[1][0],"enable") != NULL)
					{
						wdt_disable();
						setup_mode = SETUP_READY;
						putstr((u8 * )"ok\r");
					}
					else if(strstr(&gWCmd[1][0],"write") != NULL)
					{
						gOpt.env_data[0] = AscToHex(&gWCmd[2][0]); // serial 0
						gOpt.env_data[1] = AscToHex(&gWCmd[2][2]); // serial 1
						gOpt.env_data[2] = AscToHex(&gWCmd[2][4]); // serial 2
						gOpt.env_data[3] = AscToHex(&gWCmd[3][0]); // ctl

						gOpt.env_data[4] = AscToHex(&gWCmd[4][0]); // 
						gOpt.env_data[5] = AscToHex(&gWCmd[4][2]);
						gOpt.env_data[6] = AscToHex(&gWCmd[4][4]);
						gOpt.env_data[7] = AscToHex(&gWCmd[4][6]);

						gOpt.level[0] =  AscToHex(&gWCmd[5][0]);
						gOpt.level[1] =  AscToHex(&gWCmd[5][2]);
						gOpt.level[2] =  AscToHex(&gWCmd[5][4]);
						gOpt.level[3] =  AscToHex(&gWCmd[5][6]);
						gOpt.level[4] =  AscToHex(&gWCmd[5][8]);
						gOpt.level[5] =  AscToHex(&gWCmd[5][10]);
						gOpt.level[6] =  AscToHex(&gWCmd[5][12]);

						setup_mode = SETUP_WRITE;
						putstr((u8 * )"ok\r");
					}
					else if(strstr(&gWCmd[1][0],"save") != NULL)
					{
						tCtl.serial[0] = AscToHex(&gWCmd[2][0]);
						tCtl.serial[1] = AscToHex(&gWCmd[2][2]);
						tCtl.serial[2] = AscToHex(&gWCmd[2][4]);

						tCtl.adj_freq = AscToHex(&gWCmd[4][0]);
						tCtl.freq_add_sub = AscToHex(&gWCmd[4][2]);
						tCtl.adj_temper = AscToHex(&gWCmd[4][4]);
						tCtl.temper_add_sub = AscToHex(&gWCmd[4][6]);
						eeprom_busy_wait();		//EEPROM使用可能まで待・
						eeprom_write_block(&tCtl, &save_eeprom_data.gctl_data, sizeof(ctl_data)); //write 8byte 
						putstr((u8 * )"ok\r");
					}
					else
					{
						setup_mode = SETUP_NONE;
					}
				}
				else if(strstr(&gWCmd[0][0],"osccal") != NULL)
				{
					u8 osccal_old = OSCCAL;
					if(strstr(&gWCmd[1][0],"i") != NULL)
					{
						tnew_ctl_data.fix_osccal_val += 1;
						putstr((u8 * )"u\r");
					}
					else
					{
						//OSCCAL -= 1;
						tnew_ctl_data.fix_osccal_val -= 1;
						putstr((u8 * )"d\r");
					}

					if(osccal_old != tnew_ctl_data.fix_osccal_val)
					{
						OSCCAL = tnew_ctl_data.fix_osccal_val;

						//for(i=0;i<3;i++)

						eeprom_busy_wait();
						eeprom_write_block(&tnew_ctl_data.fix_osccal_val,&save_eeprom_data.gnew_ctl_data.fix_osccal_val,1);
						//eeprom_write_byte(&gctl_data.fix_osccal_val + i,&tCtl.fix_osccal_val+i);
						//my_eeprom_read_block((u16 *)&tnew_ctl_data,(u16 *)&save_eeprom_data.gnew_ctl_data,3);
						//my_eeprom_read_block((u8 *)&tnew_ctl_data,(u8 *)&save_eeprom_data.gnew_ctl_data,sizeof(ctl_data));
						putstr((u8 * )" ORG: ");
						hexToAsc(tnew_ctl_data.org_osccal_val,&tex_test[0]);
						tex_test[2] = 0x0d;
						putstr(tex_test);
						
						putstr((u8 * )"     FIX: "); 
						hexToAsc(OSCCAL,&tex_test[0]);
						tex_test[2] = 0x0d;
						putstr(tex_test);
						putstr((u8 * )" fix_ok\r");

					}

				}
				else
				{

				}
			}

			memset(rx_char,0,60);
			//gClient.status.status = MS_BOOT;
			gClient.status.status = MS_WAIT;
		}
	}
}

#endif


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






#if 1

u8 setup_oper(void)
{
	u8	ret;
	int 	length;
	fifo_t * fifo;


	fifo = &gClient.fifo;

	

	temperature_get();

	slave_fifo_preset(fifo);
	ret = cc1101_tx((u8 *)fifo,sizeof(fifo_t),0);

	if(ret == 1)
	{
		length = cc1101_rx((u8 *)fifo,0);
		if(length > 0)
		{
			gggLength = length;
			if(length == sizeof(fifo_t) && fifo->setup_mode == SETUP_WRITE)
			{
				memcpy((u8 *)tCtl.serial,(u8 *)&fifo->opt.env_data[0],3);
				tCtl.adj_freq = fifo->opt.env_data[4];
				tCtl.freq_add_sub = fifo->opt.env_data[5];
				tCtl.adj_temper = fifo->opt.env_data[6];
				tCtl.temper_add_sub = fifo->opt.env_data[7];
	
				memcpy((u8 *)&gClient.status.serial[3],(u8 *)tCtl.serial,3);
				memcpy((u8 *)&fifo->slave_id[3],(u8 *)tCtl.serial,3);

				eeprom_busy_wait();		
				eeprom_write_block(&tCtl, &save_eeprom_data.gctl_data, sizeof(ctl_data)); 

				hex_change_level(fifo->opt.level);

				//gTestMode = 0xab;
				//eeprom_busy_wait();		//EEPROM使用可能まで待・
				//eeprom_write_block(&gTestMode, &test_mode, 1); //write 8byte 
				//gTestMode = 0;

				cc1101_tx_carrier((cc1101_client *)&gClient);
				//sdelay(5);
				_delay_ms(5000);

				return 1;
			}
			fifo->length=0;
		}
	}
	return 0;
}

#endif


extern u8 pt_data[];
void setup_main(void)
{
	while(1)
	{
		if(setup_mode == SETUP_READY)
		{
			led_flasher(1,100);
		}
		else
		{
			led_flasher(5,50);
			pt_data[0] = 0xc0;			//10dBm
			//pt_data[0] = 0xc8;			//7dBm
			//pt_data[0] = 0x84;			//5dBm
			//pt_data[0] = 0x60;			//0dBm
			cc1101_8PATABLE_write_reg();
			setup_oper();

		}
	}
}


void check_env(void)
{
	u8  i;
	u8 * pD;
	u8	fixid[3];
	//u8	test;

	
	my_eeprom_read_block((u8 *)&tCtl,(u8 *)&save_eeprom_data.gctl_data,sizeof(ctl_data));
	//my_eeprom_read_block((u8 *)&tnew_ctl_data.fix_osccal_val,(u8 *)&save_eeprom_data.gnew_ctl_data.fix_osccal_val,3); 
	my_eeprom_read_block((u8 *)&tnew_ctl_data,(u8 *)&save_eeprom_data.gnew_ctl_data,sizeof(new_ctl_data)); 

	pD = (u8 *)&tCtl;

	for(i = 0 ;i  < 5 ; i++)
	{
		if(pD[i] != 0xff && pD[i] != 0x00) break;
	}

	if(i == 5)
	{
		// init;
		tCtl.adj_freq = 0x24;
		tCtl.freq_add_sub = 0x80;
		tCtl.adj_temper = 0x18;
		tCtl.temper_add_sub = 0x80;
		eeprom_busy_wait();		
		eeprom_write_block(&tCtl,&save_eeprom_data.gctl_data,sizeof(ctl_data));
	}

	my_eeprom_read_block((u8 *)level_offset,(u8 *)&save_eeprom_data.eprom_level,sizeof(level_offset));


	pD = (u8 *)&level_offset;

	for(i = 0 ;i  < sizeof(level_offset) ; i++)
	{
		if(pD[i] != 0xff && pD[i] != 0x00) break;
	}


	if(i == 5)
	{
		eeprom_busy_wait();		
		eeprom_write_block(default_offset,&save_eeprom_data.eprom_level,sizeof(level_offset));
		memcpy(level_offset,default_offset,sizeof(level_offset));

	}

	my_eeprom_read_block((u8 *)fixid,(u8 *)&save_eeprom_data.eprom_fixid,sizeof(save_eeprom_data.eprom_fixid));

	if(fixid[0] == 0x00 || fixid[0] == 0xff )
	{

	}
	else
	{
		fix_id_mel[0] = fixid[0];
		fix_id_mel[1] = fixid[1];
		fix_id_mel[2] = fixid[2];
	}

}



void test_wave_mode(void)
{
	while(1)
	{
		device_status_setup();
		
		if(gClient.status.type == MASTER)
		{
			led_flasher(50,10);
			while(1)
			{
				device_status_setup();
				if(gClient.status.type == SETUP)
				{
					led_flasher(50,5);
					random_tx_test();
				}
				_delay_ms(2000);
			}
		}
		_delay_ms(2000);
	}
}




void master2pc_out(master2pc_data_t *d)
{
		
	#if 1
	int i=0;
	cmd[i++] = 'w';
	cmd[i++] = 'r';
	cmd[i++] = ' ';
	cmd[i++] = 'e';
	cmd[i++] = 'm';
	cmd[i++] = 'g';
	cmd[i++] = ' ';
	hexToAsc(d->slave_id0,&cmd[i]);
	i+=2;
	hexToAsc(d->slave_id1,&cmd[i]);
	i+=2;
	hexToAsc(d->slave_id2,&cmd[i]);
	i+=2;
	cmd[i++] = ' ';
	hexToAsc(d->user_data0,&cmd[i]);
	i+=2;
	cmd[i++] = ' ';
	hexToAsc(d->user_data1,&cmd[i]);
	i+=2;
	cmd[i++] = ' ';
	hexToAsc(d->bat_val,&cmd[i]);
	i+=2;
	cmd[i++] = ' ';
	hexToAsc(d->RecvRssi,&cmd[i]);
	i+=2;
	cmd[i++] = ' ';
	hexToAsc(d->rssi,&cmd[i]);
	i+=2;
	cmd[i++] = 0x0d;
	cmd[i]=0;
	putstr(cmd);
	
	#endif	
	
}


ISR(TIMER0_COMPA_vect)	//タイマ割り込み
{

	#if 0

	//if(gClient.status.status!=MS_SEND2PC_DATA)
	if(gClient.status.status==MS_WAIT)
	{
		if(que_check_time != 0)
		{
			que_check_time--;
		}
		else
		{
			que_check_time = 10;		//que check time is every about 300msec
		}


		if(que_check_time == 0)
		{
			if(dequeue(&que,&tmp_d) != FAILURE)
			{
				gClient.status.status=MS_SEND2PC_DATA;
				send2pc_retry_cnt = 0;
			
			}
		}
	}

	#endif


	//if(gClient.status.status!=MS_SEND2PC_DATA)
	{
		if(que_check_time != 0)
		{
			que_check_time--;
		}
	}



	#if 0
	
	if(data_send_wait_time != 0)
	{
		data_send_wait_time--;
	}

	
	
	if(que_check_time != 0)
	{
		que_check_time--;
	}
	else
	{
		que_check_time = 10;		//que check time is every about 300msec
	}
	
	
	if (data_send_wait_time == 0 && que_check_time == 0)
	{
		if(dequeue(&que,&tmp_d) != FAILURE)
		{
			for(i=0;i<7;i++)
			{
				wdt_reset();
				master2pc_out(&tmp_d);
				_delay_ms(1000);
				memset(gWCmd,0,sizeof(gWCmd));
							
				pC = strtok(rx_char," ");
				
				j=0;
				while(pC != NULL)
				{
					strcpy(&gWCmd[i][0],pC);
					pC = strtok(NULL," ");
					j++;
				}
			
				if(j != 0)
				{
					if(strstr(&gWCmd[0][0],"OK") != NULL)
					{
						break;
					}
				}
			}			
			
			data_send_wait_time = 15;		//About 500msec wait next send data;
			//data_send_wait_time = 180;	//About 6 sec wait next send data;
			//data_send_wait_time = 90;	//About 3 sec wait next send data;
			//data_send_wait_time = 120;	//About 4 sec wait next send data;
		}
	}
	
	#endif	

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
	i2c_init();
	check_env();

	device_status_setup();

	#if 1
	if(gClient.status.type == MASTER)
	{
		timer_init();
	}
	#endif


    cc1100_init();

	cc1101_init_reg();
	pt_data[0] = 0xC0;				//10dB
	//pt_data[0] = 0xc8;			//7dBm
	//pt_data[0] = 0x84;			//5dBm
	//pt_data[0] = 0x60;			//0dB
	cc1101_8PATABLE_write_reg();

	


	//set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	led_flasher(2,10);

	#if 0
	if(gClient.status.type == SLAVE || gClient.status.type == MASTER)
	{
		EIMSK |= (1<<INT1);
		sr_status = get_sr_status();
		sr_status_old = sr_status;
	}
	
	#endif
	

	if(gClient.status.type == SLAVE || gClient.status.type == MASTER)
	{
		wdt_enable(WDTO_4S);
		WDTCSR |= (1 << WDIE);
	}

	sei();
	
	
	if(tnew_ctl_data.fix_osccal_flag != 0x01 || tnew_ctl_data.prog_ver_sub != PRG_VERSION_SUB || tnew_ctl_data.prog_ver_major != PRG_VERSION_MAJOR || tnew_ctl_data.device_type != MASTER)
	{
		if(tnew_ctl_data.fix_osccal_flag != 0x01)
		{
			tnew_ctl_data.fix_osccal_val = OSCCAL;
		}
		tnew_ctl_data.fix_osccal_flag = 0x01;
		tnew_ctl_data.prog_ver_sub = PRG_VERSION_SUB;
		tnew_ctl_data.prog_ver_major = PRG_VERSION_MAJOR;
		tnew_ctl_data.org_osccal_val = OSCCAL;
		tnew_ctl_data.device_type = MASTER;

		eeprom_busy_wait();
		//eeprom_write_block(&tnew_ctl_data.fix_osccal_val,&save_eeprom_data.gnew_ctl_data.fix_osccal_val,4);
		eeprom_write_block(&tnew_ctl_data.fix_osccal_val,&save_eeprom_data.gnew_ctl_data.fix_osccal_val,sizeof(new_ctl_data));		
		//eeprom_write_block(&tCtl, &gctl_data, sizeof(ctl_data)); //write 8byte
	}
	
	
	#if 1
	if(gClient.status.type == MASTER)
	{
		OSCCAL = tnew_ctl_data.fix_osccal_val;
	}	
	#endif
	
	
	spi_signalpin_opendrain_nonactive();

	if(gClient.status.type == SLAVE || gClient.status.type == MASTER)
		rtc_alarm_disenable();


	/* Enter the infinite loop */
	
	
    while (1)
    {
		
	
		#if 1
		if(gClient.status.type == TEST_MDOE)
		{
			//temperature_carrier_test();

			test_wave_mode();
			
		}
		else if(gClient.status.type == SETUP)
		{
			setup_mode = SETUP_TRX;
			setup_main();
		}
		else
		{
			//gClient.status.status = SL_SLEEP;
			//slave_main();
			//gClient.status.status = MS_BOOT;
			gClient.status.status = MS_WAIT;
			master_main();
		}
		
		#endif
	}
}





