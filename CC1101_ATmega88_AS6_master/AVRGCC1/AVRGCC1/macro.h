#ifndef _MACRO_H_
#define _MACRO_H_

#define						GFSK_DEF
#define						PRG_VERSION_SUB		0x0f
#define						PRG_VERSION_MAJOR	0x00



typedef unsigned char 	  u8;
typedef unsigned short	u16;
typedef unsigned char 	  uint8_t;

//extern u8 status;

//#define	bool			u8
#define false			0
#define true			1

#define	LOW				0
#define	HIGH			1


#define PCDDR	DDRC
#define PCSEL	PINC
#define PCINTIE


#define PBDDR	DDRB

#define B_CTL				REGISTER_BIT(PORTB,0)
#define CS_CC1101			REGISTER_BIT(PORTB,2)

#define CS_CC1101_LOW			CS_CC1101 =0
#define CS_CC1101_HIGH			CS_CC1101 =1


#define B_CTL_LOW			B_CTL =0
#define B_CTL_HIGH			B_CTL =1

#define OUTPUT_HIGH			OUTPUT = 1
#define OUTPUT_LOW			OUTPUT = 0


#define SCLR_HIGH			SCLR = 1
#define SCLR_LOW			SCLR = 0


enum {
	SIG_EMG=0,	
	SIG_SCH
};


enum {
	SET_ALARM=0,
	NONE_SET_ALARM	
};



enum {
	STARTUP=0,
	CHK_INPUT,
	DATA_SEND,
	CHK_DATA_SEND,
	POWER_DONW
};


enum {
	SETUP_NONE=0,
	SETUP_READY,
	SETUP_TRX,
	SETUP_WRITE
};

enum {
	MASTER=0,
	TEST_MDOE,
	SLAVE,
	SETUP,
	BRIDGE
};

enum {
	
	OSC_8MHZ=0,
	OSC_1MHZ
};




enum {
	//SL_SLEEP=0,
	//SL_SEND,
	//SL_SEND_FAILE,
	//SL_SEND_RETRY_WAIT,
	//SL_ALARM_WAIT,
//	MS_BOOT,
//	MS_DATA_RACV,
	MS_UART_RECV=0,
	MS_SEND2PC_DATA,
	MS_DATA_RACV,
	MS_WAIT

};


enum {
	SR_RESET=0,
	SR_SET
};

enum {
	OFF=0,
	ON
};


enum {
	ALARM_2SEC=0,
	ALARM_1MIN,
	ALARM_2MIN,
	ALARM_3MIN,
	ALARM_5MIN,
	ALARM_30MIN,
	ALARM_1hour,
	ALARM_24hour
};



enum {
	SLEEP_2SEC = 0,
	SLEEP_1MIN = 1,
	SLEEP_2MIN = 3,
	SLEEP_5MIN = 9
};



enum {
	AD_PIN_TEMPERATURE=0x02,
	AD_PIN_TAMPA=0x06,
	AD_PIN_BATT_LOW=0x07
};


enum {
	AD_VAL_FROM_R=0,
	AD_VAL_FROM_L=0x20
};


typedef struct _ctl_data_
{
	u8 adj_freq;
	u8 freq_add_sub;
	u8 serial[3];
	u8 adj_temper;
	u8 temper_add_sub;
}ctl_data;



typedef struct _rtc_time_
{
	
	u8 chk_hour;
	u8 chk_min;
	u8 curt_hour;
	u8 curt_min;

}rtc_time;




typedef struct _fifo_option_
{
	u8 env_data[8];
	u8 level[7];
}f_option_t;


typedef struct _txfifo_data_ 
{
	u8 length;
	u8 slave_id[6];
	u8 master_id[3];
	u8 user_data0;
	u8 user_data1;
	u8 bat_val;
	u8 master_ctl;
	u8 setup_mode;
	u8 version_l;
	u8 rssi;
	u8 lqi;
	f_option_t opt;
}fifo_t;


typedef struct _reg_slave_
{
	u8 serial[3];
	u8 slave_ctl_set;
}reg_slave;


typedef struct _reg_cmd_
{
	u8 length;
	u8 cmd[4];
}reg_cmd;


typedef struct _reg_param_
{
	u8 length;
	u8 param[4];
}reg_param;





typedef struct _client_status_
{
	u8 setup_mode;
	u8 version_l;
	u8 type;
	u8 serial[6];
	u8 sessionflag;
	u8 user_data0;
	u8 user_data1;
	u8 bat_val;
	u8 system_osc;
	u8 status;
	u8 send_err_cnt;
	u8 alert;
	u8 alert_old_temp;
	u8 make_alert;
	u8 make_alert_old;
	u8 sleep_time;
	u8 input_data;
	u8 fixed_temperature;
	u16 fixed_temperature02;
	u8 signal_type;
	u8 reserve[7];

}client_status;


typedef struct _cc1101_client_
{
	client_status status;
	fifo_t			fifo;
	ctl_data ctl;
}cc1101_client;


typedef struct _new_ctl_data_
{
	u8	fix_osccal_val;
	u8	fix_osccal_flag;
	u8	prog_ver_sub;
	u8	org_osccal_val;
	u8	prog_ver_major;
	u8	device_type;
	u8	reg[10];
}new_ctl_data;



typedef struct 
{ 
  unsigned int bit0:1; 
  unsigned int bit1:1; 
  unsigned int bit2:1; 
  unsigned int bit3:1; 
  unsigned int bit4:1; 
  unsigned int bit5:1; 
  unsigned int bit6:1; 
  unsigned int bit7:1; 
} _io_reg; 

typedef struct _time_set_
{
	u8 hour;
	u8 min;
	u8 sec;
}time_set_t;


typedef struct _save_eeprom_data_
{
	ctl_data		gctl_data;
	reg_slave		greg_slave[100];
	int				eprom_level[14];
	char			eprom_fixid[3];
	new_ctl_data	gnew_ctl_data;
}save_eeprom_data_t;



extern u8 edge_val_gdo0;
extern u8 edge_val_gdo1;

extern u8 gdo0_status;
extern u8 gdo1_status;





#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt 




#endif
