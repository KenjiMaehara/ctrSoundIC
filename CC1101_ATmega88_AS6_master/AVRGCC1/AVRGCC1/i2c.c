#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include <avr/sleep.h>
#include <util/twi.h>
#define F_CPU 8000000UL
#include <util/delay.h>


#include "macro.h"


void i2c_error(void);
void i2c_init(void);
u8 i2c_read_ack(void);
u8 i2c_read_nak(void);
void i2c_start(u8 id);
void i2c_stop(void);
void i2c_wait_int_clear(void);
void i2c_write(u8 d);
u8 rtc_1byte_data_read(u8 addr);
void rtc_1byte_data_write(u8 addr,u8 data);
void rtc_3byte_data_read(u8 start_addr,u8 * data);







// �G���[
//u8 i2c_error_flag = 0;
void i2c_error(void)
{
    //mdelay(10);
	_delay_ms(10);
	
    //i2c_error_flag = 1;
}




// �ʐM���x�̏�����
void i2c_init(void)
{
    TWSR = 0b00000000;  // 1����
    //TWBR = 0;          // 250k = 4MHz / (16 + 2 * TWBR * 1)
	//TWBR = 12;          // 250k = 4MHz / (16 + 2 * TWBR * 1)
	TWBR = 32;          // 100k = 4MHz / (16 + 2 * TWBR * 1)
}


void i2c_wait_int_clear(void)
{
	u8 i=0;

	while( !(TWCR & _BV(TWINT)) )
	{
		//udelay(250);
		_delay_us(250);
		i++;
		if(i > 100) return;
	}
}
// master 1byte���M
void i2c_write(u8 d)
{
    TWDR = d; // ���M�f�[�^
    TWCR = _BV(TWINT) | _BV(TWEN);
    i2c_wait_int_clear();//while( !(TWCR & _BV(TWINT)) ) ; // �f�[�^�̑��o�����ҋ@
    if((TWSR & TW_STATUS_MASK) != TW_MT_DATA_ACK) i2c_error();
}

// master 1byte��M(ack��Ԃ�)
u8 i2c_read_ack(void)
{
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
    i2c_wait_int_clear();//while( !(TWCR & _BV(TWINT)) ) ; // ��M�����҂�
    if((TWSR & TW_STATUS_MASK) != TW_MR_DATA_ACK) i2c_error();
    return TWDR; // �f�[�^��Ԃ�
}

// master 1byte��M(noack��Ԃ�)
u8 i2c_read_nak(void)
{
    TWCR = _BV(TWINT) | _BV(TWEN);
    i2c_wait_int_clear();//while( !(TWCR & _BV(TWINT)) ) ; // ��M�����҂�
    if((TWSR & TW_STATUS_MASK) != TW_MR_DATA_NACK) i2c_error();
    return TWDR; // �f�[�^��Ԃ�
}

// master���M�J�n�AID��slave address ���� 1
void i2c_start(u8 id)
{
    u8 s;

    // �J�n�����𑗂�
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    i2c_wait_int_clear();//while( !(TWCR & _BV(TWINT)) ) ; // �J�n�����̑��o�����ҋ@
    s = TWSR & TW_STATUS_MASK;
    if(s != TW_START && s != TW_REP_START) i2c_error();

    // �A�h���X�𑗂�
    TWDR = id;
    TWCR = _BV(TWINT) | _BV(TWEN);
    i2c_wait_int_clear();//while( !(TWCR & _BV(TWINT)) ) ; // �A�h���X�̑��o�����ҋ@
    s = TWSR & TW_STATUS_MASK;
    if(s != TW_MT_SLA_ACK && s != TW_MR_SLA_ACK) i2c_error();
}

// master���M�I��
void i2c_stop(void)
{
	int i=0;
	
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
	
	while(!(TWCR & _BV(TWSTO)))
	{
		_delay_ms(1);
		i++;
		if(i>500)
		{
			break;
		}
	}
    // status��TW_NO_INFO�ɂȂ�
}



#define RTC_R2223L (0x32 << 1)    // RICOH RTC R2223L/T��slave address


void rtc_1byte_data_write(u8 addr,u8 data)
{
    /*write*/
    //cli();
    //i2c_error_flag =  0;
    i2c_start(RTC_R2223L | TW_WRITE);
    i2c_write((addr<<4)&0xf0); // register 4
    i2c_write(data); // 14bit resolution
    i2c_stop();

    //sei();
    //if(i2c_error_flag == 1) return false;

    //return true;
}


u8 rtc_1byte_data_read(u8 addr)
{
	u8 data;

	//cli();
	i2c_start(RTC_R2223L | TW_WRITE);
	i2c_write((addr << 4)&0xF0); // register 0
	i2c_start(RTC_R2223L | TW_READ);
	data = i2c_read_nak(); // ���ʓǂݍ���(�Ō��NAK��Ԃ�)
	i2c_stop();
	//sei();
	return data;
}


void rtc_3byte_data_read(u8 start_addr,u8 * data)
{
	i2c_start(RTC_R2223L | TW_WRITE);
	i2c_write((start_addr << 4)&0xF0); // register 0
	i2c_start(RTC_R2223L | TW_READ);
	data[0] = i2c_read_ack(); // ��ʓǂݍ���
	data[1] = i2c_read_ack(); // ��ʓǂݍ���
	data[2] = i2c_read_nak(); // ���ʓǂݍ���(�Ō��NAK��Ԃ�)
	i2c_stop();
}


