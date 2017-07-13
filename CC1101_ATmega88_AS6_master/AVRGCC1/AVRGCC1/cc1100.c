/*
 * Copyright  2008-2009 INRIA/SensTools
 * 
 * <dev-team@sentools.info>
 * 
 * This software is a set of libraries designed to develop applications
 * for the WSN430 embedded hardware platform.
 * 
 * This software is governed by the CeCILL license under French law and
 * abiding by the rules of distribution of free software.  You can  use, 
 * modify and/ or redistribute the software under the terms of the CeCILL
 * license as circulated by CEA, CNRS and INRIA at the following URL
 * "http://www.cecill.info". 
 * 
 * As a counterpart to the access to the source code and  rights to copy,
 * modify and redistribute granted by the license, users are provided only
 * with a limited warranty  and the software&apos;s author,  the holder of the
 * economic rights,  and the successive licensors  have only  limited
 * liability. 
 * 
 * In this respect, the user&apos;s attention is drawn to the risks associated
 * with loading,  using,  modifying and/or developing or reproducing the
 * software by the user in light of its specific status of free software,
 * that may mean  that it is complicated to manipulate,  and f that  also
 * therefore means  that it is reserved for developers  and  experienced
 * professionals having in-depth computer knowledge. Users are therefore
 * encouraged to load and test the software&apos;s suitability as regards their
 * requirements in conditions enabling the security of their systems and/or 
 * data to be ensured and,  more generally, to use and operate it in the 
 * same conditions as regards security. 
 * 
 * The fact that you are presently reading this means that you have had
 * knowledge of the CeCILL license and that you accept its terms.
 */

#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
//#include <avr/iomx8p.h>
//#include <avr/stdint.h>
#define F_CPU 8000000UL
#include <util/delay.h>


#include "spi1.h"

#include "cc1100.h"
#include "cc1100_gdo.h"
#include "cc1100_globals.h"
#include "macro.h"

static u16 (*gdo0_cb)(void);
static u16 (*gdo2_cb)(void);



void u100delay(u8 usec);
void mdelay(u16 msec);



void cc1100_reinit(void)
{
    spi1_init();
}

void cc1100_init(void)
{

	//volatile unsigned char test_val1 = 0;
	//volatile unsigned char test_val2 = 0;
	//volatile unsigned char test_val3 = 0;
	//u8 pt_data[] = {0xC0,0x50,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7};

	gdo0_cb = 0x0;
	gdo2_cb = 0x0;
  
	spi1_init();
	GDO_INIT();
  
	spi1_select(SPI1_CC1100);
	spi1_deselect(SPI1_CC1100);
	spi1_select(SPI1_CC1100);
	spi1_deselect(SPI1_CC1100);
	//udelay(80);
	//u100delay(1);
	//mdelay(1);
	_delay_ms(1);
	spi1_select(SPI1_CC1100);
	while (spi1_read_somi()) ;
	spi1_write_single(CC1100_STROBE_SRES | CC1100_ACCESS_STROBE);
	while (spi1_read_somi()) ;
	spi1_deselect(SPI1_CC1100);

	//mdelay(1);
	_delay_ms(1);


	//setup 426.361755MHz
	cc1100_write_reg(CC1100_REG_FREQ2, 0x10);
	//cc1100_write_reg(CC1100_REG_FREQ2, 0x10);
	cc1100_write_reg(CC1100_REG_FREQ1, 0x66);
	cc1100_write_reg(CC1100_REG_FREQ0, 0x07);


}

u8 cc1100_read_reg(u8 addr)
{
	u8 reg;
	spi1_select(SPI1_CC1100);
	spi1_write_single(addr | CC1100_ACCESS_READ);
	reg = spi1_read_single();
	spi1_deselect(SPI1_CC1100);
	return reg;
}

void cc1100_write_reg(u8 addr, u8 value)
{
	spi1_select(SPI1_CC1100);
	spi1_write_single(addr | CC1100_ACCESS_WRITE);
	spi1_write_single(value);
	spi1_deselect(SPI1_CC1100);
}

u8 cc1100_strobe_cmd(u8 cmd)
{
	u8 ret;
	spi1_select(SPI1_CC1100);
	while(spi1_read_somi());
	//mdelay(3);
	ret = spi1_write_single(cmd | CC1100_ACCESS_STROBE);
	spi1_deselect(SPI1_CC1100);
	return ret;
}

/*add 20100113 blackz*/
u8 cc1100_sleep_strobe_cmd(u8 cmd)
{
	u8 ret;
	//spi1_select(SPI1_CC1100);
	//while(spi1_read_somi());
	//mdelay(3);
	ret = spi1_write_single(cmd | CC1100_ACCESS_STROBE);
	//spi1_deselect(SPI1_CC1100);
	return ret;
}

void cc1100_fifo_put(u8* buffer, u16 length)
{
  spi1_select(SPI1_CC1100);
  spi1_write_single(CC1100_DATA_FIFO_ADDR | CC1100_ACCESS_WRITE_BURST);
  spi1_write(buffer, length);
  spi1_deselect(SPI1_CC1100);
}

//void cc1100_fifo_get(u8* buffer, u16 length)
void cc1100_fifo_get(u8* buffer, u16 length)
{
	spi1_select(SPI1_CC1100);
	spi1_write_single(CC1100_DATA_FIFO_ADDR | CC1100_ACCESS_READ_BURST);
	spi1_read(buffer, length);
	spi1_deselect(SPI1_CC1100);
}

//u8 cc1100_read_status(u8 addr)
u8 cc1100_read_status(u8 addr)
{
	u8 temp;

	temp = cc1100_read_reg(addr | CC1100_ACCESS_STATUS);


	return temp;
}


void cc1100_gdo0_register_callback(u16 (*cb)(void))
{
	gdo0_cb = cb;
}

void cc1100_gdo2_register_callback(u16 (*cb)(void))
{
	gdo2_cb = cb;
}

#define STATE_IDLE    0
#define STATE_RX      1
#define STATE_TX      2
#define STATE_FSTXON  3
#define STATE_CALIB   4
#define STATE_SETTL   5
#define STATE_RXOVER  6
#define STATE_TXUNDER 7

#define WAIT_STATUS(status) \
	while ( ((cc1100_cmd_nop()>>4) & 0x7) != status) ;

void cc1100_cmd_calibrate(void)
{
	u8 status;
	cc1100_cmd_idle();
	cc1100_strobe_cmd(CC1100_STROBE_SCAL);

	do
	{
		status = ((cc1100_cmd_nop()>>4) & 0x7);
	}
	while(status  != STATE_IDLE );
	//WAIT_STATUS(STATE_IDLE);
}

void cc1100_cmd_idle(void)
{
	switch ((cc1100_cmd_nop() >> 4) & 0x7)
	{
		case STATE_RXOVER:
			cc1100_cmd_flush_rx();
			break;
		case STATE_TXUNDER:
			cc1100_cmd_flush_tx();
			break;
		default:
		cc1100_strobe_cmd(CC1100_STROBE_SIDLE);
	}
	WAIT_STATUS(STATE_IDLE);
}






u8 pt_data[] = {0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00};		//10dB


reg_t init_all[10]={
					{CC1100_REG_PKTCTRL1,0x04},
					{CC1100_REG_PKTCTRL0,0x45},
					{CC1100_REG_FSCTRL1,0x0C},
					{CC1100_REG_MCSM0,0x04},
					#ifdef GFSK_DEF
					{CC1100_REG_MDMCFG2,0x17},			//GFSK
					#else
					{CC1100_REG_MDMCFG2,0x07},			//2-FSK]
					#endif
					{CC1100_REG_PKTCTRL0,0x05},
					{CC1100_REG_MDMCFG3,0x83},
					{CC1100_REG_MDMCFG4,0x88},

					//{CC1100_REG_DEVIATN,0x12},				//4kHz

					
					#ifdef GFSK_DEF
					//{CC1100_REG_DEVIATN,0x17},				//6kHz
					//{CC1100_REG_DEVIATN,0x15},				//5.1kHz
					//{CC1100_REG_DEVIATN,0x14},				//4.7kHz
					{CC1100_REG_DEVIATN,0x13},					//4.36kHz
					//{CC1100_REG_DEVIATN,0x27},				//12kHz
					//{CC1100_REG_DEVIATN,0x07},					//3kHz
					//{CC1100_REG_DEVIATN,0x12},				//4kHz
					//{CC1100_REG_DEVIATN,0x22},				//8kHz
					//{CC1100_REG_DEVIATN,0x11},				//3.5kHz
					//{CC1100_REG_DEVIATN,0x10},				//3.2kHz
					//{CC1100_REG_DEVIATN,0x0F},				//3.0kHz
					//{CC1100_REG_DEVIATN,0x0E},				//2.8kHz
					#else
					//{CC1100_REG_DEVIATN,0x17},				//6kHz
					//{CC1100_REG_DEVIATN,0x15},				//5.1kHz
					//{CC1100_REG_DEVIATN,0x14},				//4.7kHz
					{CC1100_REG_DEVIATN,0x13},					//4.36kHz
					//{CC1100_REG_DEVIATN,0x07},					//3kHz
					//{CC1100_REG_DEVIATN,0x12},				//4kHz
					//{CC1100_REG_DEVIATN,0x22},				//8kHz	
					//{CC1100_REG_DEVIATN,0x11},				//3.5kHz
					//{CC1100_REG_DEVIATN,0x10},				//3.2kHz
					//{CC1100_REG_DEVIATN,0x0F},				//3.0kHz
					//{CC1100_REG_DEVIATN,0x0E},				//2.8kHz
					#endif
					

					{CC1100_REG_MDMCFG4,0xF8},
					{CC1100_REG_AGCCTRL1,0x50},
				};




void cc1101_init_reg(void)
{
	int i;

	for(i = 0 ;i < 10 ; i++) 
	{ 
		cc1100_write_reg(init_all[i].addr,init_all[i].data);
	} 
}


void cc1101_rtx_reg_set(u8 rtx)
{
	cc1101_init_reg();
	cc1100_write_reg(CC1100_REG_PKTCTRL1,rtx == 1 ? 0x0c : 0x04);
}






void cc1101_8PATABLE_write_reg(void)
{
	u8 i;


	spi1_select(SPI1_CC1100);
	spi1_write_single(0x3e | CC1100_ACCESS_WRITE_BURST);
	for(i=0;i<8;i++)
	{
		spi1_write_single(pt_data[i]);
	}
	spi1_deselect(SPI1_CC1100);

}




