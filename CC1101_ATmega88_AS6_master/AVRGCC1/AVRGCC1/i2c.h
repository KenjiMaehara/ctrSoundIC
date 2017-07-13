#ifndef I2C_H
#define I2C_H

void i2c_init(void);
void rtc_1byte_data_write(u8 addr,u8 data);
u8 rtc_1byte_data_read(u8 addr);
void rtc_3byte_data_read(u8 start_addr,u8 * data);

#endif
