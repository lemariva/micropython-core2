#include "bm8563.h"
#include "driver/i2c.h"
#include <string.h>

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d, line = %d", rc, __LINE__); } } while(0);
#define I2C_NUM 0

static uint8_t byte2BCD(uint8_t data) {
    return ((data / 10) << 4) + data % 10;
}

static uint8_t BCD2Byte(uint8_t data) {
    return (data >> 4) * 10 + (data & 0x0f);
}

void bm8563_init() {
    i2c_init();
    i2c_write_byte(0x51, 0x00, 0x00);
    i2c_write_byte(0x51, 0x01, 0x00);
    i2c_write_byte(0x51, 0x0D, 0x00);
}

void bm8563_setTime(rtc_date_t* data) {
    if (data == NULL) {
        return ;
    }
    uint8_t time_buf[7];
    time_buf[0] = byte2BCD(data->second);
    time_buf[1] = byte2BCD(data->minute);
    time_buf[2] = byte2BCD(data->hour);
    time_buf[3] = byte2BCD(data->day);
    time_buf[5] = byte2BCD(data->month) | (data->year >= 2000 ? 0x00 : 0x80);
    time_buf[6] = byte2BCD(data->year % 100);
    i2c_write(0x51, 0x02, time_buf, 7);
}

void bm8563_getTime(rtc_date_t* data) {
    if (data == NULL) {
        return ;
    }
    uint8_t time_buf[7];
    i2c_read(0x51, 0x02, time_buf, 7);
    data->second = BCD2Byte(time_buf[0] & 0x7f);
    data->minute = BCD2Byte(time_buf[1] & 0x7f);
    data->hour = BCD2Byte(time_buf[2] & 0x3f);
    data->day = BCD2Byte(time_buf[3] & 0x3f);
    data->month = BCD2Byte(time_buf[5] & 0x0f);
    data->year = BCD2Byte(time_buf[6]) + (time_buf[5] & 0x80 ? 1900 : 2000);
}

// -1 :disable
void bm8563_setDateIRQ(int8_t minute, int8_t hour, int8_t day, int8_t week) {
    uint8_t irq_enable = false;
    uint8_t out_buf[4] = { 0x80, 0x80, 0x80, 0x80 };
    if(minute >= 0) {
        irq_enable = true;
        out_buf[0] = byte2BCD(minute) & 0x7f;
    }

    if(hour >= 0) {
        irq_enable = true;
        out_buf[1] = byte2BCD(hour) & 0x3f;
    }

    if(day >= 0) {
        irq_enable = true;
        out_buf[2] = byte2BCD(day) & 0x3f;
    }

    if(week >= 0) {
        irq_enable = true;
        out_buf[3] = byte2BCD(week) & 0x07;
    }

    uint8_t reg_value = 0;
    i2c_read(0x51, 0x01, &reg_value, 1);
    if (irq_enable) {
        reg_value |= (1 << 1);
    } else {
        reg_value &= ~(1 << 1);
    }

    i2c_write(0x51, 0x09, out_buf, 4);
    i2c_write(0x51, 0x01, &reg_value, 1);
}

// -1: disable
int16_t bm8563_setTimerIRQ(int16_t value) {
    uint8_t reg_value = 0;
    i2c_read(0x51, 0x01, &reg_value, 1);

    if (value < 0) {
        reg_value &= ~(1 << 0);
        i2c_write(0x51, 0x01, &reg_value, 1);
        reg_value = 0x03;
        i2c_write(0x51, 0x0E, &reg_value, 1);
        return -1;
    }

    uint8_t type_value = 2;
    uint8_t div = 1;
    if (value > 255) {
        div = 60;
        type_value = 0x83;
    } else {
        type_value = 0x82;
    }
    value = (value / div) & 0xFF;
    i2c_write(0x51, 0x0F, (uint8_t *)&value, 1);
    i2c_write(0x51, 0x0E, &type_value, 1);

    reg_value |= (1 << 0);
    reg_value &= ~(1 << 7);
    i2c_write(0x51, 0x01, &reg_value, 1);
    return value * div;
}

int16_t bm8563_getTimerTime() {
    uint8_t value = 0;
    uint8_t type_value = 0;
    i2c_read(0x51, 0x0f, &value, 1);
    i2c_read(0x51, 0x0e, &type_value, 1);

    if ((type_value & 0x03) == 3) {
        return value * 60;
    } else {
        return value;
    }
}

uint8_t bm8563_getIRQ() {
    uint8_t data;
    i2c_read(0x51, 0x01, &data, 1);
    return data;
}

void bm8563_clearIRQ() {
    uint8_t data;
    i2c_read(0x51, 0x01, &data, 1);
    i2c_write_byte(0x51, 0x01, data & 0xf3);
}