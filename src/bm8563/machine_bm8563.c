/*
* Copyright 2020 LeMaRiva|Tech (Mauro Riva) info@lemariva.com
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*     http://www.apache.org/licenses/LICENSE-2.0
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/


#include <stdio.h>
#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "extmod/machine_i2c.h"
#include "esp_log.h"

#include "bm8563.h"
#include "i2cdev.h"

#include <time.h>
#include "machine_bm8563.h"
#include <math.h>

#define I2C_DEFAULT_TIMEOUT_US (10000) // 10ms
#define I2C_0_DEFAULT_SCL (GPIO_NUM_22)
#define I2C_0_DEFAULT_SDA (GPIO_NUM_21)

I2cDef pSensorBusDef;
I2cDrv pSensorsBus;

typedef struct _machine_hw_bm8563_obj_t {
    mp_obj_base_t base;
    i2c_port_t port : 8;
    gpio_num_t scl : 8;
    gpio_num_t sda : 8;
} machine_hw_bm8563_obj_t;


const mp_obj_type_t machine_hw_bm8563_type;
STATIC machine_hw_bm8563_obj_t machine_hw_bm8563_obj[I2C_NUM_MAX];

STATIC void machine_hw_bm8563_init(machine_hw_bm8563_obj_t *self, uint32_t freq, uint32_t timeout_us, bool first_init) {
    mp_int_t bm8563_code = 0;
    // bus definition
    pSensorBusDef.i2cPort = I2C_NUM_0;
    pSensorBusDef.i2cClockSpeed = freq;
    pSensorBusDef.gpioSDAPin = self->sda;
    pSensorBusDef.gpioSCLPin = self->scl;
    pSensorBusDef.gpioPullup = GPIO_PULLUP_ENABLE;
    pSensorsBus.def = &pSensorBusDef;
    
    i2cdevInit(&pSensorsBus);
    mp_hal_delay_us(100000);
    
    bm8563_code = pcf8563Init(&pSensorsBus);

    mp_hal_delay_us(100000);

    // test bm8563 
    if (bm8563_code == PCF8563_OK) {
        mp_printf(&mp_plat_print, "bm8563 I2C connection [OK].\n");
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("bm8563 I2C connection [FAIL].\n"));
    }
}

STATIC void machine_hw_bm8563_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
	//machine_hw_mpu6886_obj_t *self = MP_OBJ_TO_PTR(self_in);
	
    //mp_printf(print, "COPTER(%u, scl=%u, sda=%u, freq=%u)",
    //    self->port, self->scl, self->sda, I2C_APB_CLK_FREQ / (h + l));

}

mp_obj_t machine_hw_bm8563_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    // Parse args
	enum { ARG_scl, ARG_sda, ARG_freq, ARG_timeout };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_scl, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sda, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_freq, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 400000} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = I2C_DEFAULT_TIMEOUT_US} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Get I2C bus
    mp_int_t i2c_id = 0;

    // Get static peripheral object
    machine_hw_bm8563_obj_t *self = (machine_hw_bm8563_obj_t *)&machine_hw_bm8563_obj[i2c_id];

    bool first_init = false;
    if (self->base.type == NULL) {
        // Created for the first time, set default pins
        self->base.type = &machine_hw_bm8563_type;
        self->port = i2c_id;
        self->scl = I2C_0_DEFAULT_SCL;
        self->sda = I2C_0_DEFAULT_SDA;
        first_init = true;
    }

    // Set SCL/SDA pins if given
    if (args[ARG_scl].u_obj != MP_OBJ_NULL) {
        self->scl = mp_hal_get_pin_obj(args[ARG_scl].u_obj);
    }
    if (args[ARG_sda].u_obj != MP_OBJ_NULL) {
        self->sda = mp_hal_get_pin_obj(args[ARG_sda].u_obj);
    }

    // Initialise the I2C peripheral
    machine_hw_bm8563_init(self, args[ARG_freq].u_int, args[ARG_timeout].u_int, first_init);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_bm8563_datetime(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t bm8563_code = PCF8563_ERROR_NOTTY;
	struct tm time;
    mp_obj_t tuple[7];

    strptime("2020-11-22 16:00", "%Y-%m-%dT %H:%M", &time);
    //(year, month, day, wday, hour, minute, second)
    if (n_args < 8) {
        //get time
        bm8563_code = pcf8563_read(&time); 
        if (bm8563_code != PCF8563_OK)
            return mp_obj_new_int(bm8563_code);

        tuple[0] = mp_obj_new_int(time.tm_year + 1900);
        tuple[1] = mp_obj_new_int(time.tm_mon + 1);
        tuple[2] = mp_obj_new_int(time.tm_mday);
        tuple[3] = mp_obj_new_int(time.tm_wday);
        tuple[4] = mp_obj_new_int(time.tm_hour);
        tuple[5] = mp_obj_new_int(time.tm_min);
        tuple[6] = mp_obj_new_int(time.tm_sec);

        return mp_obj_new_tuple(7, tuple);
        
	} else {
        // set time
        time.tm_year = mp_obj_get_int(args[1]);
        time.tm_mon = mp_obj_get_int(args[2]) - 1;
        time.tm_mday = mp_obj_get_int(args[3]);
        time.tm_wday = mp_obj_get_int(args[4]);
        time.tm_hour = mp_obj_get_int(args[5]);
        time.tm_min = mp_obj_get_int(args[6]);
        time.tm_sec = mp_obj_get_int(args[7]);
        mktime(&time);

        bm8563_code = pcf8563_write(&time);

        return mp_obj_new_int(bm8563_code);
    }
    
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_bm8563_datetime_obj, 1, 8, mp_bm8563_datetime);


STATIC mp_obj_t mp_bm8563_alarm(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t bm8563_code = PCF8563_ERROR_NOTTY;
	struct tm time;
    mp_obj_t tuple[7];

    strptime("2020-11-22 16:00", "%Y-%m-%dT %H:%M", &time);
    //(year, month, day, wday, hour, minute, second)
    if (n_args < 7) {
        //get alarm
        bm8563_code = pcf8563_ioctl(PCF8563_ALARM_READ, &time); 
        if (bm8563_code != PCF8563_OK)
            return mp_obj_new_int(bm8563_code);

        tuple[0] = mp_obj_new_int(time.tm_year + 1900);
        tuple[1] = mp_obj_new_int(time.tm_mon + 1);
        tuple[2] = mp_obj_new_int(time.tm_mday);
        tuple[3] = mp_obj_new_int(time.tm_wday);
        tuple[4] = mp_obj_new_int(time.tm_hour);
        tuple[5] = mp_obj_new_int(time.tm_min);

        return mp_obj_new_tuple(6, tuple);
        
	} else {
        // set alarm
        time.tm_year = mp_obj_get_int(args[1]);
        time.tm_mon = mp_obj_get_int(args[2]) - 1;
        time.tm_mday = mp_obj_get_int(args[3]);
        time.tm_wday = mp_obj_get_int(args[4]);
        time.tm_hour = mp_obj_get_int(args[5]);
        time.tm_min = mp_obj_get_int(args[6]);
        mktime(&time);

        bm8563_code = pcf8563_ioctl(PCF8563_ALARM_SET, &time);

        return mp_obj_new_int(bm8563_code);
    }
    
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_bm8563_alarm_obj, 1, 7, mp_bm8563_alarm);


STATIC mp_obj_t mp_bm8563_timer(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t bm8563_code = PCF8563_ERROR_NOTTY;
    uint8_t timer, clock;

    if (n_args < 2) {
        //get timer
        bm8563_code = pcf8563_ioctl(PCF8563_TIMER_READ, &timer); 
        if (bm8563_code != PCF8563_OK)
            return mp_obj_new_int(bm8563_code);

        return mp_obj_new_int(timer);

	}
    
    if (n_args == 2) {
        // set timer
        timer = mp_obj_get_int(args[1]);
        bm8563_code = pcf8563_ioctl(PCF8563_TIMER_WRITE, &timer);

        return mp_obj_new_int(bm8563_code);

    } else {
        // set timer and clock
        timer = mp_obj_get_int(args[1]);
        clock = mp_obj_get_int(args[2]);
        bm8563_code = pcf8563_ioctl(PCF8563_TIMER_WRITE, &timer);
        if (bm8563_code != PCF8563_OK)
            return mp_obj_new_int(bm8563_code);
        bm8563_code = pcf8563_ioctl(PCF8563_TIMER_CONTROL_WRITE, &clock);
        
        return mp_obj_new_int(bm8563_code);
    }
    
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_bm8563_timer_obj, 1, 3, mp_bm8563_timer);


STATIC mp_obj_t mp_bm8563_activate_irq(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t bm8563_code = PCF8563_ERROR_NOTTY;
    uint8_t data = 0, type = 0;
    bool clear = false;

    if (n_args < 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("one argument is required"));
    }

    type = mp_obj_get_int(args[1]);

    if (n_args == 3) {
        clear = (bool)mp_obj_get_int(args[2]);
    }

    switch (type) {
        case (int)PCF8563_AIE:
            data = PCF8563_AIE;
            if (clear)
                data |= PCF8563_AF;
            break;
        case (int)PCF8563_TIE:
            data = PCF8563_TIE;
            if (clear)
                data |= PCF8563_TF;
            break;
        case (int)(PCF8563_AIE + PCF8563_TIE):
            data = PCF8563_AIE + PCF8563_TIE;
            if (clear)
                data |= PCF8563_AF + PCF8563_TF;
    }

    bm8563_code = pcf8563_ioctl(PCF8563_CONTROL_STATUS2_WRITE, &data);

    return mp_obj_new_int(bm8563_code);   
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_bm8563_activate_irq_obj, 1, 3, mp_bm8563_activate_irq);


STATIC mp_obj_t mp_bm8563_activate_timer(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t bm8563_code = PCF8563_ERROR_NOTTY;
    bool start = true;

    if (n_args == 2) {
        start = (bool) mp_obj_get_int(args[1]);
    }

    bm8563_code = pcf8563_start_timer(start);
    
    return mp_obj_new_int(bm8563_code);   
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_bm8563_activate_timer_obj, 1, 2, mp_bm8563_activate_timer);


STATIC const mp_rom_map_elem_t hw_bm8563_globals_table[] = {

    { MP_ROM_QSTR(MP_QSTR_datetime), MP_ROM_PTR(&mp_bm8563_datetime_obj) },
    { MP_ROM_QSTR(MP_QSTR_alarm), MP_ROM_PTR(&mp_bm8563_alarm_obj) },
    { MP_ROM_QSTR(MP_QSTR_timer), MP_ROM_PTR(&mp_bm8563_timer_obj) },
    { MP_ROM_QSTR(MP_QSTR_activate_timer), MP_ROM_PTR(&mp_bm8563_activate_timer_obj) },
     { MP_ROM_QSTR(MP_QSTR_activate_irq), MP_ROM_PTR(&mp_bm8563_activate_irq_obj) },
    { MP_ROM_QSTR(MP_QSTR_TIMER_4_096KHZ), MP_ROM_INT(PCF8563_TIMER_4_096KHZ) },
    { MP_ROM_QSTR(MP_QSTR_TIMER_64HZ), MP_ROM_INT(PCF8563_TIMER_64HZ) },
    { MP_ROM_QSTR(MP_QSTR_TIMER_1HZ), MP_ROM_INT(PCF8563_TIMER_1HZ) },
    { MP_ROM_QSTR(MP_QSTR_TIMER_1_60HZ), MP_ROM_INT(PCF8563_TIMER_1_60HZ) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_ALARM), MP_ROM_INT(PCF8563_AIE) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_TIMER), MP_ROM_INT(PCF8563_TIE) },
    
};

STATIC MP_DEFINE_CONST_DICT(hw_bm8563_globals, hw_bm8563_globals_table);

const mp_obj_type_t machine_hw_bm8563_type = {
	{ &mp_type_type },
    .name = MP_QSTR_bm8563,
    //.print = machine_hw_bm8563_print,
    .make_new = machine_hw_bm8563_make_new,
    .locals_dict = (mp_obj_dict_t *)&hw_bm8563_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bm8563, machine_hw_bm8563_type, MODULE_BM8563_ENABLED);
