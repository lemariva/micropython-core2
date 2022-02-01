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

#include "ft6336u.h"
#include "i2cdev.h"

#include <time.h>
#include "machine_ft6336u.h"
#include <math.h>

#define I2C_DEFAULT_TIMEOUT_US (10000) // 10ms
#define I2C_0_DEFAULT_SCL (GPIO_NUM_22)
#define I2C_0_DEFAULT_SDA (GPIO_NUM_21)

I2cDef pSensorBusDef;
I2cDrv pSensorsBus;

typedef struct _machine_hw_ft6336u_obj_t {
    mp_obj_base_t base;
    i2c_port_t port : 8;
    gpio_num_t scl : 8;
    gpio_num_t sda : 8;
} machine_hw_ft6336u_obj_t;


const mp_obj_type_t machine_hw_ft6336u_type;
STATIC machine_hw_ft6336u_obj_t machine_hw_ft6336u_obj[I2C_NUM_MAX];

STATIC void machine_hw_ft6336u_init(machine_hw_ft6336u_obj_t *self, uint32_t freq, uint32_t timeout_us, bool first_init) {
    mp_int_t ft6336u_code = 0;
    // bus definition
    pSensorBusDef.i2cPort = I2C_NUM_0;
    pSensorBusDef.i2cClockSpeed = freq;
    pSensorBusDef.gpioSDAPin = self->sda;
    pSensorBusDef.gpioSCLPin = self->scl;
    pSensorBusDef.gpioPullup = GPIO_PULLUP_ENABLE;
    pSensorsBus.def = &pSensorBusDef;
    
    i2cdevInit(&pSensorsBus);
    mp_hal_delay_us(100000);
    
    ft6336u_code = ft6336uInit(&pSensorsBus);

    mp_hal_delay_us(100000);

    // test ft6336u 
    if (ft6336u_code == FT6336U_OK) {
        mp_printf(&mp_plat_print, "FT6336U I2C connection [OK].\n");
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("FT6336U I2C connection [FAIL].\n"));
    }
}

STATIC void machine_hw_ft6336u_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
	//machine_hw_mpu6886_obj_t *self = MP_OBJ_TO_PTR(self_in);
	
    //mp_printf(print, "COPTER(%u, scl=%u, sda=%u, freq=%u)",
    //    self->port, self->scl, self->sda, I2C_APB_CLK_FREQ / (h + l));

}

mp_obj_t machine_hw_ft6336u_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
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
    machine_hw_ft6336u_obj_t *self = (machine_hw_ft6336u_obj_t *)&machine_hw_ft6336u_obj[i2c_id];

    bool first_init = false;
    if (self->base.type == NULL) {
        // Created for the first time, set default pins
        self->base.type = &machine_hw_ft6336u_type;
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
    machine_hw_ft6336u_init(self, args[ARG_freq].u_int, args[ARG_timeout].u_int, first_init);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_ft6336u_irq_mode(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t ft6336u_code = FT6336U_ERROR_NOTTY;
    uint8_t mode;

    if (n_args < 2) {
        mode = read_g_mode();
        return mp_obj_new_int(mode);
    }

    mode = mp_obj_get_int(args[1]);

    ft6336u_code = write_g_mode(mode);

    return mp_obj_new_int(ft6336u_code);   
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_ft6336u_irq_mode_obj, 1, 2, mp_ft6336u_irq_mode);


STATIC mp_obj_t mp_ft6336u_touch_status(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_obj_t tuple[2];
    
    tuple[0] = mp_obj_new_int(read_gesture_id());
    tuple[1] = mp_obj_new_int(read_td_status());

    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_ft6336u_touch_status_obj, 1, 1, mp_ft6336u_touch_status);

STATIC mp_obj_t mp_ft6336u_touch_check(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_obj_t tuple[3];
    
    tuple[0] = mp_obj_new_int(read_firmware_id());
    tuple[1] = mp_obj_new_int(read_release_code_id());
    tuple[2] = mp_obj_new_int(read_library_version());

    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_ft6336u_touch_check_obj, 1, 1, mp_ft6336u_touch_check);


STATIC mp_obj_t mp_ft6336u_touch_points(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_obj_t tuple_pX[6];

    FT6336U_TouchPointType touchPoint = touchPointScan();
    
    tuple_pX[0] = mp_obj_new_int(touchPoint.tp[0].status);
    tuple_pX[1] = mp_obj_new_int(touchPoint.tp[0].x);
    tuple_pX[2] = mp_obj_new_int(touchPoint.tp[0].y);

    tuple_pX[3] = mp_obj_new_int(touchPoint.tp[1].status);
    tuple_pX[4] = mp_obj_new_int(touchPoint.tp[1].x);
    tuple_pX[5] = mp_obj_new_int(touchPoint.tp[1].y);

    return mp_obj_new_tuple(6, tuple_pX);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_ft6336u_touch_points_obj, 1, 1, mp_ft6336u_touch_points);


STATIC const mp_rom_map_elem_t hw_ft6336u_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_irq_mode), MP_ROM_PTR(&mp_ft6336u_irq_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&mp_ft6336u_touch_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_check), MP_ROM_PTR(&mp_ft6336u_touch_check_obj) },
    { MP_ROM_QSTR(MP_QSTR_points), MP_ROM_PTR(&mp_ft6336u_touch_points_obj) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_POLLING), MP_ROM_INT(FT6336U_IRQ_POLLING) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_TRIGGER), MP_ROM_INT(FT6336U_IRQ_TRIGGER) },
    
};

STATIC MP_DEFINE_CONST_DICT(hw_ft6336u_globals, hw_ft6336u_globals_table);

const mp_obj_type_t machine_hw_ft6336u_type = {
	{ &mp_type_type },
    .name = MP_QSTR_ft6336u,
    //.print = machine_hw_ft6336u_print,
    .make_new = machine_hw_ft6336u_make_new,
    .locals_dict = (mp_obj_dict_t *)&hw_ft6336u_globals,
};

MP_REGISTER_MODULE(MP_QSTR_ft6336u, machine_hw_ft6336u_type, MODULE_FT6336_ENABLED);