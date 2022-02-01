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
#include "esp_log.h"

#include "ili9340.h"
#include "i2cdev.h"

#include <time.h>
#include "machine_ili9341.h"
#include <math.h>


typedef struct _machine_hw_ili9341_obj_t {
    mp_obj_base_t base;
    TFT_t *display;
    uint8_t mosi;
    uint8_t miso;
    uint8_t sclk;
    uint8_t cs;
    uint8_t dc;
    uint8_t reset;
    uint8_t bl;
} machine_hw_ili9341_obj_t;


const mp_obj_type_t machine_hw_ili9341_type;
STATIC TFT_t display;
STATIC machine_hw_ili9341_obj_t machine_hw_ili9341_obj;

STATIC void machine_hw_ili9341_internal(machine_hw_ili9341_obj_t *self) {
    mp_int_t ili9341_code = 0;

    mp_printf(&mp_plat_print, "STEPA [OK].\n");

    spi_master_init(self->display, self->mosi, self->sclk, self->cs, self->dc, -1, -1);

    mp_printf(&mp_plat_print, "SPI connection [OK].\n");

    lcdInit(self->display, self->display->_model, self->display->_width, self->display->_height, self->display->_offsetx, self->display->_offsety);

    mp_printf(&mp_plat_print, "LDC connection [OK].\n");
}

STATIC void machine_hw_ili9341_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
	//machine_hw_mpu6886_obj_t *self = MP_OBJ_TO_PTR(self_in);
	
    //mp_printf(print, "COPTER(%u, scl=%u, sda=%u, freq=%u)",
    //    self->port, self->scl, self->sda, I2C_APB_CLK_FREQ / (h + l));

}

mp_obj_t machine_hw_ili9341_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    // Parse args
	enum { ARG_mosi, ARG_miso, ARG_sclk, ARG_dc, ARG_cs, ARG_width, ARG_height, ARG_offsetX, ARG_offsetY};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mosi, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 23} },
        { MP_QSTR_miso, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 38} },
        { MP_QSTR_sclk, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 18} },
        { MP_QSTR_dc, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 15} },
        { MP_QSTR_cs, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5} },
        { MP_QSTR_width, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 320} },
        { MP_QSTR_height, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 240} },
        { MP_QSTR_offsetX, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_offsetY, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_printf(&mp_plat_print, "STEP0 [OK].\n");

    // Get static peripheral object
    machine_hw_ili9341_obj_t *self = (machine_hw_ili9341_obj_t *)&machine_hw_ili9341_obj;

    bool first_init = false;
    if (self->base.type == NULL) {
        // Created for the first time, set default pins
        self->base.type = &machine_hw_ili9341_type;
        self->display = &display;
        self->display->_model = 0x9341;
        self->display->_offsetx = args[ARG_offsetX].u_int;
        self->display->_offsety = args[ARG_offsetY].u_int;
        self->display->_width = args[ARG_width].u_int;
        self->display->_height = args[ARG_height].u_int;
        self->mosi = args[ARG_mosi].u_int;
        self->miso = args[ARG_miso].u_int;
        self->sclk = args[ARG_sclk].u_int;
        self->cs = args[ARG_cs].u_int;
        self->dc = args[ARG_dc].u_int;
        
        first_init = true;
    }

    mp_printf(&mp_plat_print, "STEP1 [OK].\n");

    // Initialise the Display peripheral
    machine_hw_ili9341_internal(self);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_ili9341_irq_mode(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t ili9341_code = -1;
    // uint8_t mode;

    // if (n_args < 2) {
    //     mode = read_g_mode();
    //     return mp_obj_new_int(mode);
    // }

    // mode = mp_obj_get_int(args[1]);

    // ili9341_code = write_g_mode(mode);

    return mp_obj_new_int(ili9341_code);   
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_ili9341_irq_mode_obj, 1, 2, mp_ili9341_irq_mode);


STATIC const mp_rom_map_elem_t hw_ili9341_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_irq_mode), MP_ROM_PTR(&mp_ili9341_irq_mode_obj) },
    // { MP_ROM_QSTR(MP_QSTR_IRQ_POLLING), MP_ROM_INT(ili9341_IRQ_POLLING) },
    // { MP_ROM_QSTR(MP_QSTR_IRQ_TRIGGER), MP_ROM_INT(ili9341_IRQ_TRIGGER) },
    
};

STATIC MP_DEFINE_CONST_DICT(hw_ili9341_globals, hw_ili9341_globals_table);

const mp_obj_type_t machine_hw_ili9341_type = {
	{ &mp_type_type },
    .name = MP_QSTR_ili9341c,
    //.print = machine_hw_ili9341_print,
    .make_new = machine_hw_ili9341_make_new,
    .locals_dict = (mp_obj_dict_t *)&hw_ili9341_globals,
};

MP_REGISTER_MODULE(MP_QSTR_ili9341c, machine_hw_ili9341_type, MODULE_ILI9341_ENABLED);