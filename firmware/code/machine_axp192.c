#include <stdio.h>
#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "extmod/machine_i2c.h"
#include "esp_log.h"

#include "axp20x.h"
#include "i2cdev.h"

#include "machine_axp192.h"
#include <math.h>

#define I2C_DEFAULT_TIMEOUT_US (10000) // 10ms
#define I2C_0_DEFAULT_SCL (GPIO_NUM_22)
#define I2C_0_DEFAULT_SDA (GPIO_NUM_21)

I2cDef pSensorBusDef;
I2cDrv pSensorsBus;

typedef struct _machine_hw_axp192_obj_t {
    mp_obj_base_t base;
    i2c_port_t port : 8;
    gpio_num_t scl : 8;
    gpio_num_t sda : 8;
} machine_hw_axp192_obj_t;


const mp_obj_type_t machine_hw_axp192_type;
STATIC machine_hw_axp192_obj_t machine_hw_axp192_obj[I2C_NUM_MAX];

STATIC void machine_hw_axp192_init(machine_hw_axp192_obj_t *self, uint32_t freq, uint32_t timeout_us, bool first_init) {
    mp_int_t axp192_code = 0;
    // bus definition
    pSensorBusDef.i2cPort = I2C_NUM_0;
    pSensorBusDef.i2cClockSpeed = freq;
    pSensorBusDef.gpioSDAPin = self->sda;
    pSensorBusDef.gpioSCLPin = self->scl;
    pSensorBusDef.gpioPullup = GPIO_PULLUP_ENABLE;
    pSensorsBus.def = &pSensorBusDef;
    
    i2cdevInit(&pSensorsBus);
    mp_hal_delay_us(100000);
    
    axp192_code = axpInit(&pSensorsBus, AXP192_SLAVE_ADDRESS, 0);

    mp_hal_delay_us(100000);

    // test axp192 
    if (axp192_code == AXP_PASS) {
        mp_printf(&mp_plat_print, "AXP192 I2C connection [OK].\n");
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("AXP192 I2C connection [FAIL].\n"));
    }
}

STATIC void machine_hw_axp192_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
	//machine_hw_mpu6886_obj_t *self = MP_OBJ_TO_PTR(self_in);
	
    //mp_printf(print, "COPTER(%u, scl=%u, sda=%u, freq=%u)",
    //    self->port, self->scl, self->sda, I2C_APB_CLK_FREQ / (h + l));

}

mp_obj_t machine_hw_axp192_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
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
    machine_hw_axp192_obj_t *self = (machine_hw_axp192_obj_t *)&machine_hw_axp192_obj[i2c_id];

    bool first_init = false;
    if (self->base.type == NULL) {
        // Created for the first time, set default pins
        self->base.type = &machine_hw_axp192_type;
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
    machine_hw_axp192_init(self, args[ARG_freq].u_int, args[ARG_timeout].u_int, first_init);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_axp192_dc_voltage(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t axp192_code = AXP_FAIL;
	if (n_args < 3) {
		mp_raise_ValueError(MP_ERROR_TEXT("two arguments are required"));
	}
    
    uint8_t ldo = mp_obj_get_int(args[1]);
    uint16_t vol_mv = mp_obj_get_int(args[2]);
    uint16_t vol_mv_max = 3500;

    if (ldo < 1 || ldo > 3)
        return mp_obj_new_int(AXP_FAIL);

    if (vol_mv < 700)
        vol_mv = 700;

    if (ldo == 1)
        vol_mv_max = 2275;

    if (vol_mv > vol_mv_max)
        vol_mv = vol_mv_max;

    switch (ldo)
    {
        case 1:
            axp192_code = setDCDC1Voltage(vol_mv);
            break;
        case 2:
            axp192_code = setDCDC2Voltage(vol_mv);
            break;      
        case 3:
            axp192_code = setDCDC3Voltage(vol_mv);
            break;
    }

    return mp_obj_new_int(axp192_code);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_dc_voltage_obj, 1, 3, mp_axp192_dc_voltage);

STATIC mp_obj_t mp_axp192_dc_enable(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t axp192_code = 0;
	if (n_args < 3) {
		mp_raise_ValueError(MP_ERROR_TEXT("two arguments are required"));
	}
    uint8_t ch = mp_obj_get_int(args[1]);
    uint8_t en = mp_obj_get_int(args[2]);

    axp192_code = setPowerOutPut(ch, en);

    return mp_obj_new_int(axp192_code);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_dc_enable_obj, 1, 3, mp_axp192_dc_enable);

STATIC mp_obj_t mp_axp192_ldo_voltage(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t axp192_code = AXP_FAIL;
	if (n_args < 3) {
		mp_raise_ValueError(MP_ERROR_TEXT("two arguments are required"));
	}
    
    uint8_t ldo = mp_obj_get_int(args[1]);
    uint16_t vol_mv = mp_obj_get_int(args[2]);


    if (ldo < 2 || ldo > 3)
        return mp_obj_new_int(AXP_FAIL);

    if (vol_mv < 1800)
        vol_mv = 1800;
    if (vol_mv > 3300)
        vol_mv = 3300;

    switch (ldo)
    {
        case 2:
            axp192_code = setLDO2Voltage(vol_mv);
            break;
        
        case 3:
            axp192_code = setLDO3Voltage(vol_mv);
            break;
    }

    return mp_obj_new_int(axp192_code);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_ldo_voltage_obj, 1, 3, mp_axp192_ldo_voltage);


STATIC mp_obj_t mp_axp192_ldo_enable(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t axp192_code = 0;
	if (n_args < 3) {
		mp_raise_ValueError(MP_ERROR_TEXT("two arguments are required"));
	}
    uint8_t ch = mp_obj_get_int(args[1]);
    uint8_t en = mp_obj_get_int(args[2]);

    axp192_code = setPowerOutPut(ch, en);

    return mp_obj_new_int(axp192_code);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_ldo_enable_obj, 1, 3, mp_axp192_ldo_enable);

STATIC mp_obj_t mp_axp192_set_bat_charging_current(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t axp192_code = 0;
	if (n_args < 2) {
		mp_raise_ValueError(MP_ERROR_TEXT("one argument is required"));
	}
    
    uint8_t current = mp_obj_get_int(args[1]);

    if (current < 0 || current > 7)
        return mp_obj_new_int(AXP_FAIL);

    axp192_code = setChargeControlCur(current);

    return mp_obj_new_int(axp192_code);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_set_bat_charging_current_obj, 1, 2, mp_axp192_set_bat_charging_current);

STATIC mp_obj_t mp_axp192_set_bat_charging_voltage(mp_uint_t n_args, const mp_obj_t *args)  {
    mp_int_t axp192_code = 0;
	if (n_args < 2) {
		mp_raise_ValueError(MP_ERROR_TEXT("one argument is required"));
	}
    
    uint8_t voltage = mp_obj_get_int(args[1]);

    if (voltage < 0 || voltage > 3)
        return mp_obj_new_int(AXP_FAIL);

    axp192_code = setChargingTargetVoltage(voltage);

    return mp_obj_new_int(axp192_code);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_set_bat_charging_voltage_obj, 1, 2, mp_axp192_set_bat_charging_voltage);

STATIC mp_obj_t mp_axp192_enable_bat_charging(mp_uint_t n_args, const mp_obj_t *args)  {
    int axp192_code = 0;
	if (n_args < 2) {
		mp_raise_ValueError(MP_ERROR_TEXT("one argument is required"));
	}
    
    uint8_t enable = mp_obj_get_int(args[1]);

    if (enable < 0 || enable > 1)
        return mp_obj_new_int(AXP_FAIL);

    setChgLEDMode(AXP20X_LED_BLINK_1HZ);

    axp192_code = enableCharging(enable);

    return mp_obj_new_int(axp192_code);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_enable_bat_charging_obj, 1, 2, mp_axp192_enable_bat_charging);

STATIC mp_obj_t mp_axp192_bat_charging_led_mode(mp_uint_t n_args, const mp_obj_t *args)  {
    int axp192_code = 0;
	if (n_args < 2) {
		mp_raise_ValueError(MP_ERROR_TEXT("one argument is required"));
	}
    
    uint8_t mode = mp_obj_get_int(args[1]);

    if (mode < 1 || mode > 3)
        return mp_obj_new_int(AXP_FAIL);

    setChgLEDMode(mode);

    return mp_obj_new_int(axp192_code);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_bat_charging_led_mode_obj, 1, 2, mp_axp192_bat_charging_led_mode);

STATIC mp_obj_t mp_axp192_battery_charging(mp_uint_t n_args, const mp_obj_t *args)  {
    int ischarging = isCharging();
    return mp_obj_new_int(ischarging);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_battery_charging_obj, 1, 1, mp_axp192_battery_charging);

STATIC mp_obj_t mp_axp192_battery_connected(mp_uint_t n_args, const mp_obj_t *args)  {
    int batconnected = isBatteryConnect();
    return mp_obj_new_int(batconnected);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_battery_connected_obj, 1, 1, mp_axp192_battery_connected);

STATIC mp_obj_t mp_axp192_shutdown(mp_uint_t n_args, const mp_obj_t *args)  {
    int shutdown = apxShutdown();
    return mp_obj_new_int(shutdown);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_shutdown_obj, 1, 1, mp_axp192_shutdown);

STATIC mp_obj_t mp_axp192_get_temp(mp_uint_t n_args, const mp_obj_t *args)  {
    float temperature = getTemp();
	return mp_obj_new_float(temperature);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_get_temp_obj, 1, 1, mp_axp192_get_temp);

STATIC mp_obj_t mp_axp192_limiting_off(mp_uint_t n_args, const mp_obj_t *args)  {
    int axp192_code = limitingOff();
	return mp_obj_new_int(axp192_code);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_axp192_limiting_off_obj, 1, 1, mp_axp192_limiting_off);


STATIC const mp_rom_map_elem_t hw_axp192_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_get_temp), MP_ROM_PTR(&mp_axp192_get_temp_obj) },
    { MP_ROM_QSTR(MP_QSTR_dc_enable), MP_ROM_PTR(&mp_axp192_dc_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_dc_voltage), MP_ROM_PTR(&mp_axp192_dc_voltage_obj) },
    { MP_ROM_QSTR(MP_QSTR_ldo_enable), MP_ROM_PTR(&mp_axp192_ldo_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_ldo_voltage), MP_ROM_PTR(&mp_axp192_ldo_voltage_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_bat_charging_current), MP_ROM_PTR(&mp_axp192_set_bat_charging_current_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_bat_charging_voltage), MP_ROM_PTR(&mp_axp192_set_bat_charging_voltage_obj) },
    { MP_ROM_QSTR(MP_QSTR_enable_bat_charging), MP_ROM_PTR(&mp_axp192_enable_bat_charging_obj) },
    { MP_ROM_QSTR(MP_QSTR_bat_charging_led_mode), MP_ROM_PTR(&mp_axp192_bat_charging_led_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_battery_charging), MP_ROM_PTR(&mp_axp192_battery_charging_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_battery_connected), MP_ROM_PTR(&mp_axp192_battery_connected_obj) },
    { MP_ROM_QSTR(MP_QSTR_shutdown), MP_ROM_PTR(&mp_axp192_shutdown_obj) },
    { MP_ROM_QSTR(MP_QSTR_limiting_off), MP_ROM_PTR(&mp_axp192_limiting_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_LED_OFF), MP_ROM_INT(AXP20X_LED_OFF) },
    { MP_ROM_QSTR(MP_QSTR_LED_BLINK_1HZ), MP_ROM_INT(AXP20X_LED_BLINK_1HZ) },
    { MP_ROM_QSTR(MP_QSTR_LED_BLINK_4HZ), MP_ROM_INT(AXP20X_LED_BLINK_4HZ) },
    { MP_ROM_QSTR(MP_QSTR_LED_LOW_LEVEL), MP_ROM_INT(AXP20X_LED_LOW_LEVEL) },
};

STATIC MP_DEFINE_CONST_DICT(hw_axp192_globals, hw_axp192_globals_table);

const mp_obj_type_t machine_hw_axp192_type = {
	{ &mp_type_type },
    .name = MP_QSTR_axp192,
    //.print = machine_hw_axp192_print,
    .make_new = machine_hw_axp192_make_new,
    .locals_dict = (mp_obj_dict_t *)&hw_axp192_globals,
};

