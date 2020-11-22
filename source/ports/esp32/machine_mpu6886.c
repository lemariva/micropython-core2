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

#include "mpu6886.h"
#include "i2cdev.h"
#include "imu_types.h"

#include "machine_mpu6886.h"
#include <math.h>

const mp_float_t BETA = 0.7557497f;
const mp_float_t DEG2RAD = 0.0174533f;

static Axis3i16 gGyroRaw;
static Axis3f gOffsetGyroRaw;
static Axis3i16 gAccelRaw;

#define I2C_DEFAULT_TIMEOUT_US (10000) // 10ms
#define I2C_0_DEFAULT_SCL (GPIO_NUM_22)
#define I2C_0_DEFAULT_SDA (GPIO_NUM_21)
#define SENSORS_MPU6886_BUFF_LEN 14

#define SENSORS_GYRO_FS_CFG       MPU6886_GYRO_FS_500
#define SENSORS_DEG_PER_LSB_CFG   MPU6886_DEG_PER_LSB_500

#define SENSORS_ACCEL_FS_CFG      MPU6886_ACCEL_FS_4
#define SENSORS_G_PER_LSB_CFG     MPU6886_G_PER_LSB_4


static uint8_t buffer[SENSORS_MPU6886_BUFF_LEN] = {0};

I2cDef pSensorBusDef;
I2cDrv pSensorsBus;

float accelRange = 0.0f;
float gyroRange = 0.0f;

typedef struct _quaternion_obj_t {
    mp_float_t q0;
    mp_float_t q1;
    mp_float_t q2;
    mp_float_t q3;
} _quaternion_obj_t;
_quaternion_obj_t q_obj;

typedef struct _machine_hw_mpu6886_obj_t {
    mp_obj_base_t base;
    i2c_port_t port : 8;
    gpio_num_t scl : 8;
    gpio_num_t sda : 8;
    int freq;
    int timeout;
} machine_hw_mpu6886_obj_t;


const mp_obj_type_t machine_hw_mpu6886_type;
STATIC machine_hw_mpu6886_obj_t machine_hw_mpu6886_obj[I2C_NUM_MAX];

mp_float_t inv_sqrt(mp_float_t x) {
    long i;
    mp_float_t y = x;
    mp_float_t halfx = 0.5f * x;
	memcpy(&i, &y, sizeof(mp_float_t));
	i = 0x5f3759df - (i>>1);
    memcpy(&y, &i, sizeof(long));
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void processAccGyroMeasurements(const uint8_t *buffer, Axis3i16 *accelRaw, Axis3i16 *gyroRaw)
{
    /* sensors step 2.1 read from buffer */
    accelRaw->x = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw->y = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw->z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw->x = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw->y = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw->z = (((int16_t)buffer[12]) << 8) | buffer[13];
}

STATIC mp_obj_t mpu6886_quaternion_init() {
    q_obj.q0 = 1.0f;
    q_obj.q1 = 0.0f;
    q_obj.q2 = 0.0f;
    q_obj.q3 = 0.0f;

    return mp_const_true;
}

STATIC void machine_hw_mpu6886_internal(machine_hw_mpu6886_obj_t *self) {
    mp_int_t mpu6886_code = 0;
    // bus definition
    pSensorBusDef.i2cPort = I2C_NUM_0;
    pSensorBusDef.i2cClockSpeed = self->freq;
    pSensorBusDef.gpioSDAPin = self->sda;
    pSensorBusDef.gpioSCLPin = self->scl;
    pSensorBusDef.gpioPullup = GPIO_PULLUP_ENABLE;
    pSensorsBus.def = &pSensorBusDef;
    
    i2cdevInit(&pSensorsBus);
    mp_hal_delay_us(100000);
    
    // init mpu6886
    mpu6886_code = mpu6886Init(&pSensorsBus);
    if (mpu6886_code == MPU6886_OK) {
       mp_printf(&mp_plat_print, "MPU6886 I2C connection [OK].\n");
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("MPU6886 I2C connection [FAIL].\n"));
    }
    mp_hal_delay_us(100000);

    // Reset mpu6886
    mpu6886Reset();
    mp_hal_delay_us(100000);

    // Set mpu6886 clock source auto
    mpu6886SetClockSource(MPU6886_CLOCK_AUTO);
    mp_hal_delay_us(100000);
    
    // Set gyro full scale range - mpu6886
    mpu6886SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);

    // Set accelerometer full scale range - mpu6886
    mpu6886SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);
        
    // Range - mpu6886
    accelRange = mpu6886GetFullScaleAccelGPL(); 
    gyroRange = mpu6886GetFullScaleGyroDPL();
    mp_hal_delay_us(100000);
    
	// quaternion init
	mpu6886_quaternion_init();
}

STATIC void machine_hw_mpu6886_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
	//machine_hw_mpu6886_obj_t *self = MP_OBJ_TO_PTR(self_in);
	
    //mp_printf(print, "COPTER(%u, scl=%u, sda=%u, freq=%u)",
    //    self->port, self->scl, self->sda, I2C_APB_CLK_FREQ / (h + l));

}

mp_obj_t machine_hw_mpu6886_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
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
    machine_hw_mpu6886_obj_t *self = (machine_hw_mpu6886_obj_t *)&machine_hw_mpu6886_obj[i2c_id];

    if (self->base.type == NULL) {
        // Created for the first time, set default pins
        self->base.type = &machine_hw_mpu6886_type;
        self->port = i2c_id;
        self->scl = I2C_0_DEFAULT_SCL;
        self->sda = I2C_0_DEFAULT_SDA;
        self->timeout = args[ARG_timeout].u_int;
        self->freq = args[ARG_freq].u_int;
    }

    // Set SCL/SDA pins if given
    if (args[ARG_scl].u_obj != MP_OBJ_NULL) {
        self->scl = mp_hal_get_pin_obj(args[ARG_scl].u_obj);
    }
    if (args[ARG_sda].u_obj != MP_OBJ_NULL) {
        self->sda = mp_hal_get_pin_obj(args[ARG_sda].u_obj);
    }

    // Initialise the I2C peripheral
    machine_hw_mpu6886_internal(self);

    return MP_OBJ_FROM_PTR(self);
}
// inv_sqrt(x)
STATIC mp_obj_t mp_inv_sqrt(mp_uint_t n_args, const mp_obj_t *args)  {
    long i;
	if (n_args == 1) {
		mp_raise_ValueError(MP_ERROR_TEXT("an argument is required"));
	}
    mp_float_t y = mp_obj_get_float(args[1]);
    mp_float_t halfx = 0.5f * mp_obj_get_float(args[1]);
	memcpy(&i, &y, sizeof(mp_float_t));
	i = 0x5f3759df - (i>>1);
    memcpy(&y, &i, sizeof(long));
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return mp_obj_new_float(y);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_inv_sqrt_obj, 1, 2, mp_inv_sqrt);

STATIC mp_obj_t machine_hw_mpu6886_ahrs(size_t n_args, const mp_obj_t *args) {

    mp_float_t recipNorm;
	mp_float_t s0, s1, s2, s3;
	mp_float_t qDot1, qDot2, qDot3, qDot4;
	mp_float_t _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Data
    mp_float_t acc_x = mp_obj_get_float(args[1]);
    mp_float_t acc_y = mp_obj_get_float(args[2]);
    mp_float_t acc_z = mp_obj_get_float(args[3]);

    mp_float_t gyro_x = mp_obj_get_float(args[4]) * DEG2RAD;
    mp_float_t gyro_y = mp_obj_get_float(args[5]) * DEG2RAD;
    mp_float_t gyro_z = mp_obj_get_float(args[6]) * DEG2RAD;

    mp_float_t delta_ts = mp_obj_get_float(args[7]);

    // Local variables
    mp_float_t q0 = q_obj.q0;
    mp_float_t q1 = q_obj.q1;
    mp_float_t q2 = q_obj.q2;
    mp_float_t q3 = q_obj.q3;
    
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gyro_x - q2 * gyro_y - q3 * gyro_z);
	qDot2 = 0.5f * (q0 * gyro_x + q2 * gyro_z - q3 * gyro_y);
	qDot3 = 0.5f * (q0 * gyro_y - q1 * gyro_z + q3 * gyro_x);
	qDot4 = 0.5f * (q0 * gyro_z + q1 * gyro_y - q2 * gyro_x);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((acc_x == 0.0f) && (acc_y == 0.0f) && (acc_z  == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = inv_sqrt(acc_x * acc_x + acc_y * acc_y + acc_z  * acc_z);
		acc_x *= recipNorm;
		acc_y *= recipNorm;
		acc_z *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * acc_x + _4q0 * q1q1 - _2q1 * acc_y;
		s1 = _4q1 * q3q3 - _2q3 * acc_x + 4.0f * q0q0 * q1 - _2q0 * acc_y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * acc_z ;
		s2 = 4.0f * q0q0 * q2 + _2q0 * acc_x + _4q2 * q3q3 - _2q3 * acc_y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * acc_z ;
		s3 = 4.0f * q1q1 * q3 - _2q1 * acc_x + 4.0f * q2q2 * q3 - _2q2 * acc_y;
		recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= BETA * s0;
		qDot2 -= BETA * s1;
		qDot3 -= BETA * s2;
		qDot4 -= BETA * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * delta_ts;
	q1 += qDot2 * delta_ts;
	q2 += qDot3 * delta_ts;
	q3 += qDot4 * delta_ts;

	// Normalise quaternion
	recipNorm =  inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	mp_float_t roll = (float)atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)/DEG2RAD;
    mp_float_t yaw = (float)atan2(q1*q2 + q0*q3, q0*q0 + q1*q1 - 0.5f)/DEG2RAD;
	mp_float_t pitch = (float)asin(-2.0f * (q1*q3 - q0*q2))/DEG2RAD;

    mp_obj_t tuple[3];
    tuple[0] = mp_obj_new_float(pitch);
    tuple[1] = mp_obj_new_float(roll);
    tuple[2] = mp_obj_new_float(yaw);

    // save result
    q_obj.q0 = q0;
    q_obj.q1 = q1;
    q_obj.q2 = q2;
    q_obj.q3 = q3;

    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_hw_mpu6886_ahrs_obj, 8, 8, machine_hw_mpu6886_ahrs);

STATIC mp_obj_t machine_hw_mpu6886_calibrate(size_t n_args, const mp_obj_t *args) {
    mp_obj_t tuple[3];
    mp_int_t count = mp_obj_get_int(args[1]);
    float n = (float) count;
    
    gOffsetGyroRaw.x = 0.0f;
    gOffsetGyroRaw.y = 0.0f;
    gOffsetGyroRaw.z = 0.0f;

    while(count > 0)
    { 
        i2cdevReadReg8(&pSensorsBus, MPU6886_DEFAULT_ADDRESS, MPU6886_RA_ACCEL_XOUT_H, SENSORS_MPU6886_BUFF_LEN, buffer);
        processAccGyroMeasurements(&buffer[0], &gAccelRaw, &gGyroRaw);
        mp_hal_delay_us(100000);
        gOffsetGyroRaw.x += (float)gGyroRaw.x;
        gOffsetGyroRaw.y += (float)gGyroRaw.y;
        gOffsetGyroRaw.z += (float)gGyroRaw.z;
        count -= 1;
    }

    gOffsetGyroRaw.x /= n;
    gOffsetGyroRaw.y /= n;
    gOffsetGyroRaw.z /= n;

    tuple[0] = mp_obj_new_float(gOffsetGyroRaw.x);
    tuple[1] = mp_obj_new_float(gOffsetGyroRaw.y);
    tuple[2] = mp_obj_new_float(gOffsetGyroRaw.z);

    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_hw_mpu6886_calibrate_obj, 2, 2, machine_hw_mpu6886_calibrate);

STATIC mp_obj_t machine_hw_mpu6886_motion(size_t n_args, const mp_obj_t *args) {
    mp_obj_t tuple[6];
    
    i2cdevReadReg8(&pSensorsBus, MPU6886_DEFAULT_ADDRESS, MPU6886_RA_ACCEL_XOUT_H, SENSORS_MPU6886_BUFF_LEN, buffer);

    processAccGyroMeasurements(&buffer[0], &gAccelRaw, &gGyroRaw);
    
    tuple[0] = mp_obj_new_float((float)gAccelRaw.x * accelRange);
    tuple[1] = mp_obj_new_float((float)gAccelRaw.y * accelRange);
    tuple[2] = mp_obj_new_float((float)gAccelRaw.z * accelRange);
    tuple[3] = mp_obj_new_float(((float)gGyroRaw.x - gOffsetGyroRaw.x) * gyroRange );
    tuple[4] = mp_obj_new_float(((float)gGyroRaw.y - gOffsetGyroRaw.y) * gyroRange );
    tuple[5] = mp_obj_new_float(((float)gGyroRaw.z - gOffsetGyroRaw.z) * gyroRange );

    return mp_obj_new_tuple(6, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_hw_mpu6886_motion_obj, 1, 1, machine_hw_mpu6886_motion);


STATIC const mp_rom_map_elem_t hw_mpu6886_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_inv_sqrt), MP_ROM_PTR(&mp_inv_sqrt_obj) },
    { MP_ROM_QSTR(MP_QSTR_ahrs), MP_ROM_PTR(&machine_hw_mpu6886_ahrs_obj) },
    { MP_ROM_QSTR(MP_QSTR_motion), MP_ROM_PTR(&machine_hw_mpu6886_motion_obj) },
    { MP_ROM_QSTR(MP_QSTR_gyro_cal), MP_ROM_PTR(&machine_hw_mpu6886_calibrate_obj) },
};
STATIC MP_DEFINE_CONST_DICT(hw_mpu6886_globals, hw_mpu6886_globals_table);

const mp_obj_type_t machine_hw_mpu6886_type = {
	{ &mp_type_type },
    .name = MP_QSTR_mpu6886,
    //.print = machine_hw_mpu6886_print,
    .make_new = machine_hw_mpu6886_make_new,
    .locals_dict = (mp_obj_dict_t *)&hw_mpu6886_globals,
};


