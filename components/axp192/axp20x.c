/////////////////////////////////////////////////////////////////
/*
MIT License

Copyright (c) 2019 lewis he

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

axp20x.cpp - Arduino library for X-Power AXP202 chip.
Created by Lewis he on April 1, 2019.
github:https://github.com/lewisxhe/AXP202X_Libraries
*/
/////////////////////////////////////////////////////////////////

/*
Copyright (c) 2020 Mauro Riva
Adapted by Mauro Riva (lemariva.com) for MicroPython support
*/

#include "axp20x.h"
#include <math.h>

const uint8_t startupParams[] = {
    0b00000000,
    0b01000000,
    0b10000000,
    0b11000000
};

const uint8_t longPressParams[] = {
    0b00000000,
    0b00010000,
    0b00100000,
    0b00110000
};

const uint8_t shutdownParams[] = {
    0b00000000,
    0b00000001,
    0b00000010,
    0b00000011
};

const uint8_t targetVolParams[] = {
    0b00000000,
    0b00100000,
    0b01000000,
    0b01100000
};

const uint8_t dcEnableOffset[] = {
    -1, 
    0x00, 
    0x04, 
    0x01
};

const uint8_t gPIOXReadOffset[] = {
    4, 5, 6, 4, 5
};

const uint8_t gPIOXWriteOffset[] = {
    0, 1, 2, 0, 1
};

static uint8_t devAddr, _irq[5], _chip_id, _gpio[4];
static uint8_t _outputReg;
static I2C_Dev *I2Cx;
static uint8_t buffer[14];
static bool isInit;
char _isAxp173;


// Power Output Control register
int _axp_probe()
{
    uint8_t data;
    if (_isAxp173) {
        //!Axp173 does not have a chip ID, read the status register to see if it reads normally
        i2cdevReadByte(I2Cx, devAddr, 0x01, buffer);
        data = buffer[0];

        if (data == 0 || data == 0xFF) {
            return AXP_FAIL;
        }
        _chip_id = AXP173_CHIP_ID;
        i2cdevReadByte(I2Cx, devAddr, AXP202_LDO234_DC23_CTL, buffer);
        _outputReg = buffer[0];
        AXP_DEBUG("OUTPUT Register 0x%x\n", _outputReg);
        isInit = true;
        return AXP_PASS;
    }
    i2cdevReadByte(I2Cx, devAddr, AXP202_IC_TYPE, buffer);
    _chip_id = buffer[0];
    AXP_DEBUG("chip id detect 0x%x\n", _chip_id);
    if (_chip_id == AXP202_CHIP_ID || _chip_id == AXP192_CHIP_ID) {
        AXP_DEBUG("Detect CHIP :%s\n", _chip_id == AXP202_CHIP_ID ? "AXP202" : "AXP192");
        i2cdevReadByte(I2Cx, devAddr, AXP202_LDO234_DC23_CTL, buffer);
        _outputReg = buffer[0];
        AXP_DEBUG("OUTPUT Register 0x%x\n", _outputReg);
        isInit = true;
        return AXP_PASS;
    }
    return AXP_FAIL;
}

int axpInit(I2C_Dev *i2cPort, uint8_t addr, bool isAxp173)
{
    if (isInit) {
        return AXP_PASS;
    }

    I2Cx = i2cPort;
    devAddr = addr;
    isInit = true;

    return _axp_probe();
}


//Only axp192 chip
bool isDCDC1Enable()
{
    if (_chip_id == AXP192_CHIP_ID)
        return IS_OPEN(_outputReg, AXP192_DCDC1);
    else if (_chip_id == AXP173_CHIP_ID)
        return IS_OPEN(_outputReg, AXP173_DCDC1);
    return 0;
}

bool isExtenEnable()
{
    if (_chip_id == AXP192_CHIP_ID)
        return IS_OPEN(_outputReg, AXP192_EXTEN);
    else if (_chip_id == AXP202_CHIP_ID)
        return IS_OPEN(_outputReg, AXP202_EXTEN);
    else if (_chip_id == AXP173_CHIP_ID) {
        uint8_t data;
        i2cdevReadByte(I2Cx, devAddr, AXP173_EXTEN_DC2_CTL, buffer);
        data = buffer[0];
        return IS_OPEN(data, AXP173_CTL_EXTEN_BIT);
    }
    return 0;
}

bool isLDO2Enable()
{
    if (_chip_id == AXP173_CHIP_ID) {
        return IS_OPEN(_outputReg, AXP173_LDO2);
    }
    //axp192 same axp202 ldo2 bit
    return IS_OPEN(_outputReg, AXP202_LDO2);
}

bool isLDO3Enable()
{
    if (_chip_id == AXP192_CHIP_ID)
        return IS_OPEN(_outputReg, AXP192_LDO3);
    else if (_chip_id == AXP202_CHIP_ID)
        return IS_OPEN(_outputReg, AXP202_LDO3);
    else if (_chip_id == AXP173_CHIP_ID)
        return IS_OPEN(_outputReg, AXP173_LDO3);
    return 0;
}

bool isLDO4Enable()
{
    if (_chip_id == AXP202_CHIP_ID)
        return IS_OPEN(_outputReg, AXP202_LDO4);
    if (_chip_id == AXP173_CHIP_ID)
        return IS_OPEN(_outputReg, AXP173_LDO4);
    return 0;
}

bool isDCDC2Enable()
{
    if (_chip_id == AXP173_CHIP_ID) {
        uint8_t data;
        i2cdevReadByte(I2Cx, devAddr, AXP173_EXTEN_DC2_CTL, buffer);
data = buffer[0];
        return IS_OPEN(data, AXP173_CTL_DC2_BIT);
    }
    //axp192 same axp202 dc2 bit
    return IS_OPEN(_outputReg, AXP202_DCDC2);
}

bool isDCDC3Enable()
{
    if (_chip_id == AXP173_CHIP_ID)
        return 0;
    //axp192 same axp202 dc3 bit
    return IS_OPEN(_outputReg, AXP202_DCDC3);
}

int setPowerOutPut(uint8_t ch, uint8_t en, bool dc)
{
    uint8_t data;
    uint8_t val = 0;
    if (!isInit)
        return AXP_NOT_INIT;

    //! Axp173 cannot use the REG12H register to control
    //! DC2 and EXTEN. It is necessary to control REG10H separately.
    if (_chip_id == AXP173_CHIP_ID) {
        i2cdevReadByte(I2Cx, devAddr, AXP173_EXTEN_DC2_CTL, buffer);
        data = buffer[0];
        if (ch & AXP173_DCDC2) {
            data = en ? data | BIT_MASK(AXP173_CTL_DC2_BIT) : data & (~BIT_MASK(AXP173_CTL_DC2_BIT));
            ch &= (~BIT_MASK(AXP173_DCDC2));
            i2cdevWriteByte(I2Cx, devAddr, AXP173_EXTEN_DC2_CTL, data);
        } else if (ch & AXP173_EXTEN) {
            data = en ? data | BIT_MASK(AXP173_CTL_EXTEN_BIT) : data & (~BIT_MASK(AXP173_CTL_EXTEN_BIT));
            ch &= (~BIT_MASK(AXP173_EXTEN));
            i2cdevWriteByte(I2Cx, devAddr, AXP173_EXTEN_DC2_CTL, data);
        }
    }

    i2cdevReadByte(I2Cx, devAddr, AXP202_LDO234_DC23_CTL, buffer);
    data = buffer[0];
    
    if (!dc) {   //LDO
        if (en) {
            data |= (1 << ch);
        } else {
            data &= (~(1 << ch));
        }
    } else {      //DCDC
        if (en) {
            data |= (1 << dcEnableOffset[ch]);
        } else {
            data &= (~(1 << dcEnableOffset[ch]));
        }
    }

    if (_chip_id == AXP202_CHIP_ID) {
        FORCED_OPEN_DCDC3(data); //! Must be forced open in T-Watch
    }

    i2cdevWriteByte(I2Cx, devAddr, AXP202_LDO234_DC23_CTL, data);


    i2cdevReadByte(I2Cx, devAddr, AXP202_LDO234_DC23_CTL, buffer);
    val = buffer[0];
    if (data == val) {
        _outputReg = val;
        return AXP_PASS;
    }
    return AXP_FAIL;
}

bool isCharging()
{
    uint8_t reg;
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadByte(I2Cx, devAddr, AXP202_MODE_CHGSTATUS, buffer);
    reg = buffer[0];
    return IS_OPEN(reg, 6);
}

bool isBatteryConnect()
{
    uint8_t reg;
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadByte(I2Cx, devAddr, AXP202_MODE_CHGSTATUS, buffer);
    reg = buffer[0];
    return IS_OPEN(reg, 5);
}

uint16_t _getRegistH8L5(uint8_t regh8, uint8_t regl5)
{
    uint8_t hv, lv;
    i2cdevReadByte(I2Cx, devAddr, regh8, buffer);
    hv = buffer[0];
    i2cdevReadByte(I2Cx, devAddr, regl5, buffer);
    lv = buffer[0];
    return (hv << 5) | (lv & 0x1F);
}

uint16_t _getRegistResult(uint8_t regh8, uint8_t regl4)
{
    uint8_t hv, lv;
    i2cdevReadByte(I2Cx, devAddr, regh8, buffer);
    hv = buffer[0];
    i2cdevReadByte(I2Cx, devAddr, regl4, buffer);
    lv = buffer[0];
    return (hv << 4) | (lv & 0x0F);
}

float getAcinVoltage()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_ACIN_VOL_H8, AXP202_ACIN_VOL_L4) * AXP202_ACIN_VOLTAGE_STEP;
}

float getAcinCurrent()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_ACIN_CUR_H8, AXP202_ACIN_CUR_L4) * AXP202_ACIN_CUR_STEP;
}

float getVbusVoltage()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_VBUS_VOL_H8, AXP202_VBUS_VOL_L4) * AXP202_VBUS_VOLTAGE_STEP;
}

float getVbusCurrent()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_VBUS_CUR_H8, AXP202_VBUS_CUR_L4) * AXP202_VBUS_CUR_STEP;
}

float getTemp()
{
    if (!isInit)
        return AXP_NOT_INIT;
    // Internal temperature
    // 000H => -144.7℃
    // STEP => 0.1℃
    // FFFH => 264.8℃
    return _getRegistResult(AXP202_INTERNAL_TEMP_H8, AXP202_INTERNAL_TEMP_L4)  * AXP202_INTERNAL_TEMP_STEP  - 144.7;
}

float getTSTemp()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_TS_IN_H8, AXP202_TS_IN_L4) * AXP202_TS_PIN_OUT_STEP;
}

float getGPIO0Voltage()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_GPIO0_VOL_ADC_H8, AXP202_GPIO0_VOL_ADC_L4) * AXP202_GPIO0_STEP;
}

float getGPIO1Voltage()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_GPIO1_VOL_ADC_H8, AXP202_GPIO1_VOL_ADC_L4) * AXP202_GPIO1_STEP;
}

/*
Note: the battery power formula:
Pbat =2* register value * Voltage LSB * Current LSB / 1000.
(Voltage LSB is 1.1mV; Current LSB is 0.5mA, and unit of calculation result is mW.)
*/
float getBattInpower()
{
    float rslt;
    uint8_t hv, mv, lv;
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadByte(I2Cx, devAddr, AXP202_BAT_POWERH8, buffer);
    hv = buffer[0];
    i2cdevReadByte(I2Cx, devAddr, AXP202_BAT_POWERM8, buffer);
    mv = buffer[0];
    i2cdevReadByte(I2Cx, devAddr, AXP202_BAT_POWERL8, buffer);
    lv = buffer[0];
    rslt = (hv << 16) | (mv << 8) | lv;
    rslt = 2 * rslt * 1.1 * 0.5 / 1000;
    return rslt;
}

float getBattVoltage()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_BAT_AVERVOL_H8, AXP202_BAT_AVERVOL_L4) * AXP202_BATT_VOLTAGE_STEP;
}

float getBattChargeCurrent()
{
    if (!isInit)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _getRegistResult(AXP202_BAT_AVERCHGCUR_H8, AXP202_BAT_AVERCHGCUR_L4) * AXP202_BATT_CHARGE_CUR_STEP;
    case AXP192_CHIP_ID:
        return _getRegistH8L5(AXP202_BAT_AVERCHGCUR_H8, AXP202_BAT_AVERCHGCUR_L5) * AXP202_BATT_CHARGE_CUR_STEP;
    default:
        return AXP_FAIL;
    }
}

float getBattDischargeCurrent()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistH8L5(AXP202_BAT_AVERDISCHGCUR_H8, AXP202_BAT_AVERDISCHGCUR_L5) * AXP202_BATT_DISCHARGE_CUR_STEP;
}

float getSysIPSOUTVoltage()
{
    if (!isInit)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_APS_AVERVOL_H8, AXP202_APS_AVERVOL_L4);
}

/*
Coulomb calculation formula:
C= 65536 * current LSB *（charge coulomb counter value - discharge coulomb counter value） /
3600 / ADC sample rate. Refer to REG84H setting for ADC sample rate；the current LSB is
0.5mA；unit of the calculation result is mAh. ）
*/
uint32_t getBattChargeCoulomb()
{
    uint8_t buffer[4];
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadReg8(I2Cx, devAddr, 0xB0, 4, buffer);
    return (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
}

uint32_t getBattDischargeCoulomb()
{
    uint8_t buffer[4];
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadReg8(I2Cx, devAddr, 0xB4, 4, buffer);
    return (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
}

float getCoulombData()
{
    if (!isInit)
        return AXP_NOT_INIT;
    uint32_t charge = getBattChargeCoulomb(), discharge = getBattDischargeCoulomb();
    uint8_t rate = getAdcSamplingRate();
    float result = 65536.0 * 0.5 * (charge - discharge) / 3600.0 / rate;
    return result;
}


//-------------------------------------------------------
// New Coulomb functions  by MrFlexi
//-------------------------------------------------------

uint8_t getCoulombRegister()
{
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadByte(I2Cx, devAddr, AXP202_COULOMB_CTL, buffer);
    return buffer[0];
}


int setCoulombRegister(uint8_t val)
{
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevWriteByte(I2Cx, devAddr, AXP202_COULOMB_CTL, val);
    return AXP_PASS;
}


int EnableCoulombcounter(void)
{

    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val = 0x80;
    i2cdevWriteByte(I2Cx, devAddr, AXP202_COULOMB_CTL, val);
    return AXP_PASS;
}

int DisableCoulombcounter(void)
{

    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val = 0x00;
    i2cdevWriteByte(I2Cx, devAddr, AXP202_COULOMB_CTL, val);
    return AXP_PASS;
}

int StopCoulombcounter(void)
{

    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val = 0xB8;
    i2cdevWriteByte(I2Cx, devAddr, AXP202_COULOMB_CTL, val);
    return AXP_PASS;
}


int ClearCoulombcounter(void)
{

    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val = 0xA0;
    i2cdevWriteByte(I2Cx, devAddr, AXP202_COULOMB_CTL, val);
    return AXP_PASS;
}

//-------------------------------------------------------
// END
//-------------------------------------------------------



uint8_t getAdcSamplingRate()
{
    //axp192 same axp202 aregister address 0x84
    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_ADC_SPEED, buffer);
    val = buffer[0];
    return 25 * (int)pow(2, (val & 0xC0) >> 6);
}

int setAdcSamplingRate(axp_adc_sampling_rate_t rate)
{
    //axp192 same axp202 aregister address 0x84
    if (!isInit)
        return AXP_NOT_INIT;
    if (rate > AXP_ADC_SAMPLING_RATE_200HZ)
        return AXP_FAIL;
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_ADC_SPEED, buffer);
    val = buffer[0];
    uint8_t rw = rate;
    val &= 0x3F;
    val |= (rw << 6);
    i2cdevWriteByte(I2Cx, devAddr, AXP202_ADC_SPEED, val);
    return AXP_PASS;
}

int setTSfunction(axp_ts_pin_function_t func)
{
    //axp192 same axp202 aregister address 0x84
    if (!isInit)
        return AXP_NOT_INIT;
    if (func > AXP_TS_PIN_FUNCTION_ADC)
        return AXP_FAIL;
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_ADC_SPEED, buffer);
    val = buffer[0];
    uint8_t rw = func;
    val &= 0xFA;
    val |= (rw << 2);
    i2cdevWriteByte(I2Cx, devAddr, AXP202_ADC_SPEED, val);
    return AXP_PASS;
}

int setTScurrent(axp_ts_pin_current_t current)
{
    //axp192 same axp202 aregister address 0x84
    if (!isInit)
        return AXP_NOT_INIT;
    if (current > AXP_TS_PIN_CURRENT_80UA)
        return AXP_FAIL;
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_ADC_SPEED, buffer);
    val = buffer[0];
    uint8_t rw = current;
    val &= 0xCF;
    val |= (rw << 4);
    i2cdevWriteByte(I2Cx, devAddr, AXP202_ADC_SPEED, val);
    return AXP_PASS;
}

int setTSmode(axp_ts_pin_mode_t mode)
{
    //axp192 same axp202 aregister address 0x84
    if (!isInit)
        return AXP_NOT_INIT;
    if (mode > AXP_TS_PIN_MODE_ENABLE)
        return AXP_FAIL;
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_ADC_SPEED, buffer);
    val = buffer[0];
    uint8_t rw = mode;
    val &= 0xFC;
    val |= rw;
    i2cdevWriteByte(I2Cx, devAddr, AXP202_ADC_SPEED, val);

    // TS pin ADC function enable/disable
    if (mode == AXP_TS_PIN_MODE_DISABLE)
        adc1Enable(AXP202_TS_PIN_ADC1, 0);
    else
        adc1Enable(AXP202_TS_PIN_ADC1, 1);
    return AXP_PASS;
}

int adc1Enable(uint16_t params, char en)
{
    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_ADC_EN1, buffer);
    val = buffer[0];
    if (en)
        val |= params;
    else
        val &= ~(params);
    i2cdevWriteByte(I2Cx, devAddr, AXP202_ADC_EN1, val);
    return AXP_PASS;
}

int adc2Enable(uint16_t params, char en)
{
    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_ADC_EN2, buffer);
    val = buffer[0];
    if (en)
        val |= params;
    else
        val &= ~(params);
    i2cdevWriteByte(I2Cx, devAddr,AXP202_ADC_EN2, val);
    return AXP_PASS;
}

int enableIRQ(uint64_t params, char en)
{
    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val, val1;
    if (params & 0xFF) {
        val1 = params & 0xFF;
        i2cdevReadByte(I2Cx, devAddr, AXP202_INTEN1, buffer);
        val = buffer[0];
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN1, val);
        i2cdevWriteByte(I2Cx, devAddr, AXP202_INTEN1, val);
    }
    if (params & 0xFF00) {
        val1 = params >> 8;
        i2cdevReadByte(I2Cx, devAddr, AXP202_INTEN2, buffer);
        val = buffer[0];
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN2, val);
        i2cdevWriteByte(I2Cx, devAddr, AXP202_INTEN2, val);
    }

    if (params & 0xFF0000) {
        val1 = params >> 16;
        i2cdevReadByte(I2Cx, devAddr, AXP202_INTEN3, buffer);
        val = buffer[0];
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN3, val);
        i2cdevWriteByte(I2Cx, devAddr, AXP202_INTEN3, val);
    }

    if (params & 0xFF000000) {
        val1 = params >> 24;
        i2cdevReadByte(I2Cx, devAddr, AXP202_INTEN4, buffer);
        val = buffer[0];
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN4, val);
        i2cdevWriteByte(I2Cx, devAddr, AXP202_INTEN4, val);
    }

    if (params & 0xFF00000000) {
        val1 = params >> 32;
        i2cdevReadByte(I2Cx, devAddr, AXP202_INTEN5, buffer);
        val = buffer[0];
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN5, val);
        i2cdevWriteByte(I2Cx, devAddr, AXP202_INTEN5, val);
    }
    return AXP_PASS;
}

int readIRQ()
{
    if (!isInit)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP192_CHIP_ID:
        for (int i = 0; i < 4; ++i) {
            i2cdevReadByte(I2Cx, devAddr, AXP192_INTSTS1 + i, buffer);
            _irq[i] = buffer[0];
        }
        i2cdevReadByte(I2Cx, devAddr, AXP192_INTSTS5, buffer);
        _irq[4] = buffer[0];
        return AXP_PASS;

    case AXP202_CHIP_ID:
        for (int i = 0; i < 5; ++i) {
            i2cdevReadByte(I2Cx, devAddr, AXP202_INTSTS1 + i, buffer);
            _irq[i] = buffer[0];
        }
        return AXP_PASS;
    default:
        return AXP_FAIL;
    }
}

void clearIRQ()
{
    uint8_t val = 0xFF;
    switch (_chip_id) {
    case AXP192_CHIP_ID:
        for (int i = 0; i < 3; i++) {
            i2cdevWriteByte(I2Cx, devAddr, AXP192_INTSTS1 + i, val);
        }
        i2cdevWriteByte(I2Cx, devAddr, AXP192_INTSTS5, val);
        break;
    case AXP202_CHIP_ID:
        for (int i = 0; i < 5; i++) {
            i2cdevWriteByte(I2Cx, devAddr, AXP202_INTSTS1 + i, val);
        }
        break;
    default:
        break;
    }
    memset(_irq, 0, sizeof(_irq));
}

bool isAcinOverVoltageIRQ()
{
    return (bool)(_irq[0] & BIT_MASK(7));
}

bool isAcinPlugInIRQ()
{
    return (bool)(_irq[0] & BIT_MASK(6));
}

bool isAcinRemoveIRQ()
{
    return (bool)(_irq[0] & BIT_MASK(5));
}

bool isVbusOverVoltageIRQ()
{
    return (bool)(_irq[0] & BIT_MASK(4));
}

bool isVbusPlugInIRQ()
{
    return (bool)(_irq[0] & BIT_MASK(3));
}

bool isVbusRemoveIRQ()
{
    return (bool)(_irq[0] & BIT_MASK(2));
}

bool isVbusLowVHOLDIRQ()
{
    return (bool)(_irq[0] & BIT_MASK(1));
}

bool isBattPlugInIRQ()
{
    return (bool)(_irq[1] & BIT_MASK(7));
}
bool isBattRemoveIRQ()
{
    return (bool)(_irq[1] & BIT_MASK(6));
}
bool isBattEnterActivateIRQ()
{
    return (bool)(_irq[1] & BIT_MASK(5));
}
bool isBattExitActivateIRQ()
{
    return (bool)(_irq[1] & BIT_MASK(4));
}
bool isChargingIRQ()
{
    return (bool)(_irq[1] & BIT_MASK(3));
}
bool isChargingDoneIRQ()
{
    return (bool)(_irq[1] & BIT_MASK(2));
}
bool isBattTempLowIRQ()
{
    return (bool)(_irq[1] & BIT_MASK(1));
}
bool isBattTempHighIRQ()
{
    return (bool)(_irq[1] & BIT_MASK(0));
}

bool isPEKShortPressIRQ()
{
    return (bool)(_irq[2] & BIT_MASK(1));
}

bool isPEKLongtPressIRQ()
{
    return (bool)(_irq[2] & BIT_MASK(0));
}

bool isTimerTimeoutIRQ()
{
    return (bool)(_irq[4] & BIT_MASK(7));
}

bool isVBUSPlug()
{
    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t reg;
    i2cdevReadByte(I2Cx, devAddr, AXP202_STATUS, buffer);
    reg = buffer[0];
    return IS_OPEN(reg, 5);
}

int setDCDC2Voltage(uint16_t mv)
{
    if (!isInit)
        return AXP_NOT_INIT;
    if (mv < 700) {
        AXP_DEBUG("DCDC2:Below settable voltage:700mV~2275mV");
        mv = 700;
    }
    if (mv > 2275) {
        AXP_DEBUG("DCDC2:Above settable voltage:700mV~2275mV");
        mv = 2275;
    }
    uint8_t val = (mv - 700) / 25;
    //! axp173/192/202 same register
    i2cdevWriteByte(I2Cx, devAddr, AXP202_DC2OUT_VOL, val);
    return AXP_PASS;
}

uint16_t getDCDC2Voltage()
{
    uint8_t val = 0;
    //! axp173/192/202 same register
    i2cdevReadByte(I2Cx, devAddr, AXP202_DC2OUT_VOL, buffer);
    val = buffer[0];
    return val * 25 + 700;
}

uint16_t getDCDC3Voltage()
{
    if (!isInit)
        return 0;
    if (_chip_id == AXP173_CHIP_ID)return AXP_NOT_SUPPORT;
    uint8_t val = 0;
    i2cdevReadByte(I2Cx, devAddr, AXP202_DC3OUT_VOL, buffer);
    val = buffer[0];
    return val * 25 + 700;
}

int setDCDC3Voltage(uint16_t mv)
{
    if (!isInit)
        return AXP_NOT_INIT;
    if (_chip_id == AXP173_CHIP_ID)return AXP_NOT_SUPPORT;
    if (mv < 700) {
        AXP_DEBUG("DCDC3:Below settable voltage:700mV~3500mV");
        mv = 700;
    }
    if (mv > 3500) {
        AXP_DEBUG("DCDC3:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }
    uint8_t val = (mv - 700) / 25;
    i2cdevWriteByte(I2Cx, devAddr, AXP202_DC3OUT_VOL, val);
    return AXP_PASS;
}

int setLDO2Voltage(uint16_t mv)
{
    uint8_t rVal, wVal;
    if (!isInit)
        return AXP_NOT_INIT;
    if (mv < 1800) {
        AXP_DEBUG("LDO2:Below settable voltage:1800mV~3300mV");
        mv = 1800;
    }
    if (mv > 3300) {
        AXP_DEBUG("LDO2:Above settable voltage:1800mV~3300mV");
        mv = 3300;
    }
    wVal = (mv - 1800) / 100;
    if (_chip_id == AXP202_CHIP_ID) {
        i2cdevReadByte(I2Cx, devAddr, AXP202_LDO24OUT_VOL, buffer);
        rVal = buffer[0];
        rVal &= 0x0F;
        rVal |= (wVal << 4);
        i2cdevWriteByte(I2Cx, devAddr, AXP202_LDO24OUT_VOL, rVal);
        return AXP_PASS;
    } else if (_chip_id == AXP192_CHIP_ID || _chip_id == AXP173_CHIP_ID) {
        i2cdevReadByte(I2Cx, devAddr, AXP192_LDO23OUT_VOL, buffer);
        rVal = buffer[0];
        rVal &= 0x0F;
        rVal |= (wVal << 4);
        i2cdevWriteByte(I2Cx, devAddr, AXP192_LDO23OUT_VOL, rVal);
        return AXP_PASS;
    }
    return AXP_FAIL;
}

uint16_t getLDO2Voltage()
{
    uint8_t rVal;
    if (_chip_id == AXP202_CHIP_ID) {
        i2cdevReadByte(I2Cx, devAddr, AXP202_LDO24OUT_VOL, buffer);
        rVal = buffer[0];
        rVal &= 0xF0;
        rVal >>= 4;
        return rVal * 100 + 1800;
    } else if (_chip_id == AXP192_CHIP_ID || _chip_id == AXP173_CHIP_ID ) {
        i2cdevReadByte(I2Cx, devAddr, AXP192_LDO23OUT_VOL, buffer);
        rVal = buffer[0];
        AXP_DEBUG("get result:%x\n", rVal);
        rVal &= 0xF0;
        rVal >>= 4;
        return rVal * 100 + 1800;
    }
    return 0;
}

int setLDO3Voltage(uint16_t mv)
{
    uint8_t rVal;
    if (!isInit)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID && mv < 700) {
        AXP_DEBUG("LDO3:Below settable voltage:700mV~3500mV");
        mv = 700;
    } else if (_chip_id == AXP192_CHIP_ID && mv < 1800) {
        AXP_DEBUG("LDO3:Below settable voltage:1800mV~3300mV");
        mv = 1800;
    }

    if (_chip_id == AXP202_CHIP_ID && mv > 3500) {
        AXP_DEBUG("LDO3:Above settable voltage:700mV~3500mV");
        mv = 3500;
    } else if (_chip_id == AXP192_CHIP_ID && mv > 3300) {
        AXP_DEBUG("LDO3:Above settable voltage:1800mV~3300mV");
        mv = 3300;
    }

    if (_chip_id == AXP202_CHIP_ID) {
        i2cdevReadByte(I2Cx, devAddr, AXP202_LDO3OUT_VOL, buffer);
        rVal = buffer[0];
        rVal &= 0x80;
        rVal |= ((mv - 700) / 25);
        i2cdevWriteByte(I2Cx, devAddr, AXP202_LDO3OUT_VOL, rVal);
        return AXP_PASS;
    } else if (_chip_id == AXP192_CHIP_ID || _chip_id == AXP173_CHIP_ID) {
        i2cdevReadByte(I2Cx, devAddr, AXP192_LDO23OUT_VOL, buffer);
        rVal = buffer[0];
        rVal &= 0xF0;
        rVal |= ((mv - 1800) / 100);
        i2cdevWriteByte(I2Cx, devAddr, AXP192_LDO23OUT_VOL, rVal);
        return AXP_PASS;
    }
    return AXP_FAIL;
}

uint16_t getLDO3Voltage()
{
    uint8_t rVal;
    if (!isInit)
        return AXP_NOT_INIT;

    if (_chip_id == AXP202_CHIP_ID) {
        i2cdevReadByte(I2Cx, devAddr, AXP202_LDO3OUT_VOL, buffer);
        rVal = buffer[0];
        if (rVal & 0x80) {
            //! According to the hardware N_VBUSEN Pin selection
            return getVbusVoltage() * 1000;
        } else {
            return (rVal & 0x7F) * 25 + 700;
        }
    } else if (_chip_id == AXP192_CHIP_ID || _chip_id == AXP173_CHIP_ID) {
        i2cdevReadByte(I2Cx, devAddr, AXP192_LDO23OUT_VOL, buffer);
        rVal = buffer[0];
        rVal &= 0x0F;
        return rVal * 100 + 1800;
    }
    return 0;
}


uint16_t getLDO4Voltage()
{
    const uint16_t ldo4_table[] = {1250, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2500, 2700, 2800, 3000, 3100, 3200, 3300};
    if (!isInit)
        return 0;
    uint8_t val = 0;
    switch (_chip_id) {
    case AXP173_CHIP_ID:
        i2cdevReadByte(I2Cx, devAddr, AXP173_LDO4_VLOTAGE, buffer);
        val = buffer[0];
        return val * 25 + 700;
    case AXP202_CHIP_ID:
        i2cdevReadByte(I2Cx, devAddr, AXP202_LDO24OUT_VOL, buffer);
        val = buffer[0];
        val &= 0xF;
        return ldo4_table[val];
        break;
    case AXP192_CHIP_ID:
    default:
        break;
    }
    return 0;
}


//! Only AXP202 support
// 0 : LDO  1 : DCIN
int setLDO3Mode(uint8_t mode)
{
    uint8_t val;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_FAIL;
    i2cdevReadByte(I2Cx, devAddr, AXP202_LDO3OUT_VOL, buffer);
    val = buffer[0];
    if (mode) {
        val |= BIT_MASK(7);
    } else {
        val &= (~BIT_MASK(7));
    }
    i2cdevWriteByte(I2Cx, devAddr, AXP202_LDO3OUT_VOL, val);
    return AXP_PASS;
}

int setStartupTime(uint8_t param)
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    if (param > sizeof(startupParams) / sizeof(startupParams[0]))
        return AXP_INVALID;
    i2cdevReadByte(I2Cx, devAddr, AXP202_POK_SET, buffer);
    val = buffer[0];
    val &= (~0b11000000);
    val |= startupParams[param];
    i2cdevWriteByte(I2Cx, devAddr, AXP202_POK_SET, val);
    return AXP_PASS;
}

int setlongPressTime(uint8_t param)
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    if (param > sizeof(longPressParams) / sizeof(longPressParams[0]))
        return AXP_INVALID;
    i2cdevReadByte(I2Cx, devAddr, AXP202_POK_SET, buffer);
    val = buffer[0];
    val &= (~0b00110000);
    val |= longPressParams[param];
    i2cdevWriteByte(I2Cx, devAddr, AXP202_POK_SET, val);
    return AXP_PASS;
}

int setShutdownTime(uint8_t param)
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    if (param > sizeof(shutdownParams) / sizeof(shutdownParams[0]))
        return AXP_INVALID;
    i2cdevReadByte(I2Cx, devAddr, AXP202_POK_SET, buffer);
    val = buffer[0];
    val &= (~0b00000011);
    val |= shutdownParams[param];
    i2cdevWriteByte(I2Cx, devAddr, AXP202_POK_SET, val);
    return AXP_PASS;
}

int setTimeOutShutdown(char en)
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadByte(I2Cx, devAddr, AXP202_POK_SET, buffer);
    val = buffer[0];
    if (en)
        val |= (1 << 3);
    else
        val &= (~(1 << 3));
    i2cdevWriteByte(I2Cx, devAddr, AXP202_POK_SET, val);
    return AXP_PASS;
}

int apxShutdown()
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadByte(I2Cx, devAddr, AXP202_OFF_CTL, buffer);
    val = buffer[0];
    val |= (1 << 7);
    i2cdevWriteByte(I2Cx, devAddr, AXP202_OFF_CTL, val);
    return AXP_PASS;
}

float getSettingChargeCurrent()
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
    val = buffer[0];
    val &= 0b00000111;
    float cur = 300.0 + val * 100.0;
    AXP_DEBUG("Setting Charge current : %.2f mA\n", cur);
    return cur;
}

bool isChargingEnable()
{
    uint8_t val;
    if (!isInit)
        return false;
    i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
    val = buffer[0];
    if (val & (1 << 7)) {
        AXP_DEBUG("Charging enable is enable\n");
        val = 1;
    } else {
        AXP_DEBUG("Charging enable is disable\n");
        val = 0;
    }
    return val;
}

int enableCharging(char en)
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
    val = buffer[0];
    val |= (1 << 7);
    i2cdevWriteByte(I2Cx, devAddr, AXP202_CHARGE1, val);
    return AXP_PASS;
}

int setChargingTargetVoltage(axp_chargeing_vol_t param)
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    if (param > sizeof(targetVolParams) / sizeof(targetVolParams[0]))
        return AXP_INVALID;
    i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
    val = buffer[0];
    val &= ~(0b01100000);
    val |= targetVolParams[param];
    i2cdevWriteByte(I2Cx, devAddr, AXP202_CHARGE1, val);
    return AXP_PASS;
}

int getBattPercentage()
{
    if (!isInit)
        return AXP_NOT_INIT;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_NOT_SUPPORT;
    uint8_t val;
    if (!isBatteryConnect())
        return 0;
    i2cdevReadByte(I2Cx, devAddr, AXP202_BATT_PERCENTAGE, buffer);
    val = buffer[0];
    if (!(val & BIT_MASK(7))) {
        return val & (~BIT_MASK(7));
    }
    return 0;
}

int setChgLEDMode(axp_chgled_mode_t mode)
{
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_OFF_CTL, buffer);
    val = buffer[0];
    val &= 0b11001111;
    val |= BIT_MASK(3);
    switch (mode) {
    case AXP20X_LED_OFF:
        i2cdevWriteByte(I2Cx, devAddr, AXP202_OFF_CTL, val);
        break;
    case AXP20X_LED_BLINK_1HZ:
        val |= 0b00010000;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_OFF_CTL, val);
        break;
    case AXP20X_LED_BLINK_4HZ:
        val |= 0b00100000;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_OFF_CTL, val);
        break;
    case AXP20X_LED_LOW_LEVEL:
        val |= 0b00110000;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_OFF_CTL, val);
        break;
    default:
        return AXP_FAIL;
    }
    return AXP_PASS;
}

int debugCharging()
{
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
    val = buffer[0];
    AXP_DEBUG("SRC REG:0x%x\n", val);
    if (val & (1 << 7)) {
        AXP_DEBUG("Charging enable is enable\n");
    } else {
        AXP_DEBUG("Charging enable is disable\n");
    }
    AXP_DEBUG("Charging target-voltage : 0x%x\n", ((val & 0b01100000) >> 5) & 0b11);
    if (val & (1 << 4)) {
        AXP_DEBUG("end when the charge current is lower than 15%% of the set value\n");
    } else {
        AXP_DEBUG(" end when the charge current is lower than 10%% of the set value\n");
    }
    val &= 0b00000111;
    AXP_DEBUG("Charge current : %.2f mA\n", 300.0 + val * 100.0);
    return AXP_PASS;
}

int debugStatus()
{
    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val, val1, val2;
    i2cdevReadByte(I2Cx, devAddr, AXP202_STATUS, buffer);
    val = buffer[0];
    i2cdevReadByte(I2Cx, devAddr, AXP202_MODE_CHGSTATUS, buffer);
    val1 = buffer[0];
    i2cdevReadByte(I2Cx, devAddr, AXP202_IPS_SET, buffer);
    val2 = buffer[0];
    AXP_DEBUG("AXP202_STATUS:   AXP202_MODE_CHGSTATUS   AXP202_IPS_SET\n");
    AXP_DEBUG("0x%x\t\t\t 0x%x\t\t\t 0x%x\n", val, val1, val2);
    return AXP_PASS;
}

int limitingOff()
{
    if (!isInit)
        return AXP_NOT_INIT;
    uint8_t val;
    i2cdevReadByte(I2Cx, devAddr, AXP202_IPS_SET, buffer);
    val = buffer[0];
    if (_chip_id == AXP202_CHIP_ID) {
        val |= 0x03;
    } else {
        val &= ~(1 << 1);
    }
    i2cdevWriteByte(I2Cx, devAddr, AXP202_IPS_SET, val);
    return AXP_PASS;
}

// Only AXP129 chip and AXP173
int setDCDC1Voltage(uint16_t mv)
{
    if (!isInit)
        return AXP_NOT_INIT;
    if (_chip_id != AXP192_CHIP_ID && _chip_id != AXP173_CHIP_ID)
        return AXP_FAIL;
    if (mv < 700) {
        AXP_DEBUG("DCDC1:Below settable voltage:700mV~3500mV");
        mv = 700;
    }
    if (mv > 3500) {
        AXP_DEBUG("DCDC1:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }
    uint8_t val = (mv - 700) / 25;
    //! axp192 and axp173 dc1 control register same
    i2cdevWriteByte(I2Cx, devAddr, AXP192_DC1_VLOTAGE, val);
    return AXP_PASS;
}

// Only AXP129 chip and AXP173
uint16_t getDCDC1Voltage()
{
    if (_chip_id != AXP192_CHIP_ID && _chip_id != AXP173_CHIP_ID)
        return AXP_FAIL;
    uint8_t val = 0;
    //! axp192 and axp173 dc1 control register same
    i2cdevReadByte(I2Cx, devAddr, AXP192_DC1_VLOTAGE, buffer);
    val = buffer[0];
    return val * 25 + 700;
}


/***********************************************
 *              !!! TIMER FUNCTION !!!
 * *********************************************/

int setTimer(uint8_t minutes)
{
    if (!isInit)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID) {
        if (minutes > 63) {
            return AXP_ARG_INVALID;
        }
        i2cdevWriteByte(I2Cx, devAddr, AXP202_TIMER_CTL, minutes);
        return AXP_PASS;
    }
    return AXP_NOT_SUPPORT;
}

int offTimer()
{
    if (!isInit)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID) {
        uint8_t minutes = 0x80;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_TIMER_CTL, minutes);
        return AXP_PASS;
    }
    return AXP_NOT_SUPPORT;
}

int clearTimerStatus()
{
    if (!isInit)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID) {
        uint8_t val;
        i2cdevReadByte(I2Cx, devAddr, AXP202_TIMER_CTL, buffer);
        val = buffer[0];
        val |= 0x80;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_TIMER_CTL, val);
        return AXP_PASS;
    }
    return AXP_NOT_SUPPORT;
}

/***********************************************
 *              !!! GPIO FUNCTION !!!
 * *********************************************/

int _axp192_gpio_0_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0b101;
    case AXP_IO_INPUT_MODE:
        return 0b001;
    case AXP_IO_LDO_MODE:
        return 0b010;
    case AXP_IO_ADC_MODE:
        return 0b100;
    case AXP_IO_FLOATING_MODE:
        return 0b111;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 0;
    case AXP_IO_OUTPUT_HIGH_MODE:
    case AXP_IO_PWM_OUTPUT_MODE:
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int _axp192_gpio_1_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0b101;
    case AXP_IO_INPUT_MODE:
        return 0b001;
    case AXP_IO_ADC_MODE:
        return 0b100;
    case AXP_IO_FLOATING_MODE:
        return 0b111;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 0;
    case AXP_IO_PWM_OUTPUT_MODE:
        return 0b010;
    case AXP_IO_OUTPUT_HIGH_MODE:
    case AXP_IO_LDO_MODE:
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int _axp192_gpio_3_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_EXTERN_CHARGING_CTRL_MODE:
        return 0;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int _axp192_gpio_4_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_EXTERN_CHARGING_CTRL_MODE:
        return 0;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_ADC_MODE:
        return 3;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int _axp192_gpio_set(axp_gpio_t gpio, axp_gpio_mode_t mode)
{
    int rslt;
    uint8_t val;
    switch (gpio) {
    case AXP_GPIO_0: {
        rslt = _axp192_gpio_0_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP192_GPIO0_CTL, buffer);
        val = buffer[0];
        val &= 0xF8;
        val |= (uint8_t)rslt;
        i2cdevWriteByte(I2Cx, devAddr, AXP192_GPIO0_CTL, val);
        return AXP_PASS;
    }
    case AXP_GPIO_1: {
        rslt = _axp192_gpio_1_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP192_GPIO1_CTL, buffer);
        val = buffer[0];
        val &= 0xF8;
        val |= (uint8_t)rslt;
        i2cdevWriteByte(I2Cx, devAddr, AXP192_GPIO1_CTL, val);
        return AXP_PASS;
    }
    case AXP_GPIO_2: {
        rslt = _axp192_gpio_1_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP192_GPIO2_CTL, buffer);
        val = buffer[0];
        val &= 0xF8;
        val |= (uint8_t)rslt;
        i2cdevWriteByte(I2Cx, devAddr, AXP192_GPIO2_CTL, val);
        return AXP_PASS;
    }
    case AXP_GPIO_3: {
        rslt = _axp192_gpio_3_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP192_GPIO34_CTL, buffer);
        val = buffer[0];
        val &= 0xFC;
        val |= (uint8_t)rslt;
        i2cdevWriteByte(I2Cx, devAddr, AXP192_GPIO34_CTL, val);
        return AXP_PASS;
    }
    case AXP_GPIO_4: {
        rslt = _axp192_gpio_4_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP192_GPIO34_CTL, buffer);
        val = buffer[0];
        val &= 0xF3;
        val |= (uint8_t)rslt;
        i2cdevWriteByte(I2Cx, devAddr, AXP192_GPIO34_CTL, val);
        return AXP_PASS;
    }
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int _axp202_gpio_0_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_OUTPUT_HIGH_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_LDO_MODE:
        return 3;
    case AXP_IO_ADC_MODE:
        return 4;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int _axp202_gpio_1_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_OUTPUT_HIGH_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_ADC_MODE:
        return 4;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int _axp202_gpio_2_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_FLOATING_MODE:
        return 1;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int _axp202_gpio_3_select(axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_INPUT_MODE:
        return 1;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 0;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int _axp202_gpio_set(axp_gpio_t gpio, axp_gpio_mode_t mode)
{
    uint8_t val;
    int rslt;
    switch (gpio) {
    case AXP_GPIO_0: {
        rslt = _axp202_gpio_0_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP202_GPIO0_CTL, buffer);
        val = buffer[0];
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_GPIO0_CTL, val);
        return AXP_PASS;
    }
    case AXP_GPIO_1: {
        rslt = _axp202_gpio_1_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP202_GPIO1_CTL, buffer);
        val = buffer[0];
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_GPIO1_CTL, val);
        return AXP_PASS;
    }
    case AXP_GPIO_2: {
        rslt = _axp202_gpio_2_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP202_GPIO2_CTL, buffer);
        val = buffer[0];
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_GPIO2_CTL, val);
        return AXP_PASS;
    }
    case AXP_GPIO_3: {
        rslt = _axp202_gpio_3_select(mode);
        if (rslt < 0)return rslt;
        i2cdevReadByte(I2Cx, devAddr, AXP202_GPIO3_CTL, buffer);
        val = buffer[0];
        val = rslt ? (val | BIT_MASK(2)) : (val & (~BIT_MASK(2)));
        i2cdevWriteByte(I2Cx, devAddr, AXP202_GPIO3_CTL, val);
        return AXP_PASS;
    }
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int setGPIOMode(axp_gpio_t gpio, axp_gpio_mode_t mode)
{
    if (!isInit)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _axp202_gpio_set(gpio, mode);
        break;
    case AXP192_CHIP_ID:
        return _axp192_gpio_set(gpio, mode);
        break;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int _axp_irq_mask(axp_gpio_irq_t irq)
{
    switch (irq) {
    case AXP_IRQ_NONE:
        return 0;
    case AXP_IRQ_RISING:
        return BIT_MASK(7);
    case AXP_IRQ_FALLING:
        return BIT_MASK(6);
    case AXP_IRQ_DOUBLE_EDGE:
        return 0b1100000;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int _axp202_gpio_irq_set(axp_gpio_t gpio, axp_gpio_irq_t irq)
{
    uint8_t reg;
    uint8_t val;
    int mask;
    mask = _axp_irq_mask(irq);

    if (mask < 0)return mask;
    switch (gpio) {
    case AXP_GPIO_0:
        reg = AXP202_GPIO0_CTL;
        break;
    case AXP_GPIO_1:
        reg = AXP202_GPIO1_CTL;
        break;
    case AXP_GPIO_2:
        reg = AXP202_GPIO2_CTL;
        break;
    case AXP_GPIO_3:
        reg = AXP202_GPIO3_CTL;
        break;
    default:
        return AXP_NOT_SUPPORT;
    }
    i2cdevReadByte(I2Cx, devAddr, reg, buffer);
    val = buffer[0];
    val = mask == 0 ? (val & 0b00111111) : (val | mask);
    i2cdevWriteByte(I2Cx, devAddr, reg, val);
    return AXP_PASS;
}


int setGPIOIrq(axp_gpio_t gpio, axp_gpio_irq_t irq)
{
    if (!isInit)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _axp202_gpio_irq_set(gpio, irq);
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        return AXP_NOT_SUPPORT;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int setLDO5Voltage(axp_ldo5_table_t vol)
{
    const uint8_t params[] = {
        0b11111000, //1.8V
        0b11111001, //2.5V
        0b11111010, //2.8V
        0b11111011, //3.0V
        0b11111100, //3.1V
        0b11111101, //3.3V
        0b11111110, //3.4V
        0b11111111, //3.5V
    };
    if (!isInit)
        return AXP_NOT_INIT;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_NOT_SUPPORT;
    if (vol > sizeof(params) / sizeof(params[0]))
        return AXP_ARG_INVALID;
    uint8_t val = 0;
    i2cdevReadByte(I2Cx, devAddr, AXP202_GPIO0_VOL, buffer);
    val = buffer[0];
    val &= 0b11111000;
    val |= params[vol];
    i2cdevWriteByte(I2Cx, devAddr, AXP202_GPIO0_VOL, val);
    return AXP_PASS;
}

int _axp192_gpio_write(axp_gpio_t gpio, uint8_t val)
{
    uint8_t reg;
    uint8_t wVal = 0;
    switch (gpio) {
    case AXP_GPIO_0:
    case AXP_GPIO_1:
    case AXP_GPIO_2:
        reg = AXP192_GPIO012_SIGNAL;
        i2cdevReadByte(I2Cx, devAddr, reg, buffer);
        break;
    case AXP_GPIO_3:
    case AXP_GPIO_4:
        reg = AXP192_GPIO34_SIGNAL;
        i2cdevReadByte(I2Cx, devAddr, reg, buffer);
        break;
    default:
        return AXP_NOT_SUPPORT;
    }
    if (val == 1) {
        buffer[0] |= 1 << gPIOXWriteOffset[gpio];
    }else {
        buffer[0] &= ~(1 <<  gPIOXWriteOffset[gpio]);
    }
    i2cdevWriteByte(I2Cx, devAddr, reg, buffer[0]);

    return AXP_PASS;
}

int _axp202_gpio_write(axp_gpio_t gpio, uint8_t val)
{
    uint8_t reg;
    uint8_t wVal = 0;
    switch (gpio) {
    case AXP_GPIO_0:
        reg = AXP202_GPIO0_CTL;
        break;
    case AXP_GPIO_1:
        reg = AXP202_GPIO1_CTL;
        break;
    case AXP_GPIO_2:
        reg = AXP202_GPIO2_CTL;
        if (val) {
            return AXP_NOT_SUPPORT;
        }
        break;
    case AXP_GPIO_3:
        if (val) {
            return AXP_NOT_SUPPORT;
        }
        i2cdevReadByte(I2Cx, devAddr, AXP202_GPIO3_CTL, buffer);
        wVal = buffer[0];
        wVal &= 0b11111101;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_GPIO3_CTL, wVal);
        return AXP_PASS;
    default:
        return AXP_NOT_SUPPORT;
    }
    i2cdevReadByte(I2Cx, devAddr, reg, buffer);
    wVal = buffer[0];
    wVal = val ? (wVal | 1) : (wVal & 0b11111000);
    i2cdevWriteByte(I2Cx, devAddr, reg, wVal);
    return AXP_PASS;
}

int _axp202_gpio_read(axp_gpio_t gpio)
{
    uint8_t val;
    uint8_t reg = AXP202_GPIO012_SIGNAL;
    uint8_t offset;
    switch (gpio) {
    case AXP_GPIO_0:
        offset = 4;
        break;
    case AXP_GPIO_1:
        offset = 5;
        break;
    case AXP_GPIO_2:
        offset = 6;
        break;
    case AXP_GPIO_3:
        reg = AXP202_GPIO3_CTL;
        offset = 0;
        break;
    default:
        return AXP_NOT_SUPPORT;
    }
    i2cdevReadByte(I2Cx, devAddr, reg, buffer);
    val = buffer[0];
    
    return val & BIT_MASK(offset) ? 1 : 0;
}

int gpioWrite(axp_gpio_t gpio, uint8_t val)
{
    if (!isInit)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _axp202_gpio_write(gpio, val);
    case AXP192_CHIP_ID:
        return _axp192_gpio_write(gpio, val);
    case AXP173_CHIP_ID:
        return AXP_NOT_SUPPORT;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int gpioRead(axp_gpio_t gpio)
{
    if (!isInit)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _axp202_gpio_read(gpio);
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        return AXP_NOT_SUPPORT;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int getChargeControlCur()
{
    int cur;
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
        val = buffer[0];
        val &= 0x0F;
        cur =  val * 100 + 300;
        if (cur > 1800 || cur < 300)return 0;
        return cur;
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
        val = buffer[0];
        return val & 0x0F;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int setChargeControlCur(uint16_t mA)
{
    uint8_t val;
    if (!isInit)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
        val = buffer[0];
        val &= 0b11110000;
        mA -= 300;
        val |= (mA / 100);
        i2cdevWriteByte(I2Cx, devAddr, AXP202_CHARGE1, val);
        return AXP_PASS;
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        i2cdevReadByte(I2Cx, devAddr, AXP202_CHARGE1, buffer);
        val = buffer[0];
        val &= 0b11110000;
        if (mA > AXP1XX_CHARGE_CUR_1320MA)
            mA = AXP1XX_CHARGE_CUR_1320MA;
        val |= mA;
        i2cdevWriteByte(I2Cx, devAddr, AXP202_CHARGE1, val);
        return AXP_PASS;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int setSleep()
{
    int ret;
    uint8_t val  = 0;
    ret = i2cdevReadByte(I2Cx, devAddr, AXP202_VOFF_SET, buffer);
    val = buffer[0];
    if (ret != 0)return AXP_FAIL;
    val |= _BV(3);
    ret = i2cdevWriteByte(I2Cx, devAddr, AXP202_VOFF_SET, val);
    if (ret != 0)return AXP_FAIL;
    ret = i2cdevReadByte(I2Cx, devAddr, AXP202_VOFF_SET, buffer);
    val = buffer[0];
    if (ret != 0)return AXP_FAIL;
    return (val & _BV(3)) ? AXP_PASS : AXP_FAIL;
}