/*!
 * @file Adafruit_LIS3DH.cpp
 *
 *  @mainpage Adafruit LIS3DH breakout board
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the Adafruit LIS3DH Accel breakout board
 *
 *  Designed specifically to work with the Adafruit LIS3DH Accel breakout board.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2809
 *
 *  This sensor communicates over I2C or SPI (our library code supports both) so
 * you can share it with a bunch of other sensors on the same I2C bus.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  K. Townsend / Limor Fried (Adafruit Industries)
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Arduino.h"

#include <Adafruit_LIS3DH.h>
#include <Wire.h>

 /*!
  *  @brief  Instantiates a new LIS3DH class in I2C
  *  @param  Wi
  *          optional wire object
  */
Adafruit_LIS3DH::Adafruit_LIS3DH(TwoWire* Wi)
    : _cs(-1), _mosi(-1), _miso(-1), _sck(-1), _sensorID(-1) {
    I2Cinterface = Wi;
}

/*!
 *   @brief  Instantiates a new LIS3DH class using hardware SPI
 *   @param  cspin
 *           number of CSPIN (Chip Select)
 *   @param  *theSPI
 *           optional parameter contains spi object
 */
Adafruit_LIS3DH::Adafruit_LIS3DH(int8_t cspin, SPIClass* theSPI) {
    _cs = cspin;
    _mosi = -1;
    _miso = -1;
    _sck = -1;
    _sensorID = -1;
    SPIinterface = theSPI;
}

/*!
 *   @brief  Instantiates a new LIS3DH class using software SPI
 *   @param  cspin
 *           number of CSPIN (Chip Select)
 *   @param  mosipin
 *           number of pin used for MOSI (Master Out Slave In))
 *   @param  misopin
 *           number of pin used for MISO (Master In Slave Out)
 *   @param  sckpin
 *           number of pin used for CLK (clock pin)
 */
Adafruit_LIS3DH::Adafruit_LIS3DH(int8_t cspin, int8_t mosipin, int8_t misopin,
    int8_t sckpin) {
    _cs = cspin;
    _mosi = mosipin;
    _miso = misopin;
    _sck = sckpin;
    _sensorID = -1;
}

/*!
 *  @brief  Setups the HW (reads coefficients values, etc.)
 *  @param  i2caddr
 *          i2c address (optional, fallback to default)
 *  @param  nWAI
 *          Who Am I register value - defaults to 0x33 (LIS3DH)
 *  @return true if successful
 */
bool Adafruit_LIS3DH::begin(uint8_t i2caddr, lis3dh_dataRate_t dataRate, lis3dh_powerMode_t powerMode, lis3dh_activeAxis_t activeAxis) {
    _i2caddr = i2caddr;
    if (I2Cinterface) {
        i2c_dev = new Adafruit_I2CDevice(_i2caddr, I2Cinterface);

        if (!i2c_dev->begin()) {
            return false;
        }
    }
    else if (_cs != -1) {

        // SPIinterface->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        if (_sck == -1) {
            spi_dev = new Adafruit_SPIDevice(_cs,
                500000,                // frequency
                SPI_BITORDER_MSBFIRST, // bit order
                SPI_MODE0,             // data mode
                SPIinterface);
        }
        else {
            spi_dev = new Adafruit_SPIDevice(_cs, _sck, _miso, _mosi,
                500000,                // frequency
                SPI_BITORDER_MSBFIRST, // bit order
                SPI_MODE0);            // data mode
        }

        if (!spi_dev->begin()) {
            return false;
        }
    }

    /* Check connection */
    if (getDeviceID() != LIS3DH_WHO_AM_I_VALUE) {
        /* No LIS3DH detected ... return false */
        // Serial.println(deviceid, HEX);
        return false;
    }
    // Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
    //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
    // _ctrl1.write(0x07); // enable all axes, normal mode

    writeReg(LIS3DH_REG_CTRL5, 0x80);
    delay(10);

    setDataRate(dataRate, powerMode);
    enableAxes(activeAxis);

    Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);
    // _ctrl4.write(0x80); //  BDU enabled
    _ctrl4.write(0x00); //  BDU disabled

    Adafruit_BusIO_Register _ctrl2 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL2, 1);

    _ctrl2.write(1);

    return true;
}

/*!
 *  @brief  Get Device ID from LIS3DH_REG_WHOAMI
 *  @return WHO AM I value
 */
uint8_t Adafruit_LIS3DH::getDeviceID(void) {
    Adafruit_BusIO_Register _chip_id = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_WHOAMI, 1);

    return _chip_id.read();
}
/*!
 *  @brief  Check to see if new data available
 *  @return true if there is new data available, false otherwise
 */
bool Adafruit_LIS3DH::haveNewData(void) {
    Adafruit_BusIO_Register status_2 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_STATUS2, 1);
    Adafruit_BusIO_RegisterBits zyx_data_available =
        Adafruit_BusIO_RegisterBits(&status_2, 1, 3);
    return zyx_data_available.read();
}

/*!
 *  @brief  Reads x y z values at once
 */
void Adafruit_LIS3DH::read(uint8_t fifoIdx)
{
    uint8_t register_address = LIS3DH_REG_OUT_X_L;
    uint8_t buffer[6];
    if (i2c_dev)
    {
        register_address |= 0x80; // set [7] for auto-increment
    }
    else {
        register_address |= 0x40; // set [6] for auto-increment
        register_address |= 0x80; // set [7] for read
    }

    // Adafruit_BusIO_Register xl_data = Adafruit_BusIO_Register(
    //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, register_address, 6);
    // uint8_t buffer[6];
    // xl_data.read(buffer, 6);

    if (!_fifoEnabled)
    {
        fifoIdx = 0;
    }
    fifoIdx = min(fifoIdx, min(LIS3DH_FIFO_MAX_DEPTH,getFifoSize()));

    do
    {
        readRegBuffer(register_address, buffer, 6);
        fifoIdx--;
    } while (fifoIdx + 1 > 0 && fifoIsEmpty() == false);



    x = buffer[0];
    x |= ((uint16_t)buffer[1]) << 8;
    y = buffer[2];
    y |= ((uint16_t)buffer[3]) << 8;
    z = buffer[4];
    z |= ((uint16_t)buffer[5]) << 8;

    uint8_t lsb_value = getRangeLsb();
    x_g = lsb_value * ((float)x / _dataScale);
    y_g = lsb_value * ((float)y / _dataScale);
    z_g = lsb_value * ((float)z / _dataScale);
}

/*!
 *  @brief  Read the auxilary ADC
 *  @param  adc
 *          adc index. possible values (1, 2, 3).
 *  @return auxilary ADC value
 */
int16_t Adafruit_LIS3DH::readADC(uint8_t adc) {
    if ((adc < 1) || (adc > 3))
        return 0;
    adc--; // switch to 0 indexed

    uint16_t value;
    uint8_t reg = LIS3DH_REG_OUTADC1_L + (adc * 2);

    if (i2c_dev) {
        reg |= 0x80; // set [7] for auto-increment
    }
    else {
        reg |= 0x40; // set [6] for auto-increment
        reg |= 0x80; // set [7] for read
    }

    uint8_t buffer[2];
    Adafruit_BusIO_Register adc_data =
        Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, reg, 2);

    adc_data.read(buffer, 2);

    value = buffer[0];
    value |= ((uint16_t)buffer[1]) << 8;

    return value;
}

/*!
 *   @brief  Set INT to output for single or double click
 *   @param  c
 *					 0 = turn off I1_CLICK
 *           1 = turn on all axes & singletap
 *					 2 = turn on all axes & doubletap
 *   @param  clickthresh
 *           CLICK threshold value
 *   @param  timelimit
 *           sets time limit (default 10)
 *   @param  timelatency
 *   				 sets time latency (default 20)
 *   @param  timewindow
 *   				 sets time window (default 255)
 */

void Adafruit_LIS3DH::setClick(uint8_t c, uint8_t clickthresh,
    uint8_t timelimit, uint8_t timelatency,
    uint8_t timewindow) {

    Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL3, 1);
    Adafruit_BusIO_RegisterBits i1_click =
        Adafruit_BusIO_RegisterBits(&ctrl3, 1, 7);

    Adafruit_BusIO_Register click_cfg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CLICKCFG, 1);

    if (!c) {
        // disable int
        i1_click.write(0); // disable i1 click
        click_cfg.write(0);
        return;
    }
    // else...

    i1_click.write(1); // enable i1 click

    Adafruit_BusIO_Register ctrl5 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL5, 1);

    Adafruit_BusIO_RegisterBits int1_latch_bit =
        Adafruit_BusIO_RegisterBits(&ctrl5, 1, 3);
    int1_latch_bit.write(true);

    if (c == 1)
        click_cfg.write(0x15); // turn on all axes & singletap
    if (c == 2)
        click_cfg.write(0x2A); // turn on all axes & doubletap

    Adafruit_BusIO_Register click_ths = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CLICKTHS, 1);
    click_ths.write(clickthresh); // arbitrary

    Adafruit_BusIO_Register time_limit = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_TIMELIMIT, 1);
    time_limit.write(timelimit); // arbitrary

    Adafruit_BusIO_Register time_latency = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_TIMELATENCY, 1);
    time_latency.write(timelatency); // arbitrary

    Adafruit_BusIO_Register time_window = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_TIMEWINDOW, 1);
    time_window.write(timewindow); // arbitrary
}

/*!
 *   @brief  Get uint8_t for single or double click
 *   @return register LIS3DH_REG_CLICKSRC
 */
uint8_t Adafruit_LIS3DH::getClick(void) {
    Adafruit_BusIO_Register click_reg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CLICKSRC, 1);

    return click_reg.read();
}

/*!
 *   @brief  Get uint8_t for INT1 source and clear interrupt
 *   @return register LIS3DH_REG_INT1SRC
 */
uint8_t Adafruit_LIS3DH::readAndClearInterrupt(void) {
    Adafruit_BusIO_Register int_reg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_INT1SRC, 1);

    return int_reg.read();
}

/**
 * @brief Enable or disable the Data Ready interupt
 *
 * @param enable_drdy true to enable the given Data Ready interrupt on INT1,
 * false to disable it
 * @param int_pin which DRDY interrupt to enable; 1 for DRDY1, 2 for DRDY2
 * @return true: success false: failure
 */
bool Adafruit_LIS3DH::enableDRDY(bool enable_drdy, uint8_t int_pin) {
    Adafruit_BusIO_Register _ctrl3 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL3, 1);
    Adafruit_BusIO_RegisterBits _drdy1_int_enable =
        Adafruit_BusIO_RegisterBits(&_ctrl3, 1, 4);
    Adafruit_BusIO_RegisterBits _drdy2_int_enable =
        Adafruit_BusIO_RegisterBits(&_ctrl3, 1, 3);

    if (int_pin == 1) {
        return _drdy1_int_enable.write(enable_drdy);
    }
    else if (int_pin == 2) {
        return _drdy2_int_enable.write(enable_drdy);
    }
    else {
        return false;
    }
}

/*!
 *   @brief  Sets the g range for the accelerometer
 *   @param  range
 *           range value
 */
void Adafruit_LIS3DH::setRange(lis3dh_range_t range) {

    Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);

    Adafruit_BusIO_RegisterBits range_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl4, 2, 4);
    range_bits.write(range);
    _range = range;
    delay(15); // delay to let new setting settle
}

/*!
 *  @brief  Gets the g range for the accelerometer
 *  @return Returns g range value
 */
lis3dh_range_t Adafruit_LIS3DH::getRange(void) {
    Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);

    Adafruit_BusIO_RegisterBits range_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl4, 2, 4);
    _range = (lis3dh_range_t)range_bits.read();
    return _range;
}

/*!
 *  @brief  Sets the data rate for the LIS3DH (controls power consumption)
 *  @param  dataRate
 *          data rate value
 */
void Adafruit_LIS3DH::setDataRate(lis3dh_dataRate_t dataRate, lis3dh_powerMode_t powerMode) {
    Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
    Adafruit_BusIO_RegisterBits data_rate_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl1, 4, 4);


    switch (dataRate)
    {
        case LIS3DH_DATARATE_LOWPOWER_1K6HZ:
        {
            setPowerMode(LIS3DH_PM_LOW_POWER);
        }break;
        default:
        {
            setPowerMode(powerMode);
        }break;

    }

    data_rate_bits.write(dataRate);
    _odr = dataRate;
}

/*!
 *   @brief  Gets the data rate for the LIS3DH (controls power consumption)
 *   @return Returns Data Rate value
 */
lis3dh_dataRate_t Adafruit_LIS3DH::getDataRate(void) {
    Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
    Adafruit_BusIO_RegisterBits data_rate_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl1, 4, 4);

    return (lis3dh_dataRate_t)data_rate_bits.read();
}

/*!
 *  @brief  Gets the most recent sensor event
 *  @param  *event
 *          sensor event that we want to read
 *  @return true if successful
 */
bool Adafruit_LIS3DH::getEvent(sensors_event_t* event) {
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_ACCELEROMETER;
    event->timestamp = 0;

    read();

    event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
    event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
    event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;

    return true;
}

/*!
 *   @brief  Gets the sensor_t data
 *   @param  *sensor
 *           sensor that we want to write data into
 */
void Adafruit_LIS3DH::getSensor(sensor_t* sensor) {
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy(sensor->name, "LIS3DH", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _sensorID;
    sensor->type = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay = 0;
    sensor->max_value = 0;
    sensor->min_value = 0;
    sensor->resolution = 0;
}

/**
 * @brief Configure INT1
 *
 * @param intMode Interrupt mode
 * @param intEn Interrupt Enable Bits. Select which axis and thershold to trigger.
 * @param threshold Interrupt threshold in g.
 * @param duration Minimum event duration in seconds.
 * @param intPin Interrupt pin on which to set the IA1.
 * @param intLatched Interrupt is latched or not.
 * @return bool True on success.
 */
bool Adafruit_LIS3DH::int1Cfg(lis3dh_interruptMode_t intMode, lis3dh_interruptEnable_t intEn, float threshold, float duration, lis3dh_intPin_t intPin, bool intLatched)
{
    if (intPin == LIS3DH_INT1_PIN)
    {
        writeRegBits(LIS3DH_REG_CTRL3, 1, 1, 6);
        // Adafruit_BusIO_Register _ctrl_reg3 = Adafruit_BusIO_Register(
        //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL3, 1);
        // Adafruit_BusIO_RegisterBits ia1_bits =
        //     Adafruit_BusIO_RegisterBits(&_ctrl_reg3, 1, 6);
        // ia1_bits.write(1);
    }
    else
    {
        writeRegBits(LIS3DH_REG_CTRL6, 1, 1, 6);
        // Adafruit_BusIO_Register _ctrl_reg6 = Adafruit_BusIO_Register(
        //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL6, 1);
        // Adafruit_BusIO_RegisterBits ia1_bits =
        //     Adafruit_BusIO_RegisterBits(&_ctrl_reg6, 1, 6);
        // ia1_bits.write(1);
    }
    writeRegBits(LIS3DH_REG_CTRL5, 0, 1, 2);
    // Adafruit_BusIO_Register _ctrl_reg5 = Adafruit_BusIO_Register(
    //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL5, 1);

    // Adafruit_BusIO_RegisterBits _d4d_int1_bits =
    //     Adafruit_BusIO_RegisterBits(&_ctrl_reg5, 1, 2);
    // _d4d_int1_bits.write(0);
    if (intLatched)
    {
        writeRegBits(LIS3DH_REG_CTRL5, 1, 1, 3);
        // Adafruit_BusIO_RegisterBits lir_int_bits =
        //     Adafruit_BusIO_RegisterBits(&_ctrl_reg5, 1, 3);
        // lir_int_bits.write(1);
    }
    else
    {
        writeRegBits(LIS3DH_REG_CTRL5, 0, 1, 3);
        // Adafruit_BusIO_RegisterBits lir_int_bits =
        //     Adafruit_BusIO_RegisterBits(&_ctrl_reg5, 1, 3);
        // lir_int_bits.write(0);
    }

    writeReg(LIS3DH_REG_INT1THS, (uint8_t)((round((threshold * 1000) / getIntRangeLsb())) & 0x7F));

    // Adafruit_BusIO_Register _int1_ths = Adafruit_BusIO_Register(
    //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_INT1THS, 1);
    // _int1_ths.write(regThs);

    writeReg(LIS3DH_REG_INT1DUR, (uint8_t)((round((duration) / getOdrTime())) & 0x7F));

    // uint8_t regDur = (uint8_t)((round((duration) / getOdrTime())) & 0x7F);
    // Adafruit_BusIO_Register _int1_dur = Adafruit_BusIO_Register(
    //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_INT1DUR, 1);
    // _int1_dur.write(regDur);
    // Serial.printf("regDur: %d\r\n", regDur);

    writeReg(LIS3DH_REG_INT1CFG, (uint8_t)(intMode | intEn));
    // Adafruit_BusIO_Register _int1_cfg = Adafruit_BusIO_Register(
    //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_INT1CFG, 1);
    // _int1_cfg.write(intMode | intEn);
    // Serial.printf("regcfg: 0x%X\r\n", intMode | intEn);



    return true;
}

uint8_t Adafruit_LIS3DH::readInt1Status()
{
    Adafruit_BusIO_Register _int1_src = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_INT1SRC, 1);
    return _int1_src.read();

}

bool Adafruit_LIS3DH::int1Disable()
{
    Adafruit_BusIO_Register _int1Cfg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_INT1CFG, 1);

    Adafruit_BusIO_RegisterBits int_en_bits =
        Adafruit_BusIO_RegisterBits(&_int1Cfg, 6, 0);
    int_en_bits.write(LIS3DH_INT_CFG_DISABLE);
    return true;
}


float Adafruit_LIS3DH::getOdrTime()
{
    switch (getDataRate())
    {
        case LIS3DH_DATARATE_400_HZ:
        {
            return 1.0f / 400.0f;
        }break;
        case LIS3DH_DATARATE_200_HZ:
        {
            return 1.0f / 200.0f;
        }break;
        case LIS3DH_DATARATE_100_HZ:
        {
            return 1.0f / 100.0f;
        }break;
        case LIS3DH_DATARATE_50_HZ:
        {
            return 1.0f / 50.0f;
        }break;
        case LIS3DH_DATARATE_25_HZ:
        {
            return 1.0f / 25.0f;
        }break;
        case LIS3DH_DATARATE_10_HZ:
        {
            return 1.0f / 10.0f;
        }break;
        case LIS3DH_DATARATE_1_HZ:
        {
            return 1.0f;
        }break;
        case LIS3DH_DATARATE_POWERDOWN:
        {
            return 0.0f;
        }break;
        case LIS3DH_DATARATE_LOWPOWER_5KHZ:
        {
            if (_powerMode == LIS3DH_PM_NORMAL || _powerMode == LIS3DH_PM_HIGH_RES)
            {
                return 1.0f / 1344.0f;
            }
            else
            {
                return 1.0f / 5600.0f;
            }
        }break;
        case LIS3DH_DATARATE_LOWPOWER_1K6HZ:
        {
            return 1.0f / 1600.0f;
        }break;
    }
    return 0;
}

void Adafruit_LIS3DH::setPowerMode(lis3dh_powerMode_t powerMode)
{
    Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
    Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);
    Adafruit_BusIO_RegisterBits lpen_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl1, 1, 3);
    Adafruit_BusIO_RegisterBits hr_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl4, 1, 3);

    switch (powerMode)
    {
        case LIS3DH_PM_POWER_DOWN:
        {
            setDataRate(LIS3DH_DATARATE_POWERDOWN);
            hr_bits.write(0);
            _powerMode = powerMode;
            _dataScale = 1;
        }break;
        case LIS3DH_PM_LOW_POWER:
        {
            hr_bits.write(0);
            lpen_bits.write(1);
            _powerMode = powerMode;
            _dataScale = LIS3DH_LOW_POWER_MODE_DATA_SCALE;
        }break;
        case LIS3DH_PM_NORMAL:
        {
            hr_bits.write(0);
            lpen_bits.write(0);
            _powerMode = powerMode;
            _dataScale = LIS3DH_NORMAL_MODE_DATA_SCALE;
        }break;
        case LIS3DH_PM_HIGH_RES:
        {
            hr_bits.write(1);
            lpen_bits.write(0);
            _powerMode = powerMode;
            _dataScale = LIS3DH_HIGH_RES_MODE_DATA_SCALE;
        }break;
    }
}

lis3dh_powerMode_t Adafruit_LIS3DH::getPowerMode()
{
    Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
    Adafruit_BusIO_Register _ctrl4 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL4, 1);
    Adafruit_BusIO_RegisterBits lpen_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl1, 1, 3);
    Adafruit_BusIO_RegisterBits hr_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl4, 1, 3);

    if (_odr == LIS3DH_DATARATE_POWERDOWN)
    {
        _powerMode = LIS3DH_PM_POWER_DOWN;
    }
    else if (lpen_bits.read() == 1 && hr_bits.read() == 0)
    {
        _powerMode = LIS3DH_PM_LOW_POWER;
    }
    else if (lpen_bits.read() == 0 && hr_bits.read() == 0)
    {
        _powerMode = LIS3DH_PM_NORMAL;
    }
    else if (lpen_bits.read() == 0 && hr_bits.read() == 1)
    {
        _powerMode = LIS3DH_PM_HIGH_RES;
    }

    return _powerMode;
}

void Adafruit_LIS3DH::enableAxes(lis3dh_activeAxis_t activeAxis)
{
    Adafruit_BusIO_Register _ctrl1 = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LIS3DH_REG_CTRL1, 1);
    Adafruit_BusIO_RegisterBits axes_en_bits =
        Adafruit_BusIO_RegisterBits(&_ctrl1, 3, 0);

    axes_en_bits.write(activeAxis);
}

uint8_t Adafruit_LIS3DH::getDataResolution()
{
    switch (_powerMode)
    {
        case LIS3DH_PM_LOW_POWER:
        {
            return 8;
        }break;
        case LIS3DH_PM_NORMAL:
        {
            return 10;
        }break;
        case LIS3DH_PM_HIGH_RES:
        {
            return 12;
        }break;
        default:
        {
            return 0;
        }
    }
}

uint8_t Adafruit_LIS3DH::getRangeLsb()
{
    uint8_t lsb_value = 1;
    if (_range == LIS3DH_RANGE_2_G)
        lsb_value = 4;
    else if (_range == LIS3DH_RANGE_4_G)
        lsb_value = 8;
    else if (_range == LIS3DH_RANGE_8_G)
        lsb_value = 16;
    else if (_range == LIS3DH_RANGE_16_G)
        lsb_value = 48;
    return lsb_value;
}

uint8_t Adafruit_LIS3DH::getIntRangeLsb()
{
    uint8_t lsb_value = 1;
    if (_range == LIS3DH_RANGE_2_G)
        lsb_value = 16;
    else if (_range == LIS3DH_RANGE_4_G)
        lsb_value = 32;
    else if (_range == LIS3DH_RANGE_8_G)
        lsb_value = 62;
    else if (_range == LIS3DH_RANGE_16_G)
        lsb_value = 186;
    return lsb_value;
}

void Adafruit_LIS3DH::enableFifo(lis3dh_fifoMode_t fifoMode, uint8_t fifoTh)
{
    if (fifoMode == LIS3DH_FIFO_MODE_BYPASS)
    {
        return;
    }
    disableFifo();
    writeRegBits(LIS3DH_REG_FIFOCTRL, fifoTh, 5, 0);
    writeRegBits(LIS3DH_REG_FIFOCTRL, fifoMode, 2, 6);
    writeRegBits(LIS3DH_REG_CTRL5, 1, 1, 6);
    _fifoEnabled = true;
}

void Adafruit_LIS3DH::disableFifo()
{
    writeRegBits(LIS3DH_REG_CTRL5, 0, 1, 6);
    writeRegBits(LIS3DH_REG_FIFOCTRL, LIS3DH_FIFO_MODE_BYPASS, 2, 6);
    _fifoEnabled = false;
}

uint8_t Adafruit_LIS3DH::getFifoSize()
{
    return readRegBits(LIS3DH_REG_FIFOSRC, 5, 0);
}

bool Adafruit_LIS3DH::fifoIsEmpty()
{
    return readRegBits(LIS3DH_REG_FIFOSRC, 1, 5);
}

uint8_t Adafruit_LIS3DH::readReg(uint8_t regAddr)
{
    Adafruit_BusIO_Register _reg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, regAddr, 1);
    return _reg.read();
}

bool Adafruit_LIS3DH::readRegBuffer(uint8_t regAddr, uint8_t* buf, uint8_t bufLen)
{
    Adafruit_BusIO_Register _reg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, regAddr, bufLen);

    return _reg.read(buf, bufLen);
}

void Adafruit_LIS3DH::writeReg(uint8_t regAddr, uint8_t regValue)
{
    Adafruit_BusIO_Register _reg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, regAddr, 1);
    _reg.write(regValue);
}

void Adafruit_LIS3DH::writeRegBits(uint8_t regAddr, uint8_t regValue, uint8_t bits, uint8_t shift)
{
    Adafruit_BusIO_Register _reg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, regAddr, 1);

    Adafruit_BusIO_RegisterBits _reg_bits =
        Adafruit_BusIO_RegisterBits(&_reg, bits, shift);
    _reg_bits.write(regValue);
}

uint8_t Adafruit_LIS3DH::readRegBits(uint8_t regAddr, uint8_t bits, uint8_t shift)
{
    Adafruit_BusIO_Register _reg = Adafruit_BusIO_Register(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, regAddr, 1);

    Adafruit_BusIO_RegisterBits _reg_bits =
        Adafruit_BusIO_RegisterBits(&_reg, bits, shift);
    return _reg_bits.read();
}
