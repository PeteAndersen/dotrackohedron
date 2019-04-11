/**************************************************************************/
/*!
    @file     lis3dh.cpp
    @author   K. Townsend / Limor Fried (Adafruit Industries)
    @author   Peter Andersen
    @license  BSD

    This is a library for LIS3DH Acceleromoter based on the Adafruit library
    and modified for usage in this project.

    Original code: https://github.com/adafruit/Adafruit_LIS3DH

*/
/**************************************************************************/

#include "Arduino.h"
#include "lis3dh.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new LIS3DH class in hardware SPI mode
*/
/**************************************************************************/
lis3dh::lis3dh(int8_t cspin)
    : _cs(cspin), _sensorID(-1), _offset_x(0), _offset_y(0), _offset_z(0), _g_scale(1)
{
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool lis3dh::begin(void)
{
  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);
  SPI.begin();

  uint8_t deviceid = readRegister8(LIS3DH_REG_WHOAMI);
  if (deviceid != 0x33)
  {
    /* No LIS3DH detected ... return false */
    return false;
  }

  writeRegister8(LIS3DH_REG_CTRL1, 0x07); // enable all axes, normal mode
  writeRegister8(LIS3DH_REG_CTRL3, 0x10); // DRDY on INT1
  writeRegister8(LIS3DH_REG_CTRL4, 0x88); // High res & BDU enabled
  setDataRate(LIS3DH_DATARATE_400_HZ);
  setRange(LIS3DH_RANGE_2_G);

  return true;
}

bool lis3dh::calibrate(void)
{
  read();

  // Initialize for flat on surface
  _offset_x = x;
  _offset_y = y;
  _offset_z = z - _g_scale;

  return true;
}

void lis3dh::read(void)
{
  // read x y z at once
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer(LIS3DH_REG_OUT_X_L | 0x80 | 0x40); // read multiple, bit 7&6 high

  x = SPI.transfer(0xFF);
  x |= ((uint16_t)SPI.transfer(0xFF)) << 8;
  y = SPI.transfer(0xFF);
  y |= ((uint16_t)SPI.transfer(0xFF)) << 8;
  z = SPI.transfer(0xFF);
  z |= ((uint16_t)SPI.transfer(0xFF)) << 8;

  digitalWrite(_cs, HIGH);
  SPI.endTransaction(); // release the SPI bus

  x_g = (float)x / _g_scale;
  y_g = (float)y / _g_scale;
  z_g = (float)z / _g_scale;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void lis3dh::setRange(lis3dh_range_t range)
{
  uint8_t r = readRegister8(LIS3DH_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  writeRegister8(LIS3DH_REG_CTRL4, r);
  setGScale(range);
}

lis3dh_range_t lis3dh::getRange(void)
{
  lis3dh_range_t range = (lis3dh_range_t)((readRegister8(LIS3DH_REG_CTRL4) >> 4) & 0x03);
  setGScale(range);
  return range;
}

void lis3dh::setGScale(lis3dh_range_t range)
{
  if (range == LIS3DH_RANGE_16_G)
    _g_scale = 1365; // different sensitivity at 16g
  if (range == LIS3DH_RANGE_8_G)
    _g_scale = 4095;
  if (range == LIS3DH_RANGE_4_G)
    _g_scale = 8191;
  if (range == LIS3DH_RANGE_2_G)
    _g_scale = 16383;
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
void lis3dh::setDataRate(lis3dh_dataRate_t dataRate)
{
  uint8_t ctl1 = readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  writeRegister8(LIS3DH_REG_CTRL1, ctl1);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
lis3dh_dataRate_t lis3dh::getDataRate(void)
{
  return (lis3dh_dataRate_t)((readRegister8(LIS3DH_REG_CTRL1) >> 4) & 0x0F);
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void lis3dh::writeRegister8(uint8_t reg, uint8_t value)
{
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer(reg & ~0x80); // write, bit 7 low
  SPI.transfer(value);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction(); // release the SPI bus
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t lis3dh::readRegister8(uint8_t reg)
{
  uint8_t value;

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer(reg | 0x80); // read, bit 7 high
  value = SPI.transfer(0);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction(); // release the SPI bus

  return value;
}
