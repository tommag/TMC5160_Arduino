/*
MIT License

Copyright (c) 2016 Mike Estee

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
*/

#include "TMC5160.h"

TMC5160_SPI::TMC5160_SPI( uint8_t chipSelectPin, uint32_t fclk, const SPISettings &spiSettings, SPIClass &spi )
: TMC5160(fclk), _CS(chipSelectPin), _spiSettings(spiSettings), _spi(&spi)
{
	pinMode(chipSelectPin, OUTPUT);
}


// calls to read/write registers must be bracketed by the begin/endTransaction calls

void _chipSelect( uint8_t pin, bool select )
{
	digitalWrite(pin, select?LOW:HIGH);
	if( select )
		delayMicroseconds(100);   // per spec, settling time is 100us
}

void TMC5160_SPI::_beginTransaction()
{
	_spi->beginTransaction(_spiSettings);
	_chipSelect(_CS, true);
}

void TMC5160_SPI::_endTransaction()
{
	_chipSelect(_CS, false);
	_spi->endTransaction();
}

uint32_t TMC5160_SPI::readRegister(uint8_t address)
{
	// request the read for the address
	_beginTransaction();
	_spi->transfer(address);
	_spi->transfer(0x00);
	_spi->transfer(0x00);
	_spi->transfer(0x00);
	_spi->transfer(0x00);
	_endTransaction();

	// skip a beat
	#define nop() asm volatile("nop")
	nop();
	nop();
	nop();

	// read it in the second cycle
	_beginTransaction();
	_spi->transfer(address);
	uint32_t value = 0;
	value |= (uint32_t)_spi->transfer(0x00) << 24;
	value |= (uint32_t)_spi->transfer(0x00) << 16;
	value |= (uint32_t)_spi->transfer(0x00) << 8;
	value |= (uint32_t)_spi->transfer(0x00);
	_endTransaction();

	_lastRegisterReadSuccess = true; // In SPI mode there is no way to know if the TMC5130 is plugged...

	return value;
}

uint8_t TMC5160_SPI::writeRegister(uint8_t address, uint32_t data)
{
	// address register
	_beginTransaction();
	uint8_t status = _spi->transfer(address | WRITE_ACCESS);

	// send new register value
	_spi->transfer((data & 0xFF000000) >> 24);
	_spi->transfer((data & 0xFF0000) >> 16);
	_spi->transfer((data & 0xFF00) >> 8);
	_spi->transfer(data & 0xFF);
	_endTransaction();

	return status;
}


uint8_t TMC5160_SPI::readStatus()
{
 	// read general config
 	_beginTransaction();
 	uint8_t status = _spi->transfer(TMC5160_Reg::GCONF);
 	// send dummy data
 	_spi->transfer(0x00);
 	_spi->transfer(0x00);
 	_spi->transfer(0x00);
 	_spi->transfer(0x00);
 	_endTransaction();

	return status;
}
