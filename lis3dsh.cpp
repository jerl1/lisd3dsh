/*
 * mbed library program
 *  LIS3DSH MEMS motion sensor: 3-axis "nano" accelerometer, made by STMicroelectronics
 *      http://www.st.com/web/catalog/sense_power/FM89/SC444/PF252716
 *
 * Copyright (c) 2014 Jérôme Lévêque
 * Based on the work of Kenji Arai / JH1PJL for the LIS3DH
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *      Created: December   7th, 2014
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "lis3dsh.h"

#define SPI_I2C_READ      0x80

lis3dsh_base::lis3dsh_base(lis3dshDataRate data_rate, lis3dshFullScale fullscale) :
    mDataRate(data_rate),
    mFullscale(fullscale) {

    static const float fullscale_factor[] = {
        LIS3DSH_SENSITIVITY_2G, LIS3DSH_SENSITIVITY_4G, LIS3DSH_SENSITIVITY_6G,
        LIS3DSH_SENSITIVITY_8G, LIS3DSH_SENSITIVITY_16G };

    mFullscaleFactor = fullscale_factor[mFullscale];
    mAvailable = false;
}

void lis3dsh_base::init() {
    mAvailable = (read_reg(LIS3DSH_WHO_AM_I, true) == id());

    if (InitOK()) {
        write_reg(LIS3DSH_CTRL_REG4, 0x0F | (mDataRate << 4));
        write_reg(LIS3DSH_CTRL_REG5, mFullscale << 4);
    }
}

void lis3dsh_base::read_mg_data(float *dt) {
    char data[6] = {0, 0, 0, 0, 0, 0};

    if (InitOK()) {
        read_reg_data(data);
        // change data type
        dt[0] = float(short((data[1] << 8) | data[0])) * mFullscaleFactor / 15;
        dt[1] = float(short((data[3] << 8) | data[2])) * mFullscaleFactor / 15;
        dt[2] = float(short((data[5] << 8) | data[4])) * mFullscaleFactor / 15;
    } else {
        dt[0] = 0;
        dt[1] = 0;
        dt[2] = 0;
    }
}

void lis3dsh_base::read_data(float *dt) {
    read_mg_data(dt);
    dt[0] *= GRAVITY;
    dt[1] *= GRAVITY;
    dt[2] *= GRAVITY;
}

uint8_t lis3dsh_base::read_id() {
    return read_reg(LIS3DSH_WHO_AM_I);
}

//-----------------------------------------

lis3dsh_i2c::lis3dsh_i2c (PinName p_sda, PinName p_scl,
    lis3dshAddress addr, lis3dshDataRate data_rate, lis3dshFullScale fullscale) :
    lis3dsh_base(data_rate, fullscale),
    mAddr(addr) {
    mI2C = new I2C(p_sda, p_scl);
    mI2C->frequency(400000);

    init();
}

lis3dsh_i2c::~lis3dsh_i2c(void) {
    delete mI2C;
}

void lis3dsh_i2c::read_reg_data(char *data) {
    dbf[0] = LIS3DSH_OUT_X_L | SPI_I2C_READ;
    mI2C->write(mAddr, dbf, 1, true);
    mI2C->read(mAddr, data, 6, false);
}

void lis3dsh_i2c::frequency(int hz) {
    mI2C->frequency(hz);
}

uint8_t lis3dsh_i2c::read_reg(uint8_t addr, bool force) {
    if (force || InitOK()) {
        dbf[0] = addr;
        mI2C->write(mAddr, dbf, 1);
        mI2C->read(mAddr, dbf, 1);
    } else {
        dbf[0] = 0xff;
    }
    return (uint8_t)dbf[0];
}

void lis3dsh_i2c::write_reg(uint8_t addr, uint8_t data) {
    if (InitOK()) {
        dbf[0] = addr;
        dbf[1] = data;
        mI2C->write(mAddr, dbf, 2);
    }
}

//-----------------------------------------

lis3dsh_spi::lis3dsh_spi (PinName p_mosi, PinName p_miso, PinName p_sck, DigitalOut p_cs,
    lis3dshDataRate data_rate, lis3dshFullScale fullscale) :
    lis3dsh_base(data_rate, fullscale),
    mCS(p_cs) {
    mSPI = new SPI(p_mosi, p_miso, p_sck);
    mSPI->format(8, 3);
    mSPI->frequency(1000000);

    //Deselect
    mCS = 1;
    init();
}

lis3dsh_spi::~lis3dsh_spi(void) {
    delete mSPI;
}

void lis3dsh_spi::read_reg_data(char *data) {
    dbf[0] = LIS3DSH_OUT_X_L | SPI_I2C_READ;
    mCS = 0;
    mSPI->write(dbf[0]);
    data[0] = mSPI->write(0);
    data[1] = mSPI->write(0);
    data[2] = mSPI->write(0);
    data[3] = mSPI->write(0);
    data[4] = mSPI->write(0);
    data[5] = mSPI->write(0);
    mCS = 1;
}

void lis3dsh_spi::frequency(int hz) {
    mSPI->frequency(hz);
}

uint8_t lis3dsh_spi::read_reg(uint8_t addr, bool force) {
    dbf[0] = addr | SPI_I2C_READ;
    if (force || InitOK()) {
        mCS = 0;
        mSPI->write(dbf[0]);
        dbf[0] = mSPI->write(0);
        mCS = 1;
    } else {
        dbf[0] = 0xff;
    }
    return (uint8_t)dbf[0];
}

void lis3dsh_spi::write_reg(uint8_t addr, uint8_t data) {
    if (InitOK()) {
        dbf[0] = addr;
        dbf[1] = data;
        mCS = 0;
        mSPI->write(dbf[0]);
        mSPI->write(dbf[1]);
        mCS = 1;
    }
}
