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

#ifndef LIS3DSH_H
#define LIS3DSH_H

#include "mbed.h"

//  LIS3DSH Address for I2C interface
//  7bit address = 0b0011101  or 0b0011110 (0x1D or 0x1E depends on SEL)
//       -> 8bit = 0b00111010 or 0b00111100 -> 0x3A,0x3C(Read) or 0x3B,0x3D(Write)
typedef enum _lis3dshAddress {
    LIS3DSH_G_CHIP_ADDR = 0x3A, // SEL = Vdd
    LIS3DSH_V_CHIP_ADDR = 0x3C  // SEL = Ground
} lis3dshAddress;

//  Register's definition
//According to "Doc ID 022405 Rev 1"
#define LIS3DSH_OUT_T          0x0C
#define LIS3DSH_INFO1          0x0D
#define LIS3DSH_INFO2          0x0E
#define LIS3DSH_WHO_AM_I       0x0F
#define LIS3DSH_OFF_X          0x10
#define LIS3DSH_OFF_Y          0x11
#define LIS3DSH_OFF_Z          0x12
#define LIS3DSH_CS_X           0x13
#define LIS3DSH_CS_Y           0x14
#define LIS3DSH_CS_Z           0x15
#define LIS3DSH_LC_L           0x16
#define LIS3DSH_LC_H           0x17
#define LIS3DSH_STAT           0x18

#define LIS3DSH_PEAKS2         0x1A
#define LIS3DSH_VFC_1          0x1B
#define LIS3DSH_VFC_2          0x1C
#define LIS3DSH_VFC_3          0x1D
#define LIS3DSH_VFC_4          0x1E
#define LIS3DSH_THRS3          0x1F
#define LIS3DSH_CTRL_REG4      0x20

#define LIS3DSH_CTRL_REG3      0x23
#define LIS3DSH_CTRL_REG5      0x24
#define LIS3DSH_CTRL_REG6      0x25

#define LIS3DSH_STATUS         0x27

#define LIS3DSH_OUT_X_L        0x28
#define LIS3DSH_OUT_X_H        0x29
#define LIS3DSH_OUT_Y_L        0x2A
#define LIS3DSH_OUT_Y_H        0x2B
#define LIS3DSH_OUT_Z_L        0x2C
#define LIS3DSH_OUT_Z_H        0x2D
#define LIS3DSH_FIFO_CTRL      0x2E
#define LIS3DSH_FIFO_SRC       0x2F

#define LIS3DSH_DES2           0x78

#define LIS3DSH_PR2            0x7C
#define LIS3DSH_TC2_L          0x7D
#define LIS3DSH_TC2_H          0x7E
#define LIS3DSH_OUTS2          0x7F

// Output Data Rate (ODR)
typedef enum _lis3dshDataRate {
    LIS3DSH_DR_PWRDWN = 0,
    LIS3DSH_DR_3HZ125,
    LIS3DSH_DR_6HZ250,
    LIS3DSH_DR_12HZ500,
    LIS3DSH_DR_25HZ,
    LIS3DSH_DR_50HZ,
    LIS3DSH_DR_100HZ,
    LIS3DSH_DR_400HZ,
    LIS3DSH_DR_800HZ,
    LIS3DSH_DR_1600HZ
} lis3dshDataRate;

// Bandwidth (Low pass)
typedef enum _lis3dshBandwidth {
    LIS3DSH_BW_LOW = 0,
    LIS3DSH_BW_M_LOW,
    LIS3DSH_BW_M_HI,
    LIS3DSH_BW_HI
} _lis3dshBandwidth;

// Full Scale
typedef enum _lis3dshFullScale {
    LIS3DSH_FS_2G = 0,
    LIS3DSH_FS_4G,
    LIS3DSH_FS_6G,
    LIS3DSH_FS_8G,
    LIS3DSH_FS_16G
} lis3dshFullScale;

// definition for Nomalization
#define LIS3DSH_SENSITIVITY_2G  (0.001F)
#define LIS3DSH_SENSITIVITY_4G  (0.002F)
#define LIS3DSH_SENSITIVITY_6G  (0.003F)
#define LIS3DSH_SENSITIVITY_8G  (0.004F)
#define LIS3DSH_SENSITIVITY_16G (0.012F)

//Gravity at Earth's surface in m/s/s
#define GRAVITY                (9.80665F)

/** Interface for STMicronics MEMS motion sensor: 3-axis "nano" accelerometer
 *      Chip: LIS3DSH
 *
 * @code
 * #include "mbed.h"
 *
 * // I2C Communication
 * LIS3DSH acc(p_sda, p_scl, chip_addr, datarate, bandwidth, fullscale);
 *
 * int main() {
 * float f[3];
 *
 *   if (acc.InitOk()) {
 *      acc.read_data(f);
 *   }
 * }
 * @endcode
 */

class lis3dsh_base {
public:
    /**
      * return the status of the initialization
      */
    bool InitOK() { return mAvailable; }

    /** Read a float type data from acc
      * @param float type of three arry's address, e.g. float dt[3];
      * @return acc motion data unit: m/s/s(m/s2)
      * @return dt[0]->x, dt[1]->y, dt[2]->z
      */
    void read_data(float *dt);

    /** Read a float type data from acc
      * @param float type of three arry's address, e.g. float dt[3];
      * @return acc motion data unit: mg
      * @return dt[0]->x, dt[1]->y, dt[2]->z
      */
    void read_mg_data(float *dt);

    /** Read a acc ID number
      * @param none
      * @return if STM MEMS acc, it should be id()
      */
    uint8_t read_id();

    /** Set communication clock frequency
      * @param freq.
      * @return none
      */
    virtual void frequency(int hz) = 0;

protected:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param device address LIS3DSH(SA0=0 or 1), LIS3DSH_G_CHIP_ADDR or LIS3DSH_V_CHIP_ADDR
      * @param output data rate selection, power down mode, 1Hz to 5KHz
      * @param full scale selection, +/-2g to +/-16g
      */
    lis3dsh_base(lis3dshDataRate data_rate, lis3dshFullScale fullscale);

    void init();

    /** Read register (general purpose)
      * @param force (for not checking if the init are ok)
      * @param register's address
      * @return register data
      */
    virtual uint8_t read_reg(uint8_t addr, bool force = false) = 0;

    /** Write register (general purpose)
      * @param register's address
      * @param data
      * @return none
      */
    virtual void write_reg(uint8_t addr, uint8_t data) = 0;

    virtual void read_reg_data(char *data) = 0;

    /** get id of the accelerator
      * @param none
      * @return accelerator ID
      */
    uint8_t id() {return 0x3f; }

    char dbf[2];                // Working buffer
    bool mAvailable;            // mems is available
    lis3dshDataRate mDataRate;
    lis3dshFullScale mFullscale;
    float mFullscaleFactor;
};

class lis3dsh_i2c : public lis3dsh_base {
public:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param device address LIS3DSH(SA0=0 or 1), LIS3DSH_G_CHIP_ADDR or LIS3DSH_V_CHIP_ADDR
      * @param output data rate selection, power down mode, 1Hz to 5KHz
      * @param full scale selection, +/-2g to +/-16g
      */
    lis3dsh_i2c(PinName p_sda, PinName p_scl,
        lis3dshAddress addr, lis3dshDataRate data_rate, lis3dshFullScale fullscale);

    ~lis3dsh_i2c(void);

    void frequency(int hz);
    uint8_t read_reg(uint8_t addr, bool force = false);
    void write_reg(uint8_t addr, uint8_t data);

protected:
    void read_reg_data(char *data);

private:
    I2C *mI2C = NULL;
    lis3dshAddress mAddr;
};

class lis3dsh_spi : public lis3dsh_base {
public:
    /** Configure data pin
      * @param data MOSI, MISO, SCK, CE pins
      * @param other parameters -> please see LIS3DsH(PinName p_sda, PinName p_scl,...)
      */
    lis3dsh_spi(PinName p_mosi, PinName p_miso, PinName p_sck, DigitalOut p_cs,
        lis3dshDataRate data_rate, lis3dshFullScale fullscale);

    ~lis3dsh_spi(void);

    void frequency(int hz);
    uint8_t read_reg(uint8_t addr, bool force = false);
    void write_reg(uint8_t addr, uint8_t data);

protected:
    void read_reg_data(char *data);

private:
    SPI *mSPI = NULL;
    DigitalOut mCS;
};

#endif      // LIS3DSH_H
