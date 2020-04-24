"""
.. module:: iis3dwb

**************
IIS3DWB Module
**************

This module contains the driver for STMicroelectronics IIS3DWB 3-axis digital vibration sensor with low noise over an ultra-wide and flat frequency range.

In the IIS3DWB, the sensing elements of the accelerometer are implemented on the same silicon die, thus guaranteeing superior stability and robustness. (`datasheet <https://www.st.com/resource/en/datasheet/iis3dwb.pdf>`_).
    """

import struct
import spi

IIS3DWB_ID                      = 0x7B
IIS3DWB_WHO_AM_I                = 0x0F

IIS3DWB_COUNTER_BDR_REG1        = 0x0B

IIS3DWB_INT1_CTRL               = 0x0D
IIS3DWB_INT2_CTRL               = 0x0E

IIS3DWB_CTRL1_XL                = 0X10
IIS3DWB_CTRL3_C                 = 0x12

IIS3DWB_OUT_TEMP_L              = 0x20
IIS3DWB_OUT_TEMP_H              = 0x21
IIS3DWB_REG_DATA_OUT_X_L_A      = 0x28
IIS3DWB_REG_DATA_OUT_X_H_A      = 0x29
IIS3DWB_REG_DATA_OUT_Y_L_A      = 0x2A
IIS3DWB_REG_DATA_OUT_Y_H_A      = 0x2B
IIS3DWB_REG_DATA_OUT_Z_L_A      = 0x2C
IIS3DWB_REG_DATA_OUT_Z_H_A      = 0x2D

TIMEOUT_DURATION                = 1000 

IIS3DWB_CTRLx_ODR_OFF           = 0

IIS3DWB_ACC_SENS_FS_2G          = 0.061 # Sensitivity value for 2g full scale [mg/LSB] 
IIS3DWB_ACC_SENS_FS_4G          = 0.122
IIS3DWB_ACC_SENS_FS_8G          = 0.244
IIS3DWB_ACC_SENS_FS_16G         = 0.488

IIS3DWB_ACC_FS_REG_VALUE = [
    0,4,8,12
]

IIS3DWB_ACC_SENS_VALUE = [
    IIS3DWB_ACC_SENS_FS_2G,
    IIS3DWB_ACC_SENS_FS_16G,
    IIS3DWB_ACC_SENS_FS_4G,
    IIS3DWB_ACC_SENS_FS_8G
]

IIS3DWB_ODR_OFF     = 0
IIS3DWB_ODR_26k7Hz  = 5

_ODR_MASK = 0b00001111
_FS_MASK  = 0b11110000

@c_native("_iis3dwb_getfast", ["csrc/iis3dwb.c"])
def _iis3dwb_getfast(spi, sens):
    pass

@c_native("_iis3dwb_read_reg8",["csrc/iis3dwb.c"])
def _iis3dwb_read_reg8(spi,reg):
    pass

@c_native("_iis3dwb_read_reg16",["csrc/iis3dwb.c"])
def _iis3dwb_read_reg16(spi,reg):
    pass

@c_native("_iis3dwb_read_reg16x3",["csrc/iis3dwb.c"])
def _iis3dwb_read_reg16x3(spi,reg):
    pass

@c_native("_iis3dwb_write_reg8",["csrc/iis3dwb.c"])
def _iis3dwb_write_reg8(spi,reg,value):
    pass

@c_native("_iis3dwb_write_reg16",["csrc/iis3dwb.c"])
def _iis3dwb_write_reg16(spi,reg,value):
    pass

class IIS3DWB(spi.Spi):
    """

.. class:: IIS3DWB(spidrv, pin_cs, clk=5000000)

    Class which provides a simple interface to IIS3DWB features.
    
    Creates an instance of IIS3DWB class, using the specified SPI settings
    and initial device configuration

    :param spidrv: the *SPI* driver to use (SPI0, ...)
    :param pin_cs: Chip select pin to access the IIS3DWB chip
    :param clk: Clock speed, default 500 kHz

    Example: ::

        from stm.iis3dwb import iis3dwb

        ...

        vibro = iis3dwb.IIS3DWB(SPI0, D10)
        acc = vibro.get_acc_data()

    """

    def __init__(self, spidrv, pin_cs, clk=5000000):
        spi.Spi.__init__(self,pin_cs,spidrv,clock=clk)
        # for native functions
        self.spi   = spidrv & 0xFF
        self.odr   = IIS3DWB_ODR_26k7Hz
        self.fs    = 0
        self.sens  = IIS3DWB_ACC_SENS_FS_2G
        
        self.reset()
        sleep(100)

        # print(self.whoami())
        if IIS3DWB_ID != self.whoami():
            raise RuntimeError #("IIS3DWB not detected")

        self.enable(self.odr, self.fs)

    def _register_word(self, register, value=None):
        self.lock()
        self.select()
        ex = None
        ret = None
        try:
            if value is None:
                ret = _iis3dwb_read_reg16(self.spi, register)
            else:
                ret = _iis3dwb_write_reg16(self.spi, register, value)
        except Exception as e:
            ex = e
        self.unselect()
        self.unlock()
        if ex is not None:
            raise ex
        return ret

    def _register_char(self, register, value=None):
        self.lock()
        self.select()
        ex = None
        ret = None
        try:
            if value is None:
                ret = _iis3dwb_read_reg8(self.spi, register)
            else:
                ret = _iis3dwb_write_reg8(self.spi, register, value)
        except Exception as e:
            ex = e
        self.unselect()
        self.unlock()
        if ex is not None:
            raise ex
        return ret

    def _fs(self, reg, value):
        char = self._register_char(reg)
        char &= _FS_MASK # clear FS bits
        char |= value
        self._register_char(reg, char)

    def _odr(self, reg, value):
        char = self._register_char(reg)
        char &= _ODR_MASK # clear ODR bits
        char |= value<<5
        self._register_char(reg, char)
    
    def reset(self):
        """
    .. method:: reset()

        Reset the device using the internal register flag.

        """
        ctr3 = self._register_char(IIS3DWB_CTRL3_C)
        ctr3 |= 0x01
        self._register_char(IIS3DWB_CTRL3_C, ctr3)

    def enable(self, odr=IIS3DWB_ODR_26k7Hz, fs=0):
        """

.. method:: enable(odr=IIS3DWB_ODR_26k7Hz, fs=0)

        Sets the device's configuration registers for accelerometer.
    
        **Parameters:**
    
        * **odr** : sets the Output Data Rate of the device. Available values are:
    
            ====== ================= =======================
            Value  Output Data Rate  Constant Name
            ====== ================= =======================
            0x00   OFF               IIS3DWB_ODR_OFF
            0x05   26.7 KHz          IIS3DWB_ODR_26k7Hz
            ====== ================= =======================
        
        * **fs** : sets the Device Full Scale. Available values are:
    
            ====== =========== ========================== =============
            Value  Full Scale  Costant Name               in mg/LSB
            ====== =========== ========================== =============
            0x00   ±2g         IIS3DWB_ACC_SENS_FS_2G     0.061 mg/LSB
            0x01   ±4g         IIS3DWB_ACC_SENS_FS_4G     0.122 mg/LSB
            0x02   ±8g         IIS3DWB_ACC_SENS_FS_8G     0.244 mg/LSB
            0x03   ±16g        IIS3DWB_ACC_SENS_FS_16G    0.488 mg/LSB
            ====== =========== ========================== =============
    
        Returns True if configuration is successful, False otherwise.

        """
        if fs not in range(0,4):
            raise ValueError
        if odr not in [0,5]:
            raise ValueError
        try:
            self._fs(IIS3DWB_CTRL1_XL, IIS3DWB_ACC_FS_REG_VALUE[fs])
            self._odr(IIS3DWB_CTRL1_XL, odr)
        except Exception as e:
            return False
        self.odr = odr
        self.fs = fs
        self.sens = IIS3DWB_ACC_SENS_VALUE[fs]
        return True

    def disable(self):
        """

.. method:: disable()

        Disables the accelerator sensor.

        Returns True if configuration is successful, False otherwise.

        """
        if self.odr_acc == 0:
            return True
        try:
            res = self._odr(IIS3DWB_CTRL1_XL, IIS3DWB_ODR_OFF)
        except Exception as e:
            return False
        self.odr = IIS3DWB_ODR_OFF
        return True

    def whoami(self):
        """

.. method:: whoami()

        Value of the *IIS3DWB_WHO_AM_I* register (0x6B).
        
        """
        return self._register_char(IIS3DWB_WHO_AM_I)

    def get_acc_data(self, ms2=True, raw=False):
        """

.. method:: get_acc_data(ms2=True, raw=False)

        Retrieves accelerometer data in one call.

        :param ms2: If ms2 flag is True, returns data converted in m/s^2; otherwise in mg (default True)
        :param raw: If raw flag is True, returns raw register values (default False)

        Returns acc_x, acc_y, acc_z

        """

        s = self.sens
        # x = self._register_word(IIS3DWB_REG_DATA_OUT_X_L_A) * s
        # y = self._register_word(IIS3DWB_REG_DATA_OUT_y_L_A) * s
        # z = self._register_word(IIS3DWB_REG_DATA_OUT_Z_L_A) * s
        # return (x,y,z)
        self.lock()
        self.select()

        ex = None
        ret = None
        try:
            ret = _iis3dwb_read_reg16x3(self.spi, IIS3DWB_REG_DATA_OUT_X_L_A)
            # (x,y,z) = _iis3dwb_read_reg16x3(self.spi, IIS3DWB_REG_DATA_OUT_X_L_A)
        except Exception as e:
            ex = e

        self.unselect()
        self.unlock()

        if ex is not None:
            raise ex
        if raw:
            return ret

        ret = (ret[0] * s, ret[1] * s, ret[2] * s)
        # ret = (x * s, y * s, z * s)

        if ms2 == False:
            return ret
        ret = (ret[0] * 0.00981, ret[1] * 0.00981, ret[2] * 0.00981)
        
        return ret


    def get_temp_data(self, raw=False):
        """

.. method:: get_temp()

        Retrieves temperature in one call; if raw flag is enabled, returns raw register values.

        Returns temp

        """
        traw = self._register_word(IIS3DWB_OUT_TEMP_L)
        if raw:
            return traw
        temp = (traw / 256.0) + 25.0
        return temp

    def get_fast(self):
        """

.. method:: get_fast()

        Retrieves all data sensors in one call in fast way (c-code acquisition).
        
        * Temperature in °C
        * Acceleration in m/s^2

        Returns temp, acc_x, acc_y, acc_z

        """
        return _iis3dwb_getfast(self.spi, self.sens)

    def set_event_interrupt(self, pin_int, enable):
        """

.. method:: set_event_interrupt(pin_int, enable)

        Enables the interrupt pins. When data from sensor will be ready, the related interrupt pin configured will be set to high.

        :param pin_int: ID of the interrupt pin to be enabled/disabled. Available values are 1 o 2.
        :param enable: If enable flag is True, interrupt pin will be enabled, otherwise will be disabled.

        Returns True if configuration is successful, False otherwise.

        """
        if pin_int not in [1,2]:
            raise ValueError
        if enable not in [True, False]:
            raise ValueError

        value = 0b00000001
        
        if pin_int == 1:
            reg = IIS3DWB_INT1_CTRL
        elif pin_int == 2:
            reg = IIS3DWB_INT1_CTRL

        try:
            char = self._register_char(reg)
            if enable == False:
                char &= ~value
            else:
                char |= value
            self._register_char(reg, char)
        except Exception as e:
            return False
        return True