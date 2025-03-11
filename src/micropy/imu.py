from machine import Pin, I2C
from micropython import const
from struct import unpack
import math


# Consts
_IMU_ADDR        = const(0x69)
_IMU_FREQ        = const(100000)
_SDA_PIN         = const(2)
_SCL_PIN         = const(3)
_I2C_BUS_ID      = const(1)
_SIX_BYTES       = const(6)
_SENSITIVITY_DPS = const(131)
_HIGH            = const(0x01)
_PWR_MGMT_1      = const(0x06)
_GYRO_XOUT_H     = const(0x33)
_GZ_OFFSET_RAD   = -0.0132  # Calibrated


class imu_i2c:
    def __init__(self) -> None:
        self.__i2c = I2C(_I2C_BUS_ID, sda=Pin(_SDA_PIN), scl=Pin(_SCL_PIN), freq=_IMU_FREQ)
        self.__scan_i2c_bus()
        self.__initialize_imu()
        
    def __scan_i2c_bus(self) -> None:
        ''' Scan for I2C devices '''
        if not self.__i2c.scan():
            raise Exception('No I2C devices found')

    def __imu_write_register(self, reg, data) -> None:
        ''' Write a byte to a register '''
        self.__i2c.writeto(_IMU_ADDR, bytes([reg, data]))

    def __imu_read_register(self, reg, length) -> int:
        ''' Read bytes from a register '''
        self.__i2c.writeto(_IMU_ADDR, bytes([reg]))
        return self.__i2c.readfrom(_IMU_ADDR, length)

    def __initialize_imu(self) -> None:
        ''' Wake up and configure IMU '''
        self.__imu_write_register(_PWR_MGMT_1, _HIGH)

    def read_gyro_z_rps(self) -> float:
        ''' Read Z-axis gyroscope data and convert to rad/s '''
        raw_data = self.__imu_read_register(_GYRO_XOUT_H, _SIX_BYTES)
        _, _, gz = unpack('>hhh', raw_data)
        return math.radians(gz/_SENSITIVITY_DPS) - _GZ_OFFSET_RAD


# Initialize IMU
onboard_imu = imu_i2c()
    