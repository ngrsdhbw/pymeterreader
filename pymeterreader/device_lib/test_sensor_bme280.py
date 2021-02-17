import unittest
from unittest import mock

import typing as tp

from pymeterreader.device_lib.common import ChannelValue
from pymeterreader.device_lib.sensor_bme280 import Bme280Reader
import typing as tp
import unittest
from unittest import mock

from pymeterreader.device_lib.common import ChannelValue
from pymeterreader.device_lib.sensor_bme280 import Bme280Reader

pres_raw = 343552
temp_raw = 516760
humi_raw = 24840


class MockBus():
    REG_ADDR_CHIP_ID = 0xD0
    REG_ADDR_MEASUREMENT_START = 0xF7
    REG_ADDR_CALIBRATION1_START = 0x88
    REG_ADDR_CALIBRATION2_START = 0xE1
    REG_ADDR_STATUS = 0xF3
    calibration1 = [109, 111, 157, 104, 50, 0, 33, 142, 248, 214, 208, 11,
                    201, 28, 230, 255, 249, 255, 172, 38, 10, 216, 189, 16, 0, 75]
    calibration2 = [135, 1, 0, 15, 46, 3, 30]
    data = [83, 224, 0, 126, 41, 128, 97, 8]

    def read_byte_data(self, i2c_addr, register, force=None) -> int:
        return self.read_i2c_block_data(i2c_addr, register, 1, force)[0]

    def __init__(self,i2c_addr:int) -> None:
        self.i2c_addr = i2c_addr
        self.written = []
        self.open = False

    def write_byte_data(self, i2c_addr, register, value, force=None) -> None:
        assert self.open is True
        self.written.append((i2c_addr, register, value))

    def read_i2c_block_data(self, i2c_addr, register, length, force=None) -> tp.List[int]:
        if i2c_addr != self.i2c_addr:
            raise OSError(f"Can not access SMBus@{i2c_addr}")
        assert self.open is True
        if register == self.REG_ADDR_CALIBRATION1_START and length == 26:
            return self.calibration1
        elif register == self.REG_ADDR_CALIBRATION2_START and length == 7:
            return self.calibration2
        elif register == self.REG_ADDR_MEASUREMENT_START and length == 8:
            return self.data
        elif register == self.REG_ADDR_CHIP_ID and length == 1:
            return [0x60]
        elif register == self.REG_ADDR_STATUS and length == 1:
            return [0b00000000]
        raise AssertionError("Unknown request")

    def __enter__(self):
        self.open = True
        return self

    def __exit__(self, type, value, traceback) -> None:
        self.open = False

ref_channels = [ChannelValue('TEMPERATURE', 19.27, '°C'), ChannelValue('PRESSURE', 998.5556593146621, 'hPa'),
                ChannelValue('HUMIDITY', 50.93572133159069, '%')]
[ChannelValue(channel_name='TEMPERATURE', value=19.272266477266385, unit='°C'),
 ChannelValue(channel_name='PRESSURE', value=99855.59723964224, unit='Pa'),
 ChannelValue(channel_name='HUMIDITY', value=50.935725617532256, unit='%')]
ref_meter_id = 'BME280-078fc53ee157b535d787a94e8ac2f05ed6083c8d21ef77389021ae97961d7d0a'


class TestBme280(unittest.TestCase):
    @mock.patch('pymeterreader.device_lib.sensor_bme280.SMBus', autospec=True)
    def test_readout(self, mock_smbus):
        mock_smbus.return_value = MockBus(0x76)
        reader = Bme280Reader("0x76")
        sample = reader.poll()
        self.assertEqual(ref_channels, sample.channels)

    @mock.patch('pymeterreader.device_lib.sensor_bme280.SMBus', autospec=True)
    def test_read_wrong_address(self, mock_smbus):
        mock_smbus.return_value = MockBus(0x76)
        reader = Bme280Reader("0x77")
        sample = reader.poll()
        self.assertIsNone(sample)


if __name__ == '__main__':
    unittest.main()
