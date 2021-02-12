"""
Reader for BOSCH BME280 sensor.
"""
import logging
import time
import typing as tp
from dataclasses import dataclass
from threading import Lock

try:
    from smbus2 import SMBus
except ImportError:
    pass
from pymeterreader.device_lib.base import BaseReader
from pymeterreader.device_lib.common import Sample, Device

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Bme280CalibrationData:
    calibration_bytes_0to25: bytes
    calibration_bytes_26to41: bytes


class Bme280Reader(BaseReader):
    """
    Reads the Bosch BME280 using I2C
    """
    PROTOCOL = "BME280"
    # Shared Lock for access to all I2C Busses
    I2C_BUS_LOCK = Lock()
    REG_ADDR_CHIP_ID = 0xD0
    REG_ADDR_STATUS = 0xF3
    REG_ADDR_CONTROL_MEASUREMENT = 0xF4
    REG_ADDR_CONTROL_HUMIDITY = 0xF2
    REG_ADDR_CONFIG = 0xF5
    REG_ADDR_MEASUREMENT_START = 0xF7

    def __init__(self,
                 meter_address: tp.Union[str, int],
                 i2c_bus: int = 1,
                 mode: str = "forced",
                 standby_time: float = 1000,
                 irr_filter_coefficient: int = 0,
                 humidity_oversampling: int = 2,
                 pressure_oversampling: int = 2,
                 temperature_oversampling: int = 2,
                 cache_calibration: bool = True,
                 **kwargs) -> None:
        """
        Initialize BME280 Reader object
        :param meter_id: is a i2c bus id in this case
        """
        # Test if smbus library has been imported
        try:
            from smbus2 import SMBus
        except NameError:
            logger.error(
                "Could not import smbus2 library! This library is missing and Bme280Reader can not function without it!")
            raise
        super().__init__(**kwargs)
        self.standby_time = standby_time
        self.irr_filter_coefficient = irr_filter_coefficient
        self.humidity_oversampling = humidity_oversampling
        self.temperature_oversampling = temperature_oversampling
        self.pressure_oversampling = pressure_oversampling
        self.cache_calibration = cache_calibration

        # FIXME
        self.i2c_address = int(meter_address.lower(), 16)
        self.i2c_bus = i2c_bus
        self.mode = mode

        self.__reconfiguration_required = True

        self.__calibration_data: tp.Optional[Bme280CalibrationData] = None

    def poll(self) -> tp.Optional[Sample]:
        """
        Poll device
        :return: True, if successful
        """
        try:
            with Bme280Reader.I2C_BUS_LOCK:
                with SMBus(self.i2c_bus) as bus:
                    # Read Chip ID
                    chip_id = bus.read_byte_data(self.i2c_address, Bme280Reader.REG_ADDR_CHIP_ID)
                    if self.__calibration_data is None or not self.cache_calibration:
                        self.__calibration_data = self.__read_calibration_data(bus)
                    # Reconfigure sensor
                    if self.__reconfiguration_required:
                        # Put sensor in sleep mode for reconfiguration
                        self.__set_register_ctrl_meas(bus, "sleep", 0, 0)
                        # Configure humidity
                        self.__set_register_ctrl_hum(bus, self.humidity_oversampling)
                        # Configure other measurement parameters
                        self.__set_register_config(bus, self.standby_time, self.irr_filter_coefficient)
                        # Activate configuration
                        self.__set_register_ctrl_meas(bus, self.mode, self.temperature_oversampling,
                                                      self.pressure_oversampling)
                        self.__reconfiguration_required = False
                    # Wait for the measurement if running in forced mode
                    if "forced" in self.mode:
                        time_osrs_t = 2.3 * self.temperature_oversampling
                        time_osrs_p = 2.3 * self.pressure_oversampling + 0.575
                        time_osrs_h = 2.3 * self.humidity_oversampling + 0.575
                        measurement_time = 1.25 + time_osrs_t + time_osrs_p + time_osrs_h
                        # Wait for measurement to complete
                        time.sleep(measurement_time / 1000)
                        # Read measuring status from bit 3
                        status = bus.read_byte_data(self.i2c_address,Bme280Reader.REG_ADDR_STATUS)
                        if bool(status >> 3 & 1):
                            logger.error("Measurement is still in progress after maximum measurement time! Aborting...")
                            return None
                    # Read measurement registers
                    measurement = bus.read_i2c_block_data(self.i2c_address, Bme280Reader.REG_ADDR_MEASUREMENT_START, 8)
            pass
        except OSError:
            pass
        return None
        # meter id from hashsum

    def __read_calibration_data(self, bus: SMBus) -> Bme280CalibrationData:
        """
        This method reads the calibration data from the sensor
        :param bus: an open i2c bus that is already protected by a Lock
        """
        # Read calibration registers from 0xF0 to 0xA1
        calibration_bytes_0to25 = bus.read_i2c_block_data(self.i2c_address, 0x88, 26)
        # Remove unused register 0xA0
        calibration_bytes_0to25.pop(24)
        # Read calibration registers from 0xE1 to 0xE7
        calibration_bytes_26to41 = bus.read_i2c_block_data(self.i2c_address, 0xE1, 8)
        return Bme280CalibrationData(bytes(calibration_bytes_0to25), bytes(calibration_bytes_26to41))

    def __set_register_config(self, bus: SMBus, standby_time: float, irr_filter_coefficient: int) -> None:
        """
        This method configures the config register
        :param bus: an open i2c bus that is already protected by a Lock
        """
        # Set the standby time
        t_sb = 0b000
        if standby_time == 1000:
            t_sb = 0b101
        elif standby_time == 500:
            t_sb = 0b100
        elif standby_time == 250:
            t_sb = 0b011
        elif standby_time == 125:
            t_sb = 0b010
        elif standby_time == 62.5:
            t_sb = 0b001
        elif standby_time == 20:
            t_sb = 0b111
        elif standby_time == 10:
            t_sb = 0b110
        elif standby_time == 0.5:
            pass
        else:
            logger.warning(f"Standby time value {standby_time} is invalid!")
        # Set irr filter coefficient
        filter = 0b000
        if irr_filter_coefficient == 16:
            filter = 0b100
        elif irr_filter_coefficient == 8:
            filter = 0b011
        elif irr_filter_coefficient == 4:
            filter = 0b010
        elif irr_filter_coefficient == 2:
            filter = 0b001
        elif irr_filter_coefficient == 0:
            pass
        else:
            logger.warning(f"IRR filter coefficient value {irr_filter_coefficient} is invalid!")
        # Disable SPI Interface
        spi3wire_enable = 0
        # Concatenate bit sequences
        config_byte = (t_sb << 5) + (filter << 2) + spi3wire_enable
        bus.write_byte_data(self.i2c_address, Bme280Reader.REG_ADDR_CONFIG, config_byte)

    def __set_register_ctrl_hum(self, bus: SMBus, humidity_oversampling: int) -> None:
        """
        This method configures the ctrl_hum register
        :param bus: an open i2c bus that is already protected by a Lock
        """
        # Set humidity oversampling
        osrs_h = 0b000
        if humidity_oversampling == 16:
            osrs_h = 0b101
        elif humidity_oversampling == 8:
            osrs_h = 0b100
        elif humidity_oversampling == 4:
            osrs_h = 0b011
        elif humidity_oversampling == 2:
            osrs_h = 0b010
        elif humidity_oversampling == 1:
            osrs_h = 0b001
        elif humidity_oversampling == 0:
            pass
        else:
            logger.warning(f"Humidity oversampling value {humidity_oversampling} is invalid!")
        # Concatenate bit sequences
        ctrl_hum_byte = osrs_h
        bus.write_byte_data(self.i2c_address, Bme280Reader.REG_ADDR_CONTROL_HUMIDITY, ctrl_hum_byte)

    def __set_register_ctrl_meas(self, bus: SMBus, mode_str: str, temperature_oversampling: int,
                                 pressure_oversampling: int) -> None:
        """
        This method configures the ctrl_meas register
        :param bus: an open i2c bus that is already protected by a Lock
        """
        # Set temperature oversampling
        osrs_t = 0b000
        if temperature_oversampling == 16:
            osrs_t = 0b101
        elif temperature_oversampling == 8:
            osrs_t = 0b100
        elif temperature_oversampling == 4:
            osrs_t = 0b011
        elif temperature_oversampling == 2:
            osrs_t = 0b010
        elif temperature_oversampling == 1:
            osrs_t = 0b001
        elif temperature_oversampling == 0:
            pass
        else:
            logger.warning(f"Pressure oversampling value {temperature_oversampling} is invalid!")
        # Set pressure oversampling
        osrs_p = 0b000
        if pressure_oversampling == 16:
            osrs_p = 0b101
        elif pressure_oversampling == 8:
            osrs_p = 0b100
        elif pressure_oversampling == 4:
            osrs_p = 0b011
        elif pressure_oversampling == 2:
            osrs_p = 0b010
        elif pressure_oversampling == 1:
            osrs_p = 0b001
        elif pressure_oversampling == 0:
            pass
        else:
            logger.warning(f"Pressure oversampling value {pressure_oversampling} is invalid!")
        # Determine operation mode
        mode = 0b00
        if "normal" in mode_str:
            mode = 0b11
        elif "forced" in mode_str:
            mode = 0b01
        elif "sleep" in mode_str:
            pass
        else:
            logger.warning(f"Measurement mode {mode_str} is invalid!")
        # Concatenate bit sequences
        ctrl_meas_byte = (osrs_t << 5) + (osrs_p << 2) + mode
        bus.write_byte_data(self.i2c_address, Bme280Reader.REG_ADDR_CONTROL_MEASUREMENT, ctrl_meas_byte)

    @staticmethod
    def detect(**kwargs) -> tp.List[Device]:
        """
        Add available devices to list
        """
        devices: tp.List[Device] = []
        addresses = ['0x76', '0x77']
        # Only the first i2c_bus is scanned
        for address in addresses:
            reader = Bme280Reader(address, cache_calibration=False)
            sample = reader.poll()
            if sample is not None:
                devices.append(Device(sample.meter_id, f"{address}@I2C({reader.i2c_bus})", "BME280", sample.channels))
        return devices
