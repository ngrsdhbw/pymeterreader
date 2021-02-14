"""
Reader for BOSCH BME280 sensor.
"""
import logging
import time
import typing as tp
from dataclasses import dataclass
from sys import byteorder
from threading import Lock

from construct import Struct, ConstructError, Int16ub as uShort, Int16sb as sShort, Int8ub as uChar, Int8sb as sChar, \
    BitStruct, Bit, BitsInteger, Padding

try:
    from smbus2 import SMBus
except ImportError:
    pass
from pymeterreader.device_lib.base import BaseReader
from pymeterreader.device_lib.common import Sample, Device

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Bme280CalibrationData:
    dig_T1: int
    dig_T2: int
    dig_T3: int
    dig_P1: int
    dig_P2: int
    dig_P3: int
    dig_P4: int
    dig_P5: int
    dig_P6: int
    dig_P7: int
    dig_P8: int
    dig_P9: int
    dig_H1: int
    dig_H2: int
    dig_H3: int
    dig_H4: int
    dig_H5: int
    dig_H6: int


class Bme280Reader(BaseReader):
    """
    Reads the Bosch BME280 using I2C
    Device Documentation: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
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
                        osrs_t_time = 2.3 * self.temperature_oversampling
                        osrs_p_time = 2.3 * self.pressure_oversampling + 0.575
                        osrs_h_time = 2.3 * self.humidity_oversampling + 0.575
                        measurement_time = 1.25 + osrs_t_time + osrs_p_time + osrs_h_time
                        # Wait for measurement to complete
                        time.sleep(measurement_time / 1000)
                        # Read measuring status from bit 3
                        status = bus.read_byte_data(self.i2c_address,Bme280Reader.REG_ADDR_STATUS)
                        if bool(status >> 3 & 1):
                            logger.error("Measurement is still in progress after maximum measurement time! Aborting...")
                            return None
                    # Read measurement registers
                    measurement = bus.read_i2c_block_data(self.i2c_address, Bme280Reader.REG_ADDR_MEASUREMENT_START, 8)
                    # Parse measurement with struct
                    z = BitStruct("press_raw" / BitsInteger(20),Padding(4),"temp_raw" / BitsInteger(20),Padding(4),"hum_raw" / BitsInteger(16))
                    x = z.parse(bytes(measurement))
                    pass
            # Calculate fine temperature to enable temperature compensation for the other measurements
            fine_temperature = self.calculate_fine_temperature(self.__calibration_data, measurement[3], measurement[4],
                                                               measurement[5])
            temperature = self.calculate_temperature(fine_temperature)
            pressure = self.calculate_pressure(self.__calibration_data, measurement[0], measurement[1], measurement[2],
                                               fine_temperature)
            humidity = self.calculate_pressure(self.__calibration_data, measurement[6], measurement[7],
                                               fine_temperature)
            pass
        except OSError:
            pass
        except ConstructError:
            pass
        return None
        # meter id from hashsum

    @staticmethod
    def calculate_temperature(t_fine: int) -> float:
        return t_fine / 5120.0

    @staticmethod
    def calculate_fine_temperature(calibration_data: Bme280CalibrationData, temp_msb: int, temp_lsb: int,
                                   temp_xlsb_misaligned: int) -> float:
        # Drop bits 0 to 3
        temp_xlsb = temp_xlsb_misaligned >> 4
        # Calculate raw temperature integer
        temperature_raw = (temp_msb << 16) + (temp_lsb << 8) + temp_xlsb
        var1 = (temperature_raw / 16384.0 - calibration_data.dig_T1 / 1024.0) * calibration_data.dig_T2
        var2 = ((temperature_raw / 131072.0 - calibration_data.dig_T1 / 8192.0) * (
                    temperature_raw / 131072.0 - calibration_data.dig_T1 / 8192.0)) * calibration_data.dig_T3
        return var1+var2

    @staticmethod
    def calculate_pressure(calibration_data: Bme280CalibrationData, press_msb: int, press_lsb: int,
                           press_xlsb_misaligned: int, t_fine: int) -> float:
        # Drop bits 0 to 3
        press_xlsb = press_xlsb_misaligned >> 4
        # Calculate raw pressure integer
        z = BitStruct("x" / BitsInteger(20))
        z.parse([press_msb,press_lsb,press_xlsb])
        pressure_raw = (press_msb << 16) + (press_lsb << 8) + press_xlsb
        pass

    @staticmethod
    def calculate_humidity(calibration_data: Bme280CalibrationData, hum_msb: int, hum_lsb: int, t_fine: int) -> float:
        # Calculate raw humidity integer
        humidity_raw = (hum_msb << 8) + hum_lsb
        pass

    @staticmethod
    def parse_calibration_bytes(calibration_0to25: bytes, calibration_26to41: bytes) -> Bme280CalibrationData:
        # Create Comparison structs
        calibration_0to25_struct = Struct("dig_T1" / uShort,
                                          "dig_T2" / sShort,
                                          "dig_T3" / sShort,
                                          "dig_P1" / uShort,
                                          "dig_P2" / sShort,
                                          "dig_P3" / sShort,
                                          "dig_P4" / sShort,
                                          "dig_P5" / sShort,
                                          "dig_P6" / sShort,
                                          "dig_P7" / sShort,
                                          "dig_P8" / sShort,
                                          "dig_P9" / sShort,
                                          "dig_H1" / uChar)
        calibration_26to41_stuct = Struct("dig_H2" / sShort,
                                          "dig_H3" / uChar,
                                          "z" / BitStruct("dig_H4" / BitsInteger(12),
                                                    "dig_H5" / BitsInteger(12)),
                                          "dig_H6" / sChar)
        #bit endinanness wrong?
        # Parse bytes to container
        calibration_0to25_container = calibration_0to25_struct.parse(calibration_0to25)
        calibration_26to41_container = calibration_26to41_stuct.parse(calibration_26to41)
        # Unpack containers into dataclass
        calibration_dict = {**calibration_0to25_container, **calibration_26to41_container}
        # Remove construct container _io object
        calibration_dict.pop("_io", None)
        return Bme280CalibrationData(**calibration_dict)

    def __read_calibration_data(self, bus: SMBus) -> Bme280CalibrationData:
        """
        This method reads the calibration data from the sensor
        :param bus: an open i2c bus that is already protected by a Lock
        """
        # Read calibration registers from 0xF0 to 0xA1
        calibration_0to25 = bus.read_i2c_block_data(self.i2c_address, 0x88, 26)
        # Remove unused register 0xA0
        calibration_0to25.pop(24)
        # Read calibration registers from 0xE1 to 0xE7
        calibration_26to41 = bus.read_i2c_block_data(self.i2c_address, 0xE1, 7)
        # Parse bytes in separate function
        return self.parse_calibration_bytes(bytes(calibration_0to25), bytes(calibration_26to41))

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
        config_byte_struct = BitStruct("t_sb" / BitsInteger(3), "filter" / BitsInteger(3), "spi3wire_enable" / BitsInteger(2))
        config_byte = config_byte_struct.build({"t_sb": t_sb, "filter": filter, "spi3wire_enable": spi3wire_enable})
        z = int.from_bytes(config_byte, byteorder )
        bus.write_byte_data(self.i2c_address, Bme280Reader.REG_ADDR_CONFIG, z)

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
