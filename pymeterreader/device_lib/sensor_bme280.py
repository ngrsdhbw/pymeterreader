"""
Reader for a BOSCH BME280 sensor
"""
import logging
import time
import typing as tp
from dataclasses import dataclass
from hashlib import sha256
from sys import byteorder as endianness
from threading import Lock

from construct import Struct, ConstructError, Int16un as uShort, Int16sn as sShort, Int8un as uChar, Int8sn as sChar, \
    BitStruct, BitsInteger, Padding, Bit

try:
    from smbus2 import SMBus
except ImportError:
    pass
from pymeterreader.device_lib.base import BaseReader
from pymeterreader.device_lib.common import Sample, Device, ChannelValue

logger = logging.getLogger(__name__)


@dataclass(eq=True,frozen=True)
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
    REG_ADDR_CALIBRATION1_START = 0x88
    REG_ADDR_CALIBRATION2_START = 0xE1
    STRUCT_STATUS = BitStruct(Padding(4), "measuring" / Bit, Padding(2), "im_update" / Bit)
    STRUCT_MEASUREMENT = BitStruct("press_raw" / BitsInteger(20),
                                   Padding(4),
                                   "temp_raw" / BitsInteger(20),
                                   Padding(4),
                                   "hum_raw" / BitsInteger(16))
    STRUCT_CALIBRATION1 = Struct("dig_T1" / uShort,
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
    STRUCT_CALIBRATION2 = Struct("dig_H2" / sShort,
                                 "dig_H3" / uChar,
                                 "misaligned_bitsegment" / BitStruct("byte_0xE4" / BitsInteger(8),
                                                                     "bits_0xE5_left" / BitsInteger(4),
                                                                     "bits_0xE5_right" / BitsInteger(4),
                                                                     "byte_OxE6" / BitsInteger(8)),
                                 "dig_H6" / sChar)

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
        Public method for polling a Sample from the meter. Enforces that the meter_id matches.
        :return: Sample, if successful
        """
        sample = self.__fetch_sample()
        if sample is not None:
            if self.meter_id_matches(sample):
                return sample
        return None

    def __fetch_sample(self) -> tp.Optional[Sample]:
        """
        Try to retrieve a Sample from any connected meter with the current configuration
        :return: Sample, if successful
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
                        # Read measuring status
                        status = bus.read_byte_data(self.i2c_address, Bme280Reader.REG_ADDR_STATUS)
                        if Bme280Reader.STRUCT_STATUS.parse(status.to_bytes(1, endianness)) == 1:
                            logger.error("Measurement is still in progress after maximum measurement time! Aborting...")
                            return None
                    # Read measurement registers
                    measurement = bus.read_i2c_block_data(self.i2c_address, Bme280Reader.REG_ADDR_MEASUREMENT_START, 8)
            # Parse measurement
            measurement_container = Bme280Reader.STRUCT_MEASUREMENT.parse(bytes(measurement))
            # Calculate fine temperature to enable temperature compensation for the other measurements
            fine_temperature = self.calculate_fine_temperature(self.__calibration_data, measurement_container.temp_raw)
            # Calculate measurement results
            temperature = self.calculate_temperature(fine_temperature)
            pressure = self.calculate_pressure(self.__calibration_data, measurement_container.press_raw,
                                               fine_temperature)
            humidity = self.calculate_pressure(self.__calibration_data, measurement_container.hum_raw, fine_temperature)
            # Determine meter_id
            meter_id = self.derive_meter_id(self.__calibration_data, chip_id)
            # Return Sample
            return Sample(meter_id=meter_id, channels=[ChannelValue('TEMPERATURE', temperature, '°C'),
                                                       ChannelValue('PRESSURE', pressure, 'Pa'),
                                                       ChannelValue('HUMIDITY', humidity, '%')])
        except OSError:
            pass
        except ConstructError as c:
            pass
        return None

    @staticmethod
    def calculate_temperature(t_fine: float) -> float:
        return t_fine / 5120.0

    @staticmethod
    def calculate_fine_temperature(calibration_data: Bme280CalibrationData, temp_raw: int) -> float:
        var1 = (temp_raw / 16384.0 - calibration_data.dig_T1 / 1024.0) * calibration_data.dig_T2
        var2 = (
                       (temp_raw / 131072.0 - calibration_data.dig_T1 / 8192.0)
                       * (temp_raw / 131072.0 - calibration_data.dig_T1 / 8192.0)
               ) * calibration_data.dig_T3
        return var1 + var2

    @staticmethod
    def calculate_pressure(calibration_data: Bme280CalibrationData, press_raw: int, t_fine: float) -> float:
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (calibration_data.dig_P6) / 32768.0
        var2 = var2 + var1 * (calibration_data.dig_P5) * 2.0
        var2 = (var2 / 4.0) + (calibration_data.dig_P4 * 65536.0)
        var1 = (calibration_data.dig_P3 * var1 * var1 / 524288.0 + calibration_data.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * calibration_data.dig_P1
        if var1 == 0.0:
            return 0.0
        p = 1048576.0 - press_raw
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = calibration_data.dig_P9 * p * p / 2147483648.0
        var2 = p * calibration_data.dig_P8 / 32768.0
        p = p + (var1 + var2 + calibration_data.dig_P7) / 16.0
        return p

    @staticmethod
    def calculate_humidity(calibration_data: Bme280CalibrationData, hum_raw: int, t_fine: float) -> float:
        var_H = t_fine - 76800.0
        var_H = (hum_raw - (calibration_data.dig_H4 * 64.0 + calibration_data.dig_H5 / 16384.0 * var_H)) \
                * (
                        calibration_data.dig_H2 / 65536.0
                        * (
                                1.0 + calibration_data.dig_H6 / 67108864.0 * var_H
                                * (1.0 + calibration_data.dig_H3 / 67108864.0 * var_H)
                        )
                )
        var_H = var_H * (1.0 - calibration_data.dig_H1 * var_H / 524288.0)
        if var_H > 100.0:
            var_H = 100.0
        elif var_H < 0.0:
            var_H = 0.0
        return var_H

    @staticmethod
    def parse_calibration_bytes(calibration_segment1: bytes, calibration_segment2: bytes) -> Bme280CalibrationData:
        # Parse bytes to container
        calibration_segment1_container = Bme280Reader.STRUCT_CALIBRATION1.parse(calibration_segment1)
        calibration_segment2_container = Bme280Reader.STRUCT_CALIBRATION2.parse(calibration_segment2)
        # Bit order from the sensor does not allow for parsing dig_H4 and dig_h5 inside of a BitStruct with BitsInteger
        # Required order is 0xE4,0xE5[right 4 Bits],0xE6,0xE5[left 4 Bits]
        reorder_struct = BitStruct("byte_0xE4" / BitsInteger(8), "bits_0xE5_right" / BitsInteger(4),
                                   "byte_OxE6" / BitsInteger(8), "bits_0xE5_left" / BitsInteger(4))
        reorder_bitsegments = calibration_segment2_container.pop("misaligned_bitsegment")
        reorder_bitsegments.pop("_io", None)
        # Recreate bytes with correct order
        reordered_bytes = reorder_struct.build(reorder_bitsegments)
        # Parse the reordered bytes with a Bitstruct
        humidity_struct = BitStruct("dig_H4" / BitsInteger(12), "dig_H5" / BitsInteger(12))
        # Parse bytes to container
        humidity_container = humidity_struct.parse(reordered_bytes)
        # Unpack containers into dataclass
        calibration_dict = {**calibration_segment1_container, **calibration_segment2_container, **humidity_container}
        # Remove construct container _io object
        calibration_dict.pop("_io", None)
        return Bme280CalibrationData(**calibration_dict)

    def __read_calibration_data(self, bus: SMBus) -> Bme280CalibrationData:
        """
        This method reads the calibration data from the sensor
        :param bus: an open i2c bus that is already protected by a Lock
        """
        # Read calibration registers from 0x88 to 0xA1
        calibration_segment1 = bus.read_i2c_block_data(self.i2c_address, Bme280Reader.REG_ADDR_CALIBRATION1_START, 26)
        # Remove unused register 0xA0
        calibration_segment1.pop(24)
        # Read calibration registers from 0xE1 to 0xE7
        calibration_segment2 = bus.read_i2c_block_data(self.i2c_address, Bme280Reader.REG_ADDR_CALIBRATION2_START, 7)
        # Parse bytes in separate function
        return self.parse_calibration_bytes(bytes(calibration_segment1), bytes(calibration_segment2))

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
        config_int = int.from_bytes(config_byte, endianness)
        bus.write_byte_data(self.i2c_address, Bme280Reader.REG_ADDR_CONFIG, config_int)

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
        ctrl_meas_struct = BitStruct("osrs_t" / BitsInteger(3), "osrs_p" / BitsInteger(3), "mode" / BitsInteger(2))
        ctrl_meas_byte = ctrl_meas_struct.build({"osrs_t": osrs_t, "osrs_p": osrs_p, "mode": mode})
        ctrl_meas_int = int.from_bytes(ctrl_meas_byte, endianness)
        bus.write_byte_data(self.i2c_address, Bme280Reader.REG_ADDR_CONTROL_MEASUREMENT, ctrl_meas_int)

    @staticmethod
    def derive_meter_id(calibration_data: Bme280CalibrationData, chip_id: int = 0) -> str:
        """
        This method calculates a unique identifier for a sensor by hashing it´s calibration data
        :param calibration_data: Calibration Data from the sensor which should be identified by the meter_id
        :param chip_id: the optional chip id identifies the series of the sensor
        :return: str uniquely identifying the sensor
        """
        calibration_hash = sha256(str(calibration_data).encode())
        # Prefixing the sensor type guards against calibration data collisions between different sensor types
        if chip_id == 0x60:
            sensor_type = "BME280-"
        elif chip_id in [0x56, 0x57]:
            sensor_type = "BMP280(Sample)-"
        elif chip_id == 0x58:
            sensor_type = "BMP280-"
        else:
            # meter_id matching will still succeed when the prefix is not explicitly specified
            sensor_type = ""
        return f"{sensor_type}{calibration_hash.hexdigest()}"

    @staticmethod
    def detect(**kwargs) -> tp.List[Device]:
        """
        Add available devices to list
        """
        devices: tp.List[Device] = []
        addresses = ["0x76", "0x77"]
        # Only the first i2c_bus is scanned
        for address in addresses:
            reader = Bme280Reader(address, cache_calibration=False)
            sample = reader.poll()
            if sample is not None:
                devices.append(Device(sample.meter_id, f"{address}@I2C({reader.i2c_bus})", "BME280", sample.channels))
        return devices