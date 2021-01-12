"""
Detect and present meters.
"""
import typing as tp
from pymeterreader.device_lib import SmlReader, PlainReader, Bme280Reader, Device
from pymeterreader.device_lib.common import ChannelValue


def detect() -> tp.List[Device]:
    """
    Calls all detectors and returns a list of available devices.
    """
    devices = []
    channels = [ChannelValue(channel_name='129-129:199.130.3*255', value='EMH', unit=None),
                ChannelValue(channel_name='1-0:1.8.0*255', value=27390133.9, unit='Wh'),
                ChannelValue(channel_name='1-0:2.8.0*255', value=18882624.0, unit='Wh'),
                ChannelValue(channel_name='1-0:1.8.1*255', value=27390133.9, unit='Wh'),
                ChannelValue(channel_name='1-0:2.8.1*255', value=18882624.0, unit='Wh'),
                ChannelValue(channel_name='1-0:1.8.2*255', value=0, unit='Wh'),
                ChannelValue(channel_name='1-0:2.8.2*255', value=0, unit='Wh'),
                ChannelValue(channel_name='1-0:16.7.0*255', value=-961.9, unit='W'),
                ChannelValue(channel_name='129-129:199.130.5*255',
                             value='58af289a611352984cf85295237ef26670cb3d367e218b48d952789fc4a5888604012b323490ced3d96d341c9e9ccf77',
                             unit=None)]
    devices = [Device('1 EMH 00 4921570', 'socket://10.10.230.20:4000', 'sml', channels)]
    return devices
