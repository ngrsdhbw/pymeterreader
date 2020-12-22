"""
Base Reader (ABC)
Created 2020.10.12 by Oliver Schwaneberg
"""
import typing as tp
from abc import ABC, abstractmethod
from logging import warning
from pymeterreader.device_lib.common import Sample, Device


class BaseReader(ABC):
    """
    Implementation Base for a Meter Protocol
    """
    PROTOCOL = "ABSTRACT"
    BOUND_INTERFACES = set()

    @abstractmethod
    def __init__(self, meter_id: tp.Union[str, int], **kwargs):
        """
        Initialize Meter Reader object
        :param meter_id: meter identification string (e.g. '1 EMH00 12345678')
        :kwargs: implementation specific parameters
        """
        self.meter_id = meter_id
        if kwargs:
            warning(f'Unknown parameter{"s" if len(kwargs) > 1 else ""}:'
                    f' {", ".join(kwargs.keys())}')

    @staticmethod
    @abstractmethod
    def detect(**kwargs) -> tp.List[Device]:
        """
        Detect available devices on all possible interfaces
        :kwargs: parameters for the classes that implement detection
        """
        raise NotImplementedError("This is just an abstract class.")

    @abstractmethod
    def poll(self) -> tp.Optional[Sample]:
        """
        Poll the reader and retrieve a new sample
        :return: Sample, if successful else None
        """
        raise NotImplementedError("This is just an abstract class.")
