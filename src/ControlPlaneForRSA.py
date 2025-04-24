from abc import abstractmethod, ABC
from typing import Dict
from src.Flow import Flow
from src.LightPath import LightPath

"""
This is the interface that provides several methods for the
RSA Class within the Control Plane.
"""


class ControlPlaneForRSA(ABC):
    """
    This is the interface that provides several methods for the
    RSA Class within the Control Plane.
    """

    @abstractmethod
    def accept_flow(self, id: int, light_paths: LightPath) -> bool:
        pass

    @abstractmethod
    def block_flow(self, id: int) -> bool:
        pass

    @abstractmethod
    def reroute_flow(self, id: int, light_paths: LightPath) -> bool:
        pass

    @abstractmethod
    def get_flow(self, id: int) -> Flow:
        pass

    @abstractmethod
    def get_path(self, flow: Flow) -> LightPath:
        pass

    @abstractmethod
    def get_mapped_flows(self) -> Dict[Flow, LightPath]:
        pass
