from xml.etree.ElementTree import Element
from abc import ABC, abstractmethod

from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.ControlPlane import ControlPlaneForRSA
from src.TrafficGenerator import TrafficGenerator


class RSA(ABC):

    @abstractmethod
    def simulation_interface(self, xml: Element, pt: PhysicalTopology, vt: VirtualTopology, cp: ControlPlaneForRSA,
                             traffic: TrafficGenerator) -> None:
        pass

    @abstractmethod
    def flow_arrival(self, flow) -> None:
        pass

    @abstractmethod
    def flow_departure(self, flow) -> None:
        pass
