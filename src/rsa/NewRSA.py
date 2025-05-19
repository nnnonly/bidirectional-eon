import xml.etree.ElementTree as ET
from typing import List, Dict
import math
import networkx as nx
from itertools import islice

from src.rsa.RSA import RSA
from src.util.ConnectedComponent import ConnectedComponent
from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.ControlPlaneForRSA import ControlPlaneForRSA
from src.TrafficGenerator import TrafficGenerator
from src.Flow import Flow
from src.Slot import Slot


class NewRSA(RSA):
    def __init__(self):
        self.pt = None
        self.vt = None
        self.cp = None
        self.graph = None

    def simulation_interface(self, xml: ET.Element, pt: PhysicalTopology, vt: VirtualTopology, cp: ControlPlaneForRSA,
                             traffic: TrafficGenerator):
        self.pt = pt
        self.vt = vt
        self.cp = cp
        self.graph = pt.get_weighted_graph()

    def flow_arrival(self, flow: Flow) -> None:
        demand_in_slots = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())
        path = self.initialize_fipp(flow.get_source(), flow.get_destination())
        print("flow", flow.get_source(), flow.get_destination())
        print("path", path)

    def fit_connection(self, list_of_regions: Dict[int, List[Slot]], demand_in_slots: int, links: List[int],
                       flow: Flow) -> bool:
        return

    def establish_connection(self, links: List[int], slot_list: List[Slot], modulation: int, flow: Flow):
        return

    def image_and(self, image1: List[List[bool]], image2: List[List[bool]], res: List[List[bool]]) -> List[List[bool]]:
        for i in range(len(res)):
            for j in range(len(res[0])):
                res[i][j] = image1[i][j] and image2[i][j]
        return res

    def flow_departure(self, flow):
        pass

    # def create_p_cycle(self):

