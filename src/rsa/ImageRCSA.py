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


class ImageRCSA(RSA):
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
        # print("flow", flow)
        # k_path = nx.shortest_path(self.graph, flow.get_source(), flow.get_destination())
        # print("k_path: ", k_path)
        k_paths = list(islice(nx.shortest_simple_paths(self.graph, flow.get_source(), flow.get_destination(), weight="weight"), 10))
        # k_paths = list(nx.k_shortest(self.graph, flow.get_source(), flow.get_destination(), weight="weight"), 10)
        # print("k_paths: ", k_paths)
        # # k_paths = KShortestPaths().dijkstra_k_shortest_paths(self.graph, flow.get_source(), flow.get_destination(), 5)
        spectrum = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]

        for k in range(0, len(k_paths), 1):
            for i in range(0, len(spectrum), 1):
                for j in range(0, len(spectrum[i]), 1):
                    spectrum[i][j] = True
            for i in range(0, len(k_paths[k]) - 1, 1):
                spectrum = self.image_and(self.pt.get_spectrum(k_paths[k][i], k_paths[k][i + 1]),
                                          spectrum, spectrum)

            cc = ConnectedComponent()
            list_of_regions = cc.list_of_regions(spectrum)

            if list_of_regions == {}:
                continue

            links = [0 for _ in range(len(k_paths[k]) - 1)]
            for j in range(0, len(k_paths[k]) - 1, 1):
                links[j] = self.pt.get_link_id(k_paths[k][j], k_paths[k][j + 1])
            if self.fit_connection(list_of_regions, demand_in_slots, links, flow):
                return
        self.cp.block_flow(flow.get_id())
        return

    def fit_connection(self, list_of_regions: Dict[int, List[Slot]], demand_in_slots: int, links: List[int],
                       flow: Flow) -> bool:
        fitted_slot_list = []
        for key, region in list_of_regions.items():
            if len(region) >= demand_in_slots:
                for i in range(demand_in_slots):
                    fitted_slot_list.append(region[i])
                if self.establish_connection(links, fitted_slot_list, 0, flow):
                    return True
        return False

    def establish_connection(self, links: List[int], slot_list: List[Slot], modulation: int, flow: Flow):
        id = self.vt.create_light_path(links, slot_list, 0)
        if id >= 0:
            lps = self.vt.get_light_path(id)
            flow.set_links(links)
            flow.set_slot_list(slot_list)
            self.cp.accept_flow(flow.get_id(), lps)
            return True
        else:
            return False

    def image_and(self, image1: List[List[bool]], image2: List[List[bool]], res: List[List[bool]]) -> List[List[bool]]:
        for i in range(len(res)):
            for j in range(len(res[0])):
                res[i][j] = image1[i][j] and image2[i][j]
        return res

    def flow_departure(self, flow):
        pass