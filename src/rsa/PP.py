import xml.etree.ElementTree as ET
from typing import List, Dict
import math

# from sqlalchemy.testing.config import ident

from src.rsa.RSA import RSA
from src.util.ConnectedComponent import ConnectedComponent
from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.ControlPlaneForRSA import ControlPlaneForRSA
from src.TrafficGenerator import TrafficGenerator
from src.Flow import Flow
from src.Slot import Slot
import networkx as nx
import itertools



class PP(RSA):
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
        sub_graphs = self.create_subgraphs_from_slots(demand_in_slots)
        min_path, min_index, min_weight = self.find_shortest_paths(subgraphs=sub_graphs, source=flow.get_source(), destination=flow.get_destination())
        if min_weight != float('inf'):
            links = [0 for _ in range(len(min_path) - 1)]
            for j in range(0, len(min_path) - 1, 1):
                links[j] = self.pt.get_link_id(min_path[j], min_path[j + 1])
            list_slot = []
            for idx in range(min_index, min_index + demand_in_slots):
                    list_slot.append(Slot(0, idx))
            if self.establish_connection(links, list_slot, 0, flow):
                return
        else:
            self.cp.block_flow(flow.get_id())
            return

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

    def create_subgraphs_from_slots(self, demand_in_slots: int):
        N = self.pt.get_num_slots()
        graph = self.pt.get_graph()
        subgraphs = []

        for i in range(N - demand_in_slots + 1):
            subgraph = nx.Graph()
            for idx, (u, v, data) in enumerate(graph.edges(data=True)):
                # Phép AND của b phần tử trong current_combination[idx]
                spectrum = self.pt.get_spectrum(u, v)
                current_combination = [spectrum[0][k] for k in range(i, i + demand_in_slots)]
                and_result = all(current_combination)

                weight = 1 if and_result else float('inf')
                subgraph.add_edge(u, v, weight=weight)

            subgraphs.append(subgraph)
        return subgraphs

    def find_shortest_paths(self, subgraphs, source, destination):
        shortest_paths = []
        for subgraph in subgraphs:
            try:
                path = nx.shortest_path(subgraph, source, destination, weight='weight')
                shortest_paths.append((path, nx.path_weight(subgraph, path, weight='weight')))
            except nx.NetworkXNoPath:
                shortest_paths.append((None, float('inf')))
        if shortest_paths:
            min_index, min_weight_path = min(enumerate(shortest_paths), key=lambda x: x[1][1])
            min_path, min_weight = min_weight_path

            return min_path, min_index, min_weight
        else:
            return None, 0, float('inf')
