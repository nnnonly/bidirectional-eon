import xml.etree.ElementTree as ET
from typing import List, Dict, Optional, Tuple
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
from src.PCycle import PCycle
from src.ProtectingLightPath import ProtectingLightPath


class FIPPFlex(RSA):
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

    def find_working_path(self, flow: Flow, demand_in_slots: int):
        """
        Find a working path for new demand
        :param flow: Flow object
        :param: demand_in_slots: Number of slots required for the flow
        :return: working path
        """
        new_graph = self.pt.get_graph().copy()
        for u,v,edge_data in new_graph.edges(data=True):
            edge_spectrum = self.pt.get_spectrum(u, v)
            new_graph[u][v]['spectrum'] = edge_spectrum

        best_path = None
        weight_best_path = float("inf")
        best_slot_index = None

        for i in range(0, self.pt.get_num_slots() - demand_in_slots):
            search_graph = nx.Graph()

            for u, v, edge_data in new_graph.edges(data=True):
                spectrum = edge_data['spectrum']
                if all(spectrum[0][i:i + demand_in_slots]):
                    search_graph.add_edge(u, v, weight_search=1)
                else:
                    search_graph.add_edge(u, v, weight_search=float("inf"))
            try:
                # path = nx.dijkstra_path(search_graph, source=flow.get_source(), target=flow.get_destination(), weight='weight_search')
                length, path = nx.single_source_dijkstra(
                    search_graph,
                    source=flow.get_source(),
                    target=flow.get_destination(),
                    weight='weight_search'
                )
                if weight_best_path > length != float("inf"):
                    best_path = path
                    weight_best_path = length
                    best_slot_index = i
            except nx.NetworkXNoPath:
                continue
        return best_path, best_slot_index, weight_best_path

    def fippflexai(self, flow: Flow, demand_in_slots: int):
        working_path, slot_index, weight_best_path = self.find_working_path(flow, demand_in_slots)
        if working_path and weight_best_path != float("inf"):
            wp_slot_list = self.convert_slot(slot_index, demand_in_slots)
            wp_links = [0 for _ in range(len(working_path) - 1)]
            for j in range(0, len(working_path) - 1, 1):
                wp_links[j] = self.pt.get_link_id(working_path[j], working_path[j + 1])

            # Check if there is a p-cycle that can protect the flow
            for pcycle in self.vt.get_p_cycles():
                if pcycle.p_cycle_contains_flow(flow.get_source(), flow.get_destination()) and pcycle.get_reserved_slots() >= demand_in_slots:
                    if pcycle.can_add_links_disjoint(wp_links):
                        return pcycle, wp_links, wp_slot_list, True

            # Find 2 disjoint paths for p-cycle creation
            path_1_pcycle, path_2_pcycle, slot_index_pcycle = self.create_p_cycle(flow, working_path, slot_index, demand_in_slots)

            # If a p-cycle can be created to protect the flow
            if path_1_pcycle is not None and path_2_pcycle is not None:
                links_1 = [0 for _ in range(len(path_1_pcycle) - 1)]
                for j in range(0, len(path_1_pcycle) - 1, 1):
                    links_1[j] = self.pt.get_link_id(path_1_pcycle[j], path_1_pcycle[j + 1])
                links_2 = [0 for _ in range(len(path_2_pcycle) - 1)]
                for k in range(0, len(path_2_pcycle) - 1, 1):
                    links_2[k] = self.pt.get_link_id(path_2_pcycle[k], path_2_pcycle[k + 1])
                p_cycle_links = links_1 + links_2
                p_cycle_nodes = list(set(path_1_pcycle) | set(path_2_pcycle))
                new_p_cycle = PCycle(cycle_links=p_cycle_links, nodes=p_cycle_nodes, reserved_slots=demand_in_slots,
                                     slot_list=self.convert_slot(slot_index_pcycle, demand_in_slots))
                self.vt.add_p_cycles(new_p_cycle)
                return new_p_cycle, wp_links, wp_slot_list, False
        return None, None, None, False


    def convert_slot(self, index: int, demand: int):
        slot_list: List[Slot] = []
        for s_idx in range(index, index + demand):
            slot_list.append(Slot(0, s_idx))
        return slot_list


    def flow_arrival(self, flow: Flow) -> None:
        print(f"[flow_arrival] Flow {flow.get_id()} arriving")
        demand_in_slots = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())
        p_cycle, wp_links, wp_slot_list, pcycle_reuse = self.fippflexai(flow, demand_in_slots)
        if p_cycle is not None:
            establish, lp_id = self.establish_connection(links=wp_links, slot_list=wp_slot_list, flow=flow, pcycle=p_cycle, reused=pcycle_reuse)
            print("LP", lp_id)
            protected_lp = ProtectingLightPath(id=lp_id, src=flow.get_source(), dst=flow.get_destination(),
                                               links_id=wp_links, fss=demand_in_slots)
            for edge in wp_links:
                self.pt.reserve_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), wp_slot_list)
            if not pcycle_reuse:
                p_cycle_links = p_cycle.get_cycle_links()
                for p_link in p_cycle_links:
                    self.pt.reserve_slots(self.pt.get_src_link(p_link), self.pt.get_dst_link(p_link), p_cycle.get_slot_list())
            p_cycle.add_protected_lightpath(protected_lp)
            return
        else:
            self.cp.block_flow(flow.get_id())
            return

    # def establish_pcycle(self, cycle_links: List[int], nodes: List[int], slot_list: List[Slot], reserved_slots: int) -> PCycle:
    #     new_p_cycle = PCycle(cycle_links=cycle_links, nodes=nodes, reserved_slots=reserved_slots,
    #                          slot_list=slot_list)
    #     self.vt.add_p_cycles(new_p_cycle)
    #     return new_p_cycle

    def establish_connection(self, links: List[int], slot_list: List[Slot], flow: Flow, pcycle: PCycle, reused: bool):
        id = self.vt.create_light_path(flow, links, slot_list, 0, pcycle)
        if id >= 0:
            lps = self.vt.get_light_path(id)
            flow.set_links(links)
            flow.set_slot_list(slot_list)
            self.cp.accept_flow(flow.get_id(), lps, reused)
            return True, id
        else:
            return False, None

    def image_and(self, image1: List[List[bool]], image2: List[List[bool]], res: List[List[bool]]) -> List[List[bool]]:
        for i in range(len(res)):
            for j in range(len(res[0])):
                res[i][j] = image1[i][j] and image2[i][j]
        return res

    def flow_departure(self, flow):
        pass

    def create_p_cycle(self, flow: Flow, working_path: List[int], slot_index: int, demand_in_slots: int):
        """
        Create a p-cycle from primary and backup paths
        :param flow: new demand
        :param working_path: working path get from dijkstra algorithm networkx
        :param slot_index: Index of the slot in which the flow is established
        :param demand_in_slots: Demand in slots
        :return: List of edges of the p-cycle if it is possible to create, None otherwise
        """

        new_graph = self.pt.get_graph().copy()
        for u,v,edge_data in new_graph.edges(data=True):
            edge_spectrum = self.pt.get_spectrum(u, v)
            new_graph[u][v]['spectrum'] = edge_spectrum
        for i in range(0, len(working_path) - 1):
            for slot in (slot_index, slot_index + demand_in_slots):
                new_graph[working_path[i]][working_path[i+1]]['spectrum'][0][slot] = False
        best_path_1 = None
        best_path_2 = None
        weight_best_path = float("inf")
        best_slot_index = None

        for j in range(0, self.pt.get_num_slots() - demand_in_slots):
            search_graph = nx.Graph()

            for u, v, edge_data in new_graph.edges(data=True):
                spectrum = edge_data['spectrum']
                if all(spectrum[0][j:j + demand_in_slots]):
                    search_graph.add_edge(u, v, weight_search=1)
                else:
                    search_graph.add_edge(u, v, weight_search=float("inf"))
            try:
                length_1, path1, length_2, path2 = self.get_two_shortest_disjoint_paths(flow, search_graph)
                if path1 and path2 and length_1 != float("inf") and length_2 != float("inf"):
                    if weight_best_path > length_1 + length_2:
                        best_path_1 = path1
                        best_path_2 = path2
                        weight_best_path = length_1 + length_2
                        best_slot_index = j
            except nx.NetworkXNoPath:
                continue
        return best_path_1, best_path_2, best_slot_index

    def get_two_shortest_disjoint_paths(self, flow: Flow, search_graph: nx.Graph):
        try:
            # Find first shortest path
            length_1, path1 = nx.single_source_dijkstra(
                search_graph,
                source=flow.get_source(),
                target=flow.get_destination(),
                weight='weight_search'
            )

            G_copy = search_graph.copy()
            for i in range(len(path1) - 1):
                u, v = path1[i], path1[i + 1]
                if G_copy.has_edge(u, v):
                    G_copy.remove_edge(u, v)

            length_2, path2 = nx.single_source_dijkstra(
                G_copy,
                source=flow.get_source(),
                target=flow.get_destination(),
                weight='weight_search'
            )

            return length_1, path1, length_2, path2

        except nx.NetworkXNoPath:
            return None, None, None, None

    def remove_edges(self, path):
        G_modified = self.pt.get_graph().copy()
        sp1_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        G_modified.remove_edges_from(sp1_edges)

        inner_nodes = set(path[1:-1])
        remove_edges = [(u, v) for u, v in G_modified.edges if u in inner_nodes or v in inner_nodes]
        G_modified.remove_edges_from(remove_edges)
        return G_modified