import xml.etree.ElementTree as ET
from typing import List, Dict, Tuple, Optional, Set
import math
import networkx as nx
from itertools import islice
from collections import defaultdict

from src.PCycle import PCycle
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

        if len(self.vt.get_p_cycles):
            for p_cycle in self.vt.get_p_cycles():
                if p_cycle.p_cycle_contains_flow(flow.get_source(), flow.get_destination()):
                    check_protect, spectrum, p_cycle = self.extend_slot(demand_in_slots, p_cycle)
                    if check_protect:
                        links = p_cycle.get_cycle_links()
                        list_slot = []
                        for idx in range(len(spectrum)):
                            for j in range(len(spectrum[0])):
                                if spectrum[idx][j] is False:
                                    list_slot.append(Slot(idx, j))

                    else:
                        self.cp.block_flow(flow.get_id())
                        return

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

    def initialize_fipp(self, flow: Flow):
        demand_in_slots = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())
        path1, path2 = self.get_two_shortest_disjoint_paths(flow)
        if path1 is None or path2 is None:
            return None
        else:
            spectrum_path_1 = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
            spectrum_path_2 = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
            for i in range(len(path1) - 1):
                spectrum_path_1 = self.image_and(self.pt.get_spectrum(path1[i], path1[i + 1]), spectrum_path_1, spectrum_path_1)
            for i in range(len(path2) - 1):
                spectrum_path_2 = self.image_and(self.pt.get_spectrum(path1[i], path1[i + 1]), spectrum_path_2, spectrum_path_2)
            spectrum = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
            spectrum = self.image_and(spectrum_path_1, spectrum_path_2, spectrum)
            check_path, spec, slot_list = self.calculate_slot_range(spectrum, demand_in_slots)
            if check_path:
                spectrum_path_1 = self.image_and(spec, spectrum_path_1, spectrum_path_1)
                check_w_1_path, spec_w_1, slot_list_w_1 = self.calculate_slot_range(spectrum_path_1, demand_in_slots)
                if check_w_1_path:
                    links_1 = [0 for _ in range(len(path1) - 1)]
                    for j in range(0, len(path1) - 1, 1):
                        links_1[j] = self.pt.get_link_id(path1[j], path1[j + 1])
                    return True, links_1, slot_list_w_1
                else:
                    spectrum_path_2 = self.image_and(spec, spectrum_path_1, spectrum_path_2)
                    check_w_2_path, spec_w_2, slot_list_w_2 = self.calculate_slot_range(spectrum_path_2,
                                                                                        demand_in_slots)
                    if check_w_2_path:
                        links = [0 for _ in range(len(path2) - 1)]
                        for j in range(0, len(path2) - 1, 1):
                            links[j] = self.pt.get_link_id(path2[j], path2[j + 1])
                        return True, links, slot_list_w_2
                    else:
                        return False, None, None
            return False, None, None


    def get_two_shortest_disjoint_paths(self, flow: Flow):
        try:
            # Find first shortest path
            path1 = nx.shortest_path(self.pt.get_graph(), source=flow.get_source(), target=flow.get_destination(), weight=None)

            # Create copy of the graph and remove edges of path1
            G_copy = self.pt.get_graph().copy()
            for i in range(len(path1) - 1):
                u, v = path1[i], path1[i + 1]
                if G_copy.has_edge(u, v):
                    G_copy.remove_edge(u, v)

            # Find second shortest path
            path2 = nx.shortest_path(G_copy, source=flow.get_source(), target=flow.get_destination(), weight=None)

            return path1, path2

        except nx.NetworkXNoPath:
            return path1, None

        except nx.NodeNotFound:
            return None, None


    def find_shortest_working_path(self, flow: Flow, spectrum: List[List[bool]], pcycle: PCycle):
        demand_in_slots = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())
        shortest_path = nx.shortest_path(self.graph, source=flow.get_source(), target=flow.get_destination())
        for i in range(len(shortest_path) - 1):
            spectrum = self.image_and(self.pt.get_spectrum(shortest_path[i], shortest_path[i + 1]), spectrum, spectrum)
        check_path, spec, slot_list = self.calculate_slot_range(spectrum, demand_in_slots)
        links = [0 for _ in range(len(shortest_path) - 1)]
        for j in range(0, len(shortest_path) - 1, 1):
            links[j] = self.pt.get_link_id(shortest_path[j], shortest_path[j + 1])
        if check_path:
            dict_id_links = pcycle.get_id_links()
            key_set = set(dict_id_links.keys())
            dict_not_disjoint = {k: v for k, v in dict_id_links.items() if k in key_set}
            if dict_not_disjoint:
                sum = demand_in_slots
                list_disjoint_links = {}
                for k, v in dict_not_disjoint.items():
                    for edge in v:
                        sum += edge.get_fss()
                    if sum < pcycle.get_reserved_slots():
                        list_disjoint_links[k] = v
                if len(list_disjoint_links) == len(dict_id_links.keys()):
                    return True, shortest_path, links, slot_list
                else:
                    backup_paths = self.get_backup_path(flow, pcycle, links)
                    for k, v in dict_not_disjoint.items():
                        list_backup_check = []
                        list_backup_check.append(backup_paths)
                        for item in v:
                            list_backup_check.append(item.get_backup_paths())
                        if len(list_backup_check) == 0:
                            return False, None, None, None
            else:
                return False, None, None, None


    def get_backup_path(self, flow: Flow, pcycle: PCycle, working_path: List[int]):
        graph = nx.Graph()
        filtered_pcycle = [e for e in working_path if e not in pcycle.get_cycle_links()]
        for idx in filtered_pcycle:
            for u, v, data in self.pt.get_graph.edges(data=True):
                if data.get("id") == idx:
                    graph.add_edge(u, v, **data)
        paths = list(nx.all_simple_paths(graph, source=flow.get_source(), target=flow.get_destination()))
        list_backup_paths = []
        for path in paths:
            links = [0 for _ in range(len(path) - 1)]
            for j in range(0, len(path) - 1, 1):
                links[j] = self.pt.get_link_id(path[j], path[j + 1])
            list_backup_paths.append(links)
        return list_backup_paths


    def calculate_slot_range(self, spectrum: List[List[bool]], demand: int):
        slot_list: List[Slot] = []
        for c_idx, r in enumerate(spectrum):
            for i in range(len(r) - demand + 1):
                if all(r[j] is True for j in range(i, i + demand)):
                    for j in range(i, i + demand):
                        r[j] = False
                    for s_idx in range(i, i + demand):
                        slot_list.append(Slot(c_idx, s_idx))
                    return True, spectrum, slot_list
        return False, spectrum, None

    def backtrack(self, groups: List[List[List[int]]], index: int, current: List[List[int]], used: Set[int]) -> List[List[int]]:
        if index == len(groups):
            return current

        for candidate in groups[index]:
            s = set(candidate)
            if s & used:
                continue
            result = self.backtrack(self, groups, index + 1, current + [candidate], used | s)
            if result:
                return result
        return []

    def select_disjoint_sets(self, groups: List[List[List[int]]]) -> List[List[int]]:
        return self.backtrack(groups, 0, [], set())

    def extend_or_replace_false(
            lst: List[List[bool]],
            core_idx: int,
            start: int,
            end: int,
            demand: int
    ) -> Tuple[List[List[bool]], Optional[Tuple[int, List[int]]]]:
        lst_cop = [row.copy() for row in lst]

        row = lst[core_idx]
        original_false_indices = list(range(start, end + 1))
        current_len = end - start + 1
        needed = demand - current_len
        left = start - 1
        right = end + 1

        extended_indices = original_false_indices.copy()

        while needed > 0:
            if left >= 0 and row[left] is True:
                row[left] = False
                extended_indices.insert(0, left)
                left -= 1
                needed -= 1
            elif right < len(row) and row[right] is True:
                row[right] = False
                extended_indices.append(right)
                right += 1
                needed -= 1
            else:
                break

        if needed == 0:
            return lst, (core_idx, extended_indices)

        for i in original_false_indices:
            row[i] = True

        for c_idx, r in enumerate(lst):
            for i in range(len(r) - demand + 1):
                if all(r[j] is True for j in range(i, i + demand)):
                    for j in range(i, i + demand):
                        r[j] = False
                    return lst, (c_idx, list(range(i, i + demand)))
        return lst, None

    def extend_slot(self, demand: int, pcycle: PCycle):
        spectrum = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
        for edge in pcycle.get_cycle_links():
            spectrum = self.image_and(self.pt.get_spectrum(self.pt.get_src_link(edge), self.pt.get_dst_link(edge)),
                                      spectrum, spectrum)
        if not pcycle.has_sufficient_slots(demand):
            core, min_slot, max_slot = pcycle.get_core_slot_range()
            spec, idx = self.extend_or_replace_false(lst=spectrum, core_idx=core, start=min_slot, end=max_slot, demand=demand)
            if idx is not None:
                core, slots = idx
                slot_list: List[Slot] = []
                for i in slots:
                    slot_list.append(Slot(core, i))
                for edge in pcycle.get_cycle_links():
                    self.pt.release_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), slot_list)
                pcycle.set_reversed_slots(demand)
                pcycle.set_slot_list(slot_list)
                return True, spec, pcycle
            else:
                return False, None, None
        else:
            return True, spectrum, pcycle