import xml.etree.ElementTree as ET
from typing import List, Dict, Tuple, Optional, Set
import math
import networkx as nx
from itertools import islice
from collections import defaultdict

from src.PCycle import PCycle
from src.ProtectingLightPath import ProtectingLightPath
from src.rsa.RSA import RSA
from src.util.ConnectedComponent import ConnectedComponent
from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.ControlPlaneForRSA import ControlPlaneForRSA
from src.TrafficGenerator import TrafficGenerator
from src.Flow import Flow
from src.Slot import Slot
from src.util.ShortestPath import ShortestPath
from functools import lru_cache

class BfsRSA(RSA):
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

        if len(self.vt.get_p_cycles()):
            for p_cycle in self.vt.get_p_cycles():
                # Check if the p-cycle contains the flow
                if p_cycle.p_cycle_contains_flow(flow.get_source(), flow.get_destination()):
                    check_protect, spectrum, p_cycle, slot_list_p_cycle = self.extend_slot(demand_in_slots, p_cycle)
                    # check extend frequency slot
                    if check_protect:
                        # find the shortest working path
                        check_path, working_path, links, slot_list, backup_paths = self.find_shortest_working_path(flow, slot_list_p_cycle, p_cycle)
                        if check_path:
                            # create light path
                            establish, lp_id = self.establish_connection(links, slot_list, flow, p_cycle)
                            # add the light path to the p-cycle
                            protected_lp = ProtectingLightPath(id=lp_id, src=flow.get_source(), dst=flow.get_destination(), links_id=links, fss=demand_in_slots, backup_paths=backup_paths)

                            # check slots of p-cycle be extended or find other block frequency slot
                            for edge in working_path:
                                self.pt.release_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), p_cycle.get_slot_list())
                                # self.pt.reserve_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), slot_list)
                            p_cycle.add_protected_lightpath(protected_lp)
                            p_cycle.set_slot_list(slot_list_p_cycle)
                            return

        check_available, working_links, working_slot_list, backup_paths, p_cycle_links, p_cycle_nodes, slot_list_p_cycle = self.initialize_fipp(flow)
        if check_available:
            p_cycle = self.establish_pcycle(p_cycle_links, p_cycle_nodes, slot_list_p_cycle, demand_in_slots)
            # create light path
            establish, lp_id = self.establish_connection(working_links, working_slot_list, flow, p_cycle)
            if establish:
                protect_lp = ProtectingLightPath(id=lp_id, src=flow.get_source(),
                                                 dst=flow.get_destination(),
                                                 links_id=working_links, fss=demand_in_slots,
                                                 backup_paths=backup_paths)
                p_cycle.add_protected_lightpath(protect_lp)
                for j in range(0, len(p_cycle_links), 1):
                    self.pt.reserve_slots(self.pt.get_src_link(p_cycle_links[j]),
                                          self.pt.get_dst_link(p_cycle_links[j]),
                                          slot_list_p_cycle)
                return
        self.cp.block_flow(flow.get_id())
        return

    def establish_connection(self, links: List[int], slot_list: List[Slot], flow: Flow, pcycle: PCycle):
        id = self.vt.create_light_path(links, slot_list, 0, pcycle)
        if id >= 0:
            lps = self.vt.get_light_path(id)
            flow.set_links(links)
            flow.set_slot_list(slot_list)
            self.cp.accept_flow(flow.get_id(), lps)
            return True, id
        else:
            return False, None

    def establish_pcycle(self, cycle_links: List[int], nodes: List[int], slot_list: List[Slot], reserved_slots: int) -> PCycle:
        new_p_cycle = PCycle(cycle_links=cycle_links, nodes=nodes, reserved_slots=reserved_slots,
                             slot_list=slot_list)
        self.vt.add_p_cycles(new_p_cycle)
        return new_p_cycle

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
        if path1 and path2:
            spectrum_path_1 = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
            spectrum_path_2 = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
            for i in range(len(path1) - 1):
                spectrum_path_1 = self.image_and(self.pt.get_spectrum(path1[i], path1[i + 1]), spectrum_path_1, spectrum_path_1)
            for i in range(len(path2) - 1):
                spectrum_path_2 = self.image_and(self.pt.get_spectrum(path2[i], path2[i + 1]), spectrum_path_2, spectrum_path_2)
            spectrum = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
            spectrum = self.image_and(spectrum_path_1, spectrum_path_2, spectrum)
            # check frequency slot for pcycle
            check_path, spec, slot_list_p_cycle = self.calculate_slot_range(spectrum, demand_in_slots)
            if check_path:
                p_cycle_nodes = list(set(path1) | set(path2))
                for slot in slot_list_p_cycle:
                    spectrum_path_1[slot.core][slot.slot] = False
                check_w_1_path, spec_w_1, slot_list_w_1 = self.calculate_slot_range(spectrum_path_1, demand_in_slots)
                if check_w_1_path:
                    list_backup_paths = []
                    links_1 = [0 for _ in range(len(path1) - 1)]
                    for j in range(0, len(path1) - 1, 1):
                        links_1[j] = self.pt.get_link_id(path1[j], path1[j + 1])
                    links_2 = [0 for _ in range(len(path2) - 1)]
                    for k in range(0, len(path2) - 1, 1):
                        links_2[k] = self.pt.get_link_id(path2[k], path2[k + 1])
                    p_cycle_links = links_1 + links_2
                    list_backup_paths.append(links_2)
                    return True, links_1, slot_list_w_1, list_backup_paths, p_cycle_links, p_cycle_nodes, slot_list_p_cycle
                else:
                    for slot in slot_list_p_cycle:
                        spectrum_path_2[slot.core][slot.slot] = False
                    check_w_2_path, spec_w_2, slot_list_w_2 = self.calculate_slot_range(spectrum_path_2,
                                                                                        demand_in_slots)
                    if check_w_2_path:
                        links = [0 for _ in range(len(path2) - 1)]
                        for j in range(0, len(path2) - 1, 1):
                            links[j] = self.pt.get_link_id(path2[j], path2[j + 1])
                        list_backup_paths = []
                        links_1 = [0 for _ in range(len(path1) - 1)]
                        for j in range(0, len(path1) - 1, 1):
                            links_1[j] = self.pt.get_link_id(path1[j], path1[j + 1])
                        list_backup_paths.append(links_1)
                        p_cycle_links = links_1 + links
                        return True, links, slot_list_w_2, list_backup_paths, p_cycle_links, p_cycle_nodes, slot_list_p_cycle
        return False, None, None, None, None, None, None


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


    def find_shortest_working_path(self, flow: Flow, slot_list_p_cycle: List[Slot], pcycle: PCycle):
        demand_in_slots = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())
        upper_bound = self.pt.get_num_slots()
        lower_bound = 0
        mid = 0
        shortest_path_obj = ShortestPath(self.pt)
        best_path = []
        best_solution = None
        while lower_bound <= upper_bound:

            mid = (upper_bound + lower_bound) // 2
            
            graph_remove_pcycle_links = shortest_path_obj.link_pcycle_remove(pcycle= pcycle, demand_in_slots=demand_in_slots)
            modified_graph = shortest_path_obj.remove_link_based_on_FS(mid, demand_in_slots, graph_remove_pcycle_links)
            # Find the shortest path in the modified graph
            shortest_path = nx.shortest_path(modified_graph, source=flow.get_source(), target=flow.get_destination(), weight=None)
            par = [-1] * modified_graph.number_of_nodes()

            # dist[] array stores the distance of nodes from S
            dist = [float('inf')] * modified_graph.number_of_nodes()

        # Function call to find the distance of all nodes and their parent nodes
            shortest_path_obj.bfs(modified_graph, flow.get_source(), par, dist)
            if dist[flow.get_destination()] == float('inf'):
                print("Source and Destination are not connected")
                return

        # List path stores the shortest path
            path = []
            current_node = flow.get_destination()
            path.append(flow.get_destination())
            while par[current_node] != -1:
                path.append(par[current_node])
                current_node = par[current_node]

            if len(path):
                best_path = path
                best_solution = mid
                upper_bound = mid + 1
            else:
                lower_bound = mid + 1
        if best_path and best_solution:
            links = [0 for _ in range(len(best_path) - 1)]
            slot_list: List[Slot] = [(s, 0) for s in range(best_solution - demand_in_slots + 1, best_solution)]
            for i in range(len(best_path) - 1):
                links[i] = self.pt.get_link_id(best_path[i], best_path[i + 1])
            backup_paths = self.get_backup_path(flow, pcycle, links)
            return True, best_path, links, slot_list, backup_paths
        return False, None, None, None, None

    def get_backup_path(self, flow: Flow, pcycle: PCycle, working_path: List[int]):
        graph = nx.Graph()
        filtered_pcycle = [e for e in pcycle.get_cycle_links() if e not in working_path]
        for idx in filtered_pcycle:
            for u, v, data in self.pt.get_graph().edges(data=True):
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



    def select_disjoint_sets(self, groups: List[List[List[int]]]) -> List[List[int]]:
        @lru_cache(maxsize=None)
        def backtrack(index: int, used: frozenset) -> Optional[Tuple[List[int], ...]]:
            if index == len(groups):
                return ()
            for candidate in groups[index]:
                s = set(candidate)
                if s & used:
                    continue
                result = backtrack(index + 1, used | s)
                if result is not None:
                    return (tuple(candidate),) + result
            return None

        result = backtrack(0, frozenset())
        return [list(r) for r in result] if result else []

    def extend_or_replace_false(self,
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
            # spec, idx = self.extend_or_replace_false(spectrum, core, min_slot, max_slot, demand)
            if idx is not None:
                core, slots = idx
                slot_list: List[Slot] = []
                for i in slots:
                    slot_list.append(Slot(core, i))
                for edge in pcycle.get_cycle_links():
                    self.pt.release_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), slot_list)
                pcycle.set_reversed_slots(demand)
                pcycle.set_slot_list(slot_list)
                return True, spec, pcycle, slot_list
            else:
                return False, None, None, None
        else:
            return True, spectrum, pcycle, pcycle.get_slot_list()
