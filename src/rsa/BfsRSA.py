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
                    # check slots of p-cycle be extended or find other block frequency slot
                    check_protect, slot_list_p_cycle, bool_extended = self.extend_slot(demand_in_slots, p_cycle)
                    current_slot_p_cycle = p_cycle.get_slot_list()
                    # check extend frequency slot
                    if check_protect:
                        # temporary set new slot for pcycle
                        if bool_extended:
                            for edge in p_cycle.get_cycle_links():
                                self.pt.release_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), current_slot_p_cycle)
                                self.pt.reserve_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), slot_list_p_cycle)
                                p_cycle.set_slot_list(slot_list_p_cycle)
                        # find the shortest working path
                        check_path, working_path, links, slot_list, backup_paths = self.find_shortest_working_path(flow, p_cycle)
                        if check_path:
                            # create light path
                            establish, lp_id = self.establish_connection(links, slot_list, flow, p_cycle)
                            # add the light path to the p-cycle
                            protected_lp = ProtectingLightPath(id=lp_id, src=flow.get_source(),
                                                               dst=flow.get_destination(), links_id=links,
                                                               fss=demand_in_slots, backup_paths=backup_paths)

                            p_cycle.add_protected_lightpath(protected_lp)
                            p_cycle.set_reversed_slots(demand_in_slots)
                            return

                        # Khi khong the tim duoc working path thi tra p-cycle ve slot va so luong slot cu
                        else:
                            for edge in p_cycle.get_cycle_links():
                                self.pt.release_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), slot_list_p_cycle)
                                self.pt.reserve_slots(self.pt.get_src_link(edge), self.pt.get_dst_link(edge), current_slot_p_cycle)
                                p_cycle.set_slot_list(current_slot_p_cycle)
                                p_cycle.set_reversed_slots(len(current_slot_p_cycle))
                            self.cp.block_flow(flow.get_id())
                            return
                    else:
                        self.cp.block_flow(flow.get_id())
                        return


        check_available, working_links, working_slot_list, backup_paths, p_cycle_links, p_cycle_nodes, slot_list_p_cycle = self.initialize_fipp(flow)
        if check_available:
            p_cycle = self.establish_pcycle(p_cycle_links, p_cycle_nodes, slot_list_p_cycle, demand_in_slots)
            # create light path
            establish, lp_id = self.establish_connection(working_links, working_slot_list, flow, p_cycle)
            protect_lp = ProtectingLightPath(id=lp_id, src=flow.get_source(),
                                             dst=flow.get_destination(),
                                             links_id=working_links, fss=demand_in_slots,
                                             backup_paths=backup_paths)
            p_cycle.add_protected_lightpath(protect_lp)
            print("ADD PROTECTED LP", self.vt.print_light_paths())
            for j in range(0, len(p_cycle_links), 1):
                self.pt.reserve_slots(self.pt.get_src_link(p_cycle_links[j]),
                                      self.pt.get_dst_link(p_cycle_links[j]),
                                      slot_list_p_cycle)
            return
        self.cp.block_flow(flow.get_id())
        return

    def establish_connection(self, links: List[int], slot_list: List[Slot], flow: Flow, pcycle: PCycle):
        id = self.vt.create_light_path(flow, links, slot_list, 0, pcycle)
        if id >= 0:
            lps = self.vt.get_light_path(id)
            flow.set_links(links)
            flow.set_slot_list(slot_list)
            self.cp.accept_flow(flow.get_id(), lps)
            # print("ADD", self.vt.print_light_paths())
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

    def check_slot_enough_for_wp(self, path: List[int], list_slot_pcycle: List[Slot]):
        spectrum_path = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
        for i in range(len(path) - 1):
            spectrum_path = self.image_and(self.pt.get_spectrum(path[i], path[i + 1]), spectrum_path, spectrum_path)
        for slot in list_slot_pcycle:
            spectrum_path[slot.core][slot.slot] = False
        res_demand = self.find_first_fit_slot_index(spectrum_path[0], len(list_slot_pcycle))
        return res_demand

    def find_first_fit_slot_index(self, slot_list: List[int], demand_in_slots: int) -> int:
        n = len(slot_list)
        for i in range(n - demand_in_slots + 1):
            if all(slot_list[i:i + demand_in_slots]):
                return i
        return -1

    def initialize_fipp(self, flow: Flow):
        demand_in_slots = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())
        upper_bound = self.pt.get_num_slots() - 1
        lower_bound = 0
        shortest_path_obj = ShortestPath(self.pt)
        best_path_1, best_path_2 = [], []
        best_solution = None
        while lower_bound <= upper_bound:
            mid = (upper_bound + lower_bound) // 2
            if mid + demand_in_slots < self.pt.get_num_slots():
                modified_graph = shortest_path_obj.remove_link_based_on_FS(mid, demand_in_slots, self.pt.get_graph())

                # Find the shortest path in the modified graph
                path1, path2 = self.get_two_shortest_disjoint_paths(flow, modified_graph, shortest_path_obj)

                if path1 and path2:
                    best_path_1 = path1
                    best_path_2 = path2
                    best_solution = mid
                    upper_bound = mid - 1
                else:
                    lower_bound = mid + 1
            else:
                lower_bound = mid + 1
        print("Best path: ", best_path_1, best_path_2)
        if len(best_path_1) and len(best_path_2):
            link_1 = [self.pt.get_link_id(best_path_1[i], best_path_1[i + 1]) for i in range(len(best_path_1) - 1)]
            link_2 = [self.pt.get_link_id(best_path_2[i], best_path_2[i + 1]) for i in range(len(best_path_2) - 1)]
            slot_list: List[Slot] = [Slot(0, s) for s in range(best_solution, best_solution + demand_in_slots)]
            p_cycle_links = link_1 + link_2
            check_path_1 = self.check_slot_enough_for_wp(best_path_1, slot_list)
            p_cycle_nodes = list(set(best_path_1) | set(best_path_2))
            with open("/Users/nhungtrinh/Work/bidirectional-eon/out/res.txt", "a") as f:
                f.write(f"TIM DUOC SLOT PCYCLE \n")
            if check_path_1 != -1:
                slot_list_wp: List[Slot] = [Slot(0, s) for s in range(check_path_1, check_path_1 + demand_in_slots)]
                return True, link_1, slot_list_wp, link_2, p_cycle_links, p_cycle_nodes, slot_list
            elif check_path_1 == -1:
                check_path_2 = self.check_slot_enough_for_wp(best_path_2, slot_list)
                if check_path_2 != -1:
                    slot_list_wp: List[Slot] = [(0, s) for s in range(check_path_2, check_path_2 + demand_in_slots)]
                    return True, link_2, slot_list_wp, link_1, p_cycle_links, p_cycle_nodes, slot_list
        with open("/Users/nhungtrinh/Work/bidirectional-eon/out/res.txt", "a") as f:
            f.write(f"NEW-Khong tim thay duong di \n")
            f.write(f"PRESENTED PROTECTED LIGHTPATH {self.vt.print_light_paths()} \n")
            f.write(f"GRAPH {self.pt.get_graph().edges(data=True)} \n")
        print("NEW-Khong tim thay duong di \n")
        return False, None, None, None, None, None, None


    def get_two_shortest_disjoint_paths(self, flow: Flow, modified_graph: nx.Graph, path_obj: ShortestPath):
        try:
            # Find first shortest path uses bfs
            path1 = path_obj.bfs(modified_graph, flow.get_source(), flow.get_destination())

            # Create copy of the graph and remove edges of path1
            G_copy = modified_graph.copy()
            for i in range(len(path1) - 1):
                u, v = path1[i], path1[i + 1]
                if G_copy.has_edge(u, v):
                    G_copy.remove_edge(u, v)

            # Find second shortest path uses bfs
            path2 = path_obj.bfs(G_copy, flow.get_source(), flow.get_destination())

            return path1, path2

        except nx.NetworkXNoPath:
            return path1, None

        except nx.NodeNotFound:
            return None, None


    def find_shortest_working_path(self, flow: Flow, pcycle: PCycle):
        demand_in_slots = math.ceil(flow.get_rate() / self.pt.get_slot_capacity())
        upper_bound = self.pt.get_num_slots() - 1
        lower_bound = 0
        shortest_path_obj = ShortestPath(self.pt)
        best_path = []
        best_solution = None
        graph_remove_pcycle_links = shortest_path_obj.link_pcycle_remove(pcycle=pcycle, demand_in_slots=demand_in_slots)

        while lower_bound <= upper_bound:
            mid = (upper_bound + lower_bound) // 2
            if mid + demand_in_slots < self.pt.get_num_slots():
                modified_graph = shortest_path_obj.remove_link_based_on_FS(mid, demand_in_slots,
                                                                           graph_remove_pcycle_links)
                # Find the shortest path in the modified graph

                shortest_path = shortest_path_obj.bfs(modified_graph, flow.get_source(), flow.get_destination())

                if len(shortest_path):
                    best_path = shortest_path
                    best_solution = mid
                    upper_bound = mid - 1
                else:
                    lower_bound = mid + 1
            else:
                lower_bound = mid + 1
        if best_path:
            links = [0 for _ in range(len(best_path) - 1)]
            slot_list: List[Slot] = [Slot(0, s) for s in range(best_solution, best_solution + demand_in_slots)]
            for i in range(len(best_path) - 1):
                links[i] = self.pt.get_link_id(best_path[i], best_path[i + 1])
            backup_paths = self.get_backup_path(flow, pcycle, links)
            return True, best_path, links, slot_list, backup_paths

        with open("/Users/nhungtrinh/Work/bidirectional-eon/out/res.txt", "a") as f:
            f.write(f"OLD-KHong tim thay duong di \n")
            f.write(f"PRESENTED LIGHTPATH {self.vt.print_light_paths()} \n")
            f.write(f"GRAPH {self.pt.get_graph().edges(data=True)} \n")

        print("OLD-Khong tim thay duong di \n")
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
        with open("/Users/nhungtrinh/Work/bidirectional-eon/out/res.txt", "a") as f:
            f.write(f"LST {lst} \n")
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
        with open("/Users/nhungtrinh/Work/bidirectional-eon/out/res.txt", "a") as f:
            f.write(f"Khong EXTEND duoc P-CYCLE \n")
        print("Khong EXTEND duoc P-CYCLE")
        return lst, None

    def extend_slot(self, demand: int, pcycle: PCycle):
        spectrum = [[True for _ in range(self.pt.get_num_slots())] for _ in range(self.pt.get_cores())]
        bool_extended = False
        for edge in pcycle.get_cycle_links():
            spectrum = self.image_and(self.pt.get_spectrum(self.pt.get_src_link(edge), self.pt.get_dst_link(edge)),
                                      spectrum, spectrum)
        if not pcycle.has_sufficient_slots(demand):
            print("CHECK EXTENDED")
            core, min_slot, max_slot = pcycle.get_core_slot_range()
            spec, idx = self.extend_or_replace_false(lst=spectrum, core_idx=core, start=min_slot, end=max_slot, demand=demand)
            # spec, idx = self.extend_or_replace_false(spectrum, core, min_slot, max_slot, demand)
            if idx is not None:
                core, slots = idx
                slot_list: List[Slot] = [Slot(core, i) for i in slots]
                bool_extended = True
                # pcycle.set_reversed_slots(demand)
                # pcycle.set_slot_list(slot_list)
                return True, slot_list, bool_extended
            else:
                return False, None, bool_extended
        else:
            return True, pcycle.get_slot_list(), bool_extended