import xml.etree.ElementTree as ET
from typing import List, Dict, Tuple, Optional
import math
import networkx as nx
from itertools import islice

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
                        if self.establish_connection(links, list_slot, 0, flow):
                            return
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

    def find_shortest_working_path(self, flow: Flow, spectrum: List[List[bool]]):
        shortest_path = nx.shortest_path(self.graph, source=flow.get_source(), target=flow.get_destination())
        for i in range(len(shortest_path) - 1):
            spectrum = self.image_and(self.pt.get_spectrum(shortest_path[i], shortest_path[i + 1]), spectrum, spectrum)
        cc = ConnectedComponent()
        list_of_regions = cc.list_of_regions(spectrum)

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
                pcycle.set_reversed_slots(demand)
                pcycle.set_slot_list(slot_list)
                return True, spec, pcycle
            else:
                return False, None, None
        else:
            return True, spectrum, pcycle