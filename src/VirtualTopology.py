import networkx as nx
import xml.etree.ElementTree as ET
from typing import List
from src.LightPath import LightPath
from src.PhysicalTopology import PhysicalTopology
from src.Slot import Slot
from src.Tracer import Tracer
from src.PCycle import PCycle


class VirtualTopology:
    def __init__(self, xml: ET.Element, pt: PhysicalTopology, verbose: bool = False):
        self.verbose = verbose
        self.next_lightpath_id = 0
        self.pt = pt
        self.tr = Tracer.get_tracer_object()

        self.g_lightpath = nx.MultiGraph()

        self.p_cycles: List[PCycle] = []

        num_nodes = self.pt.get_num_nodes()
        for i in range(num_nodes):
            self.g_lightpath.add_node(i)

    def create_light_path(self, links: List[int], slot_list: List[Slot], modulation_level: int, p_cycle: PCycle = None) -> float:
        if len(links) < 1:
            raise ValueError("Invalid links")

        if not self.can_create_light_path(links, slot_list):
            return -1

        self.create_light_path_in_pt(links, slot_list)

        src = self.pt.get_src_link(links[0])
        dst = self.pt.get_dst_link(links[-1])
        id = self.next_lightpath_id

        lp = LightPath(id, src, dst, links, slot_list, modulation_level, p_cycle)
        self.g_lightpath.add_edge(src, dst, lightpath=lp)
        self.tr.create_lightpath(lp)
        self.next_lightpath_id += 1
        return id

    def get_light_path(self, id: float) -> LightPath:
        for src, dst, data in self.g_lightpath.edges(data=True):
            if "lightpath" in data and data["lightpath"].get_id() == id:
                return data["lightpath"]
        return None

    def can_create_light_path(self, links: List[int], slot_list: List[Slot]) -> bool:
        try:
            for link in links:
                if not self.pt.are_slots_available(self.pt.get_src_link(link), self.pt.get_dst_link(link), slot_list):
                    return False
            return True
        except ValueError:
            raise "Illegal argument for areSlotsAvailable"

    def create_light_path_in_pt(self, links: List[int], slot_list: List[Slot]) -> None:
        """Update reverse slots in PhysicalTopology"""
        for link in links:
            self.pt.reserve_slots(self.pt.get_src_link(link), self.pt.get_dst_link(link), slot_list)


    def remove_light_path(self, id: float) -> bool:
        """Remove a light path by ID from the virtual topology."""
        if id < 0:
            raise ValueError("Invalid ID")
        else:
            # Find the light path in the graph
            for src, dst, data in list(self.g_lightpath.edges(data=True)):  # Iterate over edges
                if "lightpath" in data and data["lightpath"].get_id() == id:
                    lp = data["lightpath"]
                    self.remove_light_path_from_pt(lp.get_links(), lp.get_slot_list())  # Release slots
                    self.g_lightpath.remove_edge(src, dst)  # Remove the edge from the graph
                    # self.list_nodes.remove((src, dst))
                    # self.light_path.pop(id, None)  # Remove from dictionary if it exists
                    self.tr.remove_lightpath(lp)
                    return True  # Successfully removed

        return False  # Light path not found

    def remove_light_path_from_pt(self, links: List[int], slot_list: List[Slot]) -> None:
        """Release the reserved slots in the physical topology."""
        for link in links:  # Get source and destination of the link
            src = self.pt.get_src_link(link)
            dst = self.pt.get_dst_link(link)
            self.pt.release_slots(src, dst, slot_list)

    def get_p_cycles(self) -> List[PCycle]:
        return self.p_cycles
    
    def add_p_cycles(self, cycle: PCycle):
        self.p_cycles.append(cycle)

    def remove_lp_p_cycle(self, lp: LightPath):
        p_cycle_protect = lp.get_p_cycle()
        if len(p_cycle_protect.get_protected_lightpaths()) == 1:
            for i in range(0, len(p_cycle_protect.get_cycle_links()), 1):
                self.pt.release_slots(self.pt.get_src_link(p_cycle_protect.get_cycle_links()[i]),
                                      self.pt.get_dst_link(p_cycle_protect.get_cycle_links()[i]),
                                      p_cycle_protect.get_slot_list())
            self.p_cycles.remove(p_cycle_protect)
        else:
            # Remove the light path from the P-cycle's protected light paths
            p_cycle_protect.remove_protected_lightpath(lp)

    def __str__(self):
        topo = ""
        for src in self.g_lightpath.nodes():
            for dst in self.g_lightpath.neighbors(src):
                if self.g_lightpath.has_edge(src, dst):
                    edge_data = self.g_lightpath[src][dst]
                    topo += f'{edge_data["id"]}: {src}->{dst} delay: {edge_data["delay"]} slots: {edge_data["slot"]} weight: {edge_data["weight"]}\n'
        return topo