import xml.etree.ElementTree as ET
import networkx as nx
from typing import List
from src.Slot import Slot
from src.TrafficInfo import TrafficInfo

class PhysicalTopology:
    def __init__(self, xml: ET.Element, verbose: bool):
        assert len(xml) == 2, "Only two elements are allowed in the physical topology"
        self.verbose = verbose
        self.cores = 0
        self.slots = 0
        self.slot_bw = 0.0
        self.graph = nx.Graph()
        self.load_topology(xml)

    def load_topology(self, xml: ET.Element):
        # read information from physical-topology
        try:
            if self.verbose:
                print(xml.attrib["name"])
            self.cores = int(xml.attrib.get("cores"))
            self.slots = int(xml.attrib.get("slots"))
            self.slot_bw = float(xml.attrib.get("slotsBandwidth"))

            for child in xml:
                if child.tag == "nodes":
                    for node in child:
                        assert node.tag == "node" or "id" not in node.attrib.keys(), "Invalid node element"
                        self.graph.add_node(int(node.attrib["id"]))
                elif child.tag == "links":
                    for link in child:
                        assert link.tag == "link", "Invalid link element"
                        assert "id" in link.attrib, "Invalid link element id"
                        assert "source" in link.attrib, "Invalid link element source"
                        assert "destination" in link.attrib, "Invalid link element destination"
                        assert "delay" in link.attrib, "Invalid link element delay"
                        assert "bandwidth" in link.attrib, "Invalid link element bandwidth"
                        assert "weight" in link.attrib, "Invalid link element weight"
                        assert "distance" in link.attrib, "Invalid link element distance"
                        id = int(link.attrib["id"])
                        src = int(link.attrib["source"])
                        dst = int(link.attrib["destination"])
                        delay = float(link.attrib["delay"])
                        bandwidth = float(link.attrib["bandwidth"])
                        weight = float(link.attrib["weight"])
                        distance = int(link.attrib["distance"])
                        self.graph.add_edge(src, dst, id=id, delay=delay, slot=self.slots, weight=weight, reserved_slots=set())
                else:
                    raise ValueError("Unknown element " + child.tag + " in the physical topology file!")

            if self.verbose:
                print(self.graph.number_of_nodes(), " nodes\n", self.graph.number_of_edges(), " links", sep="")
        except Exception as e:
            raise e

    def get_num_nodes(self) -> int:
        return self.graph.number_of_nodes()

    def get_cores(self) -> int:
        return self.cores

    def get_slot_capacity(self) -> float:
        return self.slot_bw

    def get_num_links(self) -> int:
        return self.graph.number_of_edges()

    def get_num_slots(self) -> int:
        return self.slots

    def get_node(self, id: int):
        return id if id in self.graph.nodes else None

    def get_graph(self):
        return self.graph

    def set_graph(self, graph):
        self.graph = graph

    def get_link(self, link_id: int):
        for edge in self.graph.edges(data=True):
            if edge[2].get("id") == link_id:  # Check if the edge has the requested ID
                return edge  # Return the full edge as a tuple (src, dst, data)

    def get_src_link(self, link_index: int):
        return self.get_link(link_index)[0] if self.get_link(link_index) else None

    def get_dst_link(self, link_index: int):
        return self.get_link(link_index)[1] if self.get_link(link_index) else None

    def get_link_dst(self, src: int, dst: int):
        return self.graph[src][dst] if self.graph.has_edge(src, dst) else None

    def has_link(self, node1: int, node2: int) -> bool:
        return self.graph.has_edge(node1, node2)

    def get_node_degree(self, node_id: int) -> int:
        return self.graph.degree[node_id] if node_id in self.graph.nodes else 0

    def get_link_id(self, src: int, dst: int) -> int:
        if self.graph.has_edge(src, dst):
            return self.graph[src][dst]["id"]
        # elif self.graph.has_edge(dst, src):
        #     return self.graph[dst][src]["id"]
        else:
            raise ValueError(f"No edge between {src} and {dst}")

    def get_weighted_graph(self):
        weighted_graph = nx.Graph()

        weighted_graph.add_nodes_from(self.graph.nodes)
        for src, dst, data in self.graph.edges(data=True):
            weight = data["weight"]
            weighted_graph.add_edge(src, dst, weight=weight)
        return weighted_graph

    # def can_groom(self, flow, slot_list: List[int]) -> bool:
    #     src, dst = flow.get_source(), flow.get_destination()
    #     if not self.G.has_edge(src, dst):
    #         return False
    #     edge_data = self.G[src][dst]
    #     return edge_data["slots"] >= len(slot_list)

    def __str__(self):
        topo = ""
        for src in self.graph.nodes():
            for dst in self.graph.neighbors(src):
                if self.graph.has_edge(src, dst):
                    edge_data = self.graph[src][dst]
                    topo += f'{edge_data["id"]}: {src}->{dst} delay: {edge_data["delay"]} slots: {edge_data["slot"]} weight: {edge_data["weight"]}\n'
        return topo

    def print_network_info(self):
        for edge in self.graph.edges(data=True):
            print(edge)

    def get_spectrum(self, src: int, dst: int) -> List[List[bool]]:
        if not self.graph.has_edge(src, dst):
            return []

        edge_data = self.graph.edges[src, dst]
        cores = self.cores
        slots = self.slots
        reserved_slots = edge_data["reserved_slots"]

        free_slots = [[True for _ in range(slots)] for _ in range(cores)]
        for core, slot in reserved_slots:
            free_slots[core][slot] = False
        return free_slots

    def are_slots_available(self, src: int, dst: int, slot_list: List[Slot]) -> bool:
        if not self.graph.has_edge(src, dst):
            return False

        edge_data = self.graph[src][dst]
        reserved_slots = edge_data["reserved_slots"]
        for i in range(0, len(slot_list), 1):
            assert 0 <= slot_list[i].core < self.cores, "Illegal argument exception"
            assert 0 <= slot_list[i].slot < self.slots, "Illegal argument exception"
            if (slot_list[i].core, slot_list[i].slot) in reserved_slots:
                return False
        return True

    def get_num_free_slots(self, src: int, dst: int) -> int:
        """Returns the number of free slots on the edge between `src` and `dst`"""
        assert self.graph.has_edge(src, dst), "Edge does not exist"

        edge_data = self.graph[src][dst]
        total_slots = self.slots * self.cores
        reserved_slots = edge_data["reserved_slots"]

        return total_slots - len(reserved_slots)

    def reserve_slots(self, src: int, dst: int, slot_list: List[Slot]) -> bool:
        try:
            assert self.graph.has_edge(src, dst), "Edge does not exist"
            edge_data = self.graph[src][dst]
            for i in range(0, len(slot_list), 1):
                assert 0 <= slot_list[i].core < self.cores, "Illegal argument exception"
                assert 0 <= slot_list[i].slot < self.slots, "Illegal argument exception"
            
            for s in slot_list:
                # Add the new slot to the reserved_slots set
                edge_data["reserved_slots"].add((s.core, s.slot))
            return True
        except Exception as e:
            raise e
            exit(1)
            return False

    # def reserve_sharing_lots(self, src: int, dst: int, slot_list: List[Slot]) -> bool:
    #     try:
    #         assert self.graph.has_edge(src, dst), "Edge does not exist"
    #         edge_data = self.graph[src][dst]
    #         for i in range(0, len(slot_list), 1):
    #             assert 0 <= slot_list[i].core < self.cores, "Illegal argument exception"
    #             assert 0 <= slot_list[i].slot < self.slots, "Illegal argument exception"
    #
    #         for s in slot_list:
    #             # Add the new slot to the reserved_sharing_slots set
    #             edge_data["sharing_slots"].add((s.core, s.slot))
    #         return True
    #     except Exception as e:
    #         raise e
    #         exit(1)
    #         return False

    def release_slots(self, src: int, dst: int, slot_list: List[Slot]) -> None:
        try:
            assert self.graph.has_edge(src, dst), "Edge does not exist"
            edge_data = self.graph[src][dst]
            for i in range(0, len(slot_list), 1):
                assert 0 <= slot_list[i].core < self.cores, "Illegal argument exception"
                assert 0 <= slot_list[i].slot < self.slots, "Illegal argument exception"
            reserved_slots = edge_data["reserved_slots"]
            for s in slot_list:
                reserved_slots.discard((s.core, s.slot))
            self.graph[src][dst]["reserved_slots"] = reserved_slots
        except Exception as e:
            raise e

    def fragmentation_ratio_1d(self, spectrum) -> float:
        total_free = sum(1 for s in spectrum if not s)
        if total_free == 0:
            return 0.0

        max_contiguous = 0
        current = 0
        for s in spectrum:
            if not s:
                current += 1
                max_contiguous = max(max_contiguous, current)
            else:
                current = 0

        return 1 - (max_contiguous / total_free)

    def fragmentation_per_link(self, src, dst) -> float:
        spectrum_matrix = self.get_spectrum(src, dst)
        frags = [self.fragmentation_ratio_1d(core) for core in spectrum_matrix]
        return sum(frags) / len(frags)

    def get_fragmentation_ratio(self, src, dst, traffic_calls: List[TrafficInfo], slot_capacity: float) -> float:
        free_slots = self.get_spectrum(src, dst)
        fragments_potential = []

        for i in range(0, len(free_slots), 1):
            if free_slots[0][i]:
                i += 1
                fragment_size = 1
                while free_slots[0][i] and i < len(free_slots) - 2:
                    fragment_size += 1
                    i += 1
                counter = 0
                for call in traffic_calls:
                    if call.get_rate() / slot_capacity >= fragment_size:
                        counter += 1
                fragments_potential.append(float(counter / len(traffic_calls)))
        sum = 0
        for potential in fragments_potential:
            sum += potential
        # print("fragments_potential", fragments_potential)
        # print("sum", sum / len(fragments_potential))
        return sum / len(fragments_potential)
    
    def get_cross_talk_per_slot(self, src, dst) -> float:
        if self.cores == 1 or self.get_num_free_slots(src, dst) == self.slots * self.cores:
            return -1.0
        aoc = 0
        free_slots = self.get_spectrum(src, dst)
        for i in range(0, len(free_slots), 1):
            for j in range(0, len(free_slots[i]), 1):
                if not free_slots[i][j]:
                    if i == 0:
                        if not free_slots[self.cores - 1][j]:
                            aoc += 1
                        if not free_slots[1][j]:
                            aoc += 1
                    elif i == self.cores - 1:
                        if not free_slots[0][j]:
                            aoc += 1
                        if not free_slots[self.cores - 2][j]:
                            aoc += 1
                    else:
                        if not free_slots[i + 1][j]:
                            aoc += 1
                        if not free_slots[i - 1][j]:
                            aoc += 1
        used_slots = self.slots * self.cores - self.get_num_free_slots(src, dst)
        return aoc / used_slots
    
    # def average_crosstalk(self) -> float:
    #     average = 0.0
    #     for i in range(0, len(self.noise), 1):
    #         for j in range(0, len(self.noise[i]), 1):
    #             average += self.noise[i][j]
    #     return average / (len(self.noise) * len(self.noise[0]))

    # def get_distance(self, src: int, dst: int) -> int:
    #     if not self.G.has_edge(src, dst):
    #         return float("inf")
    #     return self.G[src][dst].get("distance", 0)
