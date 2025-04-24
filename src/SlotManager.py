import networkx as nx
from typing import List, Tuple


class SlotManager:
    def __init__(self, graph: nx.Graph):
        self.graph = graph

    def get_spectrum(self, src: int, dst: int) -> List[List[bool]]:
        edge_data = self.graph.edges[src, dst]
        cores = edge_data["cores"]
        slots = edge_data["slots"]
        reserved_slots = edge_data["reserved_slots"]

        free_slots = [[True for _ in range(slots)] for _ in range(cores)]
        for core, slot in reserved_slots:
            free_slots[core][slot] = False

        return free_slots

    def are_slots_available(self, src: int, dst: int, slot_list: List[Tuple[int, int]]) -> bool:
        if not self.graph.has_edge(src, dst):
            return False

        reserved_slots = self.graph[src][dst].get("reserved_slots", set())

        for core, slot in slot_list:
            if (core, slot) in reserved_slots:
                return False
        return True

    def reserve_slots(self, src: int, dst: int, slot_list: List[Tuple[int, int]]) -> bool:
        if not self.graph.has_edge(src, dst):
            return False

        reserved_slots = self.graph[src][dst].get("reserved_slots", set())

        for core, slot in slot_list:
            if (core, slot) in reserved_slots:
                return False  # Slot đã bị đặt trước

        reserved_slots.update(slot_list)
        self.graph[src][dst]["reserved_slots"] = reserved_slots
        return True

    def release_slots(self, src: int, dst: int, slot_list: List[Tuple[int, int]]) -> None:
        if not self.graph.has_edge(src, dst):
            return

        reserved_slots = self.graph[src][dst].get("reserved_slots", set())

        for core, slot in slot_list:
            reserved_slots.discard((core, slot))

        self.graph[src][dst]["reserved_slots"] = reserved_slots

    def get_num_free_slots(self, src: int, dst: int) -> int:
        if not self.graph.has_edge(src, dst):
            return 0

        edge_data = self.graph[src][dst]
        total_slots = edge_data.get("slots", 0) * edge_data.get("cores", 1)
        reserved_slots = edge_data.get("reserved_slots", set())

        return total_slots - len(reserved_slots)

    def get_coupled_fibers_in_use(self, src: int, dst: int, core: int, slot: int) -> List[Tuple[int, int]]:
        if not self.graph.has_edge(src, dst):
            return []

        edge_data = self.graph[src][dst]
        cores = edge_data.get("cores", 1)
        reserved_slots = edge_data.get("reserved_slots", set())

        coupled_fibers = []

        if core == 0:
            if (cores - 1, slot) in reserved_slots:
                coupled_fibers.append((cores - 1, slot))
            if (1, slot) in reserved_slots:
                coupled_fibers.append((1, slot))
        elif core == cores - 1:
            if (0, slot) in reserved_slots:
                coupled_fibers.append((0, slot))
            if (cores - 2, slot) in reserved_slots:
                coupled_fibers.append((cores - 2, slot))
        else:
            if (core + 1, slot) in reserved_slots:
                coupled_fibers.append((core + 1, slot))
            if (core - 1, slot) in reserved_slots:
                coupled_fibers.append((core - 1, slot))

        return coupled_fibers
