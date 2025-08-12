from src.PCycle import PCycle
from src.PhysicalTopology import PhysicalTopology
from typing import Dict, List, Tuple
import math
from collections import deque
import networkx as nx

class ShortestPath():
    def __init__(self, pt: PhysicalTopology):
        self.pt = pt
        
    def link_pcycle_remove(self, pcycle: PCycle, demand_in_slots: int) -> List[int]:
        """
        Returns a list of links to be removed from the graph based on the pcycle and flow.
        """
        new_graph = self.pt.get_graph().copy()
        filtered = {k: v for k, v in pcycle.get_id_links().items()}
        for k, v in filtered.items():
            total_length = sum(path.get_fss() for path in v)
            if k in pcycle.get_cycle_links() and total_length + demand_in_slots > pcycle.get_reserved_slots():
                new_graph.remove_edge(self.pt.get_src_link(k), self.pt.get_dst_link(k))
            if k not in pcycle.get_cycle_links() and total_length > pcycle.get_reserved_slots() and len(v) > 1:
                new_graph.remove_edge(self.pt.get_src_link(k), self.pt.get_dst_link(k))
        return new_graph

    def remove_link_based_on_FS(self, mid: int, demand_in_slots: int, remove_graph_pcycle_links: nx.Graph) -> nx.Graph:
        new_graph = nx.Graph()
        # self.get_link_remove(self.pt.get_pcycle(), demand_in_slots)
        for u, v, edge_data in remove_graph_pcycle_links.edges(data=True):
            edge_spectrum = self.pt.get_spectrum(u, v)
            if all(edge_spectrum[0][mid:mid + demand_in_slots]):
                new_graph.add_edge(u, v, **edge_data)
        return new_graph
    
    def bfs(self, graph: nx.Graph, source: int, destination: int) -> List[int]:
        if source in graph.nodes() and destination in graph.nodes():
            q = deque()
            dist = {node: float('inf') for node in graph}
            par = {node: -1 for node in graph}

            dist[source] = 0
            q.append(source)

            while q:
                node = q.popleft()
                if node == destination:
                    break  # found destination

                for neighbor in graph[node]:
                    if dist[neighbor] == float('inf'):
                        dist[neighbor] = dist[node] + 1
                        par[neighbor] = node
                        q.append(neighbor)

            # reconstruct path if reachable
            if dist[destination] == float('inf'):
                return []  # no path

            path = []
            current = destination
            while current != -1:
                path.append(current)
                current = par[current]
            path.reverse()
            return path
        return []

    def find_first_fit_slot_index(self, slot_list: List[int], demand_in_slots: int) -> int:
        n = len(slot_list)
        for i in range(n - demand_in_slots + 1):
            if all(slot_list[i:i + demand_in_slots]):
                return i
        return -1
