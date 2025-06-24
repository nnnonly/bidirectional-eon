import PCycle
import Flow
from typing import Dict, List, Tuple
import math
import PhysicalTopology
from collections import deque
from networkx import nx

class ShortestPath():
    def __init__(self, pt: PhysicalTopology):
        self.pt = pt
        
    def link_pcycle_remove(self, pcycle: PCycle, demand_in_slots: int) -> List[int]:
        """
        Returns a list of links to be removed from the graph based on the pcycle and flow.
        """
        new_graph = self.pt.get_graph().copy()
        filtered = {k: v for k, v in pcycle.get_id_links.items() if len(v) >= 2}
        for k, v in filtered.items():
            total_length = sum(path.get_fss() for path in v)
            if k in pcycle.get_cycle_links() and total_length + demand_in_slots > pcycle.get_reserved_slots():
                new_graph.remove_edge(self.pt.get_src_link(k), self.pt.get_destination(k))
            if k not in pcycle.get_cycle_links() and total_length > pcycle.get_reserved_slots() and len(v) > 1:
                new_graph.remove_edge(self.pt.get_src_link(k), self.pt.get_destination(k))
        return new_graph

    def remove_link_based_on_FS(self, mid: int, demand_in_slots: int, remove_graph_pcycle_links: nx.Graph) -> nx.Graph:
        new_graph = nx.Graph()
        self.get_link_remove(self.pt.get_pcycle(), demand_in_slots)
        for u, v, edge_data in self.graph.edges(data=True):
            edge_spectrum = self.pt.get_spectrum(u, v)
            if all(edge_spectrum[mid:mid - demand_in_slots + 1]):
                new_graph.add_edge(u, v, **edge_data)
        return new_graph
    
    def bfs(self, graph, S, par, dist):
        # Queue to store the nodes in the order they are visited
        q = deque()
        # Mark the distance of the source node as 0
        dist[S] = 0
        # Push the source node to the queue
        q.append(S)

        # Iterate until the queue is not empty
        while q:
            # Pop the node at the front of the queue
            node = q.popleft()

            # Explore all the neighbors of the current node
            for neighbor in graph[node]:
                # Check if the neighboring node is not visited
                if dist[neighbor] == float('inf'):
                    # Mark the current node as the parent of the neighboring node
                    par[neighbor] = node
                    # Mark the distance of the neighboring node as the distance of the current node + 1
                    dist[neighbor] = dist[node] + 1
                    # Insert the neighboring node to the queue
                    q.append(neighbor)


    def print_shortest_distance(self, graph, S, D, V):
        # par[] array stores the parent of nodes
        par = [-1] * V

        # dist[] array stores the distance of nodes from S
        dist = [float('inf')] * V

        # Function call to find the distance of all nodes and their parent nodes
        self.bfs(graph, S, par, dist)

        if dist[D] == float('inf'):
            print("Source and Destination are not connected")
            return

        # List path stores the shortest path
        path = []
        current_node = D
        path.append(D)
        while par[current_node] != -1:
            path.append(par[current_node])
            current_node = par[current_node]

        # Printing path from source to destination
        for i in range(len(path) - 1, -1, -1):
            print(path[i], end=" ")