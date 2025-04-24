from src.graphs import Graph
import xml.etree.ElementTree as ET


class OutputManager:
    def __init__(self, xml: ET.Element):
        self.graphs = []
        for graph in xml:
            self.graphs.append(Graph(graph.attrib["name"], graph.attrib["dots-file"], 2))

    def write_all_to_files(self) -> None:
        for graph in self.graphs:
            graph.write_dots_to_file()

    def add_dot_to_graph(self, graph_name: str, value1: float, value2: float) -> None:
        try:
            self.get_graph(graph_name).get_data_set().add_dot(value1, value2)
        except Exception as e:
            pass

    def get_graph(self, graph_name: str) -> Graph:
        for g in self.graphs:
            if g.get_name() == graph_name:
                return g
        raise KeyError("get_graph: Graph not found")