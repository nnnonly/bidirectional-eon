import threading

from src.OutputManager import OutputManager
from src.PhysicalTopology import PhysicalTopology
from src.TrafficGenerator import TrafficGenerator
from src.Flow import Flow
from src.LightPath import LightPath
# from .ModulationsMuticore import ModulationsMuticore
from src.Modulations import Modulations
from src.Event import Event
from src.FlowArrivalEvent import FlowArrivalEvent
from src.FlowDepartureEvent import FlowDepartureEvent


class MyStatistics:
    singleton_object = None

    def __init__(self):
        self.verbose = False
        self.plotter = OutputManager
        self.pt = PhysicalTopology
        self.traffic = TrafficGenerator

        self.min_number_arrivals = 0
        self.number_arrivals = 0
        self.arrivals = 0
        self.departures = 0
        self.accepted = 0
        self.blocked = 0

        # Bandwidth
        self.required_bandwidth = 0
        self.blocked_bandwidth = 0

        # Network information
        self.num_nodes = 0
        self.load = 0.0
        self.total_power_consumed = 0.0
        self.sim_time = 0.0
        self.data_transmitted = 0.0

        # Modulation
        self.avg_bits_per_symbol = 0.0
        self.avg_bits_per_symbol_count = 0

        self.arrivals_pairs = [[int]]
        self.blocked_pairs = [[int]]
        self.required_bandwidth_pairs = [[int]]
        self.blocked_bandwidth_pairs = [[int]]

        self.arrivals_diff = [int]
        self.blocked_diff = [int]
        self.required_bandwidth_diff = [int]
        self.blocked_bandwidth_diff = [int]
        self.arrivals_pairs_diff = [[[int]]]
        self.blocked_pairs_diff = [[[int]]]
        self.required_bandwidth_pairs_diff = [[[int]]]
        self.blocked_bandwidth_pairs_diff = [[[int]]]
        self.number_of_used_transponders = [[int]]

    @staticmethod
    def get_my_statistics():
        print("singleton_object: ", MyStatistics.singleton_object)
        if MyStatistics.singleton_object is None:
            MyStatistics.singleton_object = MyStatistics()
        return MyStatistics.singleton_object

    def __copy__(self):
        raise Exception("CloneNotSupportedException")

    def statistics_setup(self, plotter: OutputManager, pt: PhysicalTopology, traffic: TrafficGenerator, num_nodes: int,
                         num_classes: int, min_number_arrivals: int, load: float, verbose: bool) -> None:
        self.verbose = verbose
        self.plotter = plotter
        self.pt = pt
        self.traffic = traffic

        self.num_nodes = num_nodes
        self.load = load
        self.arrivals_pairs = [[0 for _ in range(num_nodes)] for _ in range(num_nodes)]
        self.blocked_pairs = [[0 for _ in range(num_nodes)] for _ in range(num_nodes)]
        self.required_bandwidth_pairs = [[0 for _ in range(num_nodes)] for _ in range(num_nodes)]
        self.blocked_bandwidth_pairs = [[0 for _ in range(num_nodes)] for _ in range(num_nodes)]

        self.avg_bits_per_symbol = 0.0
        self.avg_bits_per_symbol_count = 0
        self.min_number_arrivals = min_number_arrivals
        self.number_of_used_transponders = [[0 for _ in range(num_nodes)] for _ in range(num_nodes)]

        self.arrivals_diff = [0 for _ in range(num_classes)]
        self.blocked_diff = [0 for _ in range(num_classes)]
        self.required_bandwidth_diff = [0 for _ in range(num_classes)]
        self.blocked_bandwidth_diff = [0 for _ in range(num_classes)]
        for i in range(0, num_classes, 1):
            self.arrivals_diff[i] = 0
            self.blocked_diff[i] = 0
            self.required_bandwidth_diff[i] = 0
            self.blocked_bandwidth_diff[i] = 0

        self.arrivals_pairs_diff = [[[0 for _ in range(num_nodes)] for _ in range(num_nodes)] for _ in
                                    range(num_classes)]
        self.blocked_pairs_diff = [[[0 for _ in range(num_nodes)] for _ in range(num_nodes)] for _ in
                                   range(num_classes)]
        self.required_bandwidth_pairs_diff = [[[0 for _ in range(num_nodes)] for _ in range(num_nodes)] for _ in
                                              range(num_classes)]
        self.blocked_bandwidth_pairs_diff = [[[0 for _ in range(num_nodes)] for _ in range(num_nodes)] for _ in
                                             range(num_classes)]
        # self.total_power_consumed = 0.0
        # self.sim_time = 0.0
        # self.data_transmitted = 0.0

    def calculate_last_statistics(self) -> None:
        self.avg_bits_per_symbol = self.avg_bits_per_symbol / self.avg_bits_per_symbol_count
        self.plotter.add_dot_to_graph("avgbps", self.load, self.avg_bits_per_symbol)
        self.plotter.add_dot_to_graph("mbbr", self.load, self.blocked_bandwidth * 1.0 / self.required_bandwidth)
        self.plotter.add_dot_to_graph("bp", self.load, (self.blocked * 1.0 / self.arrivals) * 100)
        count = 0
        bbr = 0.0
        jfi = 0.0
        sum1 = 0.0
        sum2 = 0.0

        if self.blocked == 0:
            bbr = 0.0
        else:
            bbr = (self.blocked_bandwidth * 1.0 / self.required_bandwidth) * 100

        for i in range(0, self.num_nodes, 1):
            for j in range(0, self.num_nodes, 1):
                if i != j:
                    if self.blocked_pairs[i][j] == 0:
                        bbr = 0.0
                    else:
                        bbr = (self.blocked_bandwidth_pairs[i][j] * 1.0 / self.required_bandwidth_pairs[i][j]) * 100
                    count += 1
                    sum1 += bbr
                    sum2 += bbr * bbr
        jfi = (sum1 * sum1) / (count * sum2)
        self.plotter.add_dot_to_graph("jfi", self.load, jfi)

        pcoxc = 0.0
        for i in range(0, self.num_nodes, 1):
            pcoxc += self.pt.get_node_degree(i) * 85 + 150

        pcedfa = self.pt.get_num_links() * 200
        self.total_power_consumed += self.sim_time * (pcoxc + pcedfa)
        self.plotter.add_dot_to_graph("pc", self.load, self.total_power_consumed / self.sim_time)
        self.plotter.add_dot_to_graph("ee", self.load, self.data_transmitted / self.total_power_consumed / 1000)
        self.plotter.add_dot_to_graph("data", self.load, self.data_transmitted)
        self.plotter.add_dot_to_graph("ee2", self.load,
                                      self.blocked_bandwidth / self.required_bandwidth / self.total_power_consumed / (
                                                  self.sim_time * 1000))

    def calculate_periodical_statistics(self) -> None:
        fragmentation_mean = 0.0
        average_crosstalk = 0.0

        for i in range(0, self.pt.get_num_links(), 1):
            fragmentation_mean += self.pt.fragmentation_per_link(self.pt.get_src_link(i), self.pt.get_dst_link(i))
            # average_crosstalk += self.pt.average_crosstalk(self.pt.get_src_link(i), self.pt.get_dst_link(i))

        # average_crosstalk /= self.pt.get_num_links()
        # self.plotter.add_dot_to_graph("avgcrosstalk", self.load, average_crosstalk)
        fragmentation_mean /= self.pt.get_num_links()
        # print("Fragmentation mean: ", fragmentation_mean)
        # self.plotter.add_dot_to_graph("fragmentation", self.load, fragmentation_mean)
        # mean_transponders = 0.0
        # for i in range(0, len(self.number_of_used_transponders), 1):
        #     for j in range(0, len(self.number_of_used_transponders[i]), 1):
        #         if self.number_of_used_transponders[i][j] > 0:
        #             mean_transponders += self.number_of_used_transponders[i][j]
        #
        # if mean_transponders != float('nan'):
        #     self.plotter.add_dot_to_graph("transponders", self.load, mean_transponders)

        # xtps = 0.0
        # links_xtps = 0
        # for i in range(0, self.pt.get_num_links(), 1):
        #     try:
        #         xt = self.pt.get_cross_talk_per_slot(self.pt.get_src_link(i), self.pt.get_dst_link(i))
        #         if xt > 0:
        #             xtps += xt
        #             links_xtps += 1
        #     except ValueError:
        #         print("Error: ", ValueError)
        # if xtps != 0:
        #     self.plotter.add_dot_to_graph("xtps", self.load, xtps / links_xtps)

    def accept_flow(self, flow: Flow, light_paths: LightPath) -> None:
        if self.number_arrivals > self.min_number_arrivals:
            self.accepted += 1
            links = len(flow.get_links()) + 1
            self.plotter.add_dot_to_graph("modulation", self.load, flow.get_modulation_level())
            self.plotter.add_dot_to_graph("hops", self.load, links)

    # def groom_flow(self, flow: Flow) -> None:
    #     if self.number_arrivals > self.min_number_arrivals:
    #         self.data_transmitted += flow.get_rate()
    #         for i in range(0, self.pt.get_cores(), 1):
    #             self.total_power_consumed += flow.get_duration() * len(flow.get_slot_list()) * Modulations.get_power_consumption(flow.get_modulation_level())

    def block_flow(self, flow: Flow) -> None:
        if self.number_arrivals > self.min_number_arrivals:
            self.blocked += 1
            cos = flow.get_cos()
            self.blocked_diff[cos] += 1
            self.blocked_bandwidth += flow.get_rate()
            self.blocked_bandwidth_diff[cos] += flow.get_rate()
            self.blocked_pairs[flow.get_source()][flow.get_destination()] += 1
            self.blocked_pairs_diff[cos][flow.get_source()][flow.get_destination()] += 1
            self.blocked_bandwidth_pairs[flow.get_source()][flow.get_destination()] += flow.get_rate()
            self.blocked_bandwidth_pairs_diff[cos][flow.get_source()][flow.get_destination()] += flow.get_rate()

    def add_event(self, event: Event) -> None:
        self.sim_time = event.get_time()
        try:
            if isinstance(event, FlowArrivalEvent):
                self.number_arrivals += 1
                if self.number_arrivals > self.min_number_arrivals:
                    cos = event.get_flow().get_cos()
                    self.arrivals += 1
                    self.arrivals_diff[cos] += 1
                    self.required_bandwidth += event.get_flow().get_rate()
                    self.required_bandwidth_diff[cos] += event.get_flow().get_rate()
                    self.arrivals_pairs[event.get_flow().get_source()][event.get_flow().get_destination()] += 1
                    self.arrivals_pairs_diff[cos][event.get_flow().get_source()][
                        event.get_flow().get_destination()] += 1
                    self.required_bandwidth_pairs[event.get_flow().get_source()][
                        event.get_flow().get_destination()] += event.get_flow().get_rate()
                    self.required_bandwidth_pairs_diff[cos][event.get_flow().get_source()][
                        event.get_flow().get_destination()] += event.get_flow().get_rate()
                if self.verbose and (self.arrivals % 10000 == 0):
                    print(self.verbose)
                    print(self.arrivals)
            elif isinstance(event, FlowDepartureEvent):
                if self.number_arrivals > self.min_number_arrivals:
                    self.departures += 1
                f = event.get_flow()
                if f.is_accepted():
                    self.number_of_used_transponders[f.get_source()][f.get_destination()] -= 1
            if self.number_arrivals % 100 == 0:
                self.calculate_periodical_statistics()
            if self.number_arrivals % 5000 == 0:
                print("MyStatistics: 5000")
        except Exception as e:
            print("Error in MyStatistics: ", e)

    def fancy_statistics(self) -> str:
        accept_prob = 0.0
        block_prob = 0.0
        bbr = 0.0
        if self.accepted == 0:
            accept_prob = 0.0
        else:
            accept_prob = (self.accepted / self.arrivals) * 100
        if self.blocked == 0:
            block_prob = 0.0
            bbr = 0.0
        else:
            block_prob = (self.blocked/ self.arrivals) * 100
            bbr = (self.blocked_bandwidth/ self.required_bandwidth) * 100

        stats = f"Arrivals \t: {self.arrivals}\n"
        stats += f"Required BW \t: {self.required_bandwidth}\n"
        stats += f"Departures \t: {self.departures}\n"
        stats += f"Accepted \t: {self.accepted}\t({accept_prob}%)\n"
        stats += f"Blocked \t: {self.blocked}\t({block_prob}%)\n"
        stats += f"BBR \t\t: {bbr}%\n"
        stats += f"\n"
        stats += f"Blocking probability per s-d pair:\n"

        for i in range(0, self.num_nodes, 1):
            for j in range(0, self.num_nodes, 1):
                stats += f"Pair ({i}->{j}) "
                stats += f"Calls ({self.arrivals_pairs[i][j]})"
                if self.blocked_pairs[i][j] == 0:
                    block_prob = 0.0
                    bbr = 0.0
                else:
                    block_prob = (self.blocked_pairs[i][j] * 1.0 / self.arrivals_pairs[i][j]) * 100
                    bbr = (self.blocked_bandwidth_pairs[i][j] * 1.0 / self.required_bandwidth_pairs[i][j]) * 100
                stats += f"\tBP ({block_prob}%)"
                stats += f"\tBBR ({bbr}%)\n"

        with open("/Users/nhungtrinh/Work/bidirectional-eon/out/stats.txt", "a") as f:
            f.write(stats)
            f.write("\n")

        return stats

    def finish(self) -> None:
        MyStatistics.singleton_object = None
