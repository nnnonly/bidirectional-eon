import xml.etree.ElementTree as ET
import time

from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.TrafficGenerator import TrafficGenerator
from src.EventScheduler import EventScheduler
from src.MyStatistics import MyStatistics
from src.OutputManager import OutputManager
from src.Tracer import Tracer
from src.ControlPlane import ControlPlane
from src.SimulationRunner import SimulationRunner


class Simulator:
    sim_name = "flexgridsim"
    sim_version = "2.0"
    verbose = False
    trace = False

    def __init__(self, sim_config_file: str, trace: bool, verbose: bool, forced_load: float, num_simulations: int):
        Simulator.trace = trace
        Simulator.verbose = verbose

        if Simulator.verbose:
            print("#################################")
            print("# Simulator: " + Simulator.sim_name + " version " + Simulator.sim_version + " #")
            print("#################################")
            print("(0) Accessing simulation file " + sim_config_file + "...")

        with open(sim_config_file, 'r') as f:
            root = ET.parse(f).getroot()
            assert root.tag == Simulator.sim_name, "Root element of the simulation file is " + root.tag + ", " + Simulator.sim_name + " is expected!"
            assert "version" in root.attrib.keys(), "Cannot find version attribute!"
            assert root.attrib[
                       "version"] <= Simulator.sim_version, "Simulation config file requires a newer version of the simulator!"
            for child in root:
                if child.tag == "rsa":
                    self.rsa = child
                elif child.tag == "trace":
                    self.trace = child.attrib
                elif child.tag == "traffic":
                    self.traffic = child
                elif child.tag == "virtual-topology":
                    self.virtual_topology = child
                elif child.tag == "physical-topology":
                    self.physical_topology = child
                elif child.tag == "graphs":
                    self.graphs = child
                else:
                    assert False, "Unknown element " + child.tag + " in the simulation file!"
            assert hasattr(self, "rsa"), "rsa element is missing!"
            assert hasattr(self, "trace"), "trace element is missing!"
            assert hasattr(self, "traffic"), "traffic element is missing!"
            assert hasattr(self, "virtual_topology"), "virtual-topology element is missing!"
            assert hasattr(self, "physical_topology"), "physical-topology element is missing!"
            assert hasattr(self, "graphs"), "graphs element is missing!"

            gp = OutputManager(self.graphs)
            for seed in range(1, num_simulations + 1, 1):
                begin_s = time.time_ns()
                begin = time.time_ns()
                if Simulator.verbose:
                    print("(0) Done. (", round((time.time_ns() - begin) * 1e-9, 3), " sec)")
                begin = time.time_ns()
                if Simulator.verbose:
                    print("(1) Loading physical topology information...")
                pt = PhysicalTopology(self.physical_topology, verbose)
                if Simulator.verbose:
                    print(pt)
                    print("(1) Done. (", round((time.time_ns() - begin) * 1e-9, 3), " sec)")

                # Extract virtual topology part
                begin = time.time_ns()
                if Simulator.verbose:
                    print("(2) Loading virtual topology information...")
                vt = VirtualTopology(self.virtual_topology, pt, verbose)
                if Simulator.verbose:
                    print(vt)
                    print("(2) Done. (", round((time.time_ns() - begin) * 1e-9, 3), " sec)")

                # Extract simulation traffic part
                begin = time.time_ns()
                if Simulator.verbose:
                    print("(3) Loading traffic information...")
                events = EventScheduler()
                traffic = TrafficGenerator(self.traffic, forced_load, verbose)
                traffic.generate_traffic(pt, events, seed)
                print("traffic: ", traffic)
                if Simulator.verbose:
                    print("(3) Done. (", round((time.time_ns() - begin) * 1e-9, 3), " sec)")

                # Load graph configuration
                begin = time.time_ns()
                if Simulator.verbose:
                    print("(4) Loading graph information...")

                # Extract simulation setup part
                begin = time.time_ns()
                if Simulator.verbose:
                    print("(4) Loading simulation setup information...")

                st = MyStatistics.get_my_statistics()
                st.statistics_setup(gp, pt, traffic, pt.get_num_nodes(), 3, 0, forced_load, Simulator.verbose)

                tr = Tracer.get_tracer_object()

                if Simulator.trace:
                    if forced_load == 0:
                        tr.set_trace_file(sim_config_file[4:-4] + ".trace")
                    else:
                        tr.set_trace_file(sim_config_file[4:-4] + "_Load_" + str(forced_load) + ".trace")
                tr.toogle_trace_writing(Simulator.trace)

                assert "module" in self.rsa.attrib, "RSA module is missing!"
                rsa_module = self.rsa.attrib["module"]
                if Simulator.verbose:
                    print("RSA module: " + rsa_module)

                cp = ControlPlane(self.rsa, events, rsa_module, pt, vt, traffic)
                if Simulator.verbose:
                    print("(4) Done. (", round((time.time_ns() - begin) * 1e-9, 3), " sec)")

                begin = time.time_ns()
                if Simulator.verbose:
                    print("(5) Running the simulation...")
                print(f"{sim_config_file} -> Load {forced_load}: Running the simulation number {seed}")

                # with open("/Users/nhungtrinh/Documents/ISIMA/networkx-flexgrid/stats.txt", "a") as f:
                #     f.write(f"{sim_config_file} -> Load {forced_load}: Running the simulation number {seed} \n")
                SimulationRunner(cp, events)
                if Simulator.verbose:
                    print("(5) Done. (", round((time.time_ns() - begin) * 1e-9, 3), " sec)")

                # with open("/Users/nhungtrinh/Documents/ISIMA/networkx-flexgrid/stats.txt", "a") as f:
                #     f.write(f"TIME: {round((time.time_ns() - begin_s) * 1e-9, 3)} sec \n")

                if Simulator.verbose:
                    if forced_load == 0:
                        print(f"Statistics ({sim_config_file}):")
                    else:
                        print(f"Statistics for {forced_load} erlangs ({sim_config_file}):")
                    print(st.fancy_statistics())
                else:
                    st.calculate_last_statistics()

                st.finish()

                if Simulator.trace:
                    tr.finish()

            gp.write_all_to_files()
