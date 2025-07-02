import xml.etree.ElementTree as ET
from typing import List, Type

from src.util.Distribution import Distribution
from src.TrafficInfo import TrafficInfo
from src.PhysicalTopology import PhysicalTopology
from src.EventScheduler import EventScheduler
from src.Flow import Flow
from src.FlowArrivalEvent import FlowArrivalEvent
from src.FlowDepartureEvent import FlowDepartureEvent


class TrafficGenerator:
    def __init__(self, xml: ET.Element, forced_load: float, verbose: bool):
        self.verbose = verbose
        self.rate = 0
        self.cos = 0
        self.weight = 0
        self.holding_time = 0.0
        self.xml = xml
        self.min_rate = []
        self.min_size = []
        self.max_size = []

        assert "calls" in xml.attrib, "calls attribute is missing!"
        self.calls = int(xml.attrib["calls"])
        self.load = int(forced_load)

        if self.load == 0:
            assert "load" in xml.attrib, "load attribute is missing!"
            self.load = float(xml.attrib["load"])

        assert "max-rate" in xml.attrib, "max-rate attribute is missing!"
        self.max_rate = int(xml.attrib["max-rate"])

        if verbose:
            print(f'{xml.attrib["calls"]} calls, {xml.attrib["load"]} erlangs.')

        call_list = []
        for child in xml:
            if child.tag == "calls":
                assert "rate" in child.attrib, "rate attribute is missing!"
                assert "cos" in child.attrib, "cos attribute is missing!"
                assert "weight" in child.attrib, "weight attribute is missing!"
                assert "holding-time" in child.attrib, "holding-time attribute is missing!"
                call_list.append(child.attrib)
            else:
                raise Exception("Unknown element " + child.tag + " in the traffic generator file!")
        self.number_calls_type = len(call_list)
        if verbose:
            print(f"{self.number_calls_type} type(s) of calls:")

        self.total_weight = 0
        self.mean_rate = 0
        self.mean_holding_time = 0
        self.calls_types_info = [TrafficInfo] * self.number_calls_type

        for i in range(0, self.number_calls_type, 1):
            self.total_weight += int(call_list[i]["weight"])

        for i in range(0, self.number_calls_type, 1):
            holding_time = float(call_list[i]["holding-time"])
            rate = int(call_list[i]["rate"])
            cos = int(call_list[i]["cos"])
            weight = int(call_list[i]["weight"])
            self.mean_rate += rate * (weight / self.total_weight)
            self.mean_holding_time += holding_time * (weight / self.total_weight)
            self.calls_types_info[i] = TrafficInfo(holding_time, rate, cos, weight)
            if verbose:
                print("#################################")
                print(f'Weight: {weight}.;')
                print(f'COS: {cos}.')
                print(f'Rate: {rate} Mbps.')
                print(f'Mean holding time: {holding_time} seconds.')

    def generate_traffic(self, pt: PhysicalTopology, events: EventScheduler, seed: int) -> None:
        weight_vector = [int] * self.total_weight
        aux = 0

        for i in range(0, self.number_calls_type, 1):
            for j in range(0, self.calls_types_info[i].get_weight(), 1):
                weight_vector[aux] = i
                aux += 1

        mean_arrival_time = (self.mean_holding_time * (self.mean_rate * 1.0 / self.max_rate)) / self.load

        type = 0
        src = 0
        dst = 0
        time = 0.0
        id = 0
        num_nodes = pt.get_num_nodes()
        dist1 = Distribution(1, seed)
        dist2 = Distribution(2, seed)
        dist3 = Distribution(3, seed)
        dist4 = Distribution(4, seed)

        if "fileSizeValues" in self.xml.attrib:
            assert False, "Not implemented yet!"

        print("SELF.CALLS: ", self.calls)
        for j in range(0, self.calls, 1):
            type = weight_vector[dist1.next_int(self.total_weight)]
            # type = weight_vector[0]
            src = dst = dist2.next_int(num_nodes)
            # src = dst = 18
            while src == dst:
                dst = dist2.next_int(num_nodes)
                # dst = 1
            holding_time = 0.0
            if "fileSizeValues" in self.xml.attrib:
                file_size = 0.0
                rate_in_gbps = self.oc_in_gigabits(self.calls_types_info[type].get_rate())
                holding_time = ((file_size / rate_in_gbps) * 8)
                # holding_time = 0.4190175227589326
            else:
                holding_time = dist4.next_exponential(self.calls_types_info[type].get_holding_time())
                # holding_time = 0.4190175227589326

            new_flow = Flow(id, src, dst, time, self.calls_types_info[type].get_rate(), holding_time,
                            self.calls_types_info[type].get_cos(), time + (holding_time * 0.5))
            event = FlowArrivalEvent(time, new_flow)
            time += dist3.next_exponential(mean_arrival_time)
            # time += 0.09715752559779525
            events.add_event(event)
            event = FlowDepartureEvent(time + holding_time, id, new_flow)
            events.add_event(event)
            id += 1

    def get_calls_types_info(self) -> List[Type[TrafficInfo]]:
        return self.calls_types_info

    @staticmethod
    def oc_in_gigabits(oc: int) -> float:
        rate_in_gbps = 0.0
        if oc == 3:
            rate_in_gbps = 0.1
        elif oc == 12:
            rate_in_gbps = 0.5
        elif oc == 24:
            rate_in_gbps = 1.0
        elif oc == 48:
            rate_in_gbps = 2.5
        elif oc == 96:
            rate_in_gbps = 5.0
        elif oc == 192:
            rate_in_gbps = 10.0
        else:
            print("invalid rate!!")
            exit(1)
        return rate_in_gbps
