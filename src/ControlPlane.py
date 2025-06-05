import xml.etree.ElementTree as ET
from typing import Dict

from src.LightPath import LightPath
from src.ControlPlaneForRSA import ControlPlaneForRSA
from src.EventScheduler import EventScheduler
from src.PhysicalTopology import PhysicalTopology
from src.VirtualTopology import VirtualTopology
from src.TrafficGenerator import TrafficGenerator
from src.Flow import Flow
from src.Tracer import Tracer
from src.MyStatistics import MyStatistics
from src.Event import Event
from src.FlowArrivalEvent import FlowArrivalEvent
from src.FlowDepartureEvent import FlowDepartureEvent
# from src.rsa import *
from src.rsa.ImageRCSA import ImageRCSA
from src.rsa.FIPP import FIPP
from src.rsa.PP import PP
from src.rsa.NewRSA import NewRSA

class ControlPlane(ControlPlaneForRSA):
    def __init__(self, xml: ET.Element, event_scheduler: EventScheduler, rsa_module: str, pt: PhysicalTopology,
                 vt: VirtualTopology, traffic: TrafficGenerator):
        self.rsa = None
        self.pt = pt
        self.vt = vt
        self.mapped_flows = {}
        self.active_flows = {}
        self.tr = Tracer.get_tracer_object()
        self.st = MyStatistics.get_my_statistics()

        try:
            # RSAClass  = globals()[rsa_module]
            # self.rsa = RSAClass()
            # self.rsa.simulation_interface(xml, pt, vt, self, traffic)
            # self.rsa = ImageRCSA()
            self.rsa = NewRSA()
            # self.rsa = FIPP()
            self.rsa.simulation_interface(xml, pt, vt, self, traffic)
        except Exception as e:
            print("Error in ControlPlane: ", e)

    def new_event(self, event: Event):
        if isinstance(event, FlowArrivalEvent):
            self.new_flow(event.get_flow())
            self.rsa.flow_arrival(event.get_flow())
        elif isinstance(event, FlowDepartureEvent):
            removed_flow = self.remove_flow(event.get_id())
            self.rsa.flow_departure(removed_flow)

    def get_flow(self, id: int) -> Flow:
        return self.active_flows.get(id)

    def accept_flow(self, id: int, light_paths: LightPath) -> bool:
        if id < 0:
            raise ValueError("Invalid ID")
        if not id in self.active_flows:
            raise ValueError("Flow not found")

        flow = self.active_flows.get(id)
        if not self.can_add_flow_to_pt(flow, light_paths):
            return False

        self.add_flow_to_pt(flow, light_paths)
        self.mapped_flows[flow] = light_paths
        self.tr.accept_flow(flow, light_paths)
        self.st.accept_flow(flow, light_paths)
        flow.set_accepted(True)
        return True

    def block_flow(self, id: int) -> bool:
        if id < 0:
            raise ValueError("Invalid ID")
        if not id in self.active_flows:
            return False
        flow = self.active_flows.get(id)

        if flow in self.mapped_flows:
            return False

        self.active_flows.pop(id)
        self.tr.block_flow(flow)
        self.st.block_flow(flow)
        return True

    def reroute_flow(self, id: int, light_path: LightPath) -> bool:
        if id < 0:
            raise ValueError("Invalid ID")
        else:
            if not id in self.active_flows:
                return False
            flow = self.active_flows.get(id)
            if not flow in self.mapped_flows:
                return False
            old_path = self.mapped_flows.get(flow)
            self.remove_flow_from_pt(flow, light_path)
            if not self.can_add_flow_to_pt(flow, light_path):
                self.add_flow_to_pt(flow, old_path)
                return False
            self.add_flow_to_pt(flow, light_path)
            self.mapped_flows[flow] = light_path
            return True

    def new_flow(self, flow: Flow) -> None:
        self.active_flows[flow.get_id()] = flow

    def remove_flow(self, id: int) -> Flow:
        if id in self.active_flows:
            flow = self.active_flows.get(id)
            if flow in self.mapped_flows:
                light_path = self.mapped_flows.get(flow)
                self.remove_flow_from_pt(flow, light_path)
                self.mapped_flows.pop(flow)
            self.active_flows.pop(id)
            return flow
        return None

    def remove_flow_from_pt(self, flow: Flow, light_paths: LightPath) -> None:
        links = light_paths.get_links()
        for j in range(0, len(links), 1):
            self.pt.release_slots(self.pt.get_src_link(links[j]), self.pt.get_dst_link(links[j]), light_paths.get_slot_list())
            # self.pt.update_noise(self.pt.get_src_link(links[j]), self.pt.get_dst_link(links[j]), light_paths.get_slot_list(), flow.get_modulation_level())
        self.vt.remove_light_path(light_paths.get_id())
        self.vt.remove_lp_p_cycle(light_paths)
        self.vt.print_light_paths()


    def can_add_flow_to_pt(self, flow: Flow, light_paths: LightPath) -> bool:
        links = light_paths.get_links()
        for j in range(0, len(links), 1):
            if self.pt.are_slots_available(self.pt.get_src_link(links[j]), self.pt.get_dst_link(links[j]), light_paths.get_slot_list()):
                return False
        return True
    def add_flow_to_pt(self, flow: Flow, light_paths: LightPath) -> None:
        links = light_paths.get_links()
        for j in range(0, len(links), 1):
            self.pt.reserve_slots(self.pt.get_src_link(links[j]), self.pt.get_dst_link(links[j]), light_paths.get_slot_list())
            # self.pt.get_link(links[j]).update_noise(light_paths.get_slot_list(), flow.get_modulation_level())

    def get_path(self, flow: Flow) -> LightPath:
        return self.mapped_flows.get(flow)

    def get_mapped_flows(self) -> Dict[Flow, LightPath]:
        return self.mapped_flows

    def can_groom(self, flow: Flow) -> bool:
        raise NotImplementedError

    def groom_flow(self, flow: Flow, lp: LightPath) -> None:
        for link_id in lp.get_links():
            self.pt.reserve_slots(self.pt.get_src_link(link_id), self.pt.get_dst_link(link_id), lp.get_slot_list())
        self.groom_flow(flow)
