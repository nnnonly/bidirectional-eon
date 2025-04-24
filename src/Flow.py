from typing import Type

from .Slot import Slot

class Flow:
    def __init__(self, id: float, src: int, dst: int, time: float, bw: int, duration: float, cos: int, deadline: float):
        if id < 0 or src < 0 or dst < 0 or bw < 1 or duration < 0 or cos < 0:
            raise ValueError("IllegalArgumentException")
        else:
            self.id = id
            self.src = src
            self.dst = dst
            self.bw = bw
            self.duration = duration
            self.cos = cos
            self.deadline = deadline
            self.accepted = False
            self.time = time
            self.modulation_level = 0
            self.groomed = False
            self.links = [int]
            self.slot_list = [Slot]

    def get_time(self) -> float:
        return self.time

    def get_id(self) -> float:
        return self.id

    def get_source(self) -> int:
        return self.src

    def get_destination(self) -> int:
        return self.dst

    def get_rate(self) -> int:
        return self.bw

    def set_rate(self, rate: int) -> None:
        self.bw = rate

    def get_duration(self) -> float:
        return self.duration

    def get_cos(self) -> int:
        return self.cos

    def get_slot_list(self) -> [Slot]:
        return self.slot_list

    def set_slot_list(self, slot_list: [Slot]) -> None:
        self.slot_list = slot_list

    def get_links(self) -> [int]:
        return self.links

    def get_link(self, i: int) -> Type[int]:
        return self.links[i]

    def set_links(self, links: [int]) -> None:
        self.links = links

    def __str__(self):
        flow = f"{self.id}: {self.src}->{self.dst} rate: {self.bw} duration: {self.duration} cos: {self.cos}"
        return flow

    def get_deadline(self) -> float:
        return self.deadline

    def set_deadline(self, deadline: float) -> None:
        self.deadline = deadline

    def to_trace(self) -> str:
        trace = f"{self.id} {self.src} {self.dst} {self.bw} {self.duration} {self.cos}"
        return trace

    def is_accepted(self) -> bool:
        return self.accepted

    def set_accepted(self, accepted: bool) -> None:
        self.accepted = accepted

    def get_modulation_level(self) -> int:
        return self.modulation_level

    def set_modulation_level(self, modulation_level: int) -> None:
        self.modulation_level = modulation_level

    def is_groomed(self) -> bool:
        return self.groomed

    def set_groomed(self, groomed: bool) -> None:
        self.groomed = groomed
