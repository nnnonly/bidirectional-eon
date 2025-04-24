from .Event import Event
from .Flow import Flow


class FlowDepartureEvent(Event):
    def __init__(self, time: float, id: int, flow: Flow):
        super().__init__(time)
        self.id = id
        self.flow = flow

    def get_id(self) -> int:
        return self.id

    def __str__(self):
        return f"Departure: {self.id}"

    def get_flow(self) -> Flow:
        return self.flow
