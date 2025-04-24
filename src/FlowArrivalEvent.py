from .Flow import Flow
from .Event import Event


class FlowArrivalEvent(Event):
    def __init__(self, time: float, flow: Flow):
        super().__init__(time)
        self.flow = flow

    def get_flow(self) -> Flow:
        return self.flow

    def __str__(self):
        return f"Arrival: {self.flow}"
