from src.ControlPlane import ControlPlane
from src.EventScheduler import EventScheduler
from src.Tracer import Tracer
from src.MyStatistics import MyStatistics


class SimulationRunner:
    def __init__(self, cp: ControlPlane, events: EventScheduler):
        tr = Tracer.get_tracer_object()
        st = MyStatistics.get_my_statistics()

        event = events.pop_event()
        while event is not None:
            tr.add(event)
            st.add_event(event)
            cp.new_event(event)
            event = events.pop_event()


