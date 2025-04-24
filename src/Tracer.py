from src.Event import Event
from src.Flow import Flow
from src.LightPath import LightPath
from src.FlowArrivalEvent import FlowArrivalEvent
from src.FlowDepartureEvent import FlowDepartureEvent


class Tracer:
    singleton_object = None

    def __init__(self):
        self.write_trace = True
        self.trace = None

    @staticmethod
    def get_tracer_object():
        if Tracer.singleton_object is None:
            Tracer.singleton_object = Tracer()
        return Tracer.singleton_object

    def __copy__(self):
        raise Exception("CloneNotSupportedException")

    def set_trace_file(self, file_name: str) -> None:
        try:
            self.trace = open(file_name, "w")
        except IOError:
            print(f"Error: Could not open file {IOError}")
            exit(1)

    def toogle_trace_writing(self, write: bool) -> None:
        self.write_trace = write

    def add(self, obj: object) -> None:
        try:
            if isinstance(obj, str):
                self.trace.write(obj + "\n")
            elif isinstance(obj, Event):
                self.add_event(obj)
        except Exception as e:
            print(e)

    def accept_flow(self, flow: Flow, lightpaths: LightPath) -> None:
        str_ = f"flow-accepted - {flow.to_trace()} {lightpaths.get_id()}"
        if self.write_trace:
            self.trace.write(str_ + '\n')

    def block_flow(self, flow: Flow) -> None:
        if self.write_trace:
            self.trace.write(f"flow-blocked - {flow.to_trace()}\n")

    def create_lightpath(self, lp: LightPath) -> None:
        if self.write_trace:
            self.trace.write(f"lightpath-created {lp.to_trace()}\n")

    def remove_lightpath(self, lp: LightPath) -> None:
        if self.write_trace:
            self.trace.write(f"lightpath-removed {lp.to_trace()}\n")

    def add_event(self, event: Event) -> None:
        try:
            if isinstance(event, FlowArrivalEvent):
                if self.write_trace:
                    self.trace.write(f"flow-arrived {event.get_time()} {event.get_flow().to_trace()}\n")
            elif isinstance(event, FlowDepartureEvent):
                if self.write_trace:
                    self.trace.write(f"flow-departed {event.get_time()} {event.get_id()} - - - - -\n")
        except Exception as e:
            print(e)

    def finish(self) -> None:
        """Finalizes the tracing actions and closes the trace file"""
        if self.trace:
            self.trace.flush()
            self.trace.close()
        Tracer._singleton_object = None
