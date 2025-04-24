from queue import PriorityQueue, Empty

from src.Event import Event


# class EventSort(Event):
#     def __eq__(self, other):
#         return self.get_time() == other.get_time()
#
#     def __lt__(self, other):
#         return self.get_time() < other.get_time()
#
#     def __gt__(self, other):
#         return self.get_time() > other.get_time()
#
#     def __le__(self, other):
#         return self.get_time() <= other.get_time()
#
#     def __ge__(self, other):
#         return self.get_time() >= other.get_time()


class EventScheduler:
    def __init__(self):
        # self.event_sort = EventSort
        self.event_queue = PriorityQueue()

    def add_event(self, event: Event):
        self.event_queue.put(event)

    def pop_event(self) -> Event:
        try:
            return self.event_queue.get_nowait()
        except Empty:
            return None
