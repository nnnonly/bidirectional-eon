from abc import ABC, abstractmethod


class Event(ABC):

    def __init__(self, time: float):
        self.time = time

    def set_time(self, time: float) -> None:
        self.time = time

    def get_time(self) -> float:
        return self.time

    def __eq__(self, other):
        return self.get_time() == other.get_time()

    def __lt__(self, other):
        return self.get_time() < other.get_time()

    def __gt__(self, other):
        return self.get_time() > other.get_time()

    def __le__(self, other):
        return self.get_time() <= other.get_time()

    def __ge__(self, other):
        return self.get_time() >= other.get_time()