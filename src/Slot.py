class Slot:
    def __init__(self, core: int, slot: int):
        self.core = core
        self.slot = slot

    def __str__(self):
        return f"({self.core}, {self.slot})"
