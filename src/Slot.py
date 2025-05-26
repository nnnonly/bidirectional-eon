class Slot:
    def __init__(self, core: int, slot: int):
        self.core = core
        self.slot = slot

    def __eq__(self, other):
        return isinstance(other, Slot) and self.core == other.core and self.slot == other.slot

    def __hash__(self):
        return hash((self.core, self.slot))

    def __repr__(self):
        return f"Slot(core={self.core}, slot={self.slot})"


    def __str__(self):
        return f"({self.core}, {self.slot})"
