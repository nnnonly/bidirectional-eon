class TrafficInfo:
    def __init__(self, holding_time: float, rate: int, cos: int, weight: int):
        self.holding_time = holding_time
        self.rate = rate
        self.cos = cos
        self.weight = weight

    def get_holding_time(self) -> float:
        return self.holding_time

    def get_rate(self) -> int:
        return self.rate

    def get_cos(self) -> int:
        return self.cos

    def get_weight(self) -> int:
        return self.weight
