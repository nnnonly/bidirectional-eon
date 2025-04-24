import math
from typing import List


class DataSet:
    def __init__(self, dimension: int):
        self.dots = list()
        self.dimension = dimension

    def get_number_of_dots(self) -> int:
        return len(self.dots)

    def get_dot_value(self, dot_index: int, value_index: int) -> float:
        try:
            dot = self.dots[dot_index]
        except IndexError:
            return float('nan')
        return dot[value_index]

    def dot_to_string(self) -> str:
        mean = self.dots_mean()
        confidence_interval = self.dots_confidence_interval()
        dot_string = str(self.get_dot_value(0, 0))
        for i in range(1, len(mean), 1):
            dot_string += f"\t{mean[i]}\t{confidence_interval[i]:}"
        return dot_string

    def add_dot(self, *values: float):
        assert len(values) == self.dimension, "Invalid dimension"
        self.dots.append(list(values))

    def dots_sum(self) -> List[float]:
        sum = [0.0] * self.dimension
        for dot in self.dots:
            for i in range(0, self.dimension, 1):
                if dot[i] is not float('nan'):
                    sum[i] += dot[i]
        return sum

    def dots_square_sum(self) -> List[float]:
        sum2 = [0.0] * self.dimension
        for dot in self.dots:
            for i in range(0, self.dimension, 1):
                if dot[i] is not float('nan'):
                    sum2[i] += dot[i] * dot[i]
        return sum2

    def dots_mean(self) -> List[float]:
        mean = self.dots_sum()
        for i in range(0, len(mean), 1):
            if len(self.dots) == 0:
                mean[i] = float('nan')
            else:
                mean[i] = mean[i] / len(self.dots)
        return mean

    def dots_standard_deviation(self) -> List[float]:
        n = len(self.dots)
        mean = self.dots_mean()
        sum2 = self.dots_square_sum()
        std_var = [0.0] * self.dimension
        for i in range(0, self.dimension, 1):
            std_var[i] = math.sqrt((sum2[i] - n * (mean[i] * mean[i])) / (n - 1))
        return std_var

    def dots_confidence_interval(self) -> List[float]:
        n = len(self.dots)
        std_var = self.dots_standard_deviation()
        confidence_interval = [0.0] * self.dimension
        for i in range(0, len(confidence_interval), 1):
            if n <= 0:
                confidence_interval[i] = float('nan')
            else:
                confidence_interval[i] = 1.96 * (std_var[i] / math.sqrt(n - 1))
        return confidence_interval
