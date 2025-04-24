class Modulations:
    distance = [10000000, 5000, 3000, 1750, 750, 250]

    @staticmethod
    def number_of_modulations():
        return 6

    @staticmethod
    def get_bandwidth(modulation_level: int) -> float:
        if modulation_level == 0:
            return 12.5
        elif modulation_level == 1:
            return 25.0
        elif modulation_level == 2:
            return 37.5
        elif modulation_level == 3:
            return 50.0
        elif modulation_level == 4:
            return 62.5
        elif modulation_level == 5:
            return 75.0
        else:
            return 0.0

    @staticmethod
    def get_modulation_level(bandwidth: float) -> int:
        if bandwidth <= 12.5:
            return 0
        elif 12.5 < bandwidth <= 25.0:
            return 1
        elif 25.0 < bandwidth <= 37.5:
            return 2
        elif 37.5 < bandwidth <= 50.0:
            return 3
        elif 50.0 < bandwidth <= 62.5:
            return 4
        elif 62.5 < bandwidth:
            return 5
        else:
            return 0

    @staticmethod
    def get_power_consumption(modulation_level: int) -> float:
        if modulation_level == 0:
            return 47.13
        elif modulation_level == 1:
            return 62.75
        elif modulation_level == 2:
            return 78.38
        elif modulation_level == 3:
            return 94.0
        elif modulation_level == 4:
            return 109.63
        elif modulation_level == 5:
            return 125.23
        else:
            return 47.3

    @staticmethod
    def get_max_distance(modulation_level: int) -> int:
        if 0 <= modulation_level <= 5:
            return Modulations.distance[modulation_level]
        else:
            return Modulations.distance[0]

    @staticmethod
    def get_modulation_by_distance(given_distance: int) -> int:
        if given_distance <= Modulations.distance[5]:
            return 5
        elif Modulations.distance[5] < given_distance <= Modulations.distance[4]:
            return 4
        elif Modulations.distance[4] < given_distance <= Modulations.distance[3]:
            return 3
        elif Modulations.distance[3] < given_distance <= Modulations.distance[2]:
            return 2
        elif Modulations.distance[2] < given_distance <= Modulations.distance[1]:
            return 1
        elif Modulations.distance[1] < given_distance:
            return 0
        else:
            return 0
