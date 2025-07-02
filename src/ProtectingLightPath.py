from typing import List, Tuple, Set
from src.Slot import Slot


class ProtectingLightPath:
    def __init__(self, id: float, src: int, dst: int, links_id: List[int], fss: int, backup_paths: List[List[int]] = []):
        if id < 0 or src < 0 or dst < 0 or len(links_id) < 1 or fss < 0:
            raise ValueError("IllegalArgumentException")
        else:
            self.id = id
            self.src = src
            self.dst = dst
            self.links_id = links_id
            self.fss = fss
            self.backup_paths = backup_paths

    def get_id(self) -> float:
        return self.id

    def get_source(self) -> int:
        return self.src

    def get_destination(self) -> int:
        return self.dst

    def get_links(self) -> List[int]:
        return self.links_id

    def get_fss(self) -> int:
        return self.fss

    def get_backup_paths(self) -> List[List[int]]:
        return self.backup_paths

    def __str__(self) -> str:
        protect_light_path = f"id:{self.id}, src: {self.src}, dst:{self.dst}, backup_paths: {self.backup_paths}, fss:{self.fss}, links_id:{self.links_id}"
        return protect_light_path
    #
    # def to_trace(self) -> str:
    #     light_path = f"{self.id} {self.src} {self.dst} "
    #     for i in range(0, len(self.links), 1):
    #         light_path += f"{self.links[i]}-"
    #     return light_path
