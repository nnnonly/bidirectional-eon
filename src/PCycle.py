from typing import List
from src.ProtectingLightPath import ProtectingLightPath
from src.Slot import Slot

class PCycle:
    def __init__(self, cycle_links: List[int], nodes: List[int], slot_list: List[Slot], reserved_slots:int = 0, protected_lightpaths:List[ProtectingLightPath] = [], be_protection: List[ProtectingLightPath] = []):
        """
        Initialize P-cycle
        :param cycle_links: List of links in P-cycle [(src1, dst1), (src2, dst2), ...]
        :param protected_lightpaths: List of protected lightpaths
        :param reserved_slots: Set of reserved spectrum slots
        """
        self.cycle_links = cycle_links
        self.nodes = nodes
        self.protected_lightpaths = protected_lightpaths if protected_lightpaths else []
        self.be_protection = be_protection if be_protection else []
        self.reserved_slots = reserved_slots
        self.slot_list = slot_list

    def add_protected_lightpath(self, lightpath):
        self.protected_lightpaths.append(lightpath)

    def remove_protected_lightpath(self, lightpath):
        if lightpath in self.protected_lightpaths:
            self.protected_lightpaths.remove(lightpath)

    def remove_be_protected_lightpath(self, lightpath):
        if lightpath in self.be_protection:
            self.be_protection.remove(lightpath)

    def get_cycle_links(self):
        return self.cycle_links

    def set_slot_list(self, slot_list: List[Slot]):
        self.slot_list = slot_list

    def get_slot_list(self) -> List[Slot]:
        return self.slot_list

    def set_reversed_slots(self, reserved_slots):
        self.reserved_slots = reserved_slots

    def p_cycle_contains_flow(self, src, dst):
        """
        Check if the P-cycle contains the flow
        :param p_cycle: List of links in P-cycle
        :param src: Source node
        :param dst: Destination node
        :return: True if the P-cycle contains the flow, False otherwise
        """
        return src in set(self.nodes) and dst in set(self.nodes)

    def has_sufficient_slots(self, required_slots):
        return self.reserved_slots >= required_slots

    def can_protect(self, primary_path):
        for link in primary_path:
            if link in self.cycle_links:  # On-cycle protection
                return True
        return False

    def get_all_lp(self) -> List[List[int]]:
        """get all lightpaths that are protected by the P-cycle"""
        paths = []
        for lp in self.protected_lightpaths:
            paths.append(lp.get_links())
        return paths

    def can_add_links_disjoint(self, new_lp: List[int]):
        """add links p-cycle can protect"""
        if self.protected_lightpaths:
            lp_protected = self.get_all_lp()
            for lp in lp_protected:
                if bool(set(lp) & set(new_lp)):
                    return False
            return True

    # tao cac set be_protection disjoint voi nhau
    def add_lp_to_be_protected(self, new_lp: List[int]):
        if self.be_protection:
            lp_protect = self.be_protection.copy()
            for lp in lp_protect:
                if bool(set(lp.get_links()) & set(new_lp.get_links())):
                    self.be_protection.remove(lp)
                    continue
        self.be_protection.append(new_lp)
        return self.be_protection

    def __str__(self):
        return f"P-cycle: {self.cycle_links}, Protected Paths: {len(self.protected_lightpaths)}, Reserved Slots: {self.reserved_slots}, Cycle Links: {self.cycle_links}, Slot List: {self.slot_list}"
