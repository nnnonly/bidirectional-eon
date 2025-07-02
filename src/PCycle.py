from typing import List, Dict
from src.ProtectingLightPath import ProtectingLightPath
from src.Slot import Slot

class PCycle:
    def __init__(self, cycle_links: List[int], nodes: List[int], slot_list: List[Slot], reserved_slots:int = 0, protected_lightpaths:List[ProtectingLightPath] = [], be_protection: List[ProtectingLightPath] = [], id_links: Dict[int, List[ProtectingLightPath]] = {}):
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
        self.id_links = id_links if id_links else {}

    def add_protected_lightpath(self, lightpath: ProtectingLightPath):
        self.protected_lightpaths.append(lightpath)
        for lp_id_link in lightpath.get_links():
            if lp_id_link not in self.id_links.keys():
                    self.id_links.setdefault(lp_id_link, []).append(lightpath)
            else:
                if lightpath not in self.id_links[lp_id_link]:
                    self.id_links[lp_id_link].append(lightpath)

    def remove_path_by_id(self, target_id: int):
        keys_to_delete = []

        for key, path_list in self.id_links.items():
            new_list = [p for p in path_list if p.id != target_id]

            if new_list:
                self.id_links[key] = new_list
            else:
                keys_to_delete.append(key)

        for key in keys_to_delete:
            del self.id_links[key]


    def remove_protected_lightpath(self, lightpath):
        paths = [p for p in self.protected_lightpaths if p.id != lightpath.id]
        self.protected_lightpaths = paths
        self.remove_path_by_id(lightpath.id)

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

    def get_reserved_slots(self):
        return self.reserved_slots

    def get_protected_lightpaths(self):
        return self.protected_lightpaths

    def get_id_links(self):
        return self.id_links

    def set_id_links(self, id_links: Dict[int, List[ProtectingLightPath]]):
        self.id_links = id_links

    def check_lp_on_cycle(self, lightpath: ProtectingLightPath):
        """
        Check if the lightpath is on P-cycle
        :param lightpath: ProtectingLightPath object
        :return: True if the lightpath is on P-cycle, False otherwise
        """
        if set(lightpath.get_links()) & set(self.cycle_links):
            return True
        return False

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

    def get_core_slot_range(self):
        if not self.slot_list:
            return None, None, None
        core = self.slot_list[0].core
        slots = [s.slot for s in self.slot_list]
        return core, min(slots), max(slots)

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
        return f"P-cycle: {self.cycle_links}, Nodes: {self.nodes}, Protected Paths: {len(self.protected_lightpaths)}, Reserved Slots: {self.reserved_slots}, Cycle Links: {self.cycle_links}, Slot List: {self.slot_list}"
