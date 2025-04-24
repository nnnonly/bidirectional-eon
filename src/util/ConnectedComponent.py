from typing import List, Dict

from src.Slot import Slot


class Dimension:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height


class ConnectedComponent:
    def __init__(self):
        self.MAX_LABELS = 80000
        self.next_label = 1

    def list_of_regions(self, image: [[bool]]) -> Dict[int, List[Slot]]:
        res = {}
        res_matrix = self.component_labeling(image)
        for i in range(0, len(res_matrix), 1):
            for j in range(0, len(res_matrix[i]), 1):
                if res_matrix[i][j] != 1:
                    if res_matrix[i][j] not in res:
                        res[res_matrix[i][j]] = []
                    res[res_matrix[i][j]].append(Slot(i, j))
        return res
    def component_labeling(self, image: [[bool]]) -> [[int]]:
        rows = len(image)
        columns = len(image[0])
        interger_image = [0] * rows * columns
        for i in range(0, rows, 1):
            for j in range(0, columns, 1):
                if image[i][j]:
                    interger_image[i * len(image[i]) + j] = 0
                else:
                    interger_image[i * len(image[i]) + j] = 1
        res = self.compact_labeling(interger_image, Dimension(rows, columns), True)
        res_matrix = [[0 for _ in range(columns)] for _ in range(rows)]
        j = 0
        k = 0
        for i in range(0, len(res), 1):
            res_matrix[k][j] = res[i]
            j += 1
            if j == columns:
                k += 1
                j = 0
        return res_matrix

    def compact_labeling(self, image: [int], d: Dimension, zero_as_bg: bool) -> [[int]]:
        label = self.labeling(image, d, zero_as_bg)
        start = [0] * (self.next_label + 1)
        for i in range(0, len(image), 1):
            if label[i] > self.next_label:
                raise Exception("bigger label than next_label found")
            start[label[i]] += 1

        start[0] = 0
        j = 1
        for i in range(1, len(start), 1):
            if start[i] != 0:
                start[i] = j
                j += 1
        self.next_label = j - 1
        for i in range(0, len(image), 1):
            label[i] = start[label[i]]
        return label

    def get_max_label(self):
        return self.next_label

    def labeling(self, image: [int], d: Dimension, zero_as_bg: bool) -> [int]:
        w = d.width
        h = d.height
        rst = [0] * w * h
        parent = [0] * self.MAX_LABELS
        labels = [0] * self.MAX_LABELS
        next_region = 1
        for y in range(0, h, 1):
            for x in range(0, w, 1):
                if image[y * w + x] == 0 and zero_as_bg:
                    continue
                k = 0
                connected = False
                if x > 0 and image[y * w + x - 1] == image[y * w + x]:
                    k = rst[y * w + x - 1]
                    connected = True

                if y > 0 and image[(y - 1) * w + x] == image[y * w + x] and (not connected or image[(y - 1) * w + x] < k):
                    k = rst[(y - 1) * w + x]
                    connected = True

                if not connected:
                    k = next_region
                    next_region += 1
                if k >= self.MAX_LABELS:
                    raise Exception("maximum number of labels reached. increase MAX_LABELS and recompile.")
                    exit(1)
                rst[y * w + x] = k

                if x > 0 and image[y * w + x - 1] == image[y * w + x] and rst[y * w + x - 1] != k:
                    parent = self.uf_union(k, rst[y * w + x - 1], parent)
                if y > 0 and image[(y - 1) * w + x] == image[y * w + x] and rst[(y - 1) * w + x] != k:
                    parent = self.uf_union(k, rst[(y - 1) * w + x], parent)
        self.next_label = 1
        for i in range(0, w * h, 1):
            if image[i] != 0 or not zero_as_bg:
                rst[i] = self.uf_find(rst[i], parent, labels)
                if not zero_as_bg:
                    rst[i] -= 1
        self.next_label -= 1
        if not zero_as_bg:
            self.next_label -= 1
        return rst

    def uf_union(self, x: int, y: int, parent: [int]):
        while parent[x] > 0:
            x = parent[x]
        while parent[y] > 0:
            y = parent[y]
        if x != y:
            if x < y:
                parent[x] = y
            else:
                parent[y] = x
        return parent

    def uf_find(self, x: int, parent: [int], label: [int]) -> int:
        while parent[x] > 0:
            x = parent[x]
        if label[x] == 0:
            label[x] = self.next_label
            self.next_label += 1
        return label[x]