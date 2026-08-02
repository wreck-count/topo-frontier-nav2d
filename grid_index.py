from typing import Protocol
from sortedcontainers import SortedKeyList
import math

EPS = 1e-3

type Bounds = tuple[float, float, float, float]

class SpatialIndex[T](Protocol):
    def insert(self, key: T, bounds: Bounds) -> list[T]:
        ...
    def remove(self, key: T) -> bool:
        ...
    def query(self, bounds: Bounds) -> list[T]:
        ...

class GridIndex[T]:
    def __init__(self, cell_size: int = 2) -> None:
        self.cell_size = cell_size
        self.rows: dict[int, SortedKeyList] = dict()
        self.elems: dict[T, Bounds] = dict()
        
    def __get_index(self, point: tuple[float, float]) -> tuple[int, int]:    
        return (math.floor(point[0] / self.cell_size), math.floor(point[1] / self.cell_size))

    def __key(self, k: tuple[int, T]):
        return k[0]
    
    def insert(self, key: T, bounds: Bounds):
        if key in self.elems:
            return
        self.elems[key] = bounds
        mix, miy, mxx, mxy = bounds
        mi_ridx, mi_cidx = self.__get_index((mix, miy))
        mx_ridx, mx_cidx = self.__get_index((mxx, mxy))
        ilist = [(yidx, key) for yidx in range(mi_cidx, mx_cidx + 1)]
        for ridx in range(mi_ridx, mx_ridx + 1):
            if ridx not in self.rows:
                self.rows[ridx] = SortedKeyList(key=self.__key)
            self.rows[ridx].update(ilist)

    def remove(self, key: T):
        bounds = self.elems.pop(key, None)
        if bounds is None:
            return False
        mix, miy, mxx, mxy = bounds
        mi_ridx, mi_cidx = self.__get_index((mix, miy))
        mx_ridx, mx_cidx = self.__get_index((mxx, mxy))
        for ridx in range(mi_ridx, mx_ridx + 1):
            row = self.rows.get(ridx)
            if row is None:
                continue
            for cidx in range(mi_cidx, mx_cidx + 1):
                row.discard((cidx, key))
            if not row:
                del self.rows[ridx]
        return True

    def query(self, bounds: Bounds):
        mix, miy, mxx, mxy = bounds
        # expand the box a little
        mix -= EPS
        miy -= EPS 
        mxx += EPS
        mxy += EPS
        mi_ridx, mi_cidx = self.__get_index((mix, miy))
        mx_ridx, mx_cidx = self.__get_index((mxx, mxy))
        result: list[T] = []
        seen: set[T] = set()
        for ridx in range(mi_ridx, mx_ridx + 1):
            if ridx not in self.rows:
                continue
            for _, elem in self.rows[ridx].irange((mi_cidx, 0), (mx_cidx, 0)):
                imix, imiy, imxx, imxy = self.elems[elem]
                if imix <= mxx and imiy <= mxy and imxx >= mix and imxy >= miy and elem not in seen:
                    result.append(elem) 
                    seen.add(elem)
        return result