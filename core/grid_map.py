"""
core/grid_map.py
Representacao do ambiente como occupancy grid.
Convencao: matriz[linha, coluna] = matriz[y, x], com 0 = livre e 1 = obstaculo.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from scipy.ndimage import binary_dilation, generate_binary_structure
from PIL import Image

SQRT2 = float(np.sqrt(2))

# 8-conexao: (dlinha, dcoluna, custo)
NEIGHBORS_8 = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
]


@dataclass
class GridMap:
    grid: np.ndarray                          # matriz original (0 livre, 1 obstaculo)
    resolution: float = 1.0                   # metros por celula (so p/ simulador)
    origin: tuple[float, float] = (0.0, 0.0)  # (x, y) do canto no mundo
    inflated: np.ndarray = field(init=False)  # grid com obstaculos dilatados

    def __post_init__(self):
        self.grid = (np.asarray(self.grid) > 0).astype(np.uint8)
        self.inflated = self.grid.copy()

    # ---------- construtores ----------
    @classmethod
    def from_png(cls, path, threshold=128, max_dim=None, **kw):
        """Carrega PNG -> occupancy grid (pixel escuro = obstaculo).
        max_dim: se o mapa for maior, reduz via max-pooling (paredes
        finas sobrevivem: bloco com qualquer pixel-obstaculo vira obstaculo)."""
        img = np.array(Image.open(path).convert("L"))
        obs = (img < threshold).astype(np.uint8)
        if max_dim and max(obs.shape) > max_dim:
            k = int(np.ceil(max(obs.shape) / max_dim))
            obs = _maxpool(obs, k)
        return cls(grid=obs, **kw)

    @classmethod
    def empty(cls, rows, cols, walls=True, **kw):
        g = np.zeros((rows, cols), dtype=np.uint8)
        if walls:
            g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 1
        return cls(grid=g, **kw)

    # ---------- inflar obstaculos ----------
    def inflate(self, robot_radius_cells: int):
        """Dilata obstaculos pelo raio do robo (em celulas) -> robo vira ponto."""
        if robot_radius_cells <= 0:
            self.inflated = self.grid.copy()
            return self
        struct = generate_binary_structure(2, 2)   # vizinhanca 8
        self.inflated = binary_dilation(
            self.grid, structure=struct, iterations=robot_radius_cells
        ).astype(np.uint8)
        return self

    # ---------- consultas ----------
    @property
    def shape(self):
        return self.grid.shape

    def in_bounds(self, cell):
        r, c = cell
        return 0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]

    def is_free(self, cell, inflated=True):
        if not self.in_bounds(cell):
            return False
        g = self.inflated if inflated else self.grid
        return g[cell[0], cell[1]] == 0

    def neighbors(self, cell, inflated=True):
        """Vizinhos livres em 8-conexao -> (vizinho, custo)."""
        r, c = cell
        for dr, dc, cost in NEIGHBORS_8:
            nb = (r + dr, c + dc)
            if not self.is_free(nb, inflated):
                continue
            # impede "cortar quina" na diagonal entre dois obstaculos
            if dr != 0 and dc != 0:
                if not self.is_free((r + dr, c), inflated) and \
                   not self.is_free((r, c + dc), inflated):
                    continue
            yield nb, cost

    def line_of_sight(self, a, b, inflated=True):
        """True se o segmento de celula a->b nao cruza obstaculo (Bresenham)."""
        for cell in bresenham(a, b):
            if not self.is_free(cell, inflated):
                return False
        return True

    # ---------- mundo <-> grid ----------
    def to_world(self, cell):
        r, c = cell
        x = self.origin[0] + (c + 0.5) * self.resolution
        y = self.origin[1] + (r + 0.5) * self.resolution
        return x, y

    def to_cell(self, point):
        x, y = point
        c = int((x - self.origin[0]) / self.resolution)
        r = int((y - self.origin[1]) / self.resolution)
        return r, c


def _maxpool(a, k):
    """Reduz a matriz por blocos kxk usando o maximo (max-pooling).
    Bloco com qualquer celula=1 -> 1. Preserva paredes finas."""
    rows, cols = a.shape
    pr, pc = (-rows) % k, (-cols) % k          # padding p/ multiplo de k
    if pr or pc:
        a = np.pad(a, ((0, pr), (0, pc)), constant_values=0)
    R, C = a.shape
    return a.reshape(R // k, k, C // k, k).max(axis=(1, 3)).astype(np.uint8)


def bresenham(a, b):
    """Gera as celulas de uma reta entre a e b (algoritmo de Bresenham)."""
    r0, c0 = a
    r1, c1 = b
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    while True:
        yield (r0, c0)
        if (r0, c0) == (r1, c1):
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r0 += sr
        if e2 < dr:
            err += dr
            c0 += sc


if __name__ == "__main__":
    m = GridMap.empty(20, 30)
    m.grid[5:15, 10] = 1          # parede vertical
    m.grid[5, 10:20] = 1          # parede horizontal
    m.inflate(robot_radius_cells=1)
    print("shape:", m.shape)
    print("livre (2,2)?    ", m.is_free((2, 2)))
    print("livre (5,10)?   ", m.is_free((5, 10)))
    print("vizinhos (2,2): ", list(m.neighbors((2, 2))))
    print("LOS (2,2)->(2,25)?", m.line_of_sight((2, 2), (2, 25)))
    print("LOS (2,2)->(18,12)?", m.line_of_sight((2, 2), (18, 12)))
