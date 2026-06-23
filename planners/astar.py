"""
planners/astar.py
Busca A* sobre o occupancy grid, com 8-conexao.
Retorna (caminho, explorados). Caminho = lista de celulas (linha, coluna).
"""
from __future__ import annotations
import heapq
import numpy as np

SQRT2 = float(np.sqrt(2))


# ---------- heuristicas ----------
def octile(a, b):
    """Admissivel e consistente p/ 8-conexao com custos 1 e sqrt(2)."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (SQRT2 - 2) * min(dr, dc)

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    return float(np.hypot(a[0] - b[0], a[1] - b[1]))

HEURISTICS = {"octile": octile, "manhattan": manhattan, "euclidean": euclidean}


# ---------- A* ----------
def astar(gmap, start, goal, heuristic="octile", inflated=True):
    start, goal = tuple(start), tuple(goal)
    if not gmap.is_free(start, inflated):
        raise ValueError(f"start {start} esta em obstaculo/fora do mapa")
    if not gmap.is_free(goal, inflated):
        raise ValueError(f"goal {goal} esta em obstaculo/fora do mapa")
    h = HEURISTICS[heuristic]

    open_heap = [(h(start, goal), 0.0, start)]   # (f, g, celula)
    came_from = {start: None}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        if current == goal:
            return _reconstruct(came_from, goal), closed
        for nb, step in gmap.neighbors(current, inflated):
            tentative = g + step
            if tentative < g_score.get(nb, float("inf")):
                g_score[nb] = tentative
                came_from[nb] = current
                heapq.heappush(open_heap, (tentative + h(nb, goal), tentative, nb))
    return None, closed   # sem caminho


def _reconstruct(came_from, goal):
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


# ---------- pos-processamento ----------
def smooth(gmap, path, inflated=True):
    """Suavizacao por line-of-sight (string pulling):
    pula waypoints intermediarios quando ha visada livre."""
    if not path or len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not gmap.line_of_sight(path[i], path[j], inflated):
            j -= 1
        out.append(path[j])
        i = j
    return out


def path_length(path):
    if not path or len(path) < 2:
        return 0.0
    return float(sum(np.hypot(path[k + 1][0] - path[k][0],
                              path[k + 1][1] - path[k][1])
                     for k in range(len(path) - 1)))


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
    from grid_map import GridMap
    from viz import show_map

    m = GridMap.empty(20, 30)
    m.grid[5:15, 10] = 1
    m.grid[5, 10:20] = 1
    m.inflate(1)

    start, goal = (2, 2), (12, 25)
    path, explored = astar(m, start, goal)
    sm = smooth(m, path)

    print("explorados:", len(explored))
    print("celulas do caminho:", len(path))
    print("comprimento bruto:   ", round(path_length(path), 2))
    print("comprimento suavizado:", round(path_length(sm), 2))

    ax = show_map(m, path=path, start=start, goal=goal, extra=explored,
                  title="A* (azul=explorado, vermelho=caminho)")
    ax.figure.savefig("_astar.png", dpi=90, bbox_inches="tight")
    print("figura salva: _astar.png")