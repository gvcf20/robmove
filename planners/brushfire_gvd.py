"""
planners/brushfire_gvd.py
Tarefa 2: Brushfire (wavefront multi-fonte) -> Diagrama de Voronoi Generalizado.

Pipeline:
  1) rotula os obstaculos em componentes conexas
  2) brushfire: BFS a partir de TODOS os obstaculos, propagando
     distancia + dono (id do obstaculo mais proximo) p/ cada celula livre
  3) GVD = celulas livres na fronteira entre donos diferentes
     (equidistantes a >= 2 obstaculos distintos)
  4) navegacao: entra no GVD, percorre o esqueleto (Dijkstra), sai p/ o goal

Tudo roda sobre o grid INFLADO -> cada celula do GVD ja e segura p/ o robo.
"""
from __future__ import annotations
from collections import deque
import heapq
import numpy as np
from scipy.ndimage import label

SQRT2 = float(np.sqrt(2))
NB8 = [(-1, 0), (1, 0), (0, -1), (0, 1),
       (-1, -1), (-1, 1), (1, -1), (1, 1)]


# ---------- 1 + 2) brushfire ----------
def brushfire(gmap, inflated=True):
    """BFS multi-fonte a partir de todos os obstaculos.
    Retorna (dist, owner, n_obstaculos):
      dist[r,c]  = nº de camadas ate o obstaculo mais proximo
      owner[r,c] = id da componente de obstaculo mais proxima (0 em obstaculo)
    """
    grid = gmap.inflated if inflated else gmap.grid
    rows, cols = grid.shape

    structure = np.ones((3, 3), dtype=int)          # 8-conexao
    labels, n = label(grid, structure=structure)    # 0=livre, 1..n=obstaculos

    dist = np.full((rows, cols), np.inf)
    owner = np.zeros((rows, cols), dtype=int)
    q = deque()

    obs_r, obs_c = np.where(grid == 1)
    for r, c in zip(obs_r, obs_c):
        dist[r, c] = 0
        owner[r, c] = labels[r, c]
        q.append((r, c))

    while q:
        r, c = q.popleft()
        for dr, dc in NB8:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                if dist[nr, nc] == np.inf:
                    dist[nr, nc] = dist[r, c] + 1
                    owner[nr, nc] = owner[r, c]
                    q.append((nr, nc))
    return dist, owner, n


# ---------- 3) extracao do GVD ----------
def extract_gvd(gmap, owner, inflated=True):
    """GVD = celula livre com algum vizinho (8) livre de DONO diferente."""
    grid = gmap.inflated if inflated else gmap.grid
    rows, cols = grid.shape
    gvd = set()
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] != 0:
                continue
            o = owner[r, c]
            for dr, dc in NB8:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                    if owner[nr, nc] != o:
                        gvd.add((r, c))
                        break
    return gvd


# ---------- 4) navegacao ----------
def _bfs_to_gvd(gmap, gvd, cell, inflated=True):
    """BFS no espaco livre ate a celula do GVD mais proxima.
    Retorna (gvd_cell, caminho cell->gvd_cell)."""
    grid = gmap.inflated if inflated else gmap.grid
    rows, cols = grid.shape
    cell = tuple(cell)
    prev = {cell: None}
    q = deque([cell])
    while q:
        u = q.popleft()
        if u in gvd:
            path = [u]
            while prev[path[-1]] is not None:
                path.append(prev[path[-1]])
            path.reverse()
            return u, path
        ur, uc = u
        for dr, dc in NB8:
            v = (ur + dr, uc + dc)
            if 0 <= v[0] < rows and 0 <= v[1] < cols \
               and grid[v[0], v[1]] == 0 and v not in prev:
                prev[v] = u
                q.append(v)
    return None, None


def _dijkstra_gvd(gvd, start, goal):
    """Menor caminho sobre o grafo do GVD (vizinhos 8 que tambem sao GVD)."""
    if start == goal:
        return [start]
    heap = [(0.0, start)]
    dist = {start: 0.0}
    prev = {start: None}
    while heap:
        d, u = heapq.heappop(heap)
        if u == goal:
            break
        if d > dist.get(u, float("inf")):
            continue
        ur, uc = u
        for dr, dc in NB8:
            v = (ur + dr, uc + dc)
            if v not in gvd:
                continue
            nd = d + (SQRT2 if dr and dc else 1.0)
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    if goal not in prev:
        return None
    path = [goal]
    while prev[path[-1]] is not None:
        path.append(prev[path[-1]])
    path.reverse()
    return path


def _dedup(path):
    out = []
    for c in path:
        if not out or out[-1] != c:
            out.append(c)
    return out


def plan(gmap, start, goal, inflated=True):
    """Planeja start->goal pelo GVD. Retorna (caminho, info)."""
    dist, owner, n = brushfire(gmap, inflated)
    gvd = extract_gvd(gmap, owner, inflated)
    info = {"gvd": gvd, "dist": dist, "owner": owner, "n_obst": n,
            "enter": None, "exit": None}
    if not gvd:
        return None, info

    enter, p_in = _bfs_to_gvd(gmap, gvd, start, inflated)
    exit_, p_out = _bfs_to_gvd(gmap, gvd, goal, inflated)
    info["enter"], info["exit"] = enter, exit_
    if enter is None or exit_ is None:
        return None, info

    mid = _dijkstra_gvd(gvd, enter, exit_)
    if mid is None:
        return None, info     # GVD desconexo entre entrada e saida

    path = _dedup(p_in + mid[1:] + list(reversed(p_out))[1:])
    return path, info


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
    from grid_map import GridMap
    from viz import show_map

    # mapa com 3 obstaculos distintos -> GVD com ramificacao
    m = GridMap.empty(25, 35)
    m.grid[6:18, 12] = 1            # parede vertical
    m.grid[6, 12:24] = 1           # parede horizontal (forma um L)
    m.grid[14:20, 24:28] = 1       # bloco solto
    m.inflate(1)

    start, goal = (3, 3), (21, 31)
    path, info = plan(m, start, goal)

    print("componentes de obstaculo:", info["n_obst"])
    print("celulas no GVD:", len(info["gvd"]))
    print("entrada no GVD:", info["enter"], " saida:", info["exit"])
    print("celulas do caminho:", len(path) if path else None)

    ax = show_map(m, path=path, start=start, goal=goal, extra=info["gvd"],
                  title="GVD (azul=esqueleto, vermelho=caminho)")
    ax.figure.savefig("_gvd.png", dpi=90, bbox_inches="tight")
    print("figura salva: _gvd.png")