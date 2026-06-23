"""
planners/brushfire_gvd.py
Tarefa 2: Brushfire (wavefront multi-fonte) -> Diagrama de Voronoi Generalizado.

Pipeline:
  1) rotula os obstaculos em componentes conexas
  2) brushfire: BFS a partir de TODOS os obstaculos, propagando p/ cada
     celula livre a distancia, o dono (componente) e o PONTO-BASE
     (coordenada do obstaculo mais proximo)
  3) GVD = celulas equidistantes a dois "obstaculos" distintos. Marcamos
     onde uma vizinha tem ponto-base distante (eixo medial, funciona mesmo
     com obstaculo unico) OU dono diferente (entre obstaculos separados).
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
    """Brushfire EXATO (propagacao tipo Dijkstra): a partir de todos os
    obstaculos, propaga o ponto-base e mede a distancia euclidiana real
    ate ele. Wavefront preciso -> pontos-base confiaveis p/ o GVD.
    Retorna (dist, owner, basis_r, basis_c, n_obstaculos)."""
    grid = gmap.inflated if inflated else gmap.grid
    rows, cols = grid.shape

    structure = np.ones((3, 3), dtype=int)          # 8-conexao
    labels, n = label(grid, structure=structure)    # 0=livre, 1..n=obstaculos

    dist = np.full((rows, cols), np.inf)
    owner = np.zeros((rows, cols), dtype=int)
    basis_r = np.full((rows, cols), -1, dtype=int)
    basis_c = np.full((rows, cols), -1, dtype=int)
    heap = []

    obs_r, obs_c = np.where(grid == 1)
    for r, c in zip(obs_r, obs_c):
        dist[r, c] = 0.0
        owner[r, c] = labels[r, c]
        basis_r[r, c], basis_c[r, c] = r, c
        heap.append((0.0, r, c))
    heapq.heapify(heap)

    while heap:
        d, r, c = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        bri, bci, own = basis_r[r, c], basis_c[r, c], owner[r, c]
        for dr, dc in NB8:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                nd = float(np.hypot(nr - bri, nc - bci))   # dist euclidiana ao base
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    owner[nr, nc] = own
                    basis_r[nr, nc], basis_c[nr, nc] = bri, bci
                    heapq.heappush(heap, (nd, nr, nc))
    return dist, owner, basis_r, basis_c, n


# ---------- 3) extracao do GVD ----------
def extract_gvd(gmap, dist, owner, basis_r, basis_c, inflated=True,
                basis_gap2=4, min_clear=1.0):
    """GVD = celula livre cuja vizinha (8) tem ponto-base distante (eixo medial)
    ou dono diferente. Com o brushfire exato, isso vira um esqueleto fino.
      basis_gap2 = limiar de distancia^2 entre pontos-base
      min_clear  = folga minima (ignora celulas coladas no obstaculo)."""
    grid = gmap.inflated if inflated else gmap.grid
    rows, cols = grid.shape
    gvd = set()
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] != 0 or dist[r, c] < min_clear:
                continue
            o, bri, bci = owner[r, c], basis_r[r, c], basis_c[r, c]
            for dr, dc in NB8:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols) or grid[nr, nc] != 0:
                    continue
                gap2 = (bri - basis_r[nr, nc]) ** 2 + (bci - basis_c[nr, nc]) ** 2
                if owner[nr, nc] != o or gap2 >= basis_gap2:
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


def _components(cells):
    """Todas as componentes 8-conexas de um conjunto de celulas."""
    cells = set(cells)
    seen, comps = set(), []
    for c in cells:
        if c in seen:
            continue
        comp, stack = [], [c]
        seen.add(c)
        while stack:
            ur, uc = stack.pop()
            comp.append((ur, uc))
            for dr, dc in NB8:
                v = (ur + dr, uc + dc)
                if v in cells and v not in seen:
                    seen.add(v)
                    stack.append(v)
        comps.append(set(comp))
    return comps


def plan(gmap, start, goal, inflated=True):
    """Planeja start->goal pelo GVD. Retorna (caminho, info).
    info['gvd'] = GVD completo (visualizacao); a navegacao usa a maior
    componente conexa do GVD (a rede principal, sem fragmentos-ruido).
    Se start/goal nao alcancarem essa rede (ex.: portas estreitas que
    fragmentam o eixo medial), retorna caminho=None -> limitacao do GVD."""
    start, goal = tuple(start), tuple(goal)
    if not gmap.is_free(start, inflated):
        raise ValueError(f"start {start} esta em obstaculo/fora do mapa")
    if not gmap.is_free(goal, inflated):
        raise ValueError(f"goal {goal} esta em obstaculo/fora do mapa")
    dist, owner, br, bc, n = brushfire(gmap, inflated)
    gvd_full = extract_gvd(gmap, dist, owner, br, bc, inflated)
    comps = _components(gvd_full)
    gvd = max(comps, key=len) if comps else set()
    info = {"gvd": gvd_full, "gvd_nav": gvd, "dist": dist, "owner": owner,
            "n_obst": n, "enter": None, "exit": None}
    if not gvd:
        return None, info

    enter, p_in = _bfs_to_gvd(gmap, gvd, start, inflated)
    exit_, p_out = _bfs_to_gvd(gmap, gvd, goal, inflated)
    info["enter"], info["exit"] = enter, exit_
    if enter is None or exit_ is None:
        return None, info

    mid = _dijkstra_gvd(gvd, enter, exit_)
    if mid is None:
        return None, info

    path = _dedup(p_in + mid[1:] + list(reversed(p_out))[1:])
    return path, info


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
    from grid_map import GridMap
    from viz import show_map

    m = GridMap.empty(25, 35)
    m.grid[6:18, 12] = 1
    m.grid[6, 12:24] = 1
    m.grid[14:20, 24:28] = 1
    m.inflate(1)

    start, goal = (3, 3), (21, 31)
    path, info = plan(m, start, goal)
    print("componentes:", info["n_obst"], "| GVD:", len(info["gvd"]),
          "| caminho:", len(path) if path else None)
    ax = show_map(m, path=path, start=start, goal=goal, extra=info["gvd"],
                  title="GVD (azul=esqueleto, vermelho=caminho)")
    ax.figure.savefig("_gvd.png", dpi=90, bbox_inches="tight")
    print("figura salva: _gvd.png")
