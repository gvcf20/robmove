"""
planners/rrt.py
Tarefa 3: RRT (Rapidly-exploring Random Tree) - planejador por amostragem.

Trabalha no espaco CONTINUO (linha, coluna como floats), nao preso ao grid.
  - amostra um ponto livre (com goal bias)
  - acha o no mais proximo da arvore
  - 'steer': avanca um passo fixo na direcao da amostra
  - checa colisao do SEGMENTO (amostrando finamente contra o grid inflado)
  - ao chegar perto do goal com visada livre, conecta e termina
Pos-processamento: suavizacao por line-of-sight (atalhos).
"""
from __future__ import annotations
import random
import numpy as np


# ---------- utilidades geometricas ----------
def _seg_free(gmap, a, b, inflated=True, res=0.5):
    """True se o segmento continuo a->b nao cruza obstaculo
    (amostra pontos a cada ~res celulas e testa is_free)."""
    ar, ac = a
    br, bc = b
    d = float(np.hypot(br - ar, bc - ac))
    n = max(1, int(d / res))
    for i in range(n + 1):
        t = i / n
        r = int(round(ar + t * (br - ar)))
        c = int(round(ac + t * (bc - ac)))
        if not gmap.is_free((r, c), inflated):
            return False
    return True


def _sample(gmap, rng, inflated=True):
    """Amostra um ponto livre uniformemente no mapa."""
    rows, cols = gmap.shape
    while True:
        r = rng.uniform(0, rows - 1)
        c = rng.uniform(0, cols - 1)
        if gmap.is_free((int(round(r)), int(round(c))), inflated):
            return (r, c)


def _nearest(arr, p):
    """Indice do no mais proximo (busca linear vetorizada)."""
    d = (arr[:, 0] - p[0]) ** 2 + (arr[:, 1] - p[1]) ** 2
    return int(d.argmin())


def _steer(a, b, step):
    """Ponto a `step` de distancia de a na direcao de b (ou b, se mais perto)."""
    ar, ac = a
    br, bc = b
    d = float(np.hypot(br - ar, bc - ac))
    if d <= step or d == 0:
        return (br, bc)
    t = step / d
    return (ar + t * (br - ar), ac + t * (bc - ac))


def path_length(path):
    if not path or len(path) < 2:
        return 0.0
    return float(sum(np.hypot(path[k + 1][0] - path[k][0],
                              path[k + 1][1] - path[k][1])
                     for k in range(len(path) - 1)))


# ---------- RRT ----------
def rrt(gmap, start, goal, step=6.0, goal_bias=0.10, goal_tol=None,
        max_iter=10000, seed=0, inflated=True):
    """Retorna (caminho, info). info traz 'edges' (arestas da arvore),
    'nodes', 'iters'. caminho=None se nao achar dentro de max_iter."""
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    if not gmap.is_free((int(round(start[0])), int(round(start[1]))), inflated):
        raise ValueError(f"start {start} esta em obstaculo/fora do mapa")
    if not gmap.is_free((int(round(goal[0])), int(round(goal[1]))), inflated):
        raise ValueError(f"goal {goal} esta em obstaculo/fora do mapa")
    if goal_tol is None:
        goal_tol = step

    rng = random.Random(seed)
    nodes = [start]
    parent = [-1]
    edges = []
    info = {"nodes": nodes, "parent": parent, "edges": edges, "iters": 0}

    for it in range(max_iter):
        info["iters"] = it + 1
        samp = goal if rng.random() < goal_bias else _sample(gmap, rng, inflated)
        arr = np.asarray(nodes)
        ni = _nearest(arr, samp)
        nn = nodes[ni]
        new = _steer(nn, samp, step)
        if not _seg_free(gmap, nn, new, inflated):
            continue
        nodes.append(new)
        parent.append(ni)
        edges.append((nn, new))
        new_idx = len(nodes) - 1

        # tenta conectar ao goal
        if np.hypot(new[0] - goal[0], new[1] - goal[1]) <= goal_tol \
           and _seg_free(gmap, new, goal, inflated):
            nodes.append(goal)
            parent.append(new_idx)
            edges.append((new, goal))
            return _reconstruct(nodes, parent), info
    return None, info


def _reconstruct(nodes, parent):
    path, i = [], len(nodes) - 1
    while i != -1:
        path.append(nodes[i])
        i = parent[i]
    path.reverse()
    return path


def smooth(gmap, path, inflated=True):
    """Suavizacao por line-of-sight: pula nos intermediarios com visada livre."""
    if not path or len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _seg_free(gmap, path[i], path[j], inflated):
            j -= 1
        out.append(path[j])
        i = j
    return out


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "core"))
    from grid_map import GridMap
    from viz import show_map

    m = GridMap.empty(40, 60)
    m.grid[8:32, 20] = 1
    m.grid[8, 20:40] = 1
    m.grid[24:40, 40] = 1
    m.inflate(1)

    start, goal = (4, 4), (36, 54)
    path, info = rrt(m, start, goal, step=4.0, seed=1)
    print("iteracoes:", info["iters"], "| nos:", len(info["nodes"]),
          "| caminho:", len(path) if path else None)
    if path:
        sm = smooth(m, path)
        print("comprimento cru:", round(path_length(path), 1),
              "| suavizado:", round(path_length(sm), 1))
        ax = show_map(m, path=path, start=start, goal=goal, tree=info["edges"],
                      title="RRT (cinza=arvore, vermelho=caminho)")
        pr = [p[0] for p in sm]; pc = [p[1] for p in sm]
        ax.plot(pc, pr, "-", color="#2ecc71", linewidth=2, zorder=4)
        ax.figure.savefig("_rrt.png", dpi=90, bbox_inches="tight")
        print("figura salva: _rrt.png")
