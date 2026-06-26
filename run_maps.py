"""
run_maps.py
Compara A* (Tarefa 1), GVD (Tarefa 2) e RRT (Tarefa 3) em todos os mapas
.png de maps/. Gera uma figura por mapa (A* | GVD | RRT) em maps_out/ e
imprime um resumo.

Uso:
    python run_maps.py                 # roda todos
    python run_maps.py cave paredes    # roda so os mapas citados
"""
import os, sys, glob, time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "core")
sys.path.insert(0, "planners")
from grid_map import GridMap
import viz
import astar as A
import brushfire_gvd as G
import rrt as R

MAX_DIM = 200          # lado maior do grid apos downsample
ROBOT_RADIUS = 1       # raio do robo em celulas
OUT = "maps_out"

# start/goal por mapa, em FRACOES do grid (linha, coluna) -> robusto a resolucao.
ENDPOINTS = {
    "circular_maze": ((0.04, 0.04), (0.50, 0.50)),   # canto -> centro do labirinto
}

# parametros do RRT por mapa (labirintos pedem passo menor / mais iteracoes).
RRT_DEFAULT = dict(step=5.0, max_iter=15000, goal_bias=0.10, seed=1)
RRT_PARAMS = {
    "square_maze":   dict(step=2.5, max_iter=60000, goal_bias=0.10, seed=3),
    "circular_maze": dict(step=4.0, max_iter=8000,  goal_bias=0.15, seed=1),
}


def rrt_params(name):
    p = dict(RRT_DEFAULT)
    p.update(RRT_PARAMS.get(name, {}))
    return p


# resolucao minima por mapa: alguns labirintos so ficam conexos (e com o
# centro alcancavel) acima de certa resolucao. circular_maze precisa de >=160,
# senao a inflacao fecha os corredores e o centro vira um bolsao isolado.
MAP_MAXDIM = {"circular_maze": 160}


def map_maxdim(name, requested):
    return max(requested, MAP_MAXDIM.get(name, 0))


def find_maps(root="maps"):
    files = []
    for f in glob.glob(os.path.join(root, "**", "*.png"), recursive=True):
        if "textura" in f.lower():
            continue
        files.append(f)
    return sorted(files)


def _nearest_in(cells, target):
    d = (cells[:, 0] - target[0]) ** 2 + (cells[:, 1] - target[1]) ** 2
    r, c = cells[d.argmin()]
    return (int(r), int(c))


def pick_endpoints(gmap, name=None):
    """start/goal dentro da MAIOR componente livre conexa (garante solucao A*).
    Se houver override em ENDPOINTS[name], usa as fracoes dadas."""
    from scipy.ndimage import label as cclabel
    rows, cols = gmap.shape
    free = (gmap.inflated == 0).astype(int)
    lab, nf = cclabel(free, structure=np.ones((3, 3), dtype=int))
    if nf == 0:
        return (1, 1), (rows - 2, cols - 2)
    sizes = np.bincount(lab.ravel()); sizes[0] = 0
    big = int(sizes.argmax())
    cells = np.argwhere(lab == big)

    if name in ENDPOINTS:
        (sr, sc), (gr, gc) = ENDPOINTS[name]
        return (_nearest_in(cells, (sr * rows, sc * cols)),
                _nearest_in(cells, (gr * rows, gc * cols)))

    (r0, c0), (r1, c1) = cells.min(0), cells.max(0)
    return _nearest_in(cells, (r0, c0)), _nearest_in(cells, (r1, c1))


def run_one(f):
    name = os.path.splitext(os.path.basename(f))[0]
    gmap = GridMap.from_png(f, max_dim=MAX_DIM).inflate(ROBOT_RADIUS)
    s, g = pick_endpoints(gmap, name)

    t = time.time(); pa, explored = A.astar(gmap, s, g);   ta = time.time() - t
    t = time.time(); pg, ginfo = G.plan(gmap, s, g);       tg = time.time() - t
    t = time.time(); pr_, rinfo = R.rrt(gmap, s, g, **rrt_params(name))
    tr = time.time() - t

    fig, axes = plt.subplots(1, 3, figsize=(20, 6.5))
    viz.show_map(gmap, path=pa, start=s, goal=g, extra=explored,
                 title=f"A*  -  {name}", ax=axes[0])
    viz.show_map(gmap, path=pg, start=s, goal=g, extra=ginfo["gvd"],
                 title=f"GVD  -  {name}", ax=axes[1])
    viz.show_map(gmap, path=pr_, start=s, goal=g, tree=rinfo["edges"],
                 title=f"RRT  -  {name}", ax=axes[2])
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f"{name}.png"), dpi=85, bbox_inches="tight")
    plt.close(fig)

    return dict(name=name, shape=gmap.shape,
                la=A.path_length(pa) if pa else None,
                lg=A.path_length(pg) if pg else None,
                lr=R.path_length(pr_) if pr_ else None,
                ta=ta, tg=tg, tr=tr)


def main():
    os.makedirs(OUT, exist_ok=True)
    wanted = [a.lower() for a in sys.argv[1:]]
    maps = find_maps()
    if wanted:
        maps = [m for m in maps
                if any(w in os.path.basename(m).lower() for w in wanted)]
    print(f"{len(maps)} mapa(s)\n")
    hdr = (f"{'mapa':16s}{'grid':>11s}{'A*':>7s}{'GVD':>7s}{'RRT':>7s}"
           f"{'tA*':>7s}{'tGVD':>7s}{'tRRT':>8s}")
    print(hdr); print("-" * len(hdr))

    def fmt(x):
        return f"{x:.0f}" if x else "-"

    for f in maps:
        r = run_one(f)
        print(f"{r['name']:16s}{str(r['shape']):>11s}"
              f"{fmt(r['la']):>7s}{fmt(r['lg']):>7s}{fmt(r['lr']):>7s}"
              f"{r['ta']:>6.2f}s{r['tg']:>6.2f}s{r['tr']:>7.2f}s")
    print(f"\n(comprimentos do caminho; '-' = sem solucao)  figuras em {OUT}/")


if __name__ == "__main__":
    main()
