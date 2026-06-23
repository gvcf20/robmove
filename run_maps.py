"""
run_maps.py
Testa A* (Tarefa 1) e GVD (Tarefa 2) em todos os mapas .png de maps/.
Gera uma figura por mapa (A* | GVD lado a lado) em maps_out/ e imprime resumo.

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

MAX_DIM = 200          # lado maior do grid apos downsample
ROBOT_RADIUS = 1       # raio do robo em celulas
OUT = "maps_out"

# start/goal por mapa, em FRACOES do grid (linha, coluna) -> robusto a resolucao.
# mapas nao listados usam cantos opostos automaticos.
ENDPOINTS = {
    "circular_maze": ((0.04, 0.04), (0.50, 0.50)),   # canto -> centro do labirinto
}


def find_maps(root="maps"):
    files = []
    for f in glob.glob(os.path.join(root, "**", "*.png"), recursive=True):
        if "textura" in f.lower():     # ignora as texturas do CoppeliaSim
            continue
        files.append(f)
    return sorted(files)


def nearest_free(gmap, target):
    """Celula livre (no grid inflado) mais proxima de um alvo."""
    free = np.argwhere(gmap.inflated == 0)
    d = (free[:, 0] - target[0]) ** 2 + (free[:, 1] - target[1]) ** 2
    r, c = free[d.argmin()]
    return (int(r), int(c))


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
        s = _nearest_in(cells, (sr * rows, sc * cols))
        g = _nearest_in(cells, (gr * rows, gc * cols))
        return s, g

    (r0, c0), (r1, c1) = cells.min(0), cells.max(0)
    return _nearest_in(cells, (r0, c0)), _nearest_in(cells, (r1, c1))


def run_one(f):
    name = os.path.splitext(os.path.basename(f))[0]
    gmap = GridMap.from_png(f, max_dim=MAX_DIM).inflate(ROBOT_RADIUS)
    s, g = pick_endpoints(gmap, name)

    t = time.time()
    pa, explored = A.astar(gmap, s, g)
    ta = time.time() - t

    t = time.time()
    pg, info = G.plan(gmap, s, g)
    tg = time.time() - t

    fig, axes = plt.subplots(1, 2, figsize=(15, 6.5))
    viz.show_map(gmap, path=pa, start=s, goal=g, extra=explored,
                 title=f"A*  -  {name}", ax=axes[0])
    viz.show_map(gmap, path=pg, start=s, goal=g, extra=info["gvd"],
                 title=f"GVD  -  {name}", ax=axes[1])
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f"{name}.png"), dpi=85, bbox_inches="tight")
    plt.close(fig)

    la = A.path_length(pa) if pa else None
    lg = A.path_length(pg) if pg else None
    return dict(name=name, shape=gmap.shape, na=len(pa) if pa else 0,
                la=la, gvd=len(info["gvd"]), lg=lg, ta=ta, tg=tg)


def main():
    os.makedirs(OUT, exist_ok=True)
    wanted = [a.lower() for a in sys.argv[1:]]
    maps = find_maps()
    if wanted:
        maps = [m for m in maps
                if any(w in os.path.basename(m).lower() for w in wanted)]
    print(f"{len(maps)} mapa(s)\n")
    hdr = f"{'mapa':18s}{'grid':>11s}{'A*cels':>8s}{'A*len':>8s}{'GVDcels':>9s}{'GVDlen':>8s}{'tA*':>7s}{'tGVD':>7s}"
    print(hdr); print("-" * len(hdr))
    for f in maps:
        r = run_one(f)
        la = f"{r['la']:.0f}" if r['la'] else "-"
        lg = f"{r['lg']:.0f}" if r['lg'] else "-"
        print(f"{r['name']:18s}{str(r['shape']):>11s}{r['na']:>8}{la:>8s}"
              f"{r['gvd']:>9}{lg:>8s}{r['ta']:>6.2f}s{r['tg']:>6.2f}s")
    print(f"\nfiguras salvas em {OUT}/")


if __name__ == "__main__":
    main()
