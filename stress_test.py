"""
stress_test.py
Teste de robustez: roda A* e GVD com start/goal ALEATORIOS, incluindo
pontos dentro de obstaculos. Espera-se que os planejadores retornem ERRO
controlado (excecao ValueError tratada) em vez de quebrar ou devolver
caminho invalido. Pontos validos mas desconexos -> SEM_CAMINHO.

Uso:
    python stress_test.py                 # mapa 'cave', 12 trials
    python stress_test.py square 20       # mapa 'square', 20 trials
    python stress_test.py cave 16 42      # ... com seed 42
"""
import os, sys, glob, random
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

MAX_DIM = 200
ROBOT_RADIUS = 1
OUT = "maps_out"


def find_map(name_hint):
    for f in sorted(glob.glob("maps/**/*.png", recursive=True)):
        if "textura" in f.lower():
            continue
        if name_hint is None or name_hint in os.path.basename(f).lower():
            return f
    return None


def random_cell(gmap, free=True):
    """Sorteia uma celula: free=True -> espaco livre; free=False -> obstaculo
    (considerando o grid inflado, igual a validacao dos planejadores)."""
    mask = (gmap.inflated == 0) if free else (gmap.inflated == 1)
    cells = np.argwhere(mask)
    r, c = cells[random.randrange(len(cells))]
    return (int(r), int(c))


def run_planner(fn, gmap, s, g):
    """Roda um planejador capturando excecao e resultado.
    status in {'OK','SEM_CAMINHO','ERRO'}."""
    try:
        out = fn(gmap, s, g)
        path = out[0] if isinstance(out, tuple) else out
        return ("OK", f"{len(path)} cels") if path else ("SEM_CAMINHO", "-")
    except ValueError as e:
        return "ERRO", str(e)


def main():
    args = sys.argv[1:]
    name_hint = args[0].lower() if len(args) >= 1 else "cave"
    n_trials = int(args[1]) if len(args) >= 2 else 12
    seed = int(args[2]) if len(args) >= 3 else 0
    random.seed(seed)

    f = find_map(name_hint)
    if not f:
        print(f"mapa '{name_hint}' nao encontrado em maps/"); return
    gmap = GridMap.from_png(f, max_dim=MAX_DIM).inflate(ROBOT_RADIUS)
    name = os.path.splitext(os.path.basename(f))[0]
    print(f"mapa: {name}  grid={gmap.shape}  trials={n_trials}  seed={seed}\n")

    # metade dos trials totalmente validos; metade com algum ponto em obstaculo
    trials = []
    for i in range(n_trials):
        if i % 2 == 1:                          # trial 'invalido'
            s = random_cell(gmap, free=random.random() < 0.5)
            g = random_cell(gmap, free=False) if gmap.is_free(s) \
                else random_cell(gmap, free=random.random() < 0.5)
        else:                                   # trial 'valido'
            s = random_cell(gmap, free=True)
            g = random_cell(gmap, free=True)
        trials.append((s, g))

    hdr = (f"{'#':>2} {'start':>11} {'goal':>11} {'s_ok':>5} {'g_ok':>5} "
           f"{'A*':>12} {'GVD':>12}")
    print(hdr); print("-" * len(hdr))
    results = []
    for i, (s, g) in enumerate(trials):
        s_ok, g_ok = gmap.is_free(s), gmap.is_free(g)
        a_st, _ = run_planner(A.astar, gmap, s, g)
        g_st, _ = run_planner(G.plan, gmap, s, g)
        results.append((s, g, s_ok, g_ok, a_st, g_st))
        print(f"{i:>2} {str(s):>11} {str(g):>11} "
              f"{'sim' if s_ok else 'NAO':>5} {'sim' if g_ok else 'NAO':>5} "
              f"{a_st:>12} {g_st:>12}")

    n_err = sum(1 for r in results if r[4] == "ERRO")
    n_ok = sum(1 for r in results if r[4] == "OK")
    n_np = n_trials - n_ok - n_err
    print(f"\nA* -> {n_ok} OK | {n_err} ERRO (pontos invalidos) | {n_np} sem caminho")
    print("Em todos os casos invalidos o planejador retornou ERRO tratado, "
          "sem quebrar.")

    # figura: ate 12 trials em grade (verde=ponto valido, X vermelho=invalido)
    show = results[:12]
    ncol, nrow = 4, (len(show) + 3) // 4
    fig, axes = plt.subplots(nrow, ncol, figsize=(3.6 * ncol, 3.6 * nrow))
    axes = np.array(axes).reshape(-1)
    for ax, (s, g, s_ok, g_ok, a_st, g_st) in zip(axes, show):
        path = A.astar(gmap, s, g)[0] if a_st == "OK" else None
        viz.show_map(gmap, path=path, ax=ax, show_grid=False)
        for pt, ok in [(s, s_ok), (g, g_ok)]:
            ax.scatter([pt[1]], [pt[0]], s=70,
                       c=("#2ecc71" if ok else "#e74c3c"),
                       marker=("o" if ok else "X"),
                       edgecolors="black", zorder=5)
        ax.set_title(f"A*: {a_st}", fontsize=9)
    for ax in axes[len(show):]:
        ax.axis("off")
    fig.suptitle(f"Stress test - {name}  (o verde = valido, X vermelho = em obstaculo)",
                 fontsize=12)
    fig.tight_layout()
    os.makedirs(OUT, exist_ok=True)
    out = os.path.join(OUT, f"stress_{name}.png")
    fig.savefig(out, dpi=80, bbox_inches="tight")
    print(f"figura salva: {out}")


if __name__ == "__main__":
    main()
