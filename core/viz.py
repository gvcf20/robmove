"""
core/viz.py
Visualizacao do grid e animacao do robo seguindo o caminho (matplotlib).
"""
from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap


def _draw_base(gmap, ax, show_inflated=True, show_grid=None):
    """Desenha obstaculos originais (escuro), a 'casca' inflada (cinza claro)
    e a malha do grid (linhas das celulas)."""
    rows, cols = gmap.shape
    img = np.zeros((rows, cols))
    if show_inflated:
        img[gmap.inflated == 1] = 1   # cinza = zona inflada
    img[gmap.grid == 1] = 2           # preto = obstaculo real
    cmap = ListedColormap(["white", "#c9c9c9", "#222222"])
    ax.imshow(img, cmap=cmap, origin="upper", interpolation="none")

    # malha do grid: linhas nas bordas das celulas (meio-inteiros).
    # auto-desabilita se o grid for grande demais (vira borrao).
    if show_grid is None:
        show_grid = max(rows, cols) <= 60
    if show_grid:
        for x in np.arange(-0.5, cols, 1):
            ax.axvline(x, color="#b8b8b8", linewidth=0.5, zorder=1)
        for y in np.arange(-0.5, rows, 1):
            ax.axhline(y, color="#b8b8b8", linewidth=0.5, zorder=1)

    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)      # y invertido p/ casar com origin='upper'
    ax.set_aspect("equal")


def show_map(gmap, path=None, start=None, goal=None, extra=None, tree=None,
             title="", show_inflated=True, show_grid=None, ax=None):
    """Plot estatico do mapa, com caminho/start/goal opcionais.
    `extra`: celulas extra p/ destacar (ex.: GVD).
    `tree` : lista de arestas (a, b) de pontos p/ desenhar (ex.: arvore RRT)."""
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 6))
    _draw_base(gmap, ax, show_inflated, show_grid)

    if tree:
        from matplotlib.collections import LineCollection
        segs = [[(a[1], a[0]), (b[1], b[0])] for a, b in tree]
        ax.add_collection(LineCollection(segs, colors="#5dade2",
                                         linewidths=0.4, zorder=1.5))

    if extra:
        er = [c[0] for c in extra]; ec = [c[1] for c in extra]
        ax.scatter(ec, er, s=4, c="#4aa3ff", alpha=0.6, zorder=2)

    if path:
        pr = [c[0] for c in path]; pc = [c[1] for c in path]
        ax.plot(pc, pr, "-", color="#ff5252", linewidth=2, zorder=3)

    if start is not None:
        ax.scatter([start[1]], [start[0]], s=120, c="#2ecc71",
                   edgecolors="black", zorder=4, label="start")
    if goal is not None:
        ax.scatter([goal[1]], [goal[0]], s=120, c="#e67e22", marker="*",
                   edgecolors="black", zorder=4, label="goal")

    if title:
        ax.set_title(title)
    if start is not None or goal is not None:
        ax.legend(loc="upper right", fontsize=8)
    return ax


def animate_path(gmap, path, start=None, goal=None, interval=80,
                 title="", save=None):
    """Anima um robo (ponto) seguindo o caminho. Simulador cinematico simples.
    Se `save` for um caminho .gif/.mp4, salva o arquivo; senao mostra na tela."""
    fig, ax = plt.subplots(figsize=(8, 6))
    _draw_base(gmap, ax)
    pr = [c[0] for c in path]; pc = [c[1] for c in path]
    ax.plot(pc, pr, "-", color="#ff5252", linewidth=1.5, alpha=0.5, zorder=2)
    if start is not None:
        ax.scatter([start[1]], [start[0]], s=120, c="#2ecc71",
                   edgecolors="black", zorder=3)
    if goal is not None:
        ax.scatter([goal[1]], [goal[0]], s=120, c="#e67e22", marker="*",
                   edgecolors="black", zorder=3)
    robot = ax.scatter([], [], s=160, c="#3498db", edgecolors="black", zorder=5)
    if title:
        ax.set_title(title)

    def update(i):
        robot.set_offsets([[pc[i], pr[i]]])
        return (robot,)

    anim = FuncAnimation(fig, update, frames=len(path),
                         interval=interval, blit=True, repeat=False)
    if save:
        anim.save(save, writer="pillow" if save.endswith(".gif") else "ffmpeg")
        print(f"animacao salva em {save}")
    else:
        plt.show()
    return anim


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")   # so p/ teste sem tela
    from grid_map import GridMap
    m = GridMap.empty(20, 30)
    m.grid[5:15, 10] = 1
    m.grid[5, 10:20] = 1
    m.inflate(1)
    fake_path = [(r, 2) for r in range(2, 18)] + [(17, c) for c in range(2, 26)]
    ax = show_map(m, path=fake_path, start=(2, 2), goal=(17, 25),
                  title="teste grid + path falso")
    ax.figure.savefig("_teste_viz.png", dpi=90, bbox_inches="tight")
    print("figura salva: _teste_viz.png")