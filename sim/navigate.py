"""
sim/navigate.py
Etapa 3: planeja, MOSTRA a fase de busca (explorados/GVD/arvore) e navega
o robo no CoppeliaSim seguindo o plano.
ABRA o CoppeliaSim (cena vazia) ANTES de rodar.

Uso (da raiz):
    python sim/navigate.py                      # paredes, A*
    python sim/navigate.py autolab gvd
    python sim/navigate.py square_maze rrt 120
planejadores: astar | gvd | rrt
"""
import sys, os, time
sys.path.insert(0, ".")
sys.path.insert(0, "core")
sys.path.insert(0, "planners")
sys.path.insert(0, "sim")

from grid_map import GridMap
import astar as A
import brushfire_gvd as G
import rrt as R
import run_maps as RM
import coppelia as C


def plan_path(gmap, name, planner, s, g):
    """Retorna (caminho, tipo, dados_da_busca). dados serve p/ visualizar
    a fase de construcao de cada algoritmo."""
    if planner == "astar":
        path, explored = A.astar(gmap, s, g)
        sm = A.smooth(gmap, path) if path else None
        ex = sorted(explored, key=lambda c: (c[0]-s[0])**2 + (c[1]-s[1])**2)
        return sm, "astar", {"cells": ex}
    if planner == "gvd":
        path, info = G.plan(gmap, s, g)
        gvd = sorted(info["gvd"], key=lambda c: (c[0]-s[0])**2 + (c[1]-s[1])**2)
        return path, "gvd", {"cells": gvd}
    if planner == "rrt":
        path, info = R.rrt(gmap, s, g, **RM.rrt_params(name))
        sm = R.smooth(gmap, path) if path else None
        return sm, "rrt", {"edges": info["edges"]}
    raise ValueError(f"planejador desconhecido: {planner}")


def show_search(sim, world, kind, data):
    """Anima a fase de busca; retorna handles de desenho p/ limpar depois."""
    if kind == "astar":
        return [C.show_points(sim, world, data["cells"],
                              color=(0.45, 0.65, 1.0))]
    if kind == "gvd":
        return [C.show_points(sim, world, data["cells"],
                              color=(0.20, 0.55, 0.95))]
    if kind == "rrt":
        return [C.show_tree(sim, world, data["edges"],
                            color=(0.30, 0.80, 0.40))]
    return []


def main():
    hint = sys.argv[1].lower() if len(sys.argv) > 1 else "paredes"
    planner = sys.argv[2].lower() if len(sys.argv) > 2 else "astar"
    max_dim = int(sys.argv[3]) if len(sys.argv) > 3 else 100

    f = next((x for x in RM.find_maps() if hint in os.path.basename(x).lower()),
             None)
    if not f:
        print(f"mapa '{hint}' nao encontrado"); return
    name = os.path.splitext(os.path.basename(f))[0]
    max_dim = RM.map_maxdim(name, max_dim)               # resolucao minima por mapa
    gmap = GridMap.from_png(f, max_dim=max_dim).inflate(RM.ROBOT_RADIUS)
    s, g = RM.pick_endpoints(gmap, name)
    print(f"mapa={name} grid={gmap.shape} planner={planner} start={s} goal={g}")

    path, kind, data = plan_path(gmap, name, planner, s, g)
    if not path:
        print("nenhum caminho encontrado p/ esse planejador/mapa."); return

    sim = C.connect()
    world, _ = C.build_scene(sim, gmap, cell=0.1, wall_h=0.4)

    print("fase 1: construcao do algoritmo (olhe o CoppeliaSim)...")
    show_search(sim, world, kind, data)
    time.sleep(1.0)

    print(f"fase 2: navegacao do caminho ({len(path)} pontos)...")
    C.draw_path(sim, world, path)
    robot_d = 2 * RM.ROBOT_RADIUS * world.cell
    robot = C.spawn_robot(sim, world, path[0], body_d=robot_d)
    C.follow_path(sim, robot, world, path, cell_speed=8.0)
    print("chegou ao goal.")


if __name__ == "__main__":
    main()
