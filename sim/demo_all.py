"""
sim/demo_all.py
Roda os 3 planejadores (A*, GVD, RRT) no MESMO mapa, em sequencia, no
CoppeliaSim: para cada um, ANIMA a fase de busca e depois navega o robo.
ABRA o CoppeliaSim (cena vazia) antes.

Uso (da raiz):
    python sim/demo_all.py                  # mapa 'cave'
    python sim/demo_all.py square_maze 120
"""
import sys, os, time
sys.path.insert(0, ".")
sys.path.insert(0, "core")
sys.path.insert(0, "planners")
sys.path.insert(0, "sim")

from grid_map import GridMap
import run_maps as RM
import coppelia as C
from navigate import plan_path, show_search


def main():
    hint = sys.argv[1].lower() if len(sys.argv) > 1 else "cave"
    max_dim = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    f = next((x for x in RM.find_maps() if hint in os.path.basename(x).lower()),
             None)
    if not f:
        print(f"mapa '{hint}' nao encontrado"); return
    name = os.path.splitext(os.path.basename(f))[0]
    max_dim = RM.map_maxdim(name, max_dim)               # resolucao minima por mapa
    gmap = GridMap.from_png(f, max_dim=max_dim).inflate(RM.ROBOT_RADIUS)
    s, g = RM.pick_endpoints(gmap, name)
    print(f"mapa={name} grid={gmap.shape} start={s} goal={g}\n")

    sim = C.connect()
    world, _ = C.build_scene(sim, gmap, cell=0.1, wall_h=0.4)
    robot_d = 2 * RM.ROBOT_RADIUS * world.cell

    handles = []
    for planner in ["astar", "gvd", "rrt"]:
        print(f"--- {planner.upper()} ---")
        C.clear_drawings(sim, handles); handles = []
        path, kind, data = plan_path(gmap, name, planner, s, g)
        if not path:
            print(f"  {planner}: sem caminho nesse mapa, pulando.\n")
            continue
        print("  fase 1: construcao...")
        handles = show_search(sim, world, kind, data)
        time.sleep(1.0)
        print(f"  fase 2: navegacao ({len(path)} pontos)...")
        C.draw_path(sim, world, path)
        robot = C.spawn_robot(sim, world, path[0], body_d=robot_d)
        C.follow_path(sim, robot, world, path, cell_speed=8.0)
        print("  ok.")
        if planner != "rrt":
            input("  ENTER p/ o proximo planejador...\n")
    print("fim.")

if __name__ == "__main__":
    main()
