"""
sim/build_scene.py
Etapa 2: gera a cena (chao + paredes) no CoppeliaSim a partir de um mapa.
ABRA o CoppeliaSim (cena vazia) ANTES de rodar.

Uso (da raiz do projeto):
    python sim/build_scene.py                 # mapa 'paredes', grid leve
    python sim/build_scene.py autolab 120     # outro mapa / resolucao
"""
import sys, os
sys.path.insert(0, "core")
sys.path.insert(0, "sim")
import glob
from grid_map import GridMap
import coppelia as C


def find_map(name_hint):
    for f in sorted(glob.glob("maps/**/*.png", recursive=True)):
        if "textura" in f.lower():
            continue
        if name_hint in os.path.basename(f).lower():
            return f
    return None


def main():
    name = sys.argv[1].lower() if len(sys.argv) > 1 else "paredes"
    max_dim = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    f = find_map(name)
    if not f:
        print(f"mapa '{name}' nao encontrado em maps/"); return
    gmap = GridMap.from_png(f, max_dim=max_dim)
    print(f"mapa: {os.path.basename(f)}  grid={gmap.grid.shape}")

    sim = C.connect()
    print("conectado. construindo cena...")
    world, root = C.build_scene(sim, gmap, cell=0.1, wall_h=0.4)
    print(f"pronto. 1 celula = {world.cell} m  ->  mapa = "
          f"{gmap.grid.shape[1]*world.cell:.1f} x "
          f"{gmap.grid.shape[0]*world.cell:.1f} m na cena.")
    print("Olhe o CoppeliaSim. Pra limpar depois: rode de novo (ele "
          "remove a cena antiga) ou apague o dummy 'Mapa' na arvore de cena.")


if __name__ == "__main__":
    main()
