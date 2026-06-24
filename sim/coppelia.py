"""
sim/coppelia.py
Biblioteca de simulacao: conexao, transformacao grid<->mundo, e geracao
da cena (chao + paredes) a partir de um occupancy grid.

A cena e construida programaticamente p/ casar exatamente com o mapa do
planejador. As paredes vem de uma decomposicao dos obstaculos em retangulos
(poucas formas grandes em vez de uma por celula).

API: CoppeliaSim 4.x ZMQ Remote API.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# ---------- conexao ----------
def connect():
    """Conecta e devolve o objeto `sim`. CoppeliaSim deve estar aberto."""
    client = RemoteAPIClient()
    sim = client.require('sim')
    return sim


# ---------- transformacao grid <-> mundo ----------
@dataclass
class World:
    """Mapeia (linha, coluna) do grid p/ (x, y) do mundo, centrado na origem.
    linha 0 (topo da imagem) -> +y; coluna 0 (esquerda) -> -x."""
    rows: int
    cols: int
    cell: float = 0.1          # metros por celula

    def to_world(self, r, c):
        x = (c - (self.cols - 1) / 2) * self.cell
        y = ((self.rows - 1) / 2 - r) * self.cell
        return float(x), float(y)


# ---------- decomposicao dos obstaculos em retangulos ----------
def rectangles(grid):
    """Cobre os obstaculos (grid==1) com retangulos disjuntos (greedy).
    Retorna lista de (r0, c0, r1, c1) inclusivos."""
    grid = np.asarray(grid)
    rows, cols = grid.shape
    used = np.zeros((rows, cols), dtype=bool)
    rects = []
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] != 1 or used[r, c]:
                continue
            c1 = c
            while c1 + 1 < cols and grid[r, c1 + 1] == 1 and not used[r, c1 + 1]:
                c1 += 1
            r1 = r
            growing = True
            while growing and r1 + 1 < rows:
                for cc in range(c, c1 + 1):
                    if grid[r1 + 1, cc] != 1 or used[r1 + 1, cc]:
                        growing = False
                        break
                if growing:
                    r1 += 1
            used[r:r1 + 1, c:c1 + 1] = True
            rects.append((r, c, r1, c1))
    return rects


# ---------- limpeza ----------
def clear_scene(sim, alias="Mapa"):
    """Remove uma cena de mapa criada anteriormente (dummy 'Mapa' e filhos)."""
    try:
        root = sim.getObject("/" + alias)
    except Exception:
        return False
    handles = sim.getObjectsInTree(root, sim.handle_all, 0)
    if handles:
        sim.removeObjects(handles)
    return True


# ---------- construcao da cena ----------
def build_scene(sim, gmap, cell=0.1, wall_h=0.4, alias="Mapa", verbose=True):
    """Cria chao + paredes a partir de gmap.grid. Retorna (world, root_handle).
    Tudo fica sob um dummy 'Mapa' p/ facilitar a limpeza."""
    clear_scene(sim, alias)
    rows, cols = gmap.grid.shape
    world = World(rows, cols, cell)

    root = sim.createDummy(0.02)
    sim.setObjectAlias(root, alias)
    sim.setObjectPosition(root, [0.0, 0.0, 0.0])

    # chao
    W, H = cols * cell, rows * cell
    floor = sim.createPrimitiveShape(sim.primitiveshape_cuboid,
                                     [W, H, 0.02], 0)
    sim.setObjectAlias(floor, "chao")
    sim.setObjectPosition(floor, [0.0, 0.0, -0.01])
    sim.setShapeColor(floor, None, sim.colorcomponent_ambient_diffuse,
                      [0.92, 0.92, 0.92])
    sim.setObjectInt32Param(floor, sim.shapeintparam_static, 1)
    sim.setObjectParent(floor, root, True)

    # paredes (a partir dos obstaculos REAIS, nao inflados)
    rects = rectangles(gmap.grid)
    if verbose:
        print(f"grid {rows}x{cols} -> {len(rects)} paredes (retangulos). "
              f"criando na cena...")
    for (r0, c0, r1, c1) in rects:
        sx = (c1 - c0 + 1) * cell
        sy = (r1 - r0 + 1) * cell
        cx, cy = world.to_world((r0 + r1) / 2.0, (c0 + c1) / 2.0)
        h = sim.createPrimitiveShape(sim.primitiveshape_cuboid,
                                     [sx, sy, wall_h], 0)
        sim.setObjectPosition(h, [cx, cy, wall_h / 2.0])
        sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse,
                          [0.25, 0.25, 0.28])
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
        sim.setObjectParent(h, root, True)

    if verbose:
        print("cena construida.")
    return world, root


# ========== ETAPA 3: robo + navegacao (seguidor cinematico) ==========
import time
import math


def spawn_robot(sim, world, start_rc, body_d=0.35, body_h=0.20, alias="Robo"):
    """Cria um robo simples (cilindro azul + marcador frontal amarelo) sob um
    dummy 'Robo', posicionado na celula start_rc. Retorna o handle do dummy
    (mova/oriente esse dummy p/ mover o robo)."""
    clear_scene(sim, alias)
    z = body_h / 2.0
    root = sim.createDummy(0.02)
    sim.setObjectAlias(root, alias)

    body = sim.createPrimitiveShape(sim.primitiveshape_cylinder,
                                    [body_d, body_d, body_h], 0)
    sim.setObjectAlias(body, "robo_corpo")
    sim.setShapeColor(body, None, sim.colorcomponent_ambient_diffuse,
                      [0.15, 0.45, 0.85])
    sim.setObjectInt32Param(body, sim.shapeintparam_static, 1)
    sim.setObjectParent(body, root, True)
    sim.setObjectPosition(body, [0.0, 0.0, 0.0], root)

    nose = sim.createPrimitiveShape(sim.primitiveshape_cuboid,
                                    [body_d * 0.5, 0.05, body_h * 1.02], 0)
    sim.setObjectAlias(nose, "robo_frente")
    sim.setShapeColor(nose, None, sim.colorcomponent_ambient_diffuse,
                      [0.95, 0.85, 0.10])
    sim.setObjectInt32Param(nose, sim.shapeintparam_static, 1)
    sim.setObjectParent(nose, root, True)
    sim.setObjectPosition(nose, [body_d * 0.35, 0.0, 0.0], root)   # +x local = frente

    x, y = world.to_world(start_rc[0], start_rc[1])
    sim.setObjectPosition(root, [x, y, z])
    sim.setObjectOrientation(root, [0.0, 0.0, 0.0])
    return root


def draw_path(sim, world, path, z=0.03, step=4, color=(0.10, 0.80, 0.25),
              alias="Caminho"):
    """Desenha o caminho no chao como esferas pequenas (subamostradas)."""
    clear_scene(sim, alias)
    root = sim.createDummy(0.01)
    sim.setObjectAlias(root, alias)
    sim.setObjectPosition(root, [0.0, 0.0, 0.0])
    pts = list(path[::step])
    if pts[-1] != path[-1]:
        pts.append(path[-1])
    for p in pts:
        x, y = world.to_world(p[0], p[1])
        m = sim.createPrimitiveShape(sim.primitiveshape_spheroid,
                                     [0.06, 0.06, 0.06], 0)
        sim.setObjectPosition(m, [x, y, z])
        sim.setShapeColor(m, None, sim.colorcomponent_ambient_diffuse,
                          list(color))
        sim.setObjectInt32Param(m, sim.shapeintparam_static, 1)
        sim.setObjectParent(m, root, True)
    return root


def follow_path(sim, robot, world, path, cell_speed=8.0, dt=0.03, z=0.10):
    """Move o robo (dummy) ao longo do caminho com controle cinematico:
    interpola posicao entre waypoints e orienta na direcao do movimento.
    cell_speed = velocidade em celulas/seg."""
    pts = [world.to_world(p[0], p[1]) for p in path]
    seg_speed = cell_speed * world.cell                    # m/s
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-9:
            continue
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(d / (seg_speed * dt)))
        for i in range(n + 1):
            t = i / n
            sim.setObjectPosition(robot, [x0 + t * (x1 - x0),
                                          y0 + t * (y1 - y0), z])
            sim.setObjectOrientation(robot, [0.0, 0.0, yaw])
            time.sleep(dt)


# ========== visualizacao da FASE DE BUSCA (drawing objects, overlay leve) ==========
def _draw_pts(sim, color, size=4):
    return sim.addDrawingObject(sim.drawing_points, size, 0.0, -1, 200000,
                                list(color))


def _draw_lines(sim, color, width=1):
    return sim.addDrawingObject(sim.drawing_lines, width, 0.0, -1, 200000,
                                list(color))


def clear_drawings(sim, handles):
    for h in handles:
        try:
            sim.removeDrawingObject(h)
        except Exception:
            pass


def show_points(sim, world, cells, color=(0.30, 0.60, 1.0), z=0.02,
                animate=True, per_frame=120, dt=0.02):
    """Desenha celulas como pontos (ex.: explorados do A*, esqueleto do GVD),
    revelando em blocos p/ dar o efeito de 'espalhamento' da busca."""
    h = _draw_pts(sim, color)
    cells = list(cells)
    for i in range(0, len(cells), per_frame):
        for p in cells[i:i + per_frame]:
            x, y = world.to_world(p[0], p[1])
            sim.addDrawingObjectItem(h, [x, y, z])
        if animate:
            time.sleep(dt)
    return h


def show_tree(sim, world, edges, color=(0.35, 0.80, 0.40), z=0.04,
              animate=True, per_frame=60, dt=0.02):
    """Desenha a arvore (lista de arestas (a,b)) crescendo - ex.: RRT."""
    h = _draw_lines(sim, color)
    for i in range(0, len(edges), per_frame):
        for a, b in edges[i:i + per_frame]:
            ax, ay = world.to_world(a[0], a[1])
            bx, by = world.to_world(b[0], b[1])
            sim.addDrawingObjectItem(h, [ax, ay, z, bx, by, z])
        if animate:
            time.sleep(dt)
    return h
