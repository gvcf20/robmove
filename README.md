# Planejamento de Movimento de Robôs — Trabalho Prático

Implementação e comparação de **três paradigmas de planejamento de caminho** em
ambientes discretizados na forma de *occupancy grid*, com navegação de um robô no
simulador **CoppeliaSim**:

1. **A\*** — busca informada em grafo (Tarefa 1)
2. **Brushfire + GVD** — Diagrama de Voronoi Generalizado (Tarefa 2)
3. **RRT** — planejamento por amostragem (Tarefa 3)

Os três rodam sobre os mesmos mapas (imagens PNG), com o mesmo par start/goal, o
que permite uma comparação direta. Há ainda um teste de robustez e a navegação
3D no CoppeliaSim com visualização da fase de construção de cada algoritmo.

---

## 1. Algoritmos implementados

### Tarefa 1 — A\* (`planners/astar.py`)

Busca em grafo sobre o grid em **8-conexão**, com custo `1.0` para movimentos
retos e `√2` para diagonais. Usa a **heurística octile**, que é admissível e
consistente para 8-conexão (garante caminho ótimo). A fila de prioridade é um
`heapq`; um `closed set` evita reexpansões. Após encontrar o caminho, há uma
**suavização por line-of-sight** (*string pulling*) que remove o zigue-zague das
diagonais. Heurísticas alternativas (`manhattan`, `euclidean`) ficam disponíveis
para comparação.

- *Pontos fortes:* completo e ótimo na resolução do grid.
- *Limitações:* custo de memória/tempo cresce com a resolução; caminho preso às
  8 direções (mitigado pela suavização).

### Tarefa 2 — Brushfire + GVD (`planners/brushfire_gvd.py`)

Constrói o **Diagrama de Voronoi Generalizado** a partir de um **brushfire
exato** (propagação tipo Dijkstra a partir de *todos* os obstáculos), que dá a
cada célula livre a distância euclidiana ao obstáculo mais próximo e o seu
**ponto-base** (a coordenada desse obstáculo). O **GVD** é o conjunto de células
cujo vizinho tem ponto-base distante (eixo medial) ou dono diferente
(equidistância a dois obstáculos). A navegação tem três trechos: entra no GVD
(BFS), percorre o esqueleto (Dijkstra sobre o grafo do GVD) e sai para o goal.
Tudo roda sobre o grid **inflado**, então toda célula do esqueleto já é segura
para o robô.

- *Pontos fortes:* caminho de **máxima folga** (mais seguro, longe das paredes).
- *Limitações:* mais longo que o A\*; sensível à resolução; em passagens
  estreitas o eixo medial **fragmenta** e a rede principal pode não conectar
  start e goal (ver mapa `icex-terceiro`).

### Tarefa 3 — RRT (`planners/rrt.py`)

*Rapidly-exploring Random Tree* no espaço **contínuo** (não preso ao grid).
A cada iteração: amostra um ponto livre (com **goal bias**), acha o nó mais
próximo da árvore, dá um passo de tamanho fixo na direção dele (*steer*), checa
colisão do **segmento** contra o grid inflado e, ao chegar perto do goal com
visada livre, conecta e termina. O caminho final passa por **suavização**
line-of-sight.

- *Pontos fortes:* simples, rápido em espaços abertos, não sofre com alta
  dimensionalidade.
- *Limitações:* probabilisticamente completo (não ótimo); caminho irregular;
  **falha em passagens estreitas** (*narrow passage problem*) — não resolve os
  labirintos densos (`square_maze` exige passo pequeno; `circular_maze` não
  resolve).

---

## 2. Estrutura do projeto

```
robmove/
├── core/
│   ├── grid_map.py        # occupancy grid: carga de PNG (com downsample),
│   │                      #   inflação de obstáculos, colisão, mundo<->grid
│   └── viz.py             # matplotlib: mapa+grade, caminho, árvore, animação
├── planners/
│   ├── astar.py           # Tarefa 1 — A* + heurísticas + suavização
│   ├── brushfire_gvd.py   # Tarefa 2 — Brushfire + GVD + navegação no esqueleto
│   └── rrt.py             # Tarefa 3 — RRT + suavização
├── sim/
│   ├── coppelia.py        # biblioteca: conexão ZMQ, cena (chão+paredes),
│   │                      #   robô, seguidor de caminho, fase de busca
│   ├── test_connection.py # teste de conexão Python <-> CoppeliaSim
│   ├── build_scene.py     # gera só a cena (chão + paredes) de um mapa
│   ├── navigate.py        # planeja + mostra a busca + navega (1 planejador)
│   └── demo_all.py        # roda A*, GVD e RRT em sequência no mesmo mapa
├── maps/                  # mapas .png (e texturas/ usadas pelo simulador)
├── run_maps.py            # compara A*|GVD|RRT em todos os mapas -> maps_out/
├── stress_test.py         # robustez: start/goal aleatórios e inválidos
├── requirements.txt
├── README.md
└── leiame.txt
```

---

## 3. Instalação

Requer **Python 3.10+**. Da raiz do projeto:

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Para a parte de simulação, instale também o cliente do CoppeliaSim e tenha o
**CoppeliaSim 4.x** instalado (no macOS Apple Silicon há build nativo):

```bash
pip install coppeliasim-zmqremoteapi-client pyzmq
```

---

## 4. Reprodução dos resultados

### 4.1 Comparação offline (sem simulador)

Roda os três planejadores em todos os mapas e gera uma figura por mapa
(`A* | GVD | RRT` lado a lado) em `maps_out/`, além de uma tabela com
comprimentos de caminho e tempos:

```bash
python run_maps.py                 # todos os mapas
python run_maps.py cave square     # apenas mapas cujo nome contém esses termos
```

### 4.2 Teste de robustez

Sorteia start/goal aleatórios (incluindo pontos dentro de obstáculos) e mostra
que os planejadores falham de forma controlada (erro tratado) em vez de quebrar:

```bash
python stress_test.py cave 12      # mapa 'cave', 12 tentativas
```

### 4.3 Simulação no CoppeliaSim

1. Abra o **CoppeliaSim** com uma **cena vazia**.
2. Teste a conexão:
   ```bash
   python sim/test_connection.py
   ```
3. Navegue um planejador num mapa (mostra a fase de construção e depois o robô):
   ```bash
   python sim/navigate.py paredes astar
   python sim/navigate.py square_maze gvd 120
   python sim/navigate.py cave rrt
   ```
   Sintaxe: `python sim/navigate.py <mapa> <astar|gvd|rrt> [max_dim]`
4. Rode os três em sequência no mesmo mapa (com pausa entre eles):
   ```bash
   python sim/demo_all.py square_maze 120
   ```

Em cada execução do simulador a navegação tem **duas fases**: (1) a construção do
algoritmo é animada — A\* espalhando os nós explorados, GVD traçando o esqueleto,
RRT crescendo a árvore — e (2) o robô percorre o caminho planejado.

---

## 5. Resultados (resumo)

Mapas testados em grid ~200 (offline) e ~100–120 (simulação). `len` = comprimento
do caminho em células.

| mapa           | A\* (len) | GVD (len) | RRT (len) |
|----------------|:---------:|:---------:|:---------:|
| autolab        |    200    |    220    |    221    |
| cave           |    237    |    278    |    249    |
| paredes        |    209    |    233    |    225    |
| square_maze    |    434    |    485    |    536    |
| circular_maze  |    568    |    625    |  falha\*  |
| icex-terceiro  |    108    |  falha\*\* |    117    |

\* RRT não resolve labirintos densos (*narrow passage*).
\*\* GVD: eixo medial fragmenta em portas estreitas; a rede principal não conecta
start e goal (mesmo o espaço livre sendo conexo — por isso A\* resolve).

**Leitura:** o A\* resolve todos; cada um dos outros dois tem uma fraqueza
demonstrada num mapa real, e onde um falha o outro resolve. O A\* tende ao
caminho mais curto, o GVD ao mais seguro (centralizado), e o RRT é o mais rápido
em espaços abertos.

---

## 6. Observações de implementação

- **Inflação dos obstáculos** pelo raio do robô transforma o robô num ponto e
  simplifica toda a checagem de colisão dos três planejadores.
- **Downsample com max-pooling** (`from_png(max_dim=...)`) reduz mapas de alta
  resolução preservando paredes finas (bloco com qualquer pixel-obstáculo vira
  obstáculo).
- **Cena gerada programaticamente:** as paredes do CoppeliaSim vêm de uma
  decomposição dos obstáculos em poucos retângulos grandes (em vez de um cubo por
  célula), então a cena casa exatamente com o mapa do planejador.