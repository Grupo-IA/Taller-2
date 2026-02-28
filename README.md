# Taller 2 — Restricciones, Adversarios y Drones

Implementación de algoritmos de **CSP (Constraint Satisfaction Problems)** y **Búsqueda Adversaria** aplicados a un simulador de drones de entrega médica. El dron debe planificar rutas de entrega evitando cazadores en un mapa con terreno variado.

---

## Estructura del proyecto

```
Drones/
├── main.py                        # Punto de entrada principal
├── algorithms/
│   ├── csp.py                     # Algoritmos CSP (backtracking, FC, AC-3, MRV+LCV)
│   ├── adversarial.py             # Agentes adversariales (Minimax, AlphaBeta, Expectimax)
│   ├── evaluation.py              # Función de evaluación para estados no terminales
│   ├── problems_csp.py            # Definición del CSP de asignación de drones
│   └── utils.py                   # BFS, Dijkstra, distancia Manhattan (con caché)
├── world/
│   ├── game.py                    # Lógica base del juego (acciones, configuraciones, Grid)
│   ├── game_state.py              # Estado del juego adversarial (dron vs cazadores)
│   ├── layout.py                  # Carga y parseo de layouts (.lay)
│   ├── rules.py                   # Reglas del juego y agentes cazadores
│   └── runner.py                  # Orquestadores de los modos CSP y adversarial
├── view/
│   ├── display.py                 # Interfaces abstractas de visualización
│   ├── graphics_display.py        # Visualización gráfica (Tkinter)
│   ├── graphics_utils.py          # Utilidades de dibujo en canvas
│   └── text_display.py            # Visualización en texto / modo silencioso
└── layouts/
    ├── csp/                       # 22 layouts para el modo CSP
    └── adversarial/               # 24 layouts para el modo adversarial
```

---

## Requisitos

- Python 3.10+
- Tkinter (incluido en la mayoría de distribuciones de Python)
- No requiere dependencias externas adicionales

---

## Ejecución

### Sintaxis general

```bash
python main.py -m <modo> -a <algoritmo> -l <layout> [opciones]
```

### Opciones disponibles

| Opción | Descripción | Default |
|--------|-------------|---------|
| `-m` | Modo de ejecución: `csp` o `adversarial` | *(requerido)* |
| `-a` | Algoritmo CSP o clase de agente adversarial | *(requerido)* |
| `-l` | Nombre del layout a cargar | *(requerido)* |
| `-d` | Profundidad de búsqueda (modo adversarial) | `1` |
| `-p` | Probabilidad de que el cazador actúe aleatoriamente (0.0–1.0) | `0.0` |
| `-n` | Número de partidas a ejecutar | `1` |
| `-t` | Mostrar salida en texto (sin ventana gráfica) | `False` |
| `-q` | Modo silencioso, sin gráficos ni texto | `False` |
| `-z` | Zoom de la ventana gráfica | `1.0` |
| `-x` | Tiempo entre frames (segundos) | `0.1` |

---

## Modo CSP

El problema de asignación de drones se modela como un CSP donde:

- **Variables:** una por cada punto de entrega (`E1, E2, ...Em`)
- **Dominios:** el conjunto de drones disponibles para cada entrega
- **Restricciones:**
  - **Capacidad:** el peso total asignado a un dron no puede superar su capacidad
  - **Batería:** la distancia total de la ruta no puede superar la batería del dron
  - **Ventana de tiempo:** el dron debe llegar al destino dentro del intervalo `[t_early, t_late]`

### Algoritmos CSP

| Algoritmo | Flag `-a` | Descripción |
|-----------|-----------|-------------|
| Backtracking básico | `backtracking` | Búsqueda por retroceso sin optimizaciones |
| Backtracking + Forward Checking | `backtracking_fc` | Elimina valores inconsistentes de vecinos tras cada asignación |
| Backtracking + AC-3 | `backtracking_ac3` | Aplica consistencia de arco antes y durante la búsqueda |
| Backtracking + MRV + LCV | `backtracking_mrv_lcv` | *(Bono)* Heurísticas MRV (mínimos valores restantes) y LCV (valor menos restrictivo) |

### Terreno en mapas CSP

| Carácter | Terreno | Costo de movimiento |
|----------|---------|-------------------|
| `.` / ` ` | Suelo normal | 1 |
| `~` | Zona de niebla | 2 |
| `^` | Montaña | 3 |
| `*` | Tormenta eléctrica | 5 |
| `%` | Muro (impasable) | — |
| `B` | Base de dron | — |
| `E` | Punto de entrega | — |

### Formato de layout CSP

Los layouts CSP incluyen una sección de parámetros separada por `---`:

```
%%%%%%%%%%%
%B.......E%
%..........%
%%%%%%%%%%%
---
drone:1:capacity=5,battery=24
delivery:1:weight=3,window=0-40
```

### Ejemplo de uso CSP

```bash
# Backtracking básico en mapa twin_bases
python main.py -m csp -a backtracking -l twin_bases

# Forward Checking en mapa con flota grande
python main.py -m csp -a backtracking_fc -l big_fleet

# AC-3 en laberinto complejo
python main.py -m csp -a backtracking_ac3 -l maze_csp

# Bono: MRV + LCV en mapa con ventanas ajustadas
python main.py -m csp -a backtracking_mrv_lcv -l tight_windows

# Modo texto sin ventana gráfica
python main.py -m csp -a backtracking -l twin_bases -t
```

### Layouts CSP disponibles

| Layout | Descripción |
|--------|-------------|
| `twin_bases` | Dos bases separadas por muros, una entrega en cada extremo |
| `triple_fleet` | Tres drones en paralelo, corredores separados por muros |
| `jungle_outpost` | Selva con niebla, múltiples bases y entregas |
| `tight_battery` | Batería justa con zonas de niebla que aumentan el costo |
| `heavy_cargo` | Cargas pesadas, dos drones con capacidades distintas |
| `storm_corridor` | Corredor de tormenta eléctrica, ruta de desvío obligatoria |
| `round_trip` | Una entrega con ventana temprana y otra tardía (espera forzada) |
| `tight_windows` | Ventanas de tiempo muy ajustadas que fuerzan asignaciones específicas |
| `big_fleet` | Cuatro drones, cinco entregas, mapa grande con terreno variado |
| `battery_edge` | Batería = costo exacto del viaje de ida y vuelta |
| `detour_required` | Tormenta bloquea ruta directa; Dijkstra obliga un desvío |
| `hero_drone` | Un dron con batería mínima, otro asume la mayoría de entregas |
| `weight_distribution` | Distribución de pesos fuerza asignación específica por capacidad |
| `fog_valley` | Valle de niebla: rutas caras que limitan la batería útil |
| `mountain_bypass` | Montañas bloquean ruta directa; desvío más largo pero necesario |
| `staggered_time` | Ventanas no superpuestas requieren drones distintos |
| `maze_csp` | Laberinto complejo con muros; rutas largas y costosas |
| `cross_mission` | Entregas en esquinas opuestas; drones se cruzan en el mapa |
| `tight_overload` | Capacidad exacta: ningún dron puede cargar más de lo asignado |
| `single_path` | Pasillo único; entregas distribuidas en el corredor |
| `corner_challenge` | Zona de niebla en el centro; drones deben rodear el mapa |
| `storm_shortcut` | Dron de batería baja sólo alcanza la entrega cercana |

---

## Modo Adversarial

El dron (agente MAX) compite contra cazadores (agentes MIN) que intentan capturarlo. El dron gana al completar todas las entregas; pierde si un cazador lo alcanza.

### Sistema de puntuación

| Evento | Puntos |
|--------|--------|
| Entrega completada | +500 |
| Capturado por cazador | -1000 |
| Penalización por turno | -1 |

### Agentes disponibles

| Agente | Flag `-a` | Descripción |
|--------|-----------|-------------|
| Minimax | `MinimaxAgent` | Búsqueda minimax estándar (dron=MAX, cazadores=MIN) |
| Alpha-Beta | `AlphaBetaAgent` | *(Bono)* Minimax con poda alpha-beta |
| Expectimax | `ExpectimaxAgent` | Modela cazadores con comportamiento mixto greedy/aleatorio |

### Modelos de cazador (flag `-p`)

| Valor `-p` | Comportamiento del cazador |
|-----------|--------------------------|
| `0.0` (default) | Greedy puro — siempre se mueve hacia el dron via BFS |
| `1.0` | Completamente aleatorio |
| `0 < p < 1` | Mixto — aleatorio con probabilidad `p`, greedy con `1-p` |

> **Nota:** `ExpectimaxAgent` modela correctamente el comportamiento mixto en el árbol de búsqueda usando la fórmula: `valor = (1-p) * min(hijos) + p * media(hijos)`

### Función de evaluación (`algorithms/evaluation.py`)

La función de evaluación se aplica a estados no terminales y puede considerar:

- **(a)** Distancia BFS del dron al punto de entrega más cercano
- **(b)** Distancia BFS de cada cazador al dron (usando solo terreno normal)
- **(c)** Distancia a posiciones seguras fuera del alcance de cazadores
- **(d)** Número de entregas pendientes
- **(e)** Puntuación actual del juego
- **(f)** Urgencia de entrega: recompensar entregas alcanzables antes que los cazadores
- **(g)** Penalización por revisitar posiciones (evitar ciclos)

Retorna un valor en el rango `[-1000, +1000]`.

### Ejemplo de uso adversarial

```bash
# Minimax profundidad 1 vs cazador greedy
python main.py -m adversarial -a MinimaxAgent -l small_hunt

# AlphaBeta profundidad 3 vs cazador mixto 30% aleatorio
python main.py -m adversarial -a AlphaBetaAgent -d 3 -p 0.3 -l jungle_chase

# Expectimax vs cazador completamente aleatorio, 10 partidas en modo silencioso
python main.py -m adversarial -a ExpectimaxAgent -p 1.0 -l open_field -n 10 -q

# Texto, paso a paso (frameTime negativo)
python main.py -m adversarial -a MinimaxAgent -l tiny_hunt -t -x -1
```

### Layouts adversariales disponibles

| Layout | Descripción |
|--------|-------------|
| `tiny_hunt` | Mapa mínimo 5×5, un cazador, una entrega |
| `small_hunt` | Mapa 7×7, un cazador, dos entregas |
| `open_field` | Campo abierto con muros simétricos |
| `drone_escape` | Corredor con muros, dron debe escapar hacia las entregas |
| `supply_run` | Terreno mixto (niebla, montaña), un cazador |
| `swamp_run` | Pantano denso de niebla, dificultad para los cazadores |
| `jungle_chase` | Selva con pasillos, dos cazadores en posiciones opuestas |
| `contested_territory` | Territorio disputado, dos cazadores cercanos al inicio |
| `maze_hunt` | Laberinto con muros, dos cazadores |
| `mountain_pass` | Paso montañoso, dos cazadores; montañas restringen movilidad |
| `wide_open` | Gran espacio abierto, cadena montañosa separa al cazador |
| `bottleneck` | Cuello de botella con muros densos |
| `gauntlet` | Corredor estrecho con dos cazadores en línea |
| `triple_threat` | Dos cazadores dispersos, laberinto de muros |
| `scattered_hunters` | Dos cazadores dispersos, terreno variado |
| `terrain_trap` | Trampa de terreno: montaña y niebla separando zonas |
| `storm_tunnel` | Túnel de tormenta eléctrica que solo el dron puede usar |
| `arena` | Arena con muro de montañas central, dos cazadores |
| `hunter_cage` | Cazadores en zona de niebla (ventaja para el dron) |
| `tunnel_escape` | Corredor de montañas inatravesable por cazadores |
| `mountain_overfly` | Cadena montañosa divide el mapa; dron sobrevuela |
| `hunter_swarm` | Tres cazadores agrupados; montañas separan zonas |
| `pacman_maze` | Laberinto denso estilo Pac-Man; pasillos estrechos |
| `desperate_run` | Montañas separan las zonas; dos cazadores en lados opuestos |

---

## Utilidades de búsqueda (`algorithms/utils.py`)

Ambos módulos tienen acceso a funciones de búsqueda con caché por layout:

```python
from algorithms.utils import bfs_distance, dijkstra, manhattan_distance

# Distancia en pasos (sin costo de terreno)
dist = bfs_distance(layout, start, goal, hunter_restricted=False)

# Distancia con costo de terreno ponderado + camino
cost, path = dijkstra(layout, start, goal)

# Distancia Manhattan (heurística)
d = manhattan_distance(pos1, pos2)
```

- `bfs_distance` con `hunter_restricted=True` solo atraviesa terreno normal (`.` y ` `), modelando el movimiento de los cazadores.
- Los resultados son cacheados por `(layout.name, start, goal)` para eficiencia.

---

## Añadir nuevos layouts

### Layout adversarial

Crea un archivo `.lay` en `layouts/adversarial/`:

```
%%%%%%%%%%
%D.......%   D = posición inicial del dron
%...E....%   E = punto de entrega
%....C...%   C = cazador
%...~....%   ~ = niebla (costo 2)
%%%%%%%%%%
```

### Layout CSP

Crea un archivo `.lay` en `layouts/csp/` con sección de parámetros:

```
%%%%%%%%%%%
%B.......E%   B = base de dron
%..........%  E = punto de entrega
%%%%%%%%%%%
---
drone:1:capacity=5,battery=24
delivery:1:weight=3,window=5-30
```

---

## Guía de implementación

### CSP (`algorithms/csp.py`)

```python
def backtracking_search(csp):
    # csp.variables           → lista de entregas a asignar
    # csp.domains[var]        → lista de drones disponibles para var
    # csp.assign(var, val, assignment)
    # csp.unassign(var, assignment)
    # csp.is_consistent(var, val, assignment)
    # csp.is_complete(assignment)
    # csp.get_unassigned_variables(assignment)
    # csp.get_neighbors(var)
    # csp.get_num_conflicts(var, val, assignment)  # para LCV
```

### Adversarial (`algorithms/adversarial.py`)

```python
def get_action(self, state):
    # state.get_legal_actions(agent_index)
    # state.generate_successor(agent_index, action)
    # state.is_win() / state.is_lose()
    # state.get_num_agents()
    # self.evaluation_function(state)
    # self.depth    → profundidad de búsqueda
    # self.prob     → probabilidad de comportamiento aleatorio del cazador
    # Agente 0 = dron (MAX), agentes 1..n-1 = cazadores (MIN/CHANCE)
```
