# 🤖 Rescue Robot — Genetic Algorithm Path Optimizer
> Real-Time Applications & Embedded Systems | ENCS4330 | Birzeit University

A Linux-based multi-processing application that uses **Genetic Algorithms (GA)** to optimize rescue robot paths inside a collapsed building modeled as a **3D grid**. The system simulates a real-world disaster scenario where small autonomous robots deliver supplies to trapped survivors while navigating debris, heat zones, and high-risk areas.

Built entirely in C using Linux IPC techniques — **pipes**, **shared memory**, and **signals** — with a pre-spawned child process pool for parallel fitness evaluation. An **OpenGL-based GUI** provides real-time 3D visualization of the robots navigating the building.

---

## 📁 Project Structure

```
.
├── src/                          # Core backend source files
│   ├── parent.c                  # Main process, coordination & GA loop
│   ├── child_pool.c              # Worker process pool management
│   ├── ipc.c                     # IPC utilities (pipes, shared memory, signals)
│   ├── grid.c                    # 3D grid map parsing & cell management
│   ├── ga.c                      # Genetic algorithm (selection, crossover, mutation)
│   ├── astar.c                   # A* pathfinding for feasible path generation
│   ├── config.c                  # Config file parsing with default fallback
│   ├── menu.c                    # Interactive menu
│   └── performance_comparison.c  # Multi-process vs single-thread benchmarking
├── include/                      # Header files
│   ├── common.h                  # Shared constants, enums & structs
│   ├── astar.h
│   ├── child_pool.h
│   ├── config.h
│   ├── ga.h
│   ├── grid.h
│   ├── ipc.h
│   ├── menu.h
│   └── performance_comparison.h
├── rescue_robot_gui/             # OpenGL GUI visualizer
│   ├── src/
│   │   ├── gui_main.c            # GUI entry point
│   │   ├── gui.c                 # GUI logic & event loop
│   │   ├── gui_camera.c          # 3D camera controls
│   │   ├── gui_renderer.c        # 3D scene rendering
│   │   ├── gui_ipc_reader.c      # Reads robot state via shared memory
│   │   └── glad.c                # OpenGL loader
│   └── include/
│       ├── gui.h
│       ├── gui_camera.h
│       ├── gui_renderer.h
│       ├── gui_ipc_reader.h
│       ├── glad/glad.h
│       └── KHR/khrplatform.h
├── config/
│   ├── params.txt                # All GA & simulation parameters
│   ├── map.txt                   # 3D grid map definition
│   └── menu.txt                  # Menu display configuration
└── makefile
```

---

## 🏗️ Problem Overview

The application models a **collapsed building** as a 3D grid where:
- Each cell is either **empty**, an **obstacle** (debris), or a **survivor location**
- **Heat sensors** determine risk levels per cell (Safe → Low → Medium → High → Critical)
- **CO2 sensors** detect survivor presence (threshold: 800 ppm = definite survivor)
- Multiple rescue robots start from an **entry point** and navigate the building to deliver supplies to survivors

The goal is to find the **optimal set of paths** for all robots such that:
- Maximum survivors are reached
- Travel distance and time are minimized
- High-risk zones (extreme heat) are avoided
- Coverage of the building is maximized

---

## 🧬 Genetic Algorithm

### Chromosome Representation
Each chromosome represents one robot's action plan as a **sequence of moves** (up to 300 moves). Each move is one of **26 possible 3D directions** (N, S, E, W, NE, NW, SE, SW, UP, DOWN, and 16 diagonal-vertical combinations). Paths are decoded by simulating the move sequence on the grid, skipping invalid moves (walls, out-of-bounds).

### Fitness Function
```
f = w1 * survivors + w2 * coverage - w3 * length - w4 * risk - w5 * time
```

Default weights (fully configurable in `config/params.txt`):

| Weight | Default | Description |
|--------|---------|-------------|
| w1 — survivors | 0.50 | Prioritize reaching survivors (dominant term) |
| w2 — coverage | 0.10 | Reward broader exploration of the building |
| w3 — length | 0.20 | Penalize unnecessarily long paths |
| w4 — risk | 0.15 | Penalize passing through high heat/risk cells |
| w5 — time | 0.05 | Penalize slow paths |

### GA Operators
- **Initialization** — Random feasible paths using grid traversal (A* seeded)
- **Selection** — Tournament selection; fitter chromosomes are more likely chosen as parents
- **Crossover** — Path segments are exchanged between two parent chromosomes to produce offspring (rate: 0.8 by default)
- **Mutation** — Random moves are altered to explore alternative routes (rate: 0.1 by default)
- **Elitism** — Top 10% of solutions are preserved unchanged into the next generation (user-defined)

### Termination Criteria
The algorithm stops when any of the following is met:
- Maximum number of generations reached (default: 30)
- Fitness stagnation for N consecutive generations (default: 5)
- Wall-clock time limit exceeded (default: 300 seconds)

---

## 🔀 Architecture & IPC

The system uses a **multi-process architecture**:

- **Parent process** — Coordinates the GA loop: initializes the population, dispatches fitness evaluation tasks to worker children, applies GA operators, and tracks the best solutions across generations
- **Worker pool** — A fixed set of pre-spawned child processes (default: 8) that evaluate chromosome fitness in parallel. Workers are reused across generations to avoid costly fork/exec overhead
- **GUI process** — A separate process that reads the current robot state from shared memory and renders the 3D visualization in real time

### IPC Strategy

| Mechanism | Usage |
|-----------|-------|
| **Pipes** | Parent dispatches fitness evaluation tasks to worker children; workers return results |
| **Shared Memory** | Robot paths and state shared between the backend and GUI process |
| **Signals** | Worker synchronization, graceful shutdown, and event notification |

### Performance Benchmarking
The application includes a built-in benchmarking module that compares:
- **Multi-process** fitness evaluation (parallel, using the worker pool)
- **Single-threaded** fitness evaluation (sequential baseline)

Results are printed after each full GA run.

---

## ⚙️ Configuration (`config/params.txt`)

All parameters are user-defined — no recompilation needed:

```ini
# Grid Dimensions
GRID_X = 20
GRID_Y = 20
GRID_Z = 5

# Genetic Algorithm
POPULATION_SIZE = 40
NUM_GENERATIONS = 30
NUM_ROBOTS = 6
MUTATION_RATE = 0.1
CROSSOVER_RATE = 0.8
ELITISM_RATE = 0.1

# Fitness Weights
WEIGHT_SURVIVORS = 0.50
WEIGHT_LENGTH = 0.20
WEIGHT_RISK = 0.15
WEIGHT_COVERAGE = 0.10
WEIGHT_TIME = 0.05

# IPC Workers
NUM_WORKERS = 8

# Termination
MAX_STAGNATION = 5
TIME_LIMIT = 300

# Reproducibility (0 = random seed)
RANDOM_SEED = 0
```

If the config file is missing, default values are used automatically.

---

## ⚙️ Requirements

- GCC (`gcc`, C11 standard)
- A Linux / Unix environment:
  - **WSL** (Windows Subsystem for Linux) with VS Code
  - Native Linux
  - macOS with GCC via Homebrew
- **For GUI only:** OpenGL + GLFW
  ```bash
  sudo apt install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  ```

---

## 🔨 Build & Run

### 1. Clone the repository

```bash
git clone https://github.com/diaabadaha/Rescue-Robot-Genetic-Algorithm.git
cd Rescue-Robot-Genetic-Algorithm
```

### 2. Install dependencies (if needed)

On Ubuntu / WSL:
```bash
sudo apt update
sudo apt install gcc make
# For GUI:
sudo apt install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

### 3. Build

```bash
# Build both backend and GUI
make

# Build backend only
make core

# Build GUI only
make gui
```

### 4. Run

```bash
# Run the GA simulation (backend)
./bin/rescue_robot

# Run with a custom params file
./bin/rescue_robot config/params.txt

# Run the GUI visualizer (in a separate terminal)
./bin/rescue_robot_gui
```

### 5. Clean

```bash
make clean
```

---

## 💻 Running on WSL with VS Code

1. Open VS Code → press `Ctrl+Shift+P` → search **"WSL: Open Folder in WSL"**
2. Navigate to the cloned project folder
3. Open the integrated terminal with `Ctrl+` `` ` ``
4. Run the build and run commands above directly in the terminal

> Make sure the **WSL** and **C/C++** extensions are installed in VS Code.

---


## 📚 Course Info

- **Course:** ENCS4330 — Real-Time Applications & Embedded Systems
- **Institution:** Birzeit University
- **Instructor:** Dr. Hanna Bullata
- **Semester:** 1st Semester 2025/2026
