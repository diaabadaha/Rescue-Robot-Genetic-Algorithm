#ifndef GRID_H
#define GRID_H

#include "common.h"
#include "config.h"

// Grid structure - represents the 3D collapsed building
typedef struct
{
    Cell *cells;               // 1D array of cells (size = X * Y * Z)
    int size_x;                // Grid dimensions
    int size_y;
    int size_z;
    int total_cells;           // Total number of cells
    int num_survivors;         // Total survivors detected in the grid
    Vec3 *survivor_positions;  // Array of survivor positions
    Vec3 entry_point;          // Where robots start
} Grid;

// Function prototypes

// Grid creation and destruction
Grid *create_grid(int x, int y, int z);
void free_grid(Grid *grid);

// Grid cell access
Cell *get_cell(Grid *grid, int x, int y, int z);
Cell *get_cell_vec(Grid *grid, Vec3 pos);
void set_cell(Grid *grid, int x, int y, int z, Cell cell);
void set_cell_vec(Grid *grid, Vec3 pos, Cell cell);

// Map loading and generation
Grid *load_map_from_file(const char *filename);
Grid *generate_random_map(int x, int y, int z, float obstacle_density, int num_survivors);
Grid *generate_pyramid_rubble(int x, int y, int z,
                              int center_x, int center_y,
                              float max_radius,
                              float center_density, float edge_density,
                              int num_survivors);
void diffuse_co2(Grid *grid, int diffusion_iterations);

// Sensor processing
void process_grid_sensors(Grid *grid);

// Grid utilities
void print_grid_layer(Grid *grid, int z);
void print_grid_stats(Grid *grid);
int is_cell_walkable(Grid *grid, Vec3 pos);
int count_survivors(Grid *grid);

#endif // GRID_H