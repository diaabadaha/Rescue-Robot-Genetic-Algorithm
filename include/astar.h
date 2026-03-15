// include/astar.h
#ifndef ASTAR_H
#define ASTAR_H

#include "common.h"  // for Vec3
#include "grid.h"    // for Grid

// Simple path representation for A*
typedef struct {
    Vec3 *nodes;   // sequence of positions from start to goal
    int length;    // number of nodes in the path
    double cost;   // total g-cost of the path
} Path;

/**
 * Run A* on a 3D grid to find an optimal path from start to goal.
 *
 * Parameters:
 *   grid  - pointer to the grid (already loaded and processed)
 *   start - starting position (e.g. grid->entry_point)
 *   goal  - goal position (e.g. survivor position)
 *   out_path - output path; this function allocates out_path->nodes
 *
 * Returns:
 *   0  - success, out_path is filled
 *  -1  - no path found
 *  -2  - memory allocation error
 *
 * Note:
 *   Caller must call free_path(&out_path) when done.
 */
int astar_find_path(const Grid *grid, Vec3 start, Vec3 goal, Path *out_path);

// Free the memory used by a Path.
void free_path(Path *path);

double evaluate_astar_path(const Path *p, Grid *grid, Config *config);
void run_astar_baseline(Grid *grid, Config *config);

// Functions for building and evaluating A* baseline (used by parent.c)
void build_astar_baseline_paths(Grid *grid, Config *config, Chromosome *baseline);
void evaluate_astar_baseline(const Chromosome *chr, Grid *grid,
                             int *out_survivors, int *out_coverage,
                             double *out_length, double *out_risk);

#endif