#include "../include/astar.h"
#include "../include/config.h"

#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>  

// Grid dimension access helpers
#define GRID_W(g) ((g)->size_x)
#define GRID_H(g) ((g)->size_y)
#define GRID_D(g) ((g)->size_z)

// Check if position is within bounds
static inline int in_bounds(const Grid *grid, int x, int y, int z) {
    return (x >= 0 && x < GRID_W(grid) &&
            y >= 0 && y < GRID_H(grid) &&
            z >= 0 && z < GRID_D(grid));
}

// Convert (x,y,z) to a single index
static inline int index3d(const Grid *grid, int x, int y, int z) {
    return z * GRID_W(grid) * GRID_H(grid) + y * GRID_W(grid) + x;
}

// Convert index back to Vec3
static inline Vec3 index_to_vec3(const Grid *grid, int idx) {
    int wh = GRID_W(grid) * GRID_H(grid);
    int z = idx / wh;
    int rem = idx % wh;
    int y = rem / GRID_W(grid);
    int x = rem % GRID_W(grid);
    Vec3 v = { x, y, z };
    return v;
}

// Open set: find node with lowest f-score
static int open_set_pop_lowest_f(const Grid *grid,
                                 uint8_t *open_set,
                                 double *f_score) {
    int total = GRID_W(grid) * GRID_H(grid) * GRID_D(grid);
    int best_idx = -1;
    double best_f = DBL_MAX;

    for (int i = 0; i < total; ++i) {
        if (open_set[i]) {
            if (f_score[i] < best_f) {
                best_f = f_score[i];
                best_idx = i;
            }
        }
    }
    return best_idx;
}

// Reconstruct path from came_from[] arrays
static int reconstruct_path(const Grid *grid,
                            int *came_from,
                            int start_idx,
                            int goal_idx,
                            Path *out_path) {
    int total = GRID_W(grid) * GRID_H(grid) * GRID_D(grid);

    // Walk backward from goal to start to count length
    int current = goal_idx;
    int length = 1; // at least goal

    while (current != start_idx) {
        current = came_from[current];
        if (current < 0 || current >= total) {
            // Broken parent chain
            return -1;
        }
        length++;
    }

    // Allocate nodes
    Vec3 *nodes = (Vec3 *)malloc(sizeof(Vec3) * length);
    if (!nodes) return -2;

    // Fill nodes from end to start
    current = goal_idx;
    for (int i = length - 1; i >= 0; --i) {
        nodes[i] = index_to_vec3(grid, current);
        if (i == 0) break;
        current = came_from[current];
    }

    out_path->nodes = nodes;
    out_path->length = length;

    // Compute total cost as distance from start to goal (sum of step distances)
    double total_cost = 0.0;
    for (int i = 1; i < length; ++i) {
        Vec3 a = nodes[i - 1];
        Vec3 b = nodes[i];
        double dx = (double)b.x - (double)a.x;
        double dy = (double)b.y - (double)a.y;
        double dz = (double)b.z - (double)a.z;
        total_cost += sqrt(dx * dx + dy * dy + dz * dz);
    }
    out_path->cost = total_cost;

    return 0;
}

// Public functions
void free_path(Path *path) {
    if (!path) return;
    if (path->nodes) {
        free(path->nodes);
        path->nodes = NULL;
    }
    path->length = 0;
    path->cost = 0.0;
}

int astar_find_path(const Grid *grid, Vec3 start, Vec3 goal, Path *out_path) {
    if (!grid || !out_path) return -1;

    int width  = GRID_W(grid);
    int height = GRID_H(grid);
    int depth  = GRID_D(grid);

    int total = width * height * depth;
    if (total <= 0) return -1;

    // Clear out_path initially
    out_path->nodes = NULL;
    out_path->length = 0;
    out_path->cost = 0.0;

    // Check bounds for start & goal
    if (!in_bounds(grid, start.x, start.y, start.z) ||
        !in_bounds(grid, goal.x, goal.y, goal.z)) {
        return -1;
    }

    // If start == goal, trivial path of length 1
    if (start.x == goal.x && start.y == goal.y && start.z == goal.z) {
        out_path->nodes = (Vec3 *)malloc(sizeof(Vec3));
        if (!out_path->nodes) return -2;
        out_path->nodes[0] = start;
        out_path->length = 1;
        out_path->cost = 0.0;
        return 0;
    }

    // Allocate arrays
    double *g_score   = (double *)malloc(sizeof(double) * total);
    double *f_score   = (double *)malloc(sizeof(double) * total);
    int    *came_from = (int *)malloc(sizeof(int) * total);
    uint8_t *open_set   = (uint8_t *)calloc(total, sizeof(uint8_t));
    uint8_t *closed_set = (uint8_t *)calloc(total, sizeof(uint8_t));

    if (!g_score || !f_score || !came_from || !open_set || !closed_set) {
        free(g_score);
        free(f_score);
        free(came_from);
        free(open_set);
        free(closed_set);
        return -2;
    }

    // Initialize scores
    for (int i = 0; i < total; ++i) {
        g_score[i] = DBL_MAX;
        f_score[i] = DBL_MAX;
        came_from[i] = -1;
    }

    int start_idx = index3d(grid, start.x, start.y, start.z);
    int goal_idx  = index3d(grid, goal.x, goal.y, goal.z);

    // Start node scores
    g_score[start_idx] = 0.0;
    {
        double dx = (double)goal.x - (double)start.x;
        double dy = (double)goal.y - (double)start.y;
        double dz = (double)goal.z - (double)start.z;
        f_score[start_idx] = sqrt(dx * dx + dy * dy + dz * dz);
    }
    open_set[start_idx] = 1;

    // -----------------------------------
    // Main A* search loop
    // -----------------------------------
    while (1) {
        int current_idx = open_set_pop_lowest_f(grid, open_set, f_score);
        if (current_idx == -1) {
            // open set empty -> no path
            break;
        }

        if (current_idx == goal_idx) {
            // Found path
            int r = reconstruct_path(grid, came_from, start_idx, goal_idx, out_path);
            free(g_score); free(f_score); free(came_from);
            free(open_set); free(closed_set);
            return r;
        }

        open_set[current_idx] = 0;
        closed_set[current_idx] = 1;

        Vec3 current = index_to_vec3(grid, current_idx);

        // Explore all 26 neighbors
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    int nx = current.x + dx;
                    int ny = current.y + dy;
                    int nz = current.z + dz;

                    if (!in_bounds(grid, nx, ny, nz)) continue;

                    Vec3 neighbor_pos = { nx, ny, nz };

                    // Check if neighbor is walkable
                    if (!is_cell_walkable((Grid *)grid, neighbor_pos)) {
                        continue;
                    }

                    int neighbor_idx = index3d(grid, nx, ny, nz);
                    if (closed_set[neighbor_idx]) continue;

                    // Step cost (Euclidean)
                    double step_dx = (double)nx - (double)current.x;
                    double step_dy = (double)ny - (double)current.y;
                    double step_dz = (double)nz - (double)current.z;
                    double step_cost = sqrt(step_dx * step_dx +
                                            step_dy * step_dy +
                                            step_dz * step_dz);

                    double tentative_g = g_score[current_idx] + step_cost;

                    if (!open_set[neighbor_idx]) {
                        open_set[neighbor_idx] = 1; // discovered
                    } else if (tentative_g >= g_score[neighbor_idx]) {
                        // Not a better path
                        continue;
                    }

                    // This path to neighbor is the best so far
                    came_from[neighbor_idx] = current_idx;
                    g_score[neighbor_idx] = tentative_g;

                    double h_dx = (double)goal.x - (double)nx;
                    double h_dy = (double)goal.y - (double)ny;
                    double h_dz = (double)goal.z - (double)nz;
                    double h = sqrt(h_dx * h_dx + h_dy * h_dy + h_dz * h_dz);

                    f_score[neighbor_idx] = tentative_g + h;
                }
            }
        }
    }

    // No path found
    free(g_score); free(f_score); free(came_from);
    free(open_set); free(closed_set);
    return -1;
}

// Evaluate A* path cost (for fitness computation)
void evaluate_astar_baseline(const Chromosome *chr,
                             Grid *grid,
                             int *out_survivors,
                             int *out_coverage,
                             double *out_length,
                             double *out_risk)
{
    if (!chr || !grid) return;

    // Reset visited
    for (int i = 0; i < grid->total_cells; i++)
        grid->cells[i].visited = 0;

    int coverage = 0;
    int survivors = 0;
    double length = 0.0;
    double risk = 0.0;

    for (int r = 0; r < chr->num_robots; r++)
    {
        const Robot *bot = &chr->robots[r];

        for (int i = 0; i < bot->path_length; i++)
        {
            Vec3 pos = bot->path[i];
            Cell *cell = get_cell_vec(grid, pos);
            if (!cell) continue;

            if (!cell->visited) {
                cell->visited = 1;
                coverage++;

                if (cell->has_survivor)
                    survivors++;
            }


            risk += cell->risk;

            if (i > 0)
                length += euclidean_distance(bot->path[i - 1], pos);
        }
    }

    *out_survivors = survivors;
    *out_coverage  = coverage;
    *out_length    = length;
    *out_risk      = risk;
}


// A* BASELINE: MULTI-ROBOT, MULTI-SURVIVOR
#define MAX_BASELINE_STEPS 512

// Local helper: convert Vec3 to flat index
static int vec3_to_index(const Grid *grid, Vec3 p)
{
    return p.z * grid->size_x * grid->size_y +
           p.y * grid->size_x +
           p.x;
}

// Local helper: squared distance (no sqrt needed)
static double squared_distance(Vec3 a, Vec3 b)
{
    double dx = (double)a.x - (double)b.x;
    double dy = (double)a.y - (double)b.y;
    double dz = (double)a.z - (double)b.z;
    return dx * dx + dy * dy + dz * dz;
}

void build_astar_baseline_paths(Grid *grid,
                                Config *config,
                                Chromosome *baseline)
{
    if (!grid || !config || !baseline)
    {
        fprintf(stderr, "[A*] Error: NULL pointer in build_astar_baseline_paths\n");
        return;
    }

    int num_robots     = config->num_robots;
    int num_survivors  = grid->num_survivors;

    if (num_robots <= 0)
    {
        fprintf(stderr, "[A*] Warning: num_robots <= 0, nothing to plan\n");
        return;
    }

    if (num_survivors <= 0)
    {
        fprintf(stderr, "[A*] Warning: no survivors in grid, baseline trivial\n");
        memset(baseline, 0, sizeof(*baseline));
        baseline->num_robots = num_robots;
        return;
    }

    printf("[A*] Building baseline paths for %d robots and %d survivors...\n",
           num_robots, num_survivors);
    printf("[A*] MULTI-TRIP MODE: Robots must return to base after each rescue\n\n");

    // Reset chromosome
    memset(baseline, 0, sizeof(*baseline));
    baseline->num_robots = num_robots;

    // Track which survivors are already rescued
    int *rescued = (int *)calloc(num_survivors, sizeof(int));
    if (!rescued)
    {
        fprintf(stderr, "[A*] Error: failed to allocate rescued[]\n");
        return;
    }

    Vec3 entry = grid->entry_point;

    // Initialize robots at entry point
    for (int r = 0; r < num_robots; r++)
    {
        Robot *bot = &baseline->robots[r];
        memset(bot, 0, sizeof(Robot));

        bot->id         = r;
        bot->start_pos  = entry;
        bot->current_pos = entry;
        bot->entry_point = entry;
        bot->has_supplies = 1;
        bot->trips_completed = 0;

        bot->path_length = 1;
        bot->path[0]     = entry;
    }

    int survivors_rescued = 0;

    // MULTI-TRIP LOOP: Each robot does multiple round trips
    // Round-robin: give each robot one rescue trip at a time
    while (survivors_rescued < num_survivors)
    {
        int any_rescue_this_round = 0;

        for (int r = 0; r < num_robots && survivors_rescued < num_survivors; r++)
        {
            Robot *bot = &baseline->robots[r];
            
            // Robot always starts a trip from entry point
            Vec3 current_pos = entry;

            // Find nearest unrescued survivor from entry point
            int best_idx = -1;
            double best_d2 = 0.0;

            for (int s = 0; s < num_survivors; s++)
            {
                if (rescued[s])
                    continue;

                Vec3 surv = grid->survivor_positions[s];
                double d2 = squared_distance(current_pos, surv);

                if (best_idx == -1 || d2 < best_d2)
                {
                    best_idx = s;
                    best_d2  = d2;
                }
            }

            if (best_idx == -1)
            {
                // No survivors left
                break;
            }

            Vec3 survivor_pos = grid->survivor_positions[best_idx];

            printf("[A*] Robot %d → planning path to survivor %d at (%d,%d,%d)\n",
                   r, best_idx, survivor_pos.x, survivor_pos.y, survivor_pos.z);

            // LEG 1: Entry → Survivor
            Path leg_to_survivor;
            int rc1 = astar_find_path(grid, entry, survivor_pos, &leg_to_survivor);
            
            if (rc1 != 0 || leg_to_survivor.length <= 0 || !leg_to_survivor.nodes)
            {
                fprintf(stderr, "[A*] Warning: A* failed for robot %d to survivor %d (rc=%d)\n",
                        r, best_idx, rc1);
                // Mark as rescued anyway to avoid infinite loop
                rescued[best_idx] = 1;
                survivors_rescued++;
                continue;
            }

            // Append leg to robot's path (skip first node if already at entry)
            int start_idx = (bot->path_length == 1) ? 1 : 0;  // Skip entry if first trip
            for (int i = start_idx; i < leg_to_survivor.length; i++)
            {
                if (bot->path_length >= MAX_PATH_LENGTH)
                {
                    printf("[A*] Robot %d path reached MAX_PATH_LENGTH, truncating\n", r);
                    free_path(&leg_to_survivor);
                    goto done_with_robot;
                }
                bot->path[bot->path_length++] = leg_to_survivor.nodes[i];
            }

            free_path(&leg_to_survivor);

            // LEG 2: Survivor → Entry (RETURN TRIP)
            printf("[A*] Robot %d → returning to base at (%d,%d,%d)\n",
                   r, entry.x, entry.y, entry.z);

            Path leg_to_entry;
            int rc2 = astar_find_path(grid, survivor_pos, entry, &leg_to_entry);
            
            if (rc2 != 0 || leg_to_entry.length <= 0 || !leg_to_entry.nodes)
            {
                fprintf(stderr, "[A*] Warning: A* failed for robot %d return trip (rc=%d)\n",
                        r, rc2);
                // Robot got stuck at survivor location
                rescued[best_idx] = 1;
                survivors_rescued++;
                bot->trips_completed++;
                continue;
            }

            // Append return path (skip first node - already at survivor)
            for (int i = 1; i < leg_to_entry.length; i++)
            {
                if (bot->path_length >= MAX_PATH_LENGTH)
                {
                    printf("[A*] Robot %d path reached MAX_PATH_LENGTH, truncating\n", r);
                    free_path(&leg_to_entry);
                    goto done_with_robot;
                }
                bot->path[bot->path_length++] = leg_to_entry.nodes[i];
            }

            free_path(&leg_to_entry);

            // Successfully completed round trip!
            bot->trips_completed++;
            bot->current_pos = entry;
            rescued[best_idx] = 1;
            survivors_rescued++;
            any_rescue_this_round = 1;

            printf("[A*] Robot %d completed trip %d (rescued survivor %d)\n\n",
                   r, bot->trips_completed, best_idx);

done_with_robot:
            continue;
        }

        if (!any_rescue_this_round)
        {
            // All remaining survivors are unreachable
            printf("[A*] No additional survivors could be rescued (probably unreachable)\n");
            break;
        }
    }

    free(rescued);

    printf("[A*] Baseline paths built.\n");
    printf("[A*] Total survivors rescued: %d / %d\n", survivors_rescued, num_survivors);
    
    // Print trip counts
    printf("[A*] Trip counts per robot:\n");
    for (int r = 0; r < num_robots; r++)
    {
        printf("  Robot %d: %d trips completed\n", r, baseline->robots[r].trips_completed);
    }
    printf("\n");
}

static int count_astar_collisions(const Chromosome *chr, const Grid *grid)
{
    if (!chr || !grid)
        return 0;

    int total_cells = grid->size_x * grid->size_y * grid->size_z;
    int max_steps   = 0;

    // Determine the maximum path length across all robots
    for (int r = 0; r < chr->num_robots; r++)
    {
        if (chr->robots[r].path_length > max_steps)
            max_steps = chr->robots[r].path_length;
    }

    if (max_steps <= 0)
        return 0;

    int *occupancy = (int *)malloc(sizeof(int) * total_cells);
    if (!occupancy)
        return 0;

    int collisions = 0;

    for (int t = 0; t < max_steps; t++)
    {
        // Reset occupancy for this time slice
        for (int i = 0; i < total_cells; i++)
            occupancy[i] = -1;

        // Mark each robot's position at time t
        for (int r = 0; r < chr->num_robots; r++)
        {
            const Robot *bot = &chr->robots[r];
            if (bot->path_length <= 0)
                continue;

            // If robot finished earlier, it stays at its last position
            int idx_step = (t < bot->path_length) ? t : (bot->path_length - 1);
            Vec3 pos = bot->path[idx_step];

            int flat = vec3_to_index(grid, pos);
            if (flat < 0 || flat >= total_cells)
                continue;

            if (occupancy[flat] == -1)
            {
                occupancy[flat] = r;
            }
            else if (occupancy[flat] != r)
            {
                collisions++;
            }
        }
    }

    free(occupancy);
    return collisions;
}

void run_astar_baseline(Grid *grid, Config *config)
{
    printf("\n======== [A*] RUNNING BASELINE ========\n");

    if (!grid || !config)
    {
        fprintf(stderr, "[A*] Error: NULL grid or config in run_astar_baseline\n");
        return;
    }

    Chromosome baseline;
    build_astar_baseline_paths(grid, config, &baseline);

    int survivors = 0;
    int coverage = 0;
    double length = 0.0;
    double risk = 0.0;

    evaluate_astar_baseline(&baseline, grid,
                            &survivors, &coverage, &length, &risk);

    int collisions = count_astar_collisions(&baseline, grid);

    printf("[A*] Survivors reached:       %d / %d\n",
           survivors, grid->num_survivors);
    printf("[A*] Cells covered:           %d\n", coverage);
    printf("[A*] Total path length:       %.2f\n", length);
    printf("[A*] Total risk:              %.2f\n", risk);
    printf("[A*] Collision count:         %d\n\n", collisions);

    printf("\n ====== A* Baseline Complete ======\n");
}