#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "../include/grid.h"

// Create an empty grid
Grid *create_grid(int x, int y, int z)
{
    if (x <= 0 || y <= 0 || z <= 0)
    {
        fprintf(stderr, "Error: Invalid grid dimensions (%d, %d, %d)\n", x, y, z);
        return NULL;
    }

    Grid *grid = (Grid *)malloc(sizeof(Grid));
    if (!grid)
    {
        fprintf(stderr, "Error: Memory allocation failed for grid\n");
        return NULL;
    }

    grid->size_x = x;
    grid->size_y = y;
    grid->size_z = z;
    grid->total_cells = x * y * z;
    grid->num_survivors = 0;

    // Allocate cell array
    grid->cells = (Cell *)calloc(grid->total_cells, sizeof(Cell));
    if (!grid->cells)
    {
        fprintf(stderr, "Error: Memory allocation failed for cells\n");
        free(grid);
        return NULL;
    }

    // Initialize all cells as empty with ambient conditions
    for (int i = 0; i < grid->total_cells; i++)
    {
        grid->cells[i].type = CELL_EMPTY;
        grid->cells[i].heat = 20.0 + (rand() % 20); // 20-40°C ambient
        grid->cells[i].co2 = CO2_AMBIENT;            // Normal CO2
        grid->cells[i].risk = RISK_NONE;
        grid->cells[i].has_survivor = 0;
        grid->cells[i].visited = 0;
    }

    // Allocate survivor positions
    grid->survivor_positions = (Vec3 *)malloc(grid->total_cells * sizeof(Vec3));
    if (!grid->survivor_positions)
    {
        fprintf(stderr, "Error: Memory allocation failed for survivor positions\n");
        free(grid->cells);
        free(grid);
        return NULL;
    }

    // Default entry point at origin
    grid->entry_point.x = 0;
    grid->entry_point.y = 0;
    grid->entry_point.z = 0;

    return grid;
}

// Free grid memory
void free_grid(Grid *grid)
{
    if (grid)
    {
        if (grid->cells)
            free(grid->cells);
        if (grid->survivor_positions)
            free(grid->survivor_positions);
        free(grid);
    }
}

// Get cell at position (x, y, z)
Cell *get_cell(Grid *grid, int x, int y, int z)
{
    if (!grid || !grid->cells)
        return NULL;

    if (x < 0 || x >= grid->size_x ||
        y < 0 || y >= grid->size_y ||
        z < 0 || z >= grid->size_z)
    {
        return NULL; // Out of bounds
    }

    int index = coord_to_index(x, y, z, grid->size_x, grid->size_y);
    return &grid->cells[index];
}

// Get cell at Vec3 position
Cell *get_cell_vec(Grid *grid, Vec3 pos)
{
    return get_cell(grid, pos.x, pos.y, pos.z);
}

// Set cell at position
void set_cell(Grid *grid, int x, int y, int z, Cell cell)
{
    if (!grid || !grid->cells)
        return;

    if (x < 0 || x >= grid->size_x ||
        y < 0 || y >= grid->size_y ||
        z < 0 || z >= grid->size_z)
    {
        return; // Out of bounds
    }

    int index = coord_to_index(x, y, z, grid->size_x, grid->size_y);
    grid->cells[index] = cell;
}

// Set cell at Vec3 position
void set_cell_vec(Grid *grid, Vec3 pos, Cell cell)
{
    set_cell(grid, pos.x, pos.y, pos.z, cell);
}

// Check if cell is walkable
int is_cell_walkable(Grid *grid, Vec3 pos)
{
    Cell *cell = get_cell_vec(grid, pos);
    if (!cell)
        return 0; // Out of bounds

    if (cell->type == CELL_OBSTACLE)
        return 0;

    if (cell->heat >= HEAT_EXTREME)
        return 0;

    return 1;
}

// Generate pyramid-shaped rubble (bombed building)
Grid *generate_pyramid_rubble(int x, int y, int z,
                              int center_x, int center_y,
                              float max_radius,
                              float center_density, float edge_density,
                              int num_survivors)
{
    Grid *grid = create_grid(x, y, z);
    if (!grid)
        return NULL;

    srand(time(NULL));

    printf("[Grid] Generating pyramid rubble...\n");
    printf("  Center: (%d, %d)\n", center_x, center_y);
    printf("  Max radius: %.1f\n", max_radius);
    printf("  Density: center=%.2f, edge=%.2f\n", center_density, edge_density);

    // Generate pyramid-shaped obstacle distribution
    for (int zz = 0; zz < z; zz++)
    {
        for (int yy = 0; yy < y; yy++)
        {
            for (int xx = 0; xx < x; xx++)
            {
                // Calculate distance from pyramid center
                float dx = xx - center_x;
                float dy = yy - center_y;
                float dist_from_center = sqrt(dx * dx + dy * dy);

                // Normalize distance (0.0 at center, 1.0 at max_radius)
                float normalized_dist = dist_from_center / max_radius;
                if (normalized_dist > 1.0)
                    normalized_dist = 1.0;

                // Interpolate density based on distance
                // Center has high density, edges have low density
                float density = center_density * (1.0 - normalized_dist) +
                                edge_density * normalized_dist;

                // Add randomness
                float random_factor = ((float)rand() / RAND_MAX) * 0.3 - 0.15; // ±15%
                density += random_factor;
                if (density < 0.0)
                    density = 0.0;
                if (density > 1.0)
                    density = 1.0;

                // Decide if this cell is an obstacle
                float roll = (float)rand() / RAND_MAX;
                Cell *cell = get_cell(grid, xx, yy, zz);

                if (roll < density)
                {
                    // Obstacle (rubble)
                    cell->type = CELL_OBSTACLE;
                    cell->heat = 30.0 + (rand() % 90);      // 30-120°C
                    cell->co2 = CO2_AMBIENT + (rand() % 200); // Some CO2 variation
                }
                else
                {
                    // Empty (air pocket / passable space)
                    cell->type = CELL_EMPTY;
                    cell->heat = 20.0 + (rand() % 40);      // 20-60°C
                    cell->co2 = CO2_AMBIENT + (rand() % 100);
                }

                cell->risk = compute_risk_from_heat(cell->heat);
                cell->has_survivor = 0;
                cell->visited = 0;
            }
        }
    }

    // Ensure entry point is safe
    Cell *entry = get_cell(grid, grid->entry_point.x, grid->entry_point.y, grid->entry_point.z);
    if (entry)
    {
        entry->type = CELL_EMPTY;
        entry->heat = 25.0;
        entry->co2 = CO2_AMBIENT;
        entry->risk = RISK_NONE;
    }

    // Place survivors in random walkable cells within pyramid
    int placed = 0;
    int attempts = 0;
    int max_attempts = grid->total_cells * 10;
    int unreachable_survivor_placed = 0;

    printf("[Grid] Placing %d survivors...\n", num_survivors);

    while (placed < num_survivors && attempts < max_attempts)
    {
        int rx = rand() % x;
        int ry = rand() % y;
        int rz = rand() % z;

        Cell *cell = get_cell(grid, rx, ry, rz);

        // Calculate distance from center
        float dx = rx - center_x;
        float dy = ry - center_y;
        float dist = sqrt(dx * dx + dy * dy);

        if (cell && cell->type == CELL_EMPTY && dist <= max_radius)
        {
            // Place ONE unreachable survivor (high heat) for test coverage
            if (!unreachable_survivor_placed && placed == num_survivors - 1)
            {
                // Last survivor - make it unreachable due to extreme heat (≥120°C)
                cell->co2 = CO2_SURVIVOR_SOURCE;
                cell->heat = 120.0 + (rand() % 20);  // 120-140°C (IMPASSABLE!)
                cell->risk = RISK_CRITICAL;
                cell->has_survivor = 1;
                cell->survivor_helped = 0;
                
                grid->survivor_positions[placed].x = rx;
                grid->survivor_positions[placed].y = ry;
                grid->survivor_positions[placed].z = rz;
                
                placed++;
                unreachable_survivor_placed = 1;

                printf("  Survivor %d placed at (%d, %d, %d) [UNREACHABLE: %.1f°C - IMPASSABLE]\n", 
                       placed - 1, rx, ry, rz, cell->heat);
            }
            // Place regular reachable survivors
            else if (cell->heat < HEAT_HOT)
            {
                // Regular survivor with safe heat
                cell->co2 = CO2_SURVIVOR_SOURCE;
                cell->heat = 32.0 + (rand() % 8); // 32-40°C (body heat)
                cell->has_survivor = 1;
                
                grid->survivor_positions[placed].x = rx;
                grid->survivor_positions[placed].y = ry;
                grid->survivor_positions[placed].z = rz;
                
                placed++;

                printf("  Survivor %d placed at (%d, %d, %d) on floor %d\n", 
                       placed - 1, rx, ry, rz, rz);
            }
        }
        attempts++;
    }

    if (placed < num_survivors)
    {
        printf("[Grid] Warning: Only placed %d/%d survivors\n", placed, num_survivors);
    }

    // Print floor distribution
    printf("[Grid] Survivor floor distribution:\n");
    int floor_count[MAX_GRID_Z] = {0};
    for (int i = 0; i < placed; i++)
    {
        int floor = grid->survivor_positions[i].z;
        if (floor < MAX_GRID_Z)
            floor_count[floor]++;
    }
    for (int floor = 0; floor < z; floor++)
    {
        if (floor_count[floor] > 0)
            printf("  Floor %d: %d survivor(s)\n", floor, floor_count[floor]);
    }

    // Set survivor count
    grid->num_survivors = placed;
    printf("[Grid] Survivor placement summary:\n");
    printf("  - Total survivors: %d\n", placed);
    printf("  - Reachable survivors: %d\n", placed - unreachable_survivor_placed);
    printf("  - Unreachable survivors (high heat): %d\n", unreachable_survivor_placed);
     int floor_counts[MAX_GRID_Z] = {0};
    for (int i = 0; i < placed; i++)
    {
        int floor = grid->survivor_positions[i].z;
        if (floor >= 0 && floor < z)
            floor_counts[floor]++;
    }
    printf("  - Floor distribution:\n");
    for (int floor = 0; floor < z; floor++)
    {
        if (floor_counts[floor] > 0)
            printf("    Floor %d: %d survivor(s)\n", floor, floor_counts[floor]);
    }

    // Diffuse CO2 from survivors to nearby cells
    printf("[Grid] Diffusing CO2...\n");
    diffuse_co2(grid, 3); // 3 diffusion iterations

    // Process sensors to compute risk (don't reset survivor markers)
    process_grid_sensors(grid);

    printf("[Grid] Pyramid rubble generated: %d survivors detected\n", grid->num_survivors);

    return grid;
}

// Diffuse CO2 from high-concentration cells to neighbors
void diffuse_co2(Grid *grid, int diffusion_iterations)
{
    if (!grid || !grid->cells)
        return;

    // Create temporary array for new CO2 values
    float *new_co2 = (float *)malloc(grid->total_cells * sizeof(float));
    if (!new_co2)
    {
        fprintf(stderr, "Error: Failed to allocate memory for CO2 diffusion\n");
        return;
    }

    for (int iter = 0; iter < diffusion_iterations; iter++)
    {
        // Copy current CO2 values
        for (int i = 0; i < grid->total_cells; i++)
        {
            new_co2[i] = grid->cells[i].co2;
        }

        // Diffuse CO2 to neighbors
        for (int z = 0; z < grid->size_z; z++)
        {
            for (int y = 0; y < grid->size_y; y++)
            {
                for (int x = 0; x < grid->size_x; x++)
                {
                    Cell *cell = get_cell(grid, x, y, z);
                    if (!cell || cell->type == CELL_OBSTACLE)
                        continue; // Can't diffuse through obstacles

                    float total_co2 = cell->co2;
                    int neighbor_count = 1; // Include self

                    // Check 6-connected neighbors (N, S, E, W, UP, DOWN)
                    int dx[] = {0, 0, 1, -1, 0, 0};
                    int dy[] = {1, -1, 0, 0, 0, 0};
                    int dz[] = {0, 0, 0, 0, 1, -1};

                    for (int dir = 0; dir < 6; dir++)
                    {
                        int nx = x + dx[dir];
                        int ny = y + dy[dir];
                        int nz = z + dz[dir];

                        Cell *neighbor = get_cell(grid, nx, ny, nz);
                        if (neighbor && neighbor->type == CELL_EMPTY)
                        {
                            total_co2 += neighbor->co2 * CO2_DIFFUSION_RATE;
                            neighbor_count++;
                        }
                    }

                    // Average with neighbors
                    int idx = coord_to_index(x, y, z, grid->size_x, grid->size_y);
                    new_co2[idx] = total_co2 / (1.0 + neighbor_count * CO2_DIFFUSION_RATE);
                }
            }
        }

        // Update CO2 values
        for (int i = 0; i < grid->total_cells; i++)
        {
            if (grid->cells[i].type == CELL_EMPTY)
            {
                // FIXED: Preserve survivor source cells
                if (grid->cells[i].has_survivor)
                {
                    // Keep survivor cells at high CO2
                    grid->cells[i].co2 = CO2_SURVIVOR_SOURCE;
                }
                else
                {
                    // Normal diffusion for other cells
                    grid->cells[i].co2 = new_co2[i];
                }
            }
        }
    }

    free(new_co2);
}

// Process grid to compute risk and detect survivors
void process_grid_sensors(Grid *grid)
{
    if (!grid || !grid->cells)
        return;

    // FIXED: Don't reset num_survivors if survivors were manually placed
    // Keep the count from manual placement
    int additional_survivors = 0;

    for (int z = 0; z < grid->size_z; z++)
    {
        for (int y = 0; y < grid->size_y; y++)
        {
            for (int x = 0; x < grid->size_x; x++)
            {
                Cell *cell = get_cell(grid, x, y, z);
                if (!cell)
                    continue;

                // Compute risk from heat
                cell->risk = compute_risk_from_heat(cell->heat);

                // FIXED: Only detect from CO2 if not already marked as survivor
                if (!cell->has_survivor)
                {
                    // Detect survivor from CO2 sensor
                    int detected = detect_survivor_from_co2(cell->co2);
                    
                    if (detected)
                    {
                        cell->has_survivor = 1;
                        
                        // Track newly detected survivor position
                        grid->survivor_positions[grid->num_survivors + additional_survivors].x = x;
                        grid->survivor_positions[grid->num_survivors + additional_survivors].y = y;
                        grid->survivor_positions[grid->num_survivors + additional_survivors].z = z;
                        additional_survivors++;
                    }
                }
            }
        }
    }

    // Update total survivor count
    grid->num_survivors += additional_survivors;
}

// Count survivors in grid
int count_survivors(Grid *grid)
{
    if (!grid || !grid->cells)
        return 0;
    return grid->num_survivors;
}

// Generate random map (old version - kept for compatibility)
Grid *generate_random_map(int x, int y, int z, float obstacle_density, int num_survivors)
{
    Grid *grid = create_grid(x, y, z);
    if (!grid)
        return NULL;

    srand(time(NULL));

    // Place obstacles randomly
    for (int i = 0; i < grid->total_cells; i++)
    {
        float r = (float)rand() / RAND_MAX;
        if (r < obstacle_density)
        {
            grid->cells[i].type = CELL_OBSTACLE;
            grid->cells[i].heat = 40.0 + (rand() % 80);
            grid->cells[i].co2 = CO2_AMBIENT + (rand() % 100);
        }
        else
        {
            grid->cells[i].type = CELL_EMPTY;
            grid->cells[i].heat = 20.0 + (rand() % 60);
            grid->cells[i].co2 = CO2_AMBIENT + (rand() % 100);
        }
    }

    // Ensure entry point is safe
    Cell *entry = get_cell(grid, 0, 0, 0);
    if (entry)
    {
        entry->type = CELL_EMPTY;
        entry->heat = 22.0;
        entry->co2 = CO2_AMBIENT;
    }

    // Place survivors randomly
    int placed = 0;
    int attempts = 0;
    while (placed < num_survivors && attempts < grid->total_cells * 10)
    {
        int rx = rand() % x;
        int ry = rand() % y;
        int rz = rand() % z;

        Cell *cell = get_cell(grid, rx, ry, rz);
        if (cell && cell->type == CELL_EMPTY && cell->heat < HEAT_HOT)
        {
            // FIXED: Mark survivor immediately
            cell->co2 = CO2_SURVIVOR_SOURCE;
            cell->heat = 32.0 + (rand() % 8);
            cell->has_survivor = 1;
            
            // Track position
            grid->survivor_positions[placed].x = rx;
            grid->survivor_positions[placed].y = ry;
            grid->survivor_positions[placed].z = rz;
            
            placed++;
        }
        attempts++;
    }

    // Set count
    grid->num_survivors = placed;

    // Diffuse CO2
    diffuse_co2(grid, 2);

    // Process sensors
    process_grid_sensors(grid);

    return grid;
}

// Load map from file (existing implementation - unchanged)
Grid *load_map_from_file(const char *filename)
{
    FILE *file = fopen(filename, "r");
    if (!file)
    {
        fprintf(stderr, "Warning: Could not open map file '%s'\n", filename);
        return NULL;
    }

    int x, y, z;
    if (fscanf(file, "%d %d %d", &x, &y, &z) != 3)
    {
        fprintf(stderr, "Error: Invalid map file format (dimensions)\n");
        fclose(file);
        return NULL;
    }

    Grid *grid = create_grid(x, y, z);
    if (!grid)
    {
        fclose(file);
        return NULL;
    }

    if (fscanf(file, "%d %d %d", &grid->entry_point.x,
               &grid->entry_point.y, &grid->entry_point.z) != 3)
    {
        fprintf(stderr, "Error: Invalid entry point in map file\n");
        free_grid(grid);
        fclose(file);
        return NULL;
    }

    for (int zz = 0; zz < z; zz++)
    {
        for (int yy = 0; yy < y; yy++)
        {
            for (int xx = 0; xx < x; xx++)
            {
                Cell cell;

                if (fscanf(file, "%d %f %f", &cell.type, &cell.heat, &cell.co2) != 3)
                {
                    fprintf(stderr, "Error: Incomplete cell data at (%d,%d,%d)\n",
                            xx, yy, zz);
                    free_grid(grid);
                    fclose(file);
                    return NULL;
                }

                cell.visited = 0;
                cell.risk = 0;
                cell.has_survivor = 0;

                set_cell(grid, xx, yy, zz, cell);
            }
        }
    }

    fclose(file);

    process_grid_sensors(grid);

    printf("Map loaded: %d x %d x %d grid with %d survivors detected\n",
           x, y, z, grid->num_survivors);

    return grid;
}

// Print a single layer of the grid
void print_grid_layer(Grid *grid, int z)
{
    if (!grid || z < 0 || z >= grid->size_z)
    {
        printf("Invalid layer\n");
        return;
    }

    printf("\n=== Grid Layer Z=%d ===\n", z);
    for (int y = 0; y < grid->size_y; y++)
    {
        for (int x = 0; x < grid->size_x; x++)
        {
            Cell *cell = get_cell(grid, x, y, z);
            if (!cell)
            {
                printf("? ");
                continue;
            }

            if (cell->has_survivor)
            {
                printf("S "); // Survivor
            }
            else if (cell->type == CELL_OBSTACLE)
            {
                printf("# "); // Obstacle
            }
            else if (cell->heat >= HEAT_EXTREME)
            {
                printf("X "); // Extreme heat
            }
            else if (cell->heat >= HEAT_VERY_HOT)
            {
                printf("H "); // Very hot
            }
            else
            {
                printf(". "); // Empty/safe
            }
        }
        printf("\n");
    }
    printf("\n");
}

// Print grid statistics
void print_grid_stats(Grid *grid)
{
    if (!grid)
        return;

    int obstacles = 0, empty = 0, survivors = 0;
    int extreme_heat = 0, very_hot = 0, hot = 0;

    for (int i = 0; i < grid->total_cells; i++)
    {
        if (grid->cells[i].type == CELL_OBSTACLE)
        {
            obstacles++;
        }
        else
        {
            empty++;
        }

        if (grid->cells[i].has_survivor)
        {
            survivors++;
        }

        if (grid->cells[i].heat >= HEAT_EXTREME)
        {
            extreme_heat++;
        }
        else if (grid->cells[i].heat >= HEAT_VERY_HOT)
        {
            very_hot++;
        }
        else if (grid->cells[i].heat >= HEAT_HOT)
        {
            hot++;
        }
    }

    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf("                    GRID STATISTICS\n");
    printf("═══════════════════════════════════════════════════════════\n\n");
    printf("Dimensions: %d × %d × %d\n", grid->size_x, grid->size_y, grid->size_z);
    printf("Total cells: %d\n", grid->total_cells);
    printf("Entry point: (%d, %d, %d)\n\n",
           grid->entry_point.x, grid->entry_point.y, grid->entry_point.z);

    printf("Cell distribution:\n");
    printf("  Empty cells:    %d (%.1f%%)\n", empty, 100.0 * empty / grid->total_cells);
    printf("  Obstacles:      %d (%.1f%%)\n", obstacles, 100.0 * obstacles / grid->total_cells);
    printf("  Survivors:      %d (detected by CO2 >= %.0f ppm)\n\n", survivors, CO2_THRESHOLD_HIGH);

    printf("Heat zones:\n");
    printf("  Extreme (≥%.0f°C): %d cells (IMPASSABLE)\n", HEAT_EXTREME, extreme_heat);
    printf("  Very Hot (≥%.0f°C): %d cells (CRITICAL RISK)\n", HEAT_VERY_HOT, very_hot);
    printf("  Hot (≥%.0f°C):      %d cells (HIGH RISK)\n\n", HEAT_HOT, hot);
}