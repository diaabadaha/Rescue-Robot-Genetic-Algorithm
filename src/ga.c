#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../include/ga.h"

#include "child_pool.h"

// =============================================================================
// CHROMOSOME INITIALIZATION AND PATH GENERATION
// =============================================================================

// Initialize a robot with random move sequence
void init_random_robot(Robot *robot, int robot_id, Vec3 start_pos, int num_moves)
{
    if (!robot || num_moves <= 0 || num_moves > MAX_MOVES)
    {
        fprintf(stderr, "Error: Invalid robot initialization parameters\n");
        return;
    }

    robot->id = robot_id;
    robot->start_pos = start_pos;
    robot->current_pos = start_pos;
    robot->num_moves = num_moves;
    robot->path_length = 0;
    robot->status = ROBOT_IDLE;
    robot->survivors_rescued = 0;
    robot->time_taken = 0.0;

    // Multi-trip supply delivery initialization
    robot->has_supplies = 1; // Starts with supplies
    robot->trips_completed = 0;
    robot->entry_point = start_pos; // Entry/exit point

    // Fog of war initialization
    robot->num_discovered = 0;
    memset(robot->discovered_obstacles, 0, sizeof(robot->discovered_obstacles));

    // Generate PURELY RANDOM move sequence
    for (int i = 0; i < num_moves; i++)
    {
        robot->moves[i] = rand() % NUM_DIRECTIONS;
    }

    // Path will be generated later by generate_path_from_moves()
}

// Generate actual path from robot's move sequence
int generate_path_from_moves(Robot *robot, Grid *grid)
{
    if (!robot || !grid)
    {
        fprintf(stderr, "Error: NULL robot or grid in generate_path_from_moves\n");
        return 0;
    }

    // IMPORTANT: Reset discovered obstacles for this path generation
    // Fog of war applies only within a single path evaluation, not across generations
    robot->num_discovered = 0;

    // Start from robot's starting position
    Vec3 current = robot->start_pos;
    robot->path[0] = current;
    robot->path_length = 1;

    // Apply each move sequentially
    for (int i = 0; i < robot->num_moves; i++)
    {
        Vec3 delta = direction_to_delta(robot->moves[i]);
        Vec3 next = {
            current.x + delta.x,
            current.y + delta.y,
            current.z + delta.z};

        // Check if out of bounds
        if (!is_within_bounds(next, grid->size_x, grid->size_y, grid->size_z))
            continue; // Skip this move

        Cell *cell = get_cell_vec(grid, next);
        if (!cell)
            continue;

        // FOG OF WAR: Check if we already discovered this cell is an obstacle
        int is_known_obstacle = 0;
        int cell_index = next.z * (grid->size_x * grid->size_y) +
                         next.y * grid->size_x + next.x;

        for (int j = 0; j < robot->num_discovered; j++)
        {
            if (robot->discovered_obstacles[j] == cell_index)
            {
                is_known_obstacle = 1;
                break;
            }
        }

        // If we know it's an obstacle, don't try to move there
        if (is_known_obstacle)
            continue;

        // Try to move - discover if it's walkable
        if (is_cell_walkable(grid, next))
        {
            // Success! Move to this cell
            current = next;
            robot->path[robot->path_length++] = current;
            robot->current_pos = current;

            if (robot->path_length >= MAX_PATH_LENGTH)
                break; // Path buffer full
        }
        else
        {
            // Discovered obstacle! Remember it for future moves
            if (robot->num_discovered < MAX_PATH_LENGTH)
            {
                robot->discovered_obstacles[robot->num_discovered++] = cell_index;
            }
            // Move is wasted - robot stays at current position
        }
    }

    return robot->path_length;
}

// Initialize random chromosome
void init_random_chromosome(Chromosome *chromosome, int num_robots,
                            Vec3 start_pos, int num_moves)
{
    if (!chromosome || num_robots <= 0 || num_robots > MAX_ROBOTS)
    {
        fprintf(stderr, "Error: Invalid chromosome initialization parameters\n");
        return;
    }

    chromosome->num_robots = num_robots;
    chromosome->fitness = 0.0;
    chromosome->survivors_reached = 0;
    chromosome->cells_covered = 0;
    chromosome->total_path_length = 0.0;
    chromosome->total_risk = 0.0;
    chromosome->total_time = 0.0;

    // Initialize each robot with random moves
    for (int i = 0; i < num_robots; i++)
    {
        init_random_robot(&chromosome->robots[i], i, start_pos, num_moves);
    }
}

// Generate paths for all robots in chromosome
void generate_chromosome_paths(Chromosome *chromosome, Grid *grid)
{
    if (!chromosome || !grid)
    {
        fprintf(stderr, "Error: NULL chromosome or grid in generate_chromosome_paths\n");
        return;
    }

    for (int i = 0; i < chromosome->num_robots; i++)
    {
        generate_path_from_moves(&chromosome->robots[i], grid);
    }
}

// =============================================================================
// FITNESS EVALUATION
// =============================================================================

// Evaluate a robot's path and update chromosome statistics
// Evaluate a robot's path and update chromosome statistics
void evaluate_robot_path(Robot *robot, Grid *grid, Chromosome *chromosome,
                         int *cells_visited, int *survivor_rescued)
{
    if (!robot || !grid || !chromosome)
        return;

    robot->survivors_rescued = 0;
    robot->trips_completed = 0;
    robot->has_supplies = 1;

    double path_length = 0.0;
    double total_risk = 0.0;
    int currently_on_rescue_trip = 0;

    for (int i = 0; i < robot->path_length; i++)
    {
        Vec3 pos = robot->path[i];
        Cell *cell = get_cell_vec(grid, pos);

        if (!cell)
            continue;

        // MULTI-TRIP: Check if robot returned to entry point
        if (vec3_equals(pos, robot->entry_point))
        {
            if (currently_on_rescue_trip)
            {
                robot->has_supplies = 1;
                robot->trips_completed++;
                currently_on_rescue_trip = 0;
            }
        }

        // Count coverage using LOCAL array
        int cell_idx = coord_to_index(pos.x, pos.y, pos.z,
                                      grid->size_x, grid->size_y);
        if (!cells_visited[cell_idx])
        {
            cells_visited[cell_idx] = 1;
            chromosome->cells_covered++;
        }

        // Check for survivor using LOCAL array
        if (cell->has_survivor)
        {
            // Find which survivor this is
            int survivor_idx = -1;
            for (int s = 0; s < grid->num_survivors; s++)
            {
                if (vec3_equals(pos, grid->survivor_positions[s]))
                {
                    survivor_idx = s;
                    break;
                }
            }

            // Rescue if possible
            if (survivor_idx >= 0 &&
                !survivor_rescued[survivor_idx] &&
                robot->has_supplies)
            {
                robot->has_supplies = 0;
                survivor_rescued[survivor_idx] = 1;
                robot->survivors_rescued++;
                chromosome->survivors_reached++;
                currently_on_rescue_trip = 1;
            }
        }

        total_risk += cell->risk;

        if (i > 0)
        {
            path_length += euclidean_distance(robot->path[i - 1], pos);
        }
    }

    chromosome->total_path_length += path_length;
    chromosome->total_risk += total_risk;
    robot->time_taken = path_length;
    chromosome->total_time += robot->time_taken;
}
// Compute fitness for a chromosome

// First, evaluate all robots' paths (keeps existing logic)
double compute_fitness(Chromosome *chromosome, Grid *grid, Config *config)
{
    // Reset stats
    chromosome->survivors_reached = 0;
    chromosome->cells_covered = 0;
    chromosome->total_path_length = 0.0;
    chromosome->total_risk = 0.0;
    chromosome->total_time = 0.0;

    // Create LOCAL tracking arrays
    int cells_visited[MAX_GRID_X * MAX_GRID_Y * MAX_GRID_Z] = {0};
    int survivor_rescued[1000] = {0};

    // Evaluate with local tracking
    for (int i = 0; i < chromosome->num_robots; i++)
    {
        evaluate_robot_path(&chromosome->robots[i], grid, chromosome,
                            cells_visited, survivor_rescued);
    }

    // Get raw metrics
    int survivors = chromosome->survivors_reached;
    int coverage = chromosome->cells_covered;
    double path_length = chromosome->total_path_length;
    double risk = chromosome->total_risk;
    double time = chromosome->total_time;

    // Define maximum possible values for normalization
    int max_survivors = grid->num_survivors;
    int max_coverage = grid->total_cells;

    // Estimate max path length (all robots use all moves)
    double max_path_length = MAX_MOVES * chromosome->num_robots * 1.414;
    // 1.414 = sqrt(2) for diagonal moves

    // Estimate max risk (worst case: all steps at critical risk)
    double max_risk = RISK_CRITICAL * MAX_MOVES * chromosome->num_robots;

    // Max time ≈ max path length (time correlates with distance)
    double max_time = max_path_length;

    // Normalize each metric to [0, 1] range

    // Survivors: higher is better, normalize to [0, 1]
    double norm_survivors = (max_survivors > 0) ? (double)survivors / max_survivors : 0.0;

    // Coverage: higher is better, normalize to [0, 1]
    double norm_coverage = (max_coverage > 0) ? (double)coverage / max_coverage : 0.0;

    // Path length: LOWER is better, so invert to [0, 1]
    // (shorter paths get higher scores)
    double norm_path_length = (max_path_length > 0) ? 1.0 - fmin(1.0, path_length / max_path_length) : 0.0;

    // Risk: LOWER is better, so invert to [0, 1]
    // (safer paths get higher scores)
    double norm_risk = (max_risk > 0) ? 1.0 - fmin(1.0, risk / max_risk) : 0.0;

    // Time: LOWER is better, so invert to [0, 1]
    // (faster completion gets higher scores)
    double norm_time = (max_time > 0) ? 1.0 - fmin(1.0, time / max_time) : 0.0;

    // Clamp all values to [0, 1] (safety check for edge cases)
    norm_survivors = fmax(0.0, fmin(1.0, norm_survivors));
    norm_coverage = fmax(0.0, fmin(1.0, norm_coverage));
    norm_path_length = fmax(0.0, fmin(1.0, norm_path_length));
    norm_risk = fmax(0.0, fmin(1.0, norm_risk));
    norm_time = fmax(0.0, fmin(1.0, norm_time));

    //  Apply weights (should sum to 1.0 in config)
    double fitness = (config->weight_survivors * norm_survivors + // 60% priority
                      config->weight_coverage * norm_coverage +   // 12% priority
                      config->weight_length * norm_path_length +  // 12% priority
                      config->weight_time * norm_time +           // 12% priority
                      config->weight_risk * norm_risk             // 4% priority
    );

    //  Scale to [0, 1000] for readability
    fitness *= 1000.0;

    chromosome->fitness = fitness;
    return fitness;
}
// UTILITY FUNCTIONS

// Copy robot (deep copy)
void copy_robot(Robot *dest, const Robot *src)
{
    if (!dest || !src)
        return;

    dest->id = src->id;
    dest->start_pos = src->start_pos;
    dest->current_pos = src->current_pos;
    dest->num_moves = src->num_moves;
    dest->path_length = src->path_length;
    dest->status = src->status;
    dest->survivors_rescued = src->survivors_rescued;
    dest->time_taken = src->time_taken;

    // Copy multi-trip fields
    dest->has_supplies = src->has_supplies;
    dest->trips_completed = src->trips_completed;
    dest->entry_point = src->entry_point;

    // Copy fog of war fields
    dest->num_discovered = src->num_discovered;
    memcpy(dest->discovered_obstacles, src->discovered_obstacles,
           src->num_discovered * sizeof(int));

    // Copy moves
    memcpy(dest->moves, src->moves, src->num_moves * sizeof(Direction));

    // Copy path
    memcpy(dest->path, src->path, src->path_length * sizeof(Vec3));
}

// Copy chromosome (deep copy)
void copy_chromosome(Chromosome *dest, const Chromosome *src)
{
    if (!dest || !src)
        return;

    dest->num_robots = src->num_robots;
    dest->fitness = src->fitness;
    dest->survivors_reached = src->survivors_reached;
    dest->cells_covered = src->cells_covered;
    dest->total_path_length = src->total_path_length;
    dest->total_risk = src->total_risk;
    dest->total_time = src->total_time;

    for (int i = 0; i < src->num_robots; i++)
    {
        copy_robot(&dest->robots[i], &src->robots[i]);
    }
}

// Compare chromosomes by fitness (for qsort, descending order)
int compare_fitness(const void *a, const void *b)
{
    const Chromosome *c1 = (const Chromosome *)a;
    const Chromosome *c2 = (const Chromosome *)b;

    if (c1->fitness > c2->fitness)
        return -1;
    else if (c1->fitness < c2->fitness)
        return 1;
    return 0;
}

// Print robot information
void print_robot(const Robot *robot)
{
    if (!robot)
        return;

    printf("  Robot %d:\n", robot->id);
    printf("    Start: (%d, %d, %d)\n",
           robot->start_pos.x, robot->start_pos.y, robot->start_pos.z);
    printf("    Moves: %d directions\n", robot->num_moves);
    printf("    Path length: %d steps\n", robot->path_length);
    printf("    Survivors rescued: %d\n", robot->survivors_rescued);
    printf("    Time taken: %.2f\n", robot->time_taken);

    // Print first few moves
    printf("    Move sequence (first 10): ");
    int show = (robot->num_moves < 10) ? robot->num_moves : 10;
    for (int i = 0; i < show; i++)
    {
        printf("%s ", direction_name(robot->moves[i]));
    }
    if (robot->num_moves > 10)
    {
        printf("... (%d more)", robot->num_moves - 10);
    }
    printf("\n");
}

// Print chromosome information
void print_chromosome(const Chromosome *chromosome, const char *label)
{
    if (!chromosome)
        return;

    printf("\n=== %s ===\n", label ? label : "Chromosome");
    printf("Fitness: %.2f\n", chromosome->fitness);
    printf("Survivors reached: %d\n", chromosome->survivors_reached);
    printf("Cells covered: %d\n", chromosome->cells_covered);
    printf("Total path length: %.2f\n", chromosome->total_path_length);
    printf("Total risk: %.2f\n", chromosome->total_risk);
    printf("Total time: %.2f\n", chromosome->total_time);
    printf("Number of robots: %d\n", chromosome->num_robots);

    for (int i = 0; i < chromosome->num_robots; i++)
    {
        print_robot(&chromosome->robots[i]);
    }
    printf("\n");
}

// GENETIC OPERATORS (ENHANCED BUT STILL RANDOM)
int tournament_selection(Chromosome *population, int pop_size, int tournament_size)
{
    if (!population || pop_size <= 0)
    {
        fprintf(stderr, "Error: Invalid population in tournament_selection\n");
        return 0;
    }

    if (tournament_size <= 0 || tournament_size > pop_size)
    {
        fprintf(stderr, "Warning: tournament_size %d invalid, using 2\n", tournament_size);
        tournament_size = 2;
    }

    // Pick initial candidate RANDOMLY
    int best_index = rand() % pop_size;
    double best_fitness = population[best_index].fitness;

    // Evaluate the rest of the tournament participants (ALL RANDOM)
    for (int i = 1; i < tournament_size; i++)
    {
        int idx = rand() % pop_size;
        double f = population[idx].fitness;

        if (f > best_fitness)
        {
            best_index = idx;
            best_fitness = f;
        }
    }

    return best_index;
}

// ENHANCEMENT: Multi-point crossover for more diversity (still random cut points)
void crossover_robot(Robot *parent1, Robot *parent2,
                     Robot *child1, Robot *child2, int robot_id)
{
    if (!parent1 || !parent2 || !child1 || !child2)
    {
        fprintf(stderr, "Error: NULL robot pointer in crossover_robot\n");
        return;
    }

    int len1 = parent1->num_moves;
    int len2 = parent2->num_moves;
    int min_len = (len1 < len2) ? len1 : len2;

    if (min_len < 2)
    {
        fprintf(stderr, "Warning: Robot %d has too few moves for crossover\n", robot_id);
        copy_robot(child1, parent1);
        copy_robot(child2, parent2);
        return;
    }

    // RANDOM CHOICE: Use 2-point crossover 50% of the time for more diversity
    double crossover_type = (double)rand() / RAND_MAX;

    if (crossover_type < 0.5 && min_len >= 4)
    {
        // TWO-POINT CROSSOVER (more exploration)
        int cut1 = 1 + rand() % (min_len - 2);
        int cut2 = cut1 + 1 + rand() % (min_len - cut1 - 1);


        // Child1: P1[0:cut1] + P2[cut1:cut2] + P1[cut2:end]
        memcpy(child1->moves, parent1->moves, cut1 * sizeof(Direction));
        memcpy(child1->moves + cut1, parent2->moves + cut1, (cut2 - cut1) * sizeof(Direction));
        memcpy(child1->moves + cut2, parent1->moves + cut2, (min_len - cut2) * sizeof(Direction));

        // Child2: P2[0:cut1] + P1[cut1:cut2] + P2[cut2:end]
        memcpy(child2->moves, parent2->moves, cut1 * sizeof(Direction));
        memcpy(child2->moves + cut1, parent1->moves + cut1, (cut2 - cut1) * sizeof(Direction));
        memcpy(child2->moves + cut2, parent2->moves + cut2, (min_len - cut2) * sizeof(Direction));
    }
    else
    {
        // SINGLE-POINT CROSSOVER (classic)
        int cut = 1 + rand() % (min_len - 1);

        // Child1 = P1[0:cut] + P2[cut:end]
        memcpy(child1->moves, parent1->moves, cut * sizeof(Direction));
        memcpy(child1->moves + cut, parent2->moves + cut, (min_len - cut) * sizeof(Direction));

        // Child2 = P2[0:cut] + P1[cut:end]
        memcpy(child2->moves, parent2->moves, cut * sizeof(Direction));
        memcpy(child2->moves + cut, parent1->moves + cut, (min_len - cut) * sizeof(Direction));
    }

    child1->num_moves = min_len;
    child2->num_moves = min_len;

    // Reset runtime fields
    child1->id = parent1->id;
    child2->id = parent2->id;
    child1->start_pos = parent1->start_pos;
    child2->start_pos = parent2->start_pos;
    child1->path_length = 0;
    child2->path_length = 0;
    child1->survivors_rescued = 0;
    child2->survivors_rescued = 0;
    child1->time_taken = 0.0;
    child2->time_taken = 0.0;
}

void mutate_robot(Robot *robot, double mutation_rate)
{
    if (!robot || robot->num_moves <= 0)
    {
        fprintf(stderr, "Error: Invalid robot in mutate_robot\n");
        return;
    }

    // Calculate how many moves to mutate
    int num_to_mutate = (int)(robot->num_moves * mutation_rate);

    // Ensure at least 1 mutation if rate > 0 and we have moves
    if (num_to_mutate == 0 && mutation_rate > 0.0 && robot->num_moves > 0)
    {
        num_to_mutate = 1;
    }

    int mutations = 0;

    // Mutate exactly num_to_mutate random positions
    for (int m = 0; m < num_to_mutate; m++)
    {
        // Pick a random position to mutate
        int idx = rand() % robot->num_moves;

        Direction old = robot->moves[idx];
        Direction new;

        // RANDOM CHOICE: Pick mutation strategy
        double strategy = (double)rand() / RAND_MAX;

        if (strategy < 0.7)
        {
            // POINT MUTATION (70%): Replace with completely random direction
            new = (Direction)(rand() % NUM_DIRECTIONS);
        }
        else
        {
            // SWAP MUTATION (30%): Swap with another random position
            if (robot->num_moves > 1)
            {
                int swap_idx = rand() % robot->num_moves;
                new = robot->moves[swap_idx];
                robot->moves[swap_idx] = old;
            }
            else
            {
                new = (Direction)(rand() % NUM_DIRECTIONS);
            }
        }

        robot->moves[idx] = new;
        mutations++;
    }

    // ENHANCEMENT: Occasionally do a block shuffle (rare, but adds diversity)
    if (robot->num_moves >= 4)
    {
        double block_shuffle_chance = (double)rand() / RAND_MAX;
        if (block_shuffle_chance < 0.05)
        { // 5% chance
            // Shuffle a random block of moves
            int block_start = rand() % (robot->num_moves - 2);
            int block_len = 2 + rand() % 3; // 2-4 moves
            if (block_start + block_len > robot->num_moves)
            {
                block_len = robot->num_moves - block_start;
            }

            // Fisher-Yates shuffle on the block
            for (int i = block_start + block_len - 1; i > block_start; i--)
            {
                int j = block_start + rand() % (i - block_start + 1);
                Direction temp = robot->moves[i];
                robot->moves[i] = robot->moves[j];
                robot->moves[j] = temp;
            }

            mutations++;
        }
    }

    // Reset runtime fields
    robot->path_length = 0;
    robot->survivors_rescued = 0;
    robot->time_taken = 0.0;
    robot->current_pos = robot->start_pos;
}

void apply_elitism(Chromosome *population, Chromosome *new_population,
                   int pop_size, int elite_count)
{
    if (!population || !new_population)
    {
        fprintf(stderr, "Error: NULL population pointer in apply_elitism\n");
        return;
    }

    if (elite_count <= 0)
    {
        printf("[GA][ELITE] Elitism disabled (elite_count=0)\n");
        return;
    }

    if (elite_count > pop_size)
    {
        fprintf(stderr, "Warning: Elite count %d > pop_size %d, capping\n",
                elite_count, pop_size);
        elite_count = pop_size;
    }
    // Population should already be sorted (best first)
    for (int i = 0; i < elite_count; i++)
    {
        copy_chromosome(&new_population[i], &population[i]);
    }
}

// =============================================================================
// SINGLE GENERATION (Main GA loop body)
// =============================================================================

void run_single_generation(Chromosome *population,
                           Chromosome *new_population,
                           int pop_size,
                           Config *config)
{
    if (!population || !new_population || !config)
    {
        fprintf(stderr, "Error: NULL input to run_single_generation\n");
        return;
    }

    int elite_count = (int)(config->elitism_rate * pop_size);
    if (elite_count < 0)
        elite_count = 0;
    if (elite_count > pop_size)
        elite_count = pop_size;

    // Sort population by fitness
    qsort(population, pop_size, sizeof(Chromosome), compare_fitness);

    printf("[GA] Best current fitness = %.2f\n", population[0].fitness);

    // Apply elitism
    apply_elitism(population, new_population, pop_size, elite_count);

    int filled = elite_count;

    while (filled < pop_size)
    {
        // TOURNAMENT SELECTION (random)
        int p1 = tournament_selection(population, pop_size, 3);
        int p2 = tournament_selection(population, pop_size, 3);

        if (p1 == p2)
            p2 = (p2 + 1) % pop_size;

        Chromosome *parent1 = &population[p1];
        Chromosome *parent2 = &population[p2];

        // Create fresh children
        Chromosome child1 = {0};
        Chromosome child2 = {0};

        child1.num_robots = parent1->num_robots;
        child2.num_robots = parent2->num_robots;

        // CROSSOVER (random cut points)
        double roll = (double)rand() / RAND_MAX;

        if (roll < config->crossover_rate)
        {
            for (int r = 0; r < parent1->num_robots; r++)
            {
                crossover_robot(&parent1->robots[r],
                                &parent2->robots[r],
                                &child1.robots[r],
                                &child2.robots[r],
                                r);
            }
        }
        else
        {
            copy_chromosome(&child1, parent1);
            copy_chromosome(&child2, parent2);
        }

        // MUTATION (random changes)
        for (int r = 0; r < child1.num_robots; r++)
        {
            mutate_robot(&child1.robots[r], config->mutation_rate);
            mutate_robot(&child2.robots[r], config->mutation_rate);
        }

        // Reset runtime fields
        child1.fitness = 0;
        child1.survivors_reached = 0;
        child1.cells_covered = 0;
        child1.total_risk = 0;
        child1.total_time = 0;
        child1.total_path_length = 0;

        child2.fitness = 0;
        child2.survivors_reached = 0;
        child2.cells_covered = 0;
        child2.total_risk = 0;
        child2.total_time = 0;
        child2.total_path_length = 0;

        // Insert children
        copy_chromosome(&new_population[filled], &child1);
        filled++;

        if (filled < pop_size)
        {
            copy_chromosome(&new_population[filled], &child2);
            filled++;
        }
    }

}

void run_ga(Chromosome *population, int pop_size, Grid *grid, Config *config)
{
    if (!population || !grid || !config || pop_size <= 0)
    {
        fprintf(stderr, "Error: Invalid parameters passed to run_ga\n");
        return;
    }

    // Stable pointer to caller memory (what parent allocated)
    Chromosome *orig_population = population;

    int max_gens = config->num_generations;
    int stagnation_limit = config->max_stagnation;

    // Allocate ONE buffer and ALWAYS free THIS pointer (never free caller memory)
    Chromosome *buffer = (Chromosome *)malloc(sizeof(Chromosome) * pop_size);
    if (!buffer)
    {
        fprintf(stderr, "Error: Memory allocation failed for GA buffer\n");
        return;
    }

    // Work pointers (we can swap these safely)
    Chromosome *cur = population;
    Chromosome *next = buffer;

    double prev_best_fitness = -1e9;
    int stagnation_counter = 0;

    // Track best chromosome across the whole GA (deep copy)
    Chromosome best_chr = (Chromosome){0};
    double best_fitness_overall = -1e9;

    int evaluated = evaluate_population_parallel(cur, pop_size, grid, config);
    if (evaluated != pop_size)
    {
        fprintf(stderr, "Error: Only %d/%d individuals evaluated!\n", evaluated, pop_size);
    }

    qsort(cur, pop_size, sizeof(Chromosome), compare_fitness);

    prev_best_fitness = cur[0].fitness;
    best_fitness_overall = prev_best_fitness;
    copy_chromosome(&best_chr, &cur[0]);

    printf("[GA] Initial best fitness = %.2f\n\n", prev_best_fitness);

    // Main GA loop
    for (int gen = 1; gen <= max_gens; gen++)
    {
        run_single_generation(cur, next, pop_size, config);

        int evaluated2 = evaluate_population_parallel(next, pop_size, grid, config);
        if (evaluated2 != pop_size)
        {
            fprintf(stderr, "Error: Only %d/%d individuals evaluated in generation %d!\n",
                    evaluated2, pop_size, gen);
        }

        qsort(next, pop_size, sizeof(Chromosome), compare_fitness);

        double best_now = next[0].fitness;

        // Stagnation check
        if (best_now <= prev_best_fitness)
        {
            stagnation_counter++;
            printf("[GA] No improvement (stagnation %d/%d)\n",
                   stagnation_counter, stagnation_limit);
        }
        else
        {
            stagnation_counter = 0;
            prev_best_fitness = best_now;
        }

        // Best overall tracking
        if (best_now > best_fitness_overall)
        {
            best_fitness_overall = best_now;
            copy_chromosome(&best_chr, &next[0]);
        }

        if (stagnation_counter >= stagnation_limit)
        {
            printf("\n[GA] EARLY STOPPING — stagnation reached %d generations\n",
                   stagnation_limit);
            break;
        }

        // Swap work buffers
        Chromosome *tmp = cur;
        cur = next;
        next = tmp;
    }

    // Write best result back into caller-owned array at index 0
    copy_chromosome(&orig_population[0], &best_chr);

    printf("\n==== [GA] Full GA complete — best fitness = %.2f\n", best_fitness_overall);

    printf("\n========== Final Best Solution ==========\n");
    printf("  Best fitness: %.2f\n", best_chr.fitness);
    printf("  Survivors: %d/%d\n", best_chr.survivors_reached, grid->num_survivors);
    printf("  Coverage: %d cells\n", best_chr.cells_covered);
    printf("  Total path length: %.2f\n", best_chr.total_path_length);
    printf("  Total risk: %.2f\n", best_chr.total_risk);
    printf("  Total time: %.2f\n", best_chr.total_time);
    printf("=========================================\n\n");

    free(buffer);
}

// =============================================================================
// SEQUENTIAL EVALUATION (NO PARALLELIZATION)
// =============================================================================

int evaluate_population_sequential(Chromosome *population, int pop_size,
                                   Grid *grid, Config *config)
{
    if (!population || !grid || !config || pop_size <= 0)
    {
        fprintf(stderr, "[SEQ] Error: Invalid parameters\n");
        return -1;
    }

    printf("\n[GA-SEQ] Evaluating %d chromosomes SEQUENTIALLY (no workers)\n", pop_size);

    for (int i = 0; i < pop_size; i++)
    {
        // Generate paths from move sequences
        generate_chromosome_paths(&population[i], grid);

        // Compute fitness directly (no IPC, no workers)
        compute_fitness(&population[i], grid, config);
    }
    return pop_size;
}