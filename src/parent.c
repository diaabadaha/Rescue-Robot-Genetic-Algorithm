#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>    // fork, execl
#include <sys/types.h>  // pid_t
#include <sys/wait.h>
#include <signal.h>

#include "../include/ipc.h"
#include "../include/menu.h"
#include "../include/config.h"
#include "../include/grid.h"
#include "../include/common.h"
#include "../include/ga.h"
#include "../include/child_pool.h"
#include "../include/astar.h"
#include "../include/performance_comparison.h"

// GLOBAL STATE (Parent Process Only - Not in Shared Memory)

// Persistent population for "Run Single Generation" option
static Chromosome *global_population = NULL;
static int global_pop_size = 0;
static int generation_counter = 0;

// Global resources (initialized once, used throughout session)
static Grid *global_grid = NULL;
static Config *global_config = NULL;
static int workers_initialized = 0;
static int ga_workers_active = 0; // Track if option 1 workers are still active

// HELPER FUNCTIONS - DISPLAY
void display_layer(Grid *grid, int z)
{
    if (!grid || z < 0 || z >= grid->size_z)
    {
        fprintf(stderr, "Error: Invalid layer %d\n", z);
        return;
    }

    printf("  Layer %d (z=%d)\n", z, z);
    printf("─────────────────────────────────────────────────────────────\n");

    for (int y = grid->size_y - 1; y >= 0; y--)
    {
        printf("  ");
        for (int x = 0; x < grid->size_x; x++)
        {
            Vec3 pos = {x, y, z};
            Cell *cell = get_cell_vec(grid, pos);

            if (!cell)
            {
                printf("? ");
                continue;
            }

            // Check if this is entry point
            if (vec3_equal(pos, grid->entry_point))
            {
                printf("E ");
            }
            // Check if survivor location
            else if (cell->has_survivor)
            {
                printf("S ");
            }
            // Check if obstacle
            else if (cell->type == CELL_OBSTACLE)
            {
                printf("# ");
            }
            // Empty cell
            else
            {
                printf(". ");
            }
        }
        printf("\n");
    }
}


void display_all_layers(Grid *grid)
{
    if (!grid)
    {
        fprintf(stderr, "Error: NULL grid\n");
        return;
    }

    printf("\n============ GRID VISUALIZATION ============\n");

    printf("\nGrid Dimensions: %d x %d x %d\n",
           grid->size_x, grid->size_y, grid->size_z);
    printf("Entry Point: (%d, %d, %d)\n",
           grid->entry_point.x, grid->entry_point.y, grid->entry_point.z);
    printf("Survivors: %d\n", grid->num_survivors);

    // Show survivor positions
    printf("\nSurvivor Locations:\n");
    for (int i = 0; i < grid->num_survivors; i++)
    {
        Vec3 pos = grid->survivor_positions[i];
        printf("  Survivor %d: (%d, %d, %d)\n", i, pos.x, pos.y, pos.z);
    }

    // Display legend
    printf("\nLegend:\n");
    printf("  . = Empty cell\n");
    printf("  # = Obstacle (debris/rubble)\n");
    printf("  S = Survivor location\n");
    printf("  E = Entry point\n");

    // Display each layer
    for (int z = 0; z < grid->size_z; z++)
        display_layer(grid, z);
}

// HELPER FUNCTIONS - GA vs A* COMPARISON

/**
 * Compare GA best solution with A* baseline
 */
void compare_ga_vs_astar(Chromosome *ga_best, Chromosome *astar_baseline, Grid *grid)
{
    if (!ga_best || !astar_baseline || !grid)
    {
        fprintf(stderr, "Error: NULL parameters in compare_ga_vs_astar\n");
        return;
    }

    printf("\n=========== GA vs A* BASELINE COMPARISON ============\n");

    double survivor_ratio = 0.0;
    if (astar_baseline->survivors_reached > 0)
        survivor_ratio = (ga_best->survivors_reached * 100.0) / astar_baseline->survivors_reached;

    double coverage_ratio = 0.0;
    if (astar_baseline->cells_covered > 0)
        coverage_ratio = (ga_best->cells_covered * 100.0) / astar_baseline->cells_covered;

    double length_ratio = 0.0;
    if (astar_baseline->total_path_length > 0)
        length_ratio = (ga_best->total_path_length * 100.0) / astar_baseline->total_path_length;

    double risk_ratio = 0.0;
    if (astar_baseline->total_risk > 0)
        risk_ratio = (ga_best->total_risk * 100.0) / astar_baseline->total_risk;

    double fitness_ratio = 0.0;
    if (astar_baseline->fitness > 0)
        fitness_ratio = (ga_best->fitness * 100.0) / astar_baseline->fitness;

    printf("                    GA Best      A* Baseline    Ratio (GA/A*)\n");
    printf("─────────────────────────────────────────────────────────────\n");
    printf("Survivors:          %d / %d        %d / %d          %.1f%%\n",
           ga_best->survivors_reached, grid->num_survivors,
           astar_baseline->survivors_reached, grid->num_survivors,
           survivor_ratio);

    printf("Coverage:           %-12d %-14d %.1f%%\n",
           ga_best->cells_covered,
           astar_baseline->cells_covered,
           coverage_ratio);

    printf("Path Length:        %-12.2f %-14.2f %.1f%%\n",
           ga_best->total_path_length,
           astar_baseline->total_path_length,
           length_ratio);

    printf("Total Risk:         %-12.2f %-14.2f %.1f%%\n",
           ga_best->total_risk,
           astar_baseline->total_risk,
           risk_ratio);

    printf("Fitness:            %-12.2f %-14.2f %.1f%%\n",
           ga_best->fitness,
           astar_baseline->fitness,
           fitness_ratio);

    printf("\n─────────────────────────────────────────────────────────────\n");
    printf("Analysis:\n");

    if (ga_best->survivors_reached == astar_baseline->survivors_reached)
        printf("GA found all survivors (same as A*)\n");
    else if (ga_best->survivors_reached > astar_baseline->survivors_reached)
        printf("GA found MORE survivors than A*! (%.0f%%)\n", survivor_ratio);
    else
        printf("GA found fewer survivors than A* (%.0f%%)\n", survivor_ratio);

    if (coverage_ratio > 100.0)
        printf("GA explored %.1f%% more cells (better coverage)\n", coverage_ratio - 100.0);
    else
        printf("GA explored %.1f%% fewer cells than A*\n", 100.0 - coverage_ratio);

    if (length_ratio > 100.0)
        printf("GA path is %.1f%% longer than A* (less efficient)\n", length_ratio - 100.0);
    else
        printf("GA path is %.1f%% shorter than A*!\n", 100.0 - length_ratio);

    if (risk_ratio > 100.0)
        printf("GA took %.1f%% more risk than A*\n", risk_ratio - 100.0);
    else
        printf("GA took %.1f%% less risk than A*\n", 100.0 - risk_ratio);

    printf("\nOverall: GA solution is %.1f%% as good as A* optimal baseline\n",
           fitness_ratio);
}

// =============================================================================
// HELPER FUNCTIONS - POPULATION MANAGEMENT
// =============================================================================

/**
 * Initialize global population for "Run Single Generation"
 */
void init_global_population()
{
    if (global_population != NULL)
    {
        printf("[MAIN] Warning: Global population already exists, freeing old one\n");
        free(global_population);
    }

    global_pop_size = global_config->population_size;
    global_population = (Chromosome *)malloc(sizeof(Chromosome) * global_pop_size);

    if (!global_population)
    {
        fprintf(stderr, "[MAIN] Error: Failed to allocate global population\n");
        global_pop_size = 0;
        return;
    }

    // Initialize with random chromosomes
    for (int i = 0; i < global_pop_size; i++)
    {
        init_random_chromosome(&global_population[i],
                               global_config->num_robots,
                               global_grid->entry_point,
                               200);
    }

    generation_counter = 0;
}

static int robot_path_contains(const Robot *rb, Vec3 p)
{
    if (!rb || !rb->path || rb->path_length <= 0) return 0;

    int plen = rb->path_length;
    if (plen > MAX_PATH_LENGTH) plen = MAX_PATH_LENGTH;

    for (int i = 0; i < plen; i++)
        if (vec3_equal(rb->path[i], p)) return 1;

    return 0;
}

static void shm_update_survivor_helped_from_best(Chromosome *best)
{
    if (!best) return;

    SharedMemory *shm = get_shared_memory();
    if (!shm || shm->initialized != 1) return;

    Grid *shared_grid = get_shared_grid(shm);
    if (!shared_grid) return;

    for (int s = 0; s < shared_grid->num_survivors; s++)
    {
        Vec3 sp = shared_grid->survivor_positions[s];
        Cell *c = get_cell_vec(shared_grid, sp);
        if (!c) continue;

        int reached = 0;
        for (int r = 0; r < best->num_robots; r++)
        {
            if (robot_path_contains(&best->robots[r], sp))
            {
                reached = 1;
                break;
            }
        }

        c->has_survivor = 1;
        c->survivor_helped = reached ? 1 : 0;
    }

    free_shared_grid(shared_grid);
}

// =============================================================================
// MENU OPTIONS
// =============================================================================

/**
 * Menu Option 1: Run Full Genetic Algorithm
 */
void menu_run_full_ga()
{
    printf("\n======= Parallel Fitness Evaluation =======\n");

    // Launch GUI (child process)
    pid_t gui_pid = fork();
    if (gui_pid == 0)
    {
        execl("./bin/rescue_robot_gui", "rescue_robot_gui", NULL);
        perror("[PARENT] Failed to launch GUI");
        _exit(1);
    }

    int pop_size = global_config->population_size;

    Chromosome *temp_population = (Chromosome *)malloc(sizeof(Chromosome) * pop_size);
    if (!temp_population)
    {
        fprintf(stderr, "[GA] Error: Failed to allocate temporary population\n");
        if (gui_pid > 0)
        {
            kill(gui_pid, SIGTERM);
            waitpid(gui_pid, NULL, 0);
        }
        return;
    }

    for (int i = 0; i < pop_size; i++)
    {
        init_random_chromosome(&temp_population[i],
                               global_config->num_robots,
                               global_grid->entry_point,
                               200);
    }

    if (init_worker_pool(global_config->num_workers, global_config, global_grid) == -1)
    {
        fprintf(stderr, "[GA] Error: Failed to initialize worker pool\n");
        free(temp_population);

        if (gui_pid > 0)
        {
            kill(gui_pid, SIGTERM);
            waitpid(gui_pid, NULL, 0);
        }
        return;
    }

    run_ga(temp_population, pop_size, global_grid, global_config);

    // Ensure best chromosome has paths/fitness computed (safe for GUI + printing)
    generate_chromosome_paths(&temp_population[0], global_grid);
    compute_fitness(&temp_population[0], global_grid, global_config);
    // Publish FINAL best to GUI slot 0 (persistent)
SharedMemory *shm = get_shared_memory();
if (shm)
{
    sem_wait(&shm->slot_mutex[0]);
    shm->jobs[0].chromosome = temp_population[0];
    shm->jobs[0].fitness = temp_population[0].fitness;
    shm->jobs[0].survivors_reached = temp_population[0].survivors_reached;
    shm->jobs[0].cells_covered = temp_population[0].cells_covered;
    shm->jobs[0].total_path_length = temp_population[0].total_path_length;
    shm->jobs[0].total_risk = temp_population[0].total_risk;
    shm->jobs[0].total_time = temp_population[0].total_time;
    shm->jobs[0].job_id = -999;          // reserved marker (optional)
    shm->jobs[0].status = JOB_DONE;      // IMPORTANT
    sem_post(&shm->slot_mutex[0]);
}


    // Update shared memory survivor colors based on best path
    shm_update_survivor_helped_from_best(&temp_population[0]);

    // Let GUI poll the final state
    sleep(1);

    // NOTE: unreachable-survivor debug analysis removed (it was causing crashes)

    // Summary
    printf("GA: survivors=%d/%d cov=%d len=%.2f risk=%.2f fit=%.2f\n",
           temp_population[0].survivors_reached, global_grid->num_survivors,
           temp_population[0].cells_covered, temp_population[0].total_path_length,
           temp_population[0].total_risk, temp_population[0].fitness);

    // A* baseline
    Chromosome astar_baseline;
    memset(&astar_baseline, 0, sizeof(Chromosome));
    build_astar_baseline_paths(global_grid, global_config, &astar_baseline);

    int astar_survivors = 0;
    int astar_coverage = 0;
    double astar_path_length = 0.0;
    double astar_risk = 0.0;

    evaluate_astar_baseline(&astar_baseline, global_grid,
                            &astar_survivors, &astar_coverage,
                            &astar_path_length, &astar_risk);

    printf("A*: survivors=%d/%d cov=%d len=%.2f risk=%.2f\n",
           astar_survivors, global_grid->num_survivors,
           astar_coverage, astar_path_length, astar_risk);

    // Cleanup pool + population (after GUI got the final view)
    // Keep GUI open so you can watch the final solution
if (gui_pid > 0)
{
    printf("[GUI] Close the GUI window to continue...\n");
    waitpid(gui_pid, NULL, 0);   // waits until user closes GUI
}

cleanup_worker_pool();
workers_initialized = 0;
free(temp_population);
}

/**
 * Menu Option 2: Load and Display Map
 */
void menu_load_display_map()
{
    printf("\n ===== Grid Map Display =====\n");

    if (!global_grid)
    {
        fprintf(stderr, "[MAP] Error: Grid not loaded\n");
        return;
    }

    print_grid_stats(global_grid);
    display_all_layers(global_grid);
}

/**
 * Menu Option 3: Load/Generate New Map
 */
void menu_load_generate_new_map()
{
    printf("\n===== Load/Generate New Map =====\n");
    printf("1. Load map from file\n");
    printf("2. Generate random map\n");
    printf("Enter choice (1-2): ");

    int choice;
    if (scanf("%d", &choice) != 1)
    {
        while (getchar() != '\n')
            ;
        printf("[MAP] Invalid input\n");
        return;
    }
    while (getchar() != '\n')
        ;

    Grid *new_grid = NULL;

    if (choice == 1)
    {
        // Load from file
        printf("Enter map file path: ");
        char filepath[256];
        if (fgets(filepath, sizeof(filepath), stdin))
        {
            filepath[strcspn(filepath, "\n")] = 0;

            if (strlen(filepath) == 0)
            {
                printf("[MAP] Error: No path provided\n");
                return;
            }

            new_grid = load_map_from_file(filepath);
            if (!new_grid)
            {
                printf("[MAP] Error: Invalid path or file format '%s'\n", filepath);
                return;
            }
            printf("[MAP] Successfully loaded from '%s'\n\n", filepath);
        }
    }
    else if (choice == 2)
    {
        // Generate random map
        printf("[MAP] Generating random pyramid map (10×10×3, 8 survivors)...\n");
        new_grid = generate_pyramid_rubble(10, 10, 3, 5, 5, 3.0, 0.6, 0.2, 8);
        if (!new_grid)
        {
            printf("[MAP] Error: Failed to generate map\n");
            return;
        }
        printf("[MAP] Random map generated successfully\n\n");
    }
    else
    {
        printf("[MAP] Invalid choice\n");
        return;
    }

    // Replace global grid
    if (global_grid)
    {
        free_grid(global_grid);
    }
    global_grid = new_grid;

    // Reset population if it exists (new map = new problem)
    if (global_population)
    {
        free(global_population);
        global_population = NULL;
        global_pop_size = 0;
        generation_counter = 0;
        printf("[MAP] Previous population cleared (new map loaded)\n");
    }

    // Display the new map
    print_grid_stats(global_grid);
    display_all_layers(global_grid);
}

/**
 * Menu Option 3: View Current Configuration
 */
void menu_view_config()
{
    printf("\n");
    printf("===== Configuration Display =====\n");

    if (!global_config)
    {
        fprintf(stderr, "[CONFIG] Error: Configuration not loaded\n");
        return;
    }

    print_config(global_config);

    printf("Global Population Status:\n");
    if (global_population)
    {
        printf("  Status: INITIALIZED\n");
        printf("  Size: %d chromosomes\n", global_pop_size);
        printf("  Current generation: %d\n", generation_counter);
        if (generation_counter > 0)
            printf("  Best fitness: %.2f\n", global_population[0].fitness);
    }
    else
    {
        printf("  Status: NOT INITIALIZED\n");
        printf("  (Will be created on first 'Run Single Generation')\n");
    }
    printf("\n");
}

/**
 * Menu Option 4: Test Worker Pool
 */
void menu_test_workers()
{
    printf("\n======= Worker Pool Test =======\n");

    // Use population size from config
    int test_pop_size = global_config->population_size;
    printf("[TEST] Testing with population size: %d\n", test_pop_size);
    printf("[TEST] Number of workers: %d\n\n", global_config->num_workers);

    // Create temporary test population
    Chromosome *test_population = (Chromosome *)malloc(sizeof(Chromosome) * test_pop_size);
    if (!test_population)
    {
        fprintf(stderr, "[TEST] Error: Failed to allocate test population\n");
        return;
    }

    // Initialize random chromosomes
    printf("[TEST] Initializing test population...\n");
    for (int i = 0; i < test_pop_size; i++)
    {
        init_random_chromosome(&test_population[i], 
                              global_config->num_robots,
                              global_grid->entry_point, 
                              250);
    }

    // Evaluate in parallel
    printf("[TEST] Evaluating population in parallel...\n\n");

    // Ensure workers are initialized for test
    if (!workers_initialized)
    {
        printf("[TEST] Initializing worker pool...\n");
        if (init_worker_pool(global_config->num_workers, global_config, global_grid) == -1)
        {
            fprintf(stderr, "[TEST] Error: Failed to initialize worker pool\n");
            free(test_population);
            return;
        }
        workers_initialized = 1;
    }

    int evaluated = evaluate_population_parallel(test_population, test_pop_size,
                                                 global_grid, global_config);

    if (evaluated != test_pop_size)
    {
        fprintf(stderr, "[TEST] Warning: Only %d/%d chromosomes evaluated\n",
                evaluated, test_pop_size);
    }
    else
    {
        printf("\n[TEST] All chromosomes evaluated successfully!\n");
    }

    // Display results summary
    printf("\n ===== Test Results =====\n");

    double total_fitness = 0.0;
    double max_fitness = test_population[0].fitness;
    double min_fitness = test_population[0].fitness;
    int best_idx = 0;

    for (int i = 0; i < test_pop_size; i++)
    {
        total_fitness += test_population[i].fitness;

        if (test_population[i].fitness > max_fitness)
        {
            max_fitness = test_population[i].fitness;
            best_idx = i;
        }

        if (test_population[i].fitness < min_fitness)
        {
            min_fitness = test_population[i].fitness;
        }
    }

    double avg_fitness = total_fitness / test_pop_size;

    printf("Statistics:\n");
    printf("  Chromosomes evaluated: %d\n", evaluated);
    printf("  Average fitness: %.2f\n", avg_fitness);
    printf("  Max fitness: %.2f (Chromosome %d)\n", max_fitness, best_idx);
    printf("  Min fitness: %.2f\n", min_fitness);
    printf("\n");

    printf("Best Chromosome (#%d):\n", best_idx);
    printf("  Fitness: %.2f\n", test_population[best_idx].fitness);
    printf("  Survivors: %d/%d\n", 
           test_population[best_idx].survivors_reached, 
           global_grid->num_survivors);
    printf("  Coverage: %d cells\n", test_population[best_idx].cells_covered);
    printf("  Path length: %.2f\n", test_population[best_idx].total_path_length);
    printf("\n");

    printf("\n[TEST] Worker pool test complete!\n\n");

    // Free test population
    free(test_population);
}

/**
 * Menu Option 5: Run Single Generation
 */
void menu_run_single_generation()
{
    // Check if global population exists
    if (!global_population)
    {
        printf("[SINGLE] init population (n=%d)\n", global_config->population_size);
        init_global_population();
    }

    // Initialize worker pool if not already initialized
    if (!workers_initialized)
    {
        printf("[SINGLE] init workers (n=%d)\n", global_config->num_workers);
        if (init_worker_pool(global_config->num_workers, global_config, global_grid) == -1)
        {
            fprintf(stderr, "[SINGLE] ERR: init_worker_pool failed\n");
            return;
        }
        workers_initialized = 1;
    }

    // Evaluate current generation if not yet evaluated
    if (generation_counter == 0)
    {
        evaluate_population_parallel(global_population, global_pop_size,
                                     global_grid, global_config);

        qsort(global_population, global_pop_size, sizeof(Chromosome), compare_fitness);

        printf("[SINGLE] gen0 fit=%.2f surv=%d/%d cov=%d\n",
               global_population[0].fitness,
               global_population[0].survivors_reached,
               global_grid->num_survivors,
               global_population[0].cells_covered);
    }

    // Create new population array for offspring
    Chromosome *new_population = (Chromosome *)malloc(sizeof(Chromosome) * global_pop_size);
    if (!new_population)
    {
        fprintf(stderr, "[SINGLE] ERR: malloc new_population\n");
        return;
    }

    generation_counter++;
    printf("[SINGLE] Running Generation %d\n", generation_counter);
    // Run single generation (selection, crossover, mutation)
    run_single_generation(global_population, new_population,
                          global_pop_size, global_config);

    // Evaluate new generation
    evaluate_population_parallel(new_population, global_pop_size,
                                 global_grid, global_config);

    qsort(new_population, global_pop_size, sizeof(Chromosome), compare_fitness);

    // Compact generation summary (one line)
    printf("[SINGLE] gen%d fit=%.2f surv=%d/%d cov=%d len=%.2f risk=%.2f\n",
           generation_counter,
           new_population[0].fitness,
           new_population[0].survivors_reached,
           global_grid->num_survivors,
           new_population[0].cells_covered,
           new_population[0].total_path_length,
           new_population[0].total_risk);

    // Replace old population with new
    free(global_population);
    global_population = new_population;
}

// =============================================================================
// INITIALIZATION AND CLEANUP
// =============================================================================

int initialize_system(const char *config_file)
{
    global_config = load_config(config_file);
    if (!global_config)
    {
        printf("[INIT] Using default configuration\n");
        global_config = create_default_config();
        if (!global_config)
        {
            fprintf(stderr, "[INIT] Error: Failed to create configuration\n");
            return -1;
        }
    }

    if (global_config->random_seed == 0)
        srand(time(NULL));
    else
        srand((unsigned int)global_config->random_seed);

    global_grid = load_map_from_file(global_config->map_file);
    if (!global_grid)
    {
        global_grid = generate_pyramid_rubble(
            global_config->grid_x,
            global_config->grid_y,
            global_config->grid_z,
            global_config->grid_x / 2,
            global_config->grid_y / 2,
            global_config->grid_x / 3.0,
            0.6, 0.2,
            3);

        if (!global_grid)
        {
            fprintf(stderr, "[INIT] Error: Failed to generate grid\n");
            free_config(global_config);
            return -1;
        }
    }

    return 0;
}

void cleanup_system()
{
    if (global_population)
    {
        free(global_population);
        global_population = NULL;
        global_pop_size = 0;
        generation_counter = 0;
    }

    if (global_grid)
    {
        free_grid(global_grid);
        global_grid = NULL;
    }

    if (global_config)
    {
        free_config(global_config);
        global_config = NULL;
    }
}

// =============================================================================
// MAIN FUNCTION
// =============================================================================

int main(int argc, char *argv[])
{
    const char *config_file = (argc > 1) ? argv[1] : "config/params.txt";

    if (initialize_system(config_file) == -1)
    {
        fprintf(stderr, "Fatal error: System initialization failed\n");
        return 1;
    }

    MenuLayout *menu = get_menu_layout(global_config->menu_file);
    if (!menu)
    {
        fprintf(stderr, "Fatal error: Failed to load menu\n");
        cleanup_system();
        return 1;
    }

    int running = 1;
    while (running)
    {
        display_menu(menu);
        MenuOption choice = get_user_choice(menu);

        switch (choice)
        {
        case MENU_RUN_GA:
            menu_run_full_ga();
            break;

        case MENU_LOAD_MAP:
            menu_load_display_map();
            break;

        case MENU_LOAD_GENERATE_MAP:
            menu_load_generate_new_map();
            break;

        case MENU_VIEW_CONFIG:
            menu_view_config();
            break;

        case MENU_TEST_WORKERS:
            menu_test_workers();
            break;

        case MENU_RUN_SINGLE_GEN:
            printf("\n===== Run Single Generation =====\n");
            menu_run_single_generation();
            break;

        case MENU_PERFORMANCE_COMPARISON:
            if (!global_grid)
            {
                printf("[PERF] Error: Map not loaded\n");
                break;
            }
            run_performance_comparison(global_grid, global_config);
            break;

        case MENU_EXIT:
            running = 0;
            break;

        case MENU_INVALID:
        default:
            printf("\n[MAIN] Invalid choice. Please try again.\n\n");
            break;
        }
    }

    free_menu_layout(menu);
    cleanup_system();
    return 0;
}
