// performance_comparison.c - Complete Performance Comparison Suite
// Tests 4 algorithm variants: A*, Sequential GA, Parallel GA (poor params), Parallel GA (tuned params)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "../include/performance_comparison.h"
#include "../include/ga.h"
#include "../include/child_pool.h"
#include "../include/astar.h"

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

double get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

Config *create_poor_config(Config *base_config)
{
    Config *poor = malloc(sizeof(Config));
    if (!poor)
        return NULL;

    // Copy base config
    memcpy(poor, base_config, sizeof(Config));

    // Set POOR weights (survivors barely matter)
    poor->weight_survivors = 0.01; // Only 1%! (vs 50% tuned)
    poor->weight_coverage = 0.50;  // Dominates
    poor->weight_length = 0.02;    // Low
    poor->weight_risk = 0.47;      // High pressure

    return poor;
}

// =============================================================================
// TEST A: A* BASELINE
// =============================================================================

void run_astar_test(Grid *grid, Config *config, PerformanceMetrics *metrics)
{
    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf(" TEST A: A* Baseline (Optimal Pathfinding)\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    Chromosome astar_baseline;
    memset(&astar_baseline, 0, sizeof(Chromosome));

    double start_time = get_time();

    // Build A* paths
    build_astar_baseline_paths(grid, config, &astar_baseline);

    // Evaluate
    int survivors = 0, coverage = 0;
    double path_length = 0.0, risk = 0.0;
    evaluate_astar_baseline(&astar_baseline, grid, &survivors, &coverage, &path_length, &risk);

    double end_time = get_time();

    // Store metrics
    metrics->astar_time = end_time - start_time;
    metrics->astar_survivors = survivors;
    metrics->astar_coverage = coverage;
    metrics->astar_path_length = path_length;
    metrics->astar_risk = risk;

    printf("[A*] Time: %.2f seconds\n", metrics->astar_time);
    printf("[A*] Survivors: %d/%d\n", survivors, grid->num_survivors);
    printf("[A*] Coverage: %d cells\n", coverage);
    printf("[A*] Path length: %.2f\n", path_length);
    printf("[A*] Risk: %.2f\n", risk);
    printf("\n═══════════════════════════════════════════════════════════\n");
}

// =============================================================================
// TEST B: SEQUENTIAL GA (Tuned Parameters)
// =============================================================================

void run_sequential_ga(Chromosome *population, int pop_size, Grid *grid, Config *config,
                       PerformanceMetrics *metrics)
{
    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf(" TEST B: Sequential GA (Tuned Parameters, NO Parallelization)\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    printf("[SEQ] Population: %d, Generations: %d\n", pop_size, config->num_generations);
    printf("[SEQ] Weights - Survivors: %.2f, Coverage: %.2f, Length: %.2f, Risk: %.2f\n\n",
           config->weight_survivors, config->weight_coverage,
           config->weight_length, config->weight_risk);

    // Initialize population
    for (int i = 0; i < pop_size; i++)
    {
        init_random_chromosome(&population[i], config->num_robots, grid->entry_point, MAX_MOVES);
    }

    double start_time = get_time();

    // Evaluate initial population sequentially
    evaluate_population_sequential(population, pop_size, grid, config);
    qsort(population, pop_size, sizeof(Chromosome), compare_fitness);

    Chromosome *new_pop = malloc(sizeof(Chromosome) * pop_size);
    if (!new_pop)
    {
        fprintf(stderr, "[SEQ] Memory allocation failed!\n");
        return;
    }

    // Run evolution with early stopping
    int max_gens = config->num_generations;
    int stagnation = 0;
    double prev_best = population[0].fitness;

    for (int gen = 1; gen <= max_gens && stagnation < config->max_stagnation; gen++)
    {
        if (gen % 10 == 0 || gen == 1)
        {
            printf("[SEQ] Generation %d/%d - Best: %.2f\n", gen, max_gens, population[0].fitness);
        }

        run_single_generation(population, new_pop, pop_size, config);
        evaluate_population_sequential(new_pop, pop_size, grid, config);
        qsort(new_pop, pop_size, sizeof(Chromosome), compare_fitness);

        // Check stagnation
        if (new_pop[0].fitness <= prev_best + 0.01)
        {
            stagnation++;
        }
        else
        {
            stagnation = 0;
            prev_best = new_pop[0].fitness;
        }

        // Swap
        Chromosome *tmp = population;
        population = new_pop;
        new_pop = tmp;
    }

    free(new_pop);

    double end_time = get_time();

    // Store metrics
    metrics->sequential_time = end_time - start_time;
    metrics->sequential_survivors = population[0].survivors_reached;
    metrics->sequential_fitness = population[0].fitness;
    metrics->sequential_coverage = population[0].cells_covered;
    metrics->sequential_path_length = population[0].total_path_length;
    metrics->sequential_risk = population[0].total_risk;

    printf("\n[SEQ] COMPLETE!\n");
    printf("[SEQ] Time: %.2f seconds\n", metrics->sequential_time);
    printf("[SEQ] Best fitness: %.2f\n", metrics->sequential_fitness);
    printf("[SEQ] Survivors: %d/%d\n", metrics->sequential_survivors, grid->num_survivors);
    printf("\n═══════════════════════════════════════════════════════════\n");
}

// =============================================================================
// TEST C: PARALLEL GA (Poor Parameters)
// =============================================================================

void run_parallel_ga_poor(Chromosome *population, int pop_size, Grid *grid, Config *poor_config,
                          PerformanceMetrics *metrics)
{
    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf(" TEST C: Parallel GA (Poor Parameters) ✗\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    printf("[POOR] Population: %d, Generations: %d\n", pop_size, poor_config->num_generations);
    printf("[POOR] Weights - Survivors: %.2f, Coverage: %.2f, Length: %.2f, Risk: %.2f\n\n",
           poor_config->weight_survivors, poor_config->weight_coverage,
           poor_config->weight_length, poor_config->weight_risk);

    // Initialize population
    for (int i = 0; i < pop_size; i++)
    {
        init_random_chromosome(&population[i], poor_config->num_robots, grid->entry_point, MAX_MOVES);
    }

    // Create worker pool for parallel tests
    printf("[POOR] Initializing worker pool (%d workers)...\n", poor_config->num_workers);
    if (init_worker_pool(poor_config->num_workers, poor_config, grid) == -1)
    {
        fprintf(stderr, "[POOR] Worker pool initialization failed!\n");
        return;
    }
    printf("[POOR] ✓ Worker pool ready\n\n");

    double start_time = get_time();

    // Run GA with poor config (uses parallel evaluation inside run_ga)
    run_ga(population, pop_size, grid, poor_config);

    double end_time = get_time();

    // Cleanup workers
    printf("\n[POOR] Cleaning up worker pool...\n");
    cleanup_worker_pool();
    sleep(1);

    // Store metrics
    metrics->parallel_poor_time = end_time - start_time;
    metrics->parallel_poor_survivors = population[0].survivors_reached;
    metrics->parallel_poor_fitness = population[0].fitness;
    metrics->parallel_poor_coverage = population[0].cells_covered;
    metrics->parallel_poor_path_length = population[0].total_path_length;
    metrics->parallel_poor_risk = population[0].total_risk;

    printf("\n[POOR] COMPLETE!\n");
    printf("[POOR] Time: %.2f seconds\n", metrics->parallel_poor_time);
    printf("[POOR] Best fitness: %.2f\n", metrics->parallel_poor_fitness);
    printf("[POOR] Survivors: %d/%d\n", metrics->parallel_poor_survivors, grid->num_survivors);
    printf("\n═══════════════════════════════════════════════════════════\n");
}

// =============================================================================
// TEST D: PARALLEL GA (Tuned Parameters)
// =============================================================================

void run_parallel_ga_tuned(Chromosome *population, int pop_size, Grid *grid, Config *config,
                           PerformanceMetrics *metrics)
{
    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf(" TEST D: Parallel GA (Tuned Parameters) ✓\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    printf("[TUNED] Population: %d, Generations: %d\n", pop_size, config->num_generations);
    printf("[TUNED] Weights - Survivors: %.2f, Coverage: %.2f, Length: %.2f, Risk: %.2f\n\n",
           config->weight_survivors, config->weight_coverage,
           config->weight_length, config->weight_risk);

    // Initialize population
    for (int i = 0; i < pop_size; i++)
    {
        init_random_chromosome(&population[i], config->num_robots, grid->entry_point, MAX_MOVES);
    }

    // Create worker pool
    printf("[TUNED] Initializing worker pool (%d workers)...\n", config->num_workers);
    if (init_worker_pool(config->num_workers, config, grid) == -1)
    {
        fprintf(stderr, "[TUNED] Worker pool initialization failed!\n");
        return;
    }
    printf("[TUNED] ✓ Worker pool ready\n\n");

    double start_time = get_time();

    // Run GA with tuned config
    run_ga(population, pop_size, grid, config);

    double end_time = get_time();

    // Cleanup workers
    printf("\n[TUNED] Cleaning up worker pool...\n");
    cleanup_worker_pool();
    sleep(1);

    // Store metrics
    metrics->parallel_tuned_time = end_time - start_time;
    metrics->parallel_tuned_survivors = population[0].survivors_reached;
    metrics->parallel_tuned_fitness = population[0].fitness;
    metrics->parallel_tuned_coverage = population[0].cells_covered;
    metrics->parallel_tuned_path_length = population[0].total_path_length;
    metrics->parallel_tuned_risk = population[0].total_risk;

    printf("\n[TUNED] COMPLETE!\n");
    printf("[TUNED] Time: %.2f seconds\n", metrics->parallel_tuned_time);
    printf("[TUNED] Best fitness: %.2f\n", metrics->parallel_tuned_fitness);
    printf("[TUNED] Survivors: %d/%d\n", metrics->parallel_tuned_survivors, grid->num_survivors);
    printf("\n═══════════════════════════════════════════════════════════\n");
}

// =============================================================================
// DISPLAY RESULTS TABLE
// =============================================================================

void display_performance_table(PerformanceMetrics *m)
{
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║              PERFORMANCE COMPARISON - map.txt (10×10×3)                       ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════════════════╝\n\n");

    printf("┌──────────────────────────────────┬──────────┬───────────┬──────────┬──────────┐\n");
    printf("│ Algorithm Variant                │   Time   │ Survivors │  Fitness │ Coverage │\n");
    printf("├──────────────────────────────────┼──────────┼───────────┼──────────┼──────────┤\n");
    printf("│ A. A* Baseline (Optimal)         │ %7.2fs │    %d/8    │    N/A   │   %4d   │\n",
           m->astar_time, m->astar_survivors, m->astar_coverage);
    printf("│ B. GA Sequential (Tuned Params)  │ %7.2fs │    %d/8    │  %6.2f  │   %4d   │\n",
           m->sequential_time, m->sequential_survivors, m->sequential_fitness, m->sequential_coverage);
    printf("│ C. GA Parallel (Poor Params) ✗   │ %7.2fs │    %d/8    │  %6.2f  │   %4d   │\n",
           m->parallel_poor_time, m->parallel_poor_survivors, m->parallel_poor_fitness, m->parallel_poor_coverage);
    printf("│ D. GA Parallel (Tuned Params) ✓  │ %7.2fs │    %d/8    │  %6.2f  │   %4d   │\n",
           m->parallel_tuned_time, m->parallel_tuned_survivors, m->parallel_tuned_fitness, m->parallel_tuned_coverage);
    printf("└──────────────────────────────────┴──────────┴───────────┴──────────┴──────────┘\n\n");

    printf("┌──────────────────────────────────┬─────────────┬──────────┬──────────────────┐\n");
    printf("│ Algorithm Variant                │ Path Length │   Risk   │  Speedup vs Seq  │\n");
    printf("├──────────────────────────────────┼─────────────┼──────────┼──────────────────┤\n");
    printf("│ A. A* Baseline (Optimal)         │   %8.2f  │  %6.2f  │   %8.2fx    │\n",
           m->astar_path_length, m->astar_risk, m->sequential_time / (m->astar_time + 0.01));
    printf("│ B. GA Sequential (Tuned Params)  │   %8.2f  │  %6.2f  │      1.00x       │\n",
           m->sequential_path_length, m->sequential_risk);
    printf("│ C. GA Parallel (Poor Params) ✗   │   %8.2f  │  %6.2f  │      %.2fx       │\n",
           m->parallel_poor_path_length, m->parallel_poor_risk, m->sequential_time / m->parallel_poor_time);
    printf("│ D. GA Parallel (Tuned Params) ✓  │   %8.2f  │  %6.2f  │      %.2fx       │\n",
           m->parallel_tuned_path_length, m->parallel_tuned_risk, m->sequential_time / m->parallel_tuned_time);
    printf("└──────────────────────────────────┴─────────────┴──────────┴──────────────────┘\n\n");
}

// =============================================================================
// MAIN COMPARISON FUNCTION
// =============================================================================

void run_performance_comparison(Grid *grid, Config *config)
{
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                      PERFORMANCE COMPARISON SUITE                             ║\n");
    printf("║                                                                               ║\n");
    printf("║  Testing 4 algorithm variants on map.txt (10×10×3, 8 survivors)              ║\n");
    printf("║                                                                               ║\n");
    printf("║  A. A* Baseline          - Optimal pathfinding                               ║\n");
    printf("║  B. GA Sequential        - No parallelization (tuned params)                 ║\n");
    printf("║  C. GA Parallel (Poor)   - WITH parallelization (poor params)                ║\n");
    printf("║  D. GA Parallel (Tuned)  - WITH parallelization (tuned params)               ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════════════════╝\n\n");

    PerformanceMetrics metrics = {0};
    int pop_size = config->population_size;

    // Allocate populations (reused across tests)
    Chromosome *population = malloc(sizeof(Chromosome) * pop_size);
    if (!population)
    {
        fprintf(stderr, "ERROR: Memory allocation failed!\n");
        return;
    }

    // TEST A: A* Baseline
    run_astar_test(grid, config, &metrics);

    printf("\n[AUTO] Proceeding to Test B...\n");
    sleep(2);

    // TEST B: Sequential GA (Tuned)
    run_sequential_ga(population, pop_size, grid, config, &metrics);

    printf("\n[AUTO] Proceeding to Test C...\n");
    sleep(2);

    // TEST C: Parallel GA (Poor Parameters)
    Config *poor_config = create_poor_config(config);
    if (!poor_config)
    {
        fprintf(stderr, "ERROR: Failed to create poor config!\n");
        free(population);
        return;
    }
    run_parallel_ga_poor(population, pop_size, grid, poor_config, &metrics);
    free(poor_config);

    printf("\n[AUTO] Waiting for worker cleanup...\n");
    sleep(3);

    printf("\n[AUTO] Proceeding to Test D (Final Test)...\n");

    // TEST D: Parallel GA (Tuned Parameters)
    run_parallel_ga_tuned(population, pop_size, grid, config, &metrics);

    // Display final comparison table
    display_performance_table(&metrics);

    // Cleanup
    free(population);

    printf("[COMPARISON] All tests complete! Press Enter to return to menu...");
    getchar();
}