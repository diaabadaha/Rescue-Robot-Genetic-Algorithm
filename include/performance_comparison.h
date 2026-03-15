#ifndef PERFORMANCE_COMPARISON_H
#define PERFORMANCE_COMPARISON_H

#include "common.h"
#include "grid.h"
#include "config.h"
#include "ga.h"

// =============================================================================
// PERFORMANCE METRICS STRUCTURE
// =============================================================================

typedef struct {
    // Test A: A* Baseline
    double astar_time;
    int astar_survivors;
    int astar_coverage;
    double astar_path_length;
    double astar_risk;
    
    // Test B: GA Sequential (Tuned Parameters)
    double sequential_time;
    int sequential_survivors;
    double sequential_fitness;
    int sequential_coverage;
    double sequential_path_length;
    double sequential_risk;
    
    // Test C: GA Parallel (Poor Parameters)
    double parallel_poor_time;
    int parallel_poor_survivors;
    double parallel_poor_fitness;
    int parallel_poor_coverage;
    double parallel_poor_path_length;
    double parallel_poor_risk;
    
    // Test D: GA Parallel (Tuned Parameters)
    double parallel_tuned_time;
    int parallel_tuned_survivors;
    double parallel_tuned_fitness;
    int parallel_tuned_coverage;
    double parallel_tuned_path_length;
    double parallel_tuned_risk;
} PerformanceMetrics;

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================

/**
 * Run complete performance comparison with 4 algorithm variants
 * Tests: A* baseline, GA sequential, GA parallel (poor), GA parallel (tuned)
 * 
 * @param grid Grid loaded from map.txt
 * @param config Base configuration (tuned parameters)
 */
void run_performance_comparison(Grid *grid, Config *config);

/**
 * Evaluate population sequentially (no parallelization)
 * 
 * @param population Array of chromosomes to evaluate
 * @param pop_size Population size
 * @param grid Grid environment
 * @param config Configuration
 * @return Number of chromosomes evaluated
 */
int evaluate_population_sequential(Chromosome *population, int pop_size,
                                   Grid *grid, Config *config);

/**
 * Create configuration with poor/untuned parameters
 * Used for Test C to demonstrate importance of parameter tuning
 * 
 * @param base_config Base configuration to copy from
 * @return New config with poor parameters (caller must free)
 */
Config* create_poor_config(Config *base_config);

/**
 * Run full GA evolution with specific configuration
 * 
 * @param population Population array
 * @param pop_size Population size
 * @param grid Grid environment
 * @param config Configuration to use
 * @param use_parallel 1 = parallel evaluation, 0 = sequential
 * @param variant_name Name for logging
 * @param metrics Metrics structure to populate
 * @param metric_index Which test (1=seq, 2=poor, 3=tuned)
 */
void run_ga_with_config(Chromosome *population, int pop_size,
                       Grid *grid, Config *config,
                       int use_parallel, const char *variant_name,
                       PerformanceMetrics *metrics, int metric_index);

/**
 * Run A* baseline test
 * 
 * @param grid Grid environment
 * @param config Configuration
 * @param metrics Metrics structure to populate
 */
void run_astar_test(Grid *grid, Config *config, PerformanceMetrics *metrics);

/**
 * Display comprehensive comparison table
 * 
 * @param m Performance metrics from all 4 tests
 */
void display_performance_table(PerformanceMetrics *m);

/**
 * Get current time in seconds (high precision)
 * 
 * @return Time in seconds since arbitrary start point
 */
double get_time(void);

#endif // PERFORMANCE_COMPARISON_H