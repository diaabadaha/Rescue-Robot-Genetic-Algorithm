#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

// Configuration structure - holds all runtime parameters
typedef struct {
    // Grid dimensions
    int grid_x;
    int grid_y;
    int grid_z;
    
    // GA parameters
    int population_size;
    int num_generations;
    int num_robots;              // Number of robots in the simulation
    
    // GA rates (0.0 - 1.0)
    double mutation_rate;
    double crossover_rate;
    double elitism_rate;        // Percentage of top solutions to preserve
    
    // Fitness function weights
    double weight_survivors;
    double weight_coverage;
    double weight_length;
    double weight_risk;
    double weight_time;
    
    // Worker processes (for parallel fitness computation)
    int num_workers;            // Number of child processes for IPC
    
    // File paths
    char map_file[256];
    char menu_file[256];
    
    // Termination criteria
    int max_stagnation;         // Stop if no improvement for N generations
    int time_limit;             // Time limit in seconds (0 = no limit)
    
    // Random seed for reproducibility
    int random_seed;            // Random seed (0 = use time-based seed)
    
} Config;

// Function prototypes
Config* load_config(const char* filename);
Config* create_default_config();
void print_config(const Config* config);
void free_config(Config* config);

#endif // CONFIG_H