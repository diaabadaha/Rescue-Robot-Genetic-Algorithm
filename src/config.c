#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../include/config.h"

// Create default configuration
Config *create_default_config() {
    Config *config = (Config *)malloc(sizeof(Config));
    if (!config) {
        fprintf(stderr, "Error: Memory allocation failed for config\n");
        return NULL;
    }

    // Grid dimensions
    config->grid_x = 20;
    config->grid_y = 20;
    config->grid_z = 5;

    // GA parameters
    config->population_size = 50;
    config->num_generations = 100;
    config->num_robots = 3; // Robots in the simulation

    // GA rates
    config->mutation_rate = 0.1;
    config->crossover_rate = 0.8;
    config->elitism_rate = 0.1;

    // Fitness weights
    config->weight_survivors = 10.0; 
    config->weight_coverage = 5.0;
    config->weight_length = 1.0;
    config->weight_risk = 2.0;
    config->weight_time = 1.5;

    // Worker processes (for parallel computation via IPC)
    config->num_workers = 4;

    // File paths
    strcpy(config->map_file, "config/map.txt");
    strcpy(config->menu_file, "config/menu.txt");

    // Termination
    config->max_stagnation = 20;
    config->time_limit = 300; // 5 minutes

    // Random seed (0 = use time-based)
    config->random_seed = 0;

    return config;
}

// Helper function to trim whitespace
static void trim_whitespace(char *str) {
    char *end;

    // Trim leading space
    while (*str == ' ' || *str == '\t')
        str++;

    // All spaces?
    if (*str == 0)
        return;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while (end > str && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) {
        end--;
    }
    end[1] = '\0';
}

// Parse a single line from config file
static void parse_config_line(Config *config, const char *line) {
    char key[256], value[256];

    // Skip empty lines and comments
    if (line[0] == '\0' || line[0] == '#' || line[0] == '\n') {
        return;
    }

    // Try to parse as key = value
    if (sscanf(line, "%255[^=] = %255[^\n]", key, value) == 2) {
        trim_whitespace(key);
        trim_whitespace(value);

        if (strcmp(key, "GRID_X") == 0) {
            config->grid_x = atoi(value);
        } else if (strcmp(key, "GRID_Y") == 0) {
            config->grid_y = atoi(value);
        } else if (strcmp(key, "GRID_Z") == 0) {
            config->grid_z = atoi(value);
        } else if (strcmp(key, "POPULATION_SIZE") == 0) {
            config->population_size = atoi(value);
        } else if (strcmp(key, "NUM_GENERATIONS") == 0) {
            config->num_generations = atoi(value);
        } else if (strcmp(key, "NUM_ROBOTS") == 0) {
            config->num_robots = atoi(value);
        } else if (strcmp(key, "MUTATION_RATE") == 0) {
            config->mutation_rate = atof(value);
        } else if (strcmp(key, "CROSSOVER_RATE") == 0) {
            config->crossover_rate = atof(value);
        } else if (strcmp(key, "ELITISM_RATE") == 0) {
            config->elitism_rate = atof(value);
        } else if (strcmp(key, "WEIGHT_SURVIVORS") == 0) {
            config->weight_survivors = atof(value);
        } else if (strcmp(key, "WEIGHT_COVERAGE") == 0) {
            config->weight_coverage = atof(value);
        } else if (strcmp(key, "WEIGHT_LENGTH") == 0) {
            config->weight_length = atof(value);
        } else if (strcmp(key, "WEIGHT_RISK") == 0) {
            config->weight_risk = atof(value);
        } else if (strcmp(key, "WEIGHT_TIME") == 0) {
            config->weight_time = atof(value);
        } else if (strcmp(key, "NUM_WORKERS") == 0) {
            config->num_workers = atoi(value);
        } else if (strcmp(key, "MAP_FILE") == 0) {
            strncpy(config->map_file, value, 255);
            config->map_file[255] = '\0';
        } else if (strcmp(key, "MENU_FILE") == 0) {
            strncpy(config->menu_file, value, 255);
            config->menu_file[255] = '\0';
        } else if (strcmp(key, "MAX_STAGNATION") == 0) {
            config->max_stagnation = atoi(value);
        } else if (strcmp(key, "TIME_LIMIT") == 0) {
            config->time_limit = atoi(value);
        } else if (strcmp(key, "RANDOM_SEED") == 0) {
            config->random_seed = atoi(value);
        }
    }
}

// Load configuration from file
Config *load_config(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Warning: Could not open config file '%s', using defaults\n", filename);
        return NULL;
    }

    // Start with default config
    Config *config = create_default_config();
    if (!config) {
        fclose(file);
        return NULL;
    }

    // Read and parse each line
    char line[512];
    while (fgets(line, sizeof(line), file)) {
        parse_config_line(config, line);
    }

    fclose(file);

    // Validate config
    if (config->population_size > MAX_POPULATION) {
        fprintf(stderr, "Warning: POPULATION_SIZE %d exceeds MAX_POPULATION %d, capping\n",
                config->population_size, MAX_POPULATION);
        config->population_size = MAX_POPULATION;
    }

    if (config->num_robots > MAX_ROBOTS) {
        fprintf(stderr, "Warning: NUM_ROBOTS %d exceeds MAX_ROBOTS %d, capping\n",
                config->num_robots, MAX_ROBOTS);
        config->num_robots = MAX_ROBOTS;
    }

    if (config->num_workers > MAX_WORKERS) {
        fprintf(stderr, "Warning: NUM_WORKERS %d exceeds MAX_WORKERS %d, capping\n",
                config->num_workers, MAX_WORKERS);
        config->num_workers = MAX_WORKERS;
    }

    return config;
}

// Print configuration (for debugging/display)
void print_config(const Config *config) {
    if (!config) {
        printf("Config is NULL\n");
        return;
    }
    
    printf("Grid Dimensions:\n");
    printf("  X: %d, Y: %d, Z: %d\n", config->grid_x, config->grid_y, config->grid_z);
    printf("  Total cells: %d\n\n", config->grid_x * config->grid_y * config->grid_z);

    printf("Genetic Algorithm:\n");
    printf("  Population size: %d chromosomes\n", config->population_size);
    printf("  Generations: %d\n", config->num_generations);
    printf("  Number of robots per solution: %d\n\n", config->num_robots);

    printf("GA Rates:\n");
    printf("  Mutation rate: %.2f\n", config->mutation_rate);
    printf("  Crossover rate: %.2f\n", config->crossover_rate);
    printf("  Elitism rate: %.2f (top %.0f%%)\n\n",
           config->elitism_rate, config->elitism_rate * 100);

    printf("Fitness Weights:\n");
    printf("  Survivors: %.2f\n", config->weight_survivors);
    printf("  Coverage: %.2f\n", config->weight_coverage);
    printf("  Path length: %.2f\n", config->weight_length);
    printf("  Risk: %.2f\n", config->weight_risk);
    printf("  Time: %.2f\n\n", config->weight_time);

    printf("Parallel Processing (IPC):\n");
    printf("  Worker processes: %d (child processes for fitness computation)\n\n", config->num_workers);

    printf("Files:\n");
    printf("  Map file: %s\n", config->map_file);
    printf("  Menu file: %s\n\n", config->menu_file);

    printf("Termination Criteria:\n");
    printf("  Max stagnation: %d generations\n", config->max_stagnation);
    printf("  Time limit: %d seconds\n\n", config->time_limit);

    printf("Random Seed:\n");
    if (config->random_seed == 0) {
        printf("  Time-based (non-deterministic)\n\n");
    } else {
        printf("  Fixed seed: %d (reproducible)\n\n", config->random_seed);
    }
}

// Free configuration
void free_config(Config *config) {
    if (config) {
        free(config);
    }
}