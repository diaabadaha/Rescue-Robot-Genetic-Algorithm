#ifndef GA_H
#define GA_H

#include "common.h"
#include "grid.h"
#include "config.h"

// CHROMOSOME INITIALIZATION AND PATH GENERATION
void init_random_robot(Robot *robot, int robot_id, Vec3 start_pos, int num_moves);
int generate_path_from_moves(Robot *robot, Grid *grid);
void init_random_chromosome(Chromosome *chromosome, int num_robots,Vec3 start_pos, int num_moves);
void generate_chromosome_paths(Chromosome *chromosome, Grid *grid);

// FITNESS EVALUATION
double compute_fitness(Chromosome *chromosome, Grid *grid, Config *config);
void evaluate_robot_path(Robot *robot, Grid *grid, Chromosome *chromosome,int*cells_visited, int *survivor_rescued);

// GENETIC OPERATORS
int tournament_selection(Chromosome *population, int pop_size, int tournament_size);
void crossover_robot(Robot *parent1, Robot *parent2,Robot *child1, Robot *child2, int robot_id);
void mutate_robot(Robot *robot, double mutation_rate);
void apply_elitism(Chromosome *population, Chromosome *new_population,int pop_size, int elite_count);

// UTILITY FUNCTIONS
void copy_chromosome(Chromosome *dest, const Chromosome *src);
void copy_robot(Robot *dest, const Robot *src);
void print_chromosome(const Chromosome *chromosome, const char *label);
void print_robot(const Robot *robot);
int compare_fitness(const void *a, const void *b);

// MAIN GA FUNCTIONS
void run_single_generation(Chromosome *population,
                           Chromosome *new_population,
                           int pop_size,
                           Config *config);

/**
 * Run full genetic algorithm
 * @param population Initial population
 * @param pop_size Population size
 * @param grid Grid environment
 * @param config Configuration
 */
void run_ga(Chromosome *population, int pop_size, Grid *grid, Config *config);


int evaluate_population_sequential(Chromosome *population, int pop_size, Grid *grid, Config *config);

#endif // GA_H