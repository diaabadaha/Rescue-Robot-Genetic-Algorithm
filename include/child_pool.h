#ifndef CHILD_POOL_H
#define CHILD_POOL_H

#include "common.h"
#include "config.h"
#include "grid.h"
#include "ipc.h"
#include "ga.h"

#include <sys/types.h>
#include <signal.h>

// WORKER POOL MANAGEMENT

int init_worker_pool(int num_workers, Config *config, Grid *grid);
void cleanup_worker_pool();
void worker_process(int worker_id, mqd_t mq_jobs, mqd_t mq_results);
int evaluate_population_parallel(Chromosome *population, int pop_size,Grid *grid, Config *config);

// HELPER FUNCTIONS (Internal use)
int setup_ipc_resources(Grid *grid);
void cleanup_ipc_resources();
SharedMemory *get_shared_memory();
mqd_t get_job_queue();
mqd_t get_result_queue();

#endif // CHILD_POOL_H