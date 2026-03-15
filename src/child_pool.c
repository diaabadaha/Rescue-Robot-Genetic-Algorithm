#include "../include/child_pool.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/wait.h>

// GLOBAL STATE (Worker Pool Management)

// Worker process IDs
static pid_t worker_pids[MAX_WORKERS];
static int num_active_workers = 0;

// IPC resources
static SharedMemory *shm = NULL;
static int shm_fd = -1;
static mqd_t mq_jobs = (mqd_t)-1;
static mqd_t mq_results = (mqd_t)-1;

// Configuration
static Config *global_config = NULL;

// HELPER FUNCTIONS (Internal)

// Setup IPC resources (shared memory + message queues)
int setup_ipc_resources(Grid *grid)
{
    // Create shared memory (calculate size correctly with grid cells)
    size_t shm_size = sizeof(SharedMemory) + (grid->total_cells * sizeof(Cell));
    shm_fd = create_shared_memory(shm_size);
    if (shm_fd == -1)
    {
        fprintf(stderr, "[POOL] Failed to create shared memory\n");
        return -1;
    }

    // Map shared memory
    shm = map_shared_memory(shm_fd, shm_size);
    if (!shm)
    {
        fprintf(stderr, "[POOL] Failed to map shared memory\n");
        close(shm_fd);
        destroy_shared_memory();
        return -1;
    }

    // Initialize shared memory
    memset(shm, 0, sizeof(SharedMemory));
    shm->initialized = 0;
    shm->num_workers = 0;

    // Initialize semaphores
    init_shared_semaphores(shm);

    // Initialize job slots
    for (int i = 0; i < MAX_WORKERS; i++)
    {
        shm->jobs[i].status = JOB_EMPTY;
        shm->jobs[i].job_id = -1;
        shm->jobs[i].fitness = 0.0;
    }

    // Copy config to shared memory
    shm->config = *global_config;

    // Copy grid to shared memory
    init_shared_grid(shm, grid);

    // Create message queues
    destroy_message_queue(MQ_JOBS_NAME);
    destroy_message_queue(MQ_RESULTS_NAME);

    mq_jobs = create_message_queue(MQ_JOBS_NAME, 0);
    if (mq_jobs == (mqd_t)-1)
    {
        fprintf(stderr, "[POOL] Failed to create job queue\n");
        cleanup_ipc_resources();
        return -1;
    }

    mq_results = create_message_queue(MQ_RESULTS_NAME, 0);
    if (mq_results == (mqd_t)-1)
    {
        fprintf(stderr, "[POOL] Failed to create result queue\n");
        cleanup_ipc_resources();
        return -1;
    }

    return 0;
}

// Cleanup IPC resources
void cleanup_ipc_resources()
{
    // Close message queues
    if (mq_jobs != (mqd_t)-1)
    {
        close_message_queue(mq_jobs);
        mq_jobs = (mqd_t)-1;
    }

    if (mq_results != (mqd_t)-1)
    {
        close_message_queue(mq_results);
        mq_results = (mqd_t)-1;
    }

    // Destroy semaphores
    if (shm)
    {
        destroy_shared_semaphores(shm);
    }

    // Unmap and destroy shared memory
    if (shm)
    {
        size_t shm_size = sizeof(SharedMemory) + (shm->grid_total_cells * sizeof(Cell));
        unmap_shared_memory(shm, shm_size);
        shm = NULL;
    }

    if (shm_fd != -1)
    {
        close(shm_fd);
        shm_fd = -1;
    }

    destroy_shared_memory();

    // Destroy message queues
    destroy_message_queue(MQ_JOBS_NAME);
    destroy_message_queue(MQ_RESULTS_NAME);
}

// Getter functions
SharedMemory *get_shared_memory()
{
    return shm;
}

mqd_t get_job_queue()
{
    return mq_jobs;
}

mqd_t get_result_queue()
{
    return mq_results;
}

// WORKER PROCESS IMPLEMENTATION
void worker_process(int worker_id, mqd_t mq_jobs, mqd_t mq_results)
{

    // Open shared memory (workers don't create, they open existing)
    int worker_shm_fd = open_shared_memory();
    if (worker_shm_fd == -1)
    {
        fprintf(stderr, "[WORKER %d] Failed to open shared memory\n", worker_id);
        exit(1);
    }

    // Map shared memory (need to know size first - read from file descriptor)
    // First map just the header to get grid size
    SharedMemory *temp = map_shared_memory(worker_shm_fd, sizeof(SharedMemory));
    size_t shm_size = sizeof(SharedMemory) + (temp->grid_total_cells * sizeof(Cell));
    unmap_shared_memory(temp, sizeof(SharedMemory));

    // Now map the full size
    SharedMemory *worker_shm = map_shared_memory(worker_shm_fd, shm_size);
    if (!worker_shm)
    {
        fprintf(stderr, "[WORKER %d] Failed to map shared memory\n", worker_id);
        close(worker_shm_fd);
        exit(1);
    }

    // Get grid from shared memory
    Grid *grid = get_shared_grid(worker_shm);
    if (!grid)
    {
        fprintf(stderr, "[WORKER %d] Failed to get grid from shared memory\n", worker_id);
        unmap_shared_memory(worker_shm, sizeof(SharedMemory));
        close(worker_shm_fd);
        exit(1);
    }

    // Main worker loop - wait for jobs
    while (1)
    {
        // Wait for job message (BLOCKS until job available)
        JobMessage job_msg;
        if (receive_job_message(mq_jobs, &job_msg) == -1)
        {
            fprintf(stderr, "[WORKER %d] Failed to receive job message\n", worker_id);
            continue; // Try again
        }

        // Get chromosome from shared memory
        int job_id;
        Chromosome *chromosome = get_job(worker_shm, job_msg.slot_id, &job_id);
        if (!chromosome)
        {
            fprintf(stderr, "[WORKER %d] Failed to get job from slot %d\n",
                    worker_id, job_msg.slot_id);
            continue;
        }

        // Generate paths from move sequences
        generate_chromosome_paths(chromosome, grid);

        // Compute fitness
        double fitness = compute_fitness(chromosome, grid, &worker_shm->config);

        // Submit result to shared memory (pass full chromosome)
        submit_result(worker_shm, job_msg.slot_id, chromosome);

        // Send result message to parent
        ResultMessage result_msg;
        result_msg.msg_type = MSG_TYPE_RESULT;
        result_msg.job_id = job_id;
        result_msg.slot_id = job_msg.slot_id;
        result_msg.worker_id = worker_id;
        result_msg.fitness = fitness;

        if (send_result_message(mq_results, &result_msg) == -1)
        {
            fprintf(stderr, "[WORKER %d] Failed to send result message\n", worker_id);
            // Continue anyway - result is in shared memory
        }
    }

    // Cleanup (never reached under normal operation)
    free_shared_grid(grid);
    unmap_shared_memory(worker_shm, sizeof(SharedMemory));
    close(worker_shm_fd);
    exit(0);
}

// WORKER POOL MANAGEMENT
int init_worker_pool(int num_workers, Config *config, Grid *grid)
{
    printf("==== Initializing worker pool ====\n");

    if (num_workers <= 0 || num_workers > MAX_WORKERS)
    {
        fprintf(stderr, "[POOL] Invalid number of workers: %d (max: %d)\n",
                num_workers, MAX_WORKERS);
        return -1;
    }

    if (!config || !grid)
    {
        fprintf(stderr, "[POOL] Invalid config or grid\n");
        return -1;
    }

    // Save config globally for workers
    global_config = config;

    // Setup IPC resources
    if (setup_ipc_resources(grid) == -1)
    {
        fprintf(stderr, "[POOL] Failed to setup IPC resources\n");
        return -1;
    }

    // Fork worker processes
    for (int i = 0; i < num_workers; i++)
    {
        pid_t pid = fork();

        if (pid == -1)
        {
            // Fork failed
            fprintf(stderr, "[POOL] Failed to fork worker %d: %s\n",
                    i, strerror(errno));

            // Cleanup already created workers
            cleanup_worker_pool();
            return -1;
        }
        else if (pid == 0)
        {

            // Call worker_process() - this never returns
            worker_process(i, mq_jobs, mq_results);

            // Should never reach here
            exit(0);
        }
        else
        {
            // PARENT PROCESS
            // Store worker PID
            worker_pids[i] = pid;
            num_active_workers++;
        }
    }

    // Update shared memory with number of workers
    shm->num_workers = num_workers;

    // Give workers a moment to initialize
    sleep(1);

    return 0;
}

void cleanup_worker_pool()
{
    printf("\n[POOL] Shutting down worker pool...\n");

    // Terminate all worker processes
    for (int i = 0; i < num_active_workers; i++)
    {
        if (worker_pids[i] > 0)
        { // Send SIGTERM
            kill(worker_pids[i], SIGTERM);
        }
    }

    // Wait for workers to terminate
    for (int i = 0; i < num_active_workers; i++)
    {
        if (worker_pids[i] > 0)
        {
            int status;
            waitpid(worker_pids[i], &status, 0);
        }
    }

    // Cleanup IPC resources
    cleanup_ipc_resources();

    // Reset state
    num_active_workers = 0;
    for (int i = 0; i < MAX_WORKERS; i++)
    {
        worker_pids[i] = 0;
    }
}

// PARALLEL FITNESS EVALUATION
int evaluate_population_parallel(Chromosome *population, int pop_size,
                                 Grid *grid, Config *config)
{
    printf("\n======= Parallel Fitness Evaluation =======\n");

    if (!population || pop_size <= 0)
    {
        fprintf(stderr, "[EVAL] Invalid population\n");
        return -1;
    }

    if (!shm || !grid || !config)
    {
        fprintf(stderr, "[EVAL] Worker pool not initialized\n");
        return -1;
    }

    int jobs_sent = 0;
    int results_received = 0;

    for (int i = 0; i < pop_size; i++)
    {
        // Find empty slot
        int slot = find_empty_slot(shm);

        // If no slot available, collect a result first to free one
        while (slot == -1)
        {
            // Collect ONE result to free up a slot
            ResultMessage result_msg;
            if (receive_result_message(mq_results, &result_msg) != -1)
            {
                // Copy metrics from JobSlot to population
                sem_wait(&shm->slot_mutex[result_msg.slot_id]);
                population[result_msg.job_id] = shm->jobs[result_msg.slot_id].chromosome;  // <-- copy paths too
                population[result_msg.job_id].fitness = shm->jobs[result_msg.slot_id].fitness;
                population[result_msg.job_id].survivors_reached = shm->jobs[result_msg.slot_id].survivors_reached;
                population[result_msg.job_id].cells_covered = shm->jobs[result_msg.slot_id].cells_covered;
                population[result_msg.job_id].total_path_length = shm->jobs[result_msg.slot_id].total_path_length;
                population[result_msg.job_id].total_risk = shm->jobs[result_msg.slot_id].total_risk;
                population[result_msg.job_id].total_time = shm->jobs[result_msg.slot_id].total_time;

                sem_post(&shm->slot_mutex[result_msg.slot_id]);

                // Clear job slot (frees it!)
                clear_job_slot(shm, result_msg.slot_id);
                results_received++;
            }
            else
            {
                usleep(1000);
            }

            // Try to find slot again
            slot = find_empty_slot(shm);
        }

        // Submit job to shared memory
        submit_job(shm, slot, &population[i], i);

        // Send job message
        JobMessage job_msg;
        job_msg.msg_type = MSG_TYPE_JOB;
        job_msg.job_id = i;
        job_msg.slot_id = slot;
        job_msg.worker_id = -1; // Any worker can take it

        if (send_job_message(mq_jobs, &job_msg) == -1)
        {
            fprintf(stderr, "[EVAL] Failed to send job %d\n", i);
            continue;
        }

        jobs_sent++;
    }

    while (results_received < jobs_sent)
    {
        // Wait for result message (BLOCKS)
        ResultMessage result_msg;
        if (receive_result_message(mq_results, &result_msg) == -1)
        {
            fprintf(stderr, "[EVAL] Failed to receive result\n");
            continue;
        }

        // Copy ALL metrics from JobSlot to population
        sem_wait(&shm->slot_mutex[result_msg.slot_id]);
        population[result_msg.job_id] = shm->jobs[result_msg.slot_id].chromosome;  // <-- copy paths too


        population[result_msg.job_id].fitness = shm->jobs[result_msg.slot_id].fitness;
        population[result_msg.job_id].survivors_reached = shm->jobs[result_msg.slot_id].survivors_reached;
        population[result_msg.job_id].cells_covered = shm->jobs[result_msg.slot_id].cells_covered;
        population[result_msg.job_id].total_path_length = shm->jobs[result_msg.slot_id].total_path_length;
        population[result_msg.job_id].total_risk = shm->jobs[result_msg.slot_id].total_risk;
        population[result_msg.job_id].total_time = shm->jobs[result_msg.slot_id].total_time;

        double fitness = shm->jobs[result_msg.slot_id].fitness;
        int survivors = shm->jobs[result_msg.slot_id].survivors_reached;
        int coverage = shm->jobs[result_msg.slot_id].cells_covered;

        sem_post(&shm->slot_mutex[result_msg.slot_id]);

        // Clear job slot for reuse
        clear_job_slot(shm, result_msg.slot_id);

        results_received++;
    }
    // Publish BEST of this evaluation to GUI slot 0 (persistent)
int best_idx = 0;
for (int i = 1; i < pop_size; i++)
{
    if (population[i].fitness > population[best_idx].fitness)
        best_idx = i;
}

sem_wait(&shm->slot_mutex[0]);
shm->jobs[0].chromosome = population[best_idx];
shm->jobs[0].fitness = population[best_idx].fitness;
shm->jobs[0].survivors_reached = population[best_idx].survivors_reached;
shm->jobs[0].cells_covered = population[best_idx].cells_covered;
shm->jobs[0].total_path_length = population[best_idx].total_path_length;
shm->jobs[0].total_risk = population[best_idx].total_risk;
shm->jobs[0].total_time = population[best_idx].total_time;
shm->jobs[0].job_id = -999;      
shm->jobs[0].status = JOB_DONE;    
sem_post(&shm->slot_mutex[0]);


    return results_received;
}