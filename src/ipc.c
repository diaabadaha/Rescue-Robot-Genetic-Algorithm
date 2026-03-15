#include "../include/ipc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>    
#include <unistd.h>   
#include <errno.h>

// SHARED MEMORY MANAGEMENT

// Create shared memory segment
int create_shared_memory(size_t size)
{
    // Create shared memory object
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        fprintf(stderr, "Error: shm_open failed: %s\n", strerror(errno));
        return -1;
    }

    // Configure the size of the shared memory segment
    if (ftruncate(shm_fd, size) == -1)
    {
        fprintf(stderr, "Error: ftruncate failed: %s\n", strerror(errno));
        close(shm_fd);
        shm_unlink(SHM_NAME);
        return -1;
    }

    return shm_fd;
}

// Open existing shared memory
int open_shared_memory()
{
    int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (shm_fd == -1)
    {
        fprintf(stderr, "Error: shm_open (open) failed: %s\n", strerror(errno));
        return -1;
    }
    return shm_fd;
}

// Map shared memory to process address space
SharedMemory *map_shared_memory(int shm_fd, size_t size)
{
    void *ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        fprintf(stderr, "Error: mmap failed: %s\n", strerror(errno));
        return NULL;
    }
    return (SharedMemory *)ptr;
}

// Unmap shared memory
void unmap_shared_memory(SharedMemory *shm, size_t size)
{
    if (shm)
    {
        munmap(shm, size);
    }
}

// Destroy shared memory segment
void destroy_shared_memory()
{
    shm_unlink(SHM_NAME);
}

// SEMAPHORE MANAGEMENT

// Initialize semaphores in shared memory
void init_shared_semaphores(SharedMemory *shm)
{
    if (!shm)
    {
        fprintf(stderr, "Error: NULL shared memory in init_shared_semaphores\n");
        return;
    }

    for (int i = 0; i < MAX_WORKERS; i++)
    {
        // Initialize semaphore:
        // - Second parameter (1) = shared between processes
        // - Third parameter (1) = initial value (unlocked)
        if (sem_init(&shm->slot_mutex[i], 1, 1) == -1)
        {
            fprintf(stderr, "Error: sem_init failed for slot %d: %s\n",
                    i, strerror(errno));
            // Try to continue with other semaphores
        }
    }
}

// Destroy semaphores in shared memory
void destroy_shared_semaphores(SharedMemory *shm)
{
    if (!shm)
    {
        fprintf(stderr, "Error: NULL shared memory in destroy_shared_semaphores\n");
        return;
    }

    for (int i = 0; i < MAX_WORKERS; i++)
    {
        if (sem_destroy(&shm->slot_mutex[i]) == -1)
        {
            fprintf(stderr, "Warning: sem_destroy failed for slot %d: %s\n",
                    i, strerror(errno));
            // Continue destroying other semaphores
        }
    }
}

// MESSAGE QUEUE MANAGEMENT

// Create message queue
mqd_t create_message_queue(const char *name, int flags)
{
    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;

    // Explicitly set to larger message size
    size_t job_size = sizeof(JobMessage);
    size_t result_size = sizeof(ResultMessage);
    attr.mq_msgsize = (job_size > result_size) ? job_size : result_size;
    attr.mq_curmsgs = 0;

    mqd_t mq = mq_open(name, O_CREAT | O_RDWR | flags, 0666, &attr);
    if (mq == (mqd_t)-1)
    {
        fprintf(stderr, "Error: mq_open (create) failed for %s: %s\n",
                name, strerror(errno));
        return (mqd_t)-1;
    }
    return mq;
}

// Open existing message queue
mqd_t open_message_queue(const char *name, int flags)
{
    mqd_t mq = mq_open(name, O_RDWR | flags);
    if (mq == (mqd_t)-1)
    {
        fprintf(stderr, "Error: mq_open (open) failed for %s: %s\n",
                name, strerror(errno));
        return (mqd_t)-1;
    }
    return mq;
}

// Close message queue
void close_message_queue(mqd_t mq)
{
    if (mq != (mqd_t)-1)
    {
        mq_close(mq);
    }
}

// Destroy message queue
void destroy_message_queue(const char *name)
{
    if (mq_unlink(name) == -1)
    {
        if (errno != ENOENT)
        { // Ignore "doesn't exist" errors
            fprintf(stderr, "Warning: mq_unlink failed for %s: %s\n",
                    name, strerror(errno));
        }
    }
}

// Send job message
int send_job_message(mqd_t mq, JobMessage *msg)
{

    if (mq_send(mq, (const char *)msg, sizeof(JobMessage), 0) == -1)
    {
        fprintf(stderr, "Error: mq_send (job) failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// Receive job message
int receive_job_message(mqd_t mq, JobMessage *msg)
{
    struct mq_attr attr;
    mq_getattr(mq, &attr);

    ssize_t bytes = mq_receive(mq, (char *)msg, attr.mq_msgsize, NULL);
    if (bytes == -1)
    {
        fprintf(stderr, "Error: mq_receive (job) failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// Send result message
int send_result_message(mqd_t mq, ResultMessage *msg)
{
    if (mq_send(mq, (const char *)msg, sizeof(ResultMessage), 0) == -1)
    {
        fprintf(stderr, "Error: mq_send (result) failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// Receive result message
int receive_result_message(mqd_t mq, ResultMessage *msg)
{
    if (mq_receive(mq, (char *)msg, sizeof(ResultMessage), NULL) == -1)
    {
        fprintf(stderr, "Error: mq_receive (result) failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// GRID MANAGEMENT

// Initialize grid in shared memory
void init_shared_grid(SharedMemory *shm, Grid *grid)
{
    if (!shm || !grid)
        return;

    // Copy grid metadata
    shm->grid_size_x = grid->size_x;
    shm->grid_size_y = grid->size_y;
    shm->grid_size_z = grid->size_z;
    shm->grid_total_cells = grid->total_cells;
    shm->grid_num_survivors = grid->num_survivors;
    shm->grid_entry_point = grid->entry_point;

    // Copy cells array
    memcpy(shm->grid_cells, grid->cells, grid->total_cells * sizeof(Cell));

    // Copy survivor positions
    memcpy(shm->grid_survivor_positions, grid->survivor_positions,
           grid->num_survivors * sizeof(Vec3));

    shm->initialized = 1;
}

// Get grid from shared memory (creates a Grid wrapper pointing to shared data)
Grid *get_shared_grid(SharedMemory *shm)
{
    if (!shm || !shm->initialized)
    {
        fprintf(stderr, "Error: Shared grid not initialized\n");
        return NULL;
    }

    // Allocate Grid structure (just the wrapper, not the data)
    Grid *grid = (Grid *)malloc(sizeof(Grid));
    if (!grid)
    {
        fprintf(stderr, "Error: Memory allocation failed for grid wrapper\n");
        return NULL;
    }

    // Point to shared memory data
    grid->size_x = shm->grid_size_x;
    grid->size_y = shm->grid_size_y;
    grid->size_z = shm->grid_size_z;
    grid->total_cells = shm->grid_total_cells;
    grid->num_survivors = shm->grid_num_survivors;
    grid->entry_point = shm->grid_entry_point;
    grid->cells = shm->grid_cells;                        
    grid->survivor_positions = shm->grid_survivor_positions;

    return grid;
}

// Free grid wrapper (not the shared data)
void free_shared_grid(Grid *grid)
{
    if (grid)
    {
        // Don't free cells or survivor_positions - they're in shared memory
        free(grid);
    }
}

// Find an empty job slot (NOW THREAD-SAFE)
int find_empty_slot(SharedMemory *shm)
{
    if (!shm)
        return -1;

    for (int i = 1; i < MAX_WORKERS; i++)
    {
        // Lock this slot's semaphore
        sem_wait(&shm->slot_mutex[i]);

        // Check if empty
        if (shm->jobs[i].status == JOB_EMPTY)
        {
            // Found empty slot! Mark as ready to reserve it
            shm->jobs[i].status = JOB_PENDING;

            // Unlock and return
            sem_post(&shm->slot_mutex[i]);
            return i;
        }

        // Not empty, unlock and continue searching
        sem_post(&shm->slot_mutex[i]);
    }

    return -1; // No empty slot found
}

// Submit job to slot (NOW THREAD-SAFE)
void submit_job(SharedMemory *shm, int slot, Chromosome *chromosome, int job_id)
{
    if (!shm || slot < 0 || slot >= MAX_WORKERS || !chromosome)
    {
        fprintf(stderr, "Error: Invalid job submission\n");
        return;
    }

    // Lock this slot
    sem_wait(&shm->slot_mutex[slot]);

    // Copy chromosome data
    shm->jobs[slot].chromosome = *chromosome;
    shm->jobs[slot].job_id = job_id;
    shm->jobs[slot].status = JOB_PENDING;
    shm->jobs[slot].fitness = 0.0;

    // Unlock
    sem_post(&shm->slot_mutex[slot]);
}

// Get job from slot (NOW THREAD-SAFE)
Chromosome *get_job(SharedMemory *shm, int slot, int *job_id)
{
    if (!shm || slot < 0 || slot >= MAX_WORKERS)
    {
        fprintf(stderr, "Error: Invalid slot\n");
        return NULL;
    }

    // Lock this slot
    sem_wait(&shm->slot_mutex[slot]);

    // Check if job is ready
    if (shm->jobs[slot].status != JOB_PENDING)
    {
        // Not ready, unlock and return NULL
        sem_post(&shm->slot_mutex[slot]);
        fprintf(stderr, "Error: Job slot %d not ready (status=%d)\n",
                slot, shm->jobs[slot].status);
        return NULL;
    }

    // Mark as computing
    shm->jobs[slot].status = JOB_COMPUTING;

    // Get job ID if requested
    if (job_id)
    {
        *job_id = shm->jobs[slot].job_id;
    }

    // Unlock
    sem_post(&shm->slot_mutex[slot]);

    // Return pointer to chromosome (it's safe to use after unlocking
    // because we marked it as COMPUTING)
    return &shm->jobs[slot].chromosome;
}

// Mark job as computing (NOW THREAD-SAFE)
void mark_job_computing(SharedMemory *shm, int slot)
{
    if (!shm || slot < 0 || slot >= MAX_WORKERS)
        return;

    sem_wait(&shm->slot_mutex[slot]);
    shm->jobs[slot].status = JOB_COMPUTING;
    sem_post(&shm->slot_mutex[slot]);
}

// Submit result (NOW THREAD-SAFE)
void submit_result(SharedMemory *shm, int slot_id, Chromosome *chromosome)
{
    if (!shm || !chromosome || slot_id < 0 || slot_id >= MAX_WORKERS)
    {
        fprintf(stderr, "Error: Invalid parameters in submit_result\n");
        return;
    }

    sem_wait(&shm->slot_mutex[slot_id]);

    // Store all computed metrics
    shm->jobs[slot_id].fitness = chromosome->fitness;
    shm->jobs[slot_id].survivors_reached = chromosome->survivors_reached;
    shm->jobs[slot_id].cells_covered = chromosome->cells_covered;
    shm->jobs[slot_id].total_path_length = chromosome->total_path_length;
    shm->jobs[slot_id].total_risk = chromosome->total_risk;
    shm->jobs[slot_id].total_time = chromosome->total_time;
    shm->jobs[slot_id].status = JOB_DONE;

    sem_post(&shm->slot_mutex[slot_id]);
}

// Get result from slot (NOW THREAD-SAFE)
double get_result(SharedMemory *shm, int slot)
{
    if (!shm || slot < 0 || slot >= MAX_WORKERS)
    {
        fprintf(stderr, "Error: Invalid slot\n");
        return -1.0;
    }

    // Lock
    sem_wait(&shm->slot_mutex[slot]);

    // Check status
    if (shm->jobs[slot].status != JOB_DONE)
    {
        sem_post(&shm->slot_mutex[slot]);
        fprintf(stderr, "Warning: Job slot %d not done (status=%d)\n",
                slot, shm->jobs[slot].status);
        return -1.0;
    }

    // Get fitness
    double fitness = shm->jobs[slot].fitness;

    // Unlock
    sem_post(&shm->slot_mutex[slot]);

    return fitness;
}

// Clear job slot (NOW THREAD-SAFE)
void clear_job_slot(SharedMemory *shm, int slot)
{
    if (!shm || slot < 0 || slot >= MAX_WORKERS)
        return;

    if (slot == 0)
        return;

    // Lock
    sem_wait(&shm->slot_mutex[slot]);

    // Clear slot
    shm->jobs[slot].status = JOB_EMPTY;
    shm->jobs[slot].job_id = -1;
    shm->jobs[slot].fitness = 0.0;

    // Unlock
    sem_post(&shm->slot_mutex[slot]);
}