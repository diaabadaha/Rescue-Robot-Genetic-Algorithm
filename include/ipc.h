#ifndef IPC_H
#define IPC_H

#include "common.h"
#include "config.h"
#include "grid.h"
#include "ga.h"
#include <mqueue.h>
#include <semaphore.h>
#define SHM_NAME "/shm_rescue_robots" 

// JOB STATUS ENUM 
typedef enum
{
    JOB_EMPTY = 0,
    JOB_PENDING = 1,
    JOB_COMPUTING = 2,
    JOB_DONE = 3
} JobStatus;

// JOB SLOT STRUCTURE 
typedef struct
{
    JobStatus status;
    int job_id;
    Chromosome chromosome;
    
    // Fitness and metrics (computed by worker)
    double fitness;
    int survivors_reached;
    int cells_covered;
    double total_path_length;
    double total_risk;
    double total_time;
} JobSlot;

// =============================================================================
// MESSAGE QUEUE CONFIGURATION
// =============================================================================

// Message queue names
#define MQ_JOBS_NAME "/mq_rescue_jobs"
#define MQ_RESULTS_NAME "/mq_rescue_results"

// Message types
#define MSG_TYPE_JOB 1
#define MSG_TYPE_RESULT 2

// Maximum workers
#define MAX_WORKERS 16

// =============================================================================
// MESSAGE STRUCTURES
// =============================================================================

// Job message (lightweight - sent via message queue)
typedef struct
{
    long msg_type;
    int job_id;
    int slot_id;
    int worker_id;
} JobMessage;

// Result message (lightweight - sent via message queue)
typedef struct
{
    long msg_type;
    int job_id;
    int slot_id;
    int worker_id;
    double fitness;
} ResultMessage;

// =============================================================================
// SHARED MEMORY STRUCTURE
// =============================================================================
typedef struct
{
    int initialized;
    int num_workers;
    
    // Job slots (protected by semaphores)
    JobSlot jobs[MAX_WORKERS];
    sem_t slot_mutex[MAX_WORKERS];

    Config config; // Add this field

    // Grid data (stored in shared memory)
    int grid_size_x;
    int grid_size_y;
    int grid_size_z;
    int grid_total_cells;
    int grid_num_survivors;
    Vec3 grid_entry_point;                   
    Vec3 grid_survivor_positions[1000];      
    Cell grid_cells[]; 
} SharedMemory;

// =============================================================================
// SHARED MEMORY FUNCTIONS
// =============================================================================

int create_shared_memory(size_t size);
int open_shared_memory();
SharedMemory *map_shared_memory(int shm_fd, size_t size);
void unmap_shared_memory(SharedMemory *shm, size_t size);
void destroy_shared_memory();

// =============================================================================
// SEMAPHORE FUNCTIONS
// =============================================================================

void init_shared_semaphores(SharedMemory *shm);
void destroy_shared_semaphores(SharedMemory *shm);

// =============================================================================
// GRID FUNCTIONS (Shared Memory)
// =============================================================================
void init_shared_grid(SharedMemory *shm, Grid *grid);
Grid *get_shared_grid(SharedMemory *shm);
void free_shared_grid(Grid *grid);

// =============================================================================
// JOB SLOT FUNCTIONS (Thread-safe with semaphores)
// =============================================================================
int find_empty_slot(SharedMemory *shm);
void submit_job(SharedMemory *shm, int slot_id, Chromosome *chromosome, int job_id);
Chromosome *get_job(SharedMemory *shm, int slot_id, int *job_id);
void mark_job_computing(SharedMemory *shm, int slot_id);
void submit_result(SharedMemory *shm, int slot_id, Chromosome *chromosome);
double get_result(SharedMemory *shm, int slot_id);
void clear_job_slot(SharedMemory *shm, int slot_id);

// =============================================================================
// MESSAGE QUEUE FUNCTIONS
// =============================================================================
mqd_t create_message_queue(const char *name, int flags);
mqd_t open_message_queue(const char *name, int flags);
void close_message_queue(mqd_t mq);
void destroy_message_queue(const char *name);
int send_job_message(mqd_t mq, JobMessage *msg);
int receive_job_message(mqd_t mq, JobMessage *msg);
int send_result_message(mqd_t mq, ResultMessage *msg);
int receive_result_message(mqd_t mq, ResultMessage *msg);

#endif // IPC_H