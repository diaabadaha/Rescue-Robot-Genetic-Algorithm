// rescue_robot_gui/src/gui_ipc_reader.c
#include "../include/gui_ipc_reader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#define SHM_NAME "/shm_rescue_robots"

// Initialize IPC reader - connect to existing shared memory (READ-ONLY)
int gui_ipc_init(GuiIpcReader *reader)
{
    if (!reader)
        return -1;

    memset(reader, 0, sizeof(GuiIpcReader));

    printf("[GUI-IPC] Attempting to connect to shared memory...\n");

    // Try to open existing shared memory (READ-ONLY)
    reader->shm_fd = shm_open(SHM_NAME, O_RDONLY, 0);
    if (reader->shm_fd < 0)
    {
        fprintf(stderr, "[GUI-IPC] Failed to open shared memory '%s': %s\n",
                SHM_NAME, strerror(errno));
        fprintf(stderr, "[GUI-IPC] Make sure the main GA program is running!\n");
        fprintf(stderr, "[GUI-IPC] Falling back to demo mode (random grid).\n");
        return -1;
    }

    // Get shared memory size
    struct stat sb;
    if (fstat(reader->shm_fd, &sb) < 0)
    {
        fprintf(stderr, "[GUI-IPC] fstat failed: %s\n", strerror(errno));
        close(reader->shm_fd);
        reader->shm_fd = -1;
        return -1;
    }

    reader->shm_size = sb.st_size;
    printf("[GUI-IPC] Shared memory size: %zu bytes\n", reader->shm_size);

    // Map it (READ-ONLY)
    reader->shm = mmap(NULL, reader->shm_size, PROT_READ, MAP_SHARED,
                       reader->shm_fd, 0);
    if (reader->shm == MAP_FAILED)
    {
        fprintf(stderr, "[GUI-IPC] mmap failed: %s\n", strerror(errno));
        close(reader->shm_fd);
        reader->shm_fd = -1;
        return -1;
    }

    printf("[GUI-IPC] Successfully mapped shared memory (read-only)\n");

    // Check if initialized
    if (reader->shm->initialized != 1)
    {
        fprintf(stderr, "[GUI-IPC] Warning: Shared memory not fully initialized yet\n");
        fprintf(stderr, "[GUI-IPC] Grid data may not be ready. Waiting...\n");
    }
    else
    {
        printf("[GUI-IPC] Grid: %dx%dx%d (%d cells)\n",
               reader->shm->grid_size_x,
               reader->shm->grid_size_y,
               reader->shm->grid_size_z,
               reader->shm->grid_total_cells);
        printf("[GUI-IPC] Survivors: %d\n", reader->shm->grid_num_survivors);
        printf("[GUI-IPC] Entry point: (%d, %d, %d)\n",
               reader->shm->grid_entry_point.x,
               reader->shm->grid_entry_point.y,
               reader->shm->grid_entry_point.z);
    }

    reader->connected = 1;
    return 0;
}

// Update GUI grid from shared memory
void gui_ipc_update_grid(GuiIpcReader *reader, Grid *gui_grid)
{
    if (!reader || !reader->connected || !reader->shm || !gui_grid)
        return;

    // Update grid metadata
    gui_grid->size_x = reader->shm->grid_size_x;
    gui_grid->size_y = reader->shm->grid_size_y;
    gui_grid->size_z = reader->shm->grid_size_z;
    gui_grid->total_cells = reader->shm->grid_total_cells;
    gui_grid->num_survivors = reader->shm->grid_num_survivors;
    gui_grid->entry_point = reader->shm->grid_entry_point;

    // Copy all cells from shared memory
    for (int i = 0; i < gui_grid->total_cells; i++)
    {
        gui_grid->cells[i] = reader->shm->grid_cells[i];
    }

    // Copy survivor positions
    for (int i = 0; i < gui_grid->num_survivors; i++)
    {
        gui_grid->survivor_positions[i] = reader->shm->grid_survivor_positions[i];
    }
}

// Get best chromosome from current jobs (highest fitness)
Chromosome *gui_ipc_get_best_chromosome(GuiIpcReader *reader)
{
    if (!reader || !reader->connected || !reader->shm)
        return NULL;

    double best_fitness = -1e9;
    int best_slot = -1;

    // Search all job slots for best fitness
    for (int i = 0; i < MAX_WORKERS; i++)
    {
        if (reader->shm->jobs[i].status == JOB_DONE)
        {
            if (reader->shm->jobs[i].fitness > best_fitness)
            {
                best_fitness = reader->shm->jobs[i].fitness;
                best_slot = i;
            }
        }
    }

    if (best_slot == -1)
        return NULL;

    // Allocate and return copy of best chromosome
    Chromosome *best = malloc(sizeof(Chromosome));
    if (!best)
        return NULL;

    *best = reader->shm->jobs[best_slot].chromosome;
    return best;
}

// Cleanup IPC reader
void gui_ipc_cleanup(GuiIpcReader *reader)
{
    if (!reader)
        return;

    if (reader->shm && reader->shm != MAP_FAILED)
    {
        munmap(reader->shm, reader->shm_size);
        reader->shm = NULL;
    }

    if (reader->shm_fd >= 0)
    {
        close(reader->shm_fd);
        reader->shm_fd = -1;
    }

    reader->connected = 0;
    printf("[GUI-IPC] Disconnected from shared memory\n");
}

// Check if shared memory is available (non-destructive test)
int gui_ipc_is_available(void)
{
    int fd = shm_open(SHM_NAME, O_RDONLY, 0);
    if (fd < 0)
        return 0;

    close(fd);
    return 1;
}


int gui_ipc_copy_best(GuiIpcReader *reader, Chromosome *out_best,
                      double *out_fitness, int *out_survivors)
{
    if (!reader || !reader->connected || !reader->shm || !out_best)
        return 0;

    double best_fitness = -1e18;
    int best_slot = -1;

    for (int i = 0; i < MAX_WORKERS; i++)
    {
        if (reader->shm->jobs[i].status == JOB_DONE)
        {
            double f = reader->shm->jobs[i].fitness;
            if (f > best_fitness)
            {
                best_fitness = f;
                best_slot = i;
            }
        }
    }

    if (best_slot < 0)
        return 0;

    *out_best = reader->shm->jobs[best_slot].chromosome;

    if (out_fitness)   *out_fitness = reader->shm->jobs[best_slot].fitness;
    if (out_survivors) *out_survivors = reader->shm->jobs[best_slot].survivors_reached;
    return 1;
}

#include <time.h>

int gui_ipc_try_connect(GuiIpcReader *r)
{
    static double last_try = 0.0;

    if (!r) return 0;
    if (r->connected) return 1;

    // throttle to ~2 tries/sec
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double now = ts.tv_sec + ts.tv_nsec / 1e9;

    if (now - last_try < 0.5)
        return 0;

    last_try = now;

    // if SHM doesn't exist yet, don't call init (prevents spam/lag)
    if (!gui_ipc_is_available())
        return 0;

    // now try real init
    return (gui_ipc_init(r) == 0);
}


Grid* gui_ipc_get_grid(GuiIpcReader *r)
{
    if (!r || !r->connected || !r->shm) return NULL;

    static Grid *g = NULL;

    // allocate once (size comes from shm)
    if (!g)
    {
        g = create_grid(r->shm->grid_size_x, r->shm->grid_size_y, r->shm->grid_size_z);
        if (!g) return NULL;
    }

    gui_ipc_update_grid(r, g);
    return g;
}

void gui_ipc_disconnect(GuiIpcReader *r)
{
    gui_ipc_cleanup(r);
}

