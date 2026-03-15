#ifndef GUI_IPC_READER_H
#define GUI_IPC_READER_H

#include "../../include/common.h"
#include "../../include/grid.h"
#include "../../include/ipc.h"



// GUI IPC Reader - reads data from main project's shared memory
typedef struct {
    SharedMemory *shm;     // Pointer to mapped shared memory
    int shm_fd;            // Shared memory file descriptor
    size_t shm_size;       // Size of shared memory
    int connected;         // 1 if connected to shared memory
} GuiIpcReader;

// Initialize IPC reader (opens existing shared memory READ-ONLY)
int gui_ipc_init(GuiIpcReader *reader);

// Update grid from shared memory (reads current state)
void gui_ipc_update_grid(GuiIpcReader *reader, Grid *gui_grid);

// Get best chromosome from shared memory
Chromosome* gui_ipc_get_best_chromosome(GuiIpcReader *reader);

// Cleanup IPC reader
void gui_ipc_cleanup(GuiIpcReader *reader);

// Check if shared memory is available (GA is running)
int gui_ipc_is_available(void);

// Copy best chromosome (no malloc). Returns 1 if found, 0 otherwise.
int gui_ipc_copy_best(GuiIpcReader *reader, Chromosome *out_best,
                      double *out_fitness, int *out_survivors);


int   gui_ipc_try_connect(GuiIpcReader *r);
Grid* gui_ipc_get_grid(GuiIpcReader *r);
void  gui_ipc_disconnect(GuiIpcReader *r);

#endif // GUI_IPC_READER_H