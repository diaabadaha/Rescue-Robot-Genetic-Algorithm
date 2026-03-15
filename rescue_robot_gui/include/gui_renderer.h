#ifndef GUI_RENDERER_H
#define GUI_RENDERER_H

#include <stddef.h>
#include "../../include/grid.h"   // Grid/Cell
#include "../../include/common.h" // Vec3, Chromosome, constants

typedef struct
{
    unsigned int prog;      // Main shader program (with lighting)
    unsigned int wire_prog; // Wireframe/border shader program
    unsigned int grid_prog; // Ground grid shader program

    // Cubes
    unsigned int vao;
    unsigned int vbo;
    size_t vtx_capacity; // floats capacity in VBO
    size_t vtx_count;    // floats currently used

    // Wireframe
    unsigned int wire_vao;
    unsigned int wire_vbo;
    size_t wire_capacity;

    // Ground grid
    unsigned int grid_vao;
    unsigned int grid_vbo;
    size_t grid_capacity;

    // Robots (spheres)
    unsigned int robot_vao;
    unsigned int robot_vbo;
    size_t robot_capacity;

    // Paths (lines)
    unsigned int path_vao;
    unsigned int path_vbo;
    size_t path_capacity;

} GuiRenderer;

// init / destroy
int renderer_init(GuiRenderer *r);
void renderer_destroy(GuiRenderer *r);

// Draw building grid (cubes)
void renderer_draw_grid(GuiRenderer *r, const Grid *grid, const float viewproj[16]);

// Draw ground reference grid
void renderer_draw_ground_grid(GuiRenderer *r, const Grid *grid, const float viewproj[16]);

// Step 5: paths + robots from best chromosome
void renderer_draw_paths(GuiRenderer *r, const Grid *grid, const Chromosome *best,
                         int step, const float viewproj[16]);

void renderer_draw_robots(GuiRenderer *r, const Grid *grid, const Chromosome *best,
                          int step, const float viewproj[16]);

#endif // GUI_RENDERER_H
