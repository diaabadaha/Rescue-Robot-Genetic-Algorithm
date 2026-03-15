#include "../include/gui_renderer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <glad/glad.h>
#include <GLFW/glfw3.h>

static void robot_color(int i, float col[3]);

// ----------------  shaders with lighting ----------------
static const char *VS_SRC =
    "#version 330 core\n"
    "layout(location=0) in vec3 aPos;\n"
    "layout(location=1) in vec3 aCol;\n"
    "layout(location=2) in vec3 aNormal;\n"
    "uniform mat4 uVP;\n"
    "out vec3 vCol;\n"
    "out vec3 vNormal;\n"
    "out vec3 vFragPos;\n"
    "void main(){\n"
    "  vCol = aCol;\n"
    "  vNormal = aNormal;\n"
    "  vFragPos = aPos;\n"
    "  gl_Position = uVP * vec4(aPos,1.0);\n"
    "}\n";

static const char *FS_SRC =
    "#version 330 core\n"
    "in vec3 vCol;\n"
    "in vec3 vNormal;\n"
    "in vec3 vFragPos;\n"
    "uniform float uTime;\n"
    "uniform vec3 uLightDir;\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "  float ambient = 0.4;\n"
    "  vec3 norm = normalize(vNormal);\n"
    "  float diff = max(dot(norm, uLightDir), 0.0);\n"
    "  float diffuse = diff * 0.6;\n"
    "  float lighting = ambient + diffuse;\n"
    "  vec3 result = vCol * lighting;\n"
    "  FragColor = vec4(result, 1.0);\n"
    "}\n";

// Wireframe/border shader
static const char *WIRE_VS_SRC =
    "#version 330 core\n"
    "layout(location=0) in vec3 aPos;\n"
    "uniform mat4 uVP;\n"
    "void main(){\n"
    "  gl_Position = uVP * vec4(aPos,1.0);\n"
    "}\n";

static const char *WIRE_FS_SRC =
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main(){ FragColor = vec4(0.1, 0.1, 0.1, 1.0); }\n";

// Grid shader (also used for paths)
static const char *GRID_VS_SRC =
    "#version 330 core\n"
    "layout(location=0) in vec3 aPos;\n"
    "layout(location=1) in vec3 aCol;\n"
    "uniform mat4 uVP;\n"
    "out vec3 vCol;\n"
    "void main(){\n"
    "  vCol = aCol;\n"
    "  gl_Position = uVP * vec4(aPos,1.0);\n"
    "}\n";

static const char *GRID_FS_SRC =
    "#version 330 core\n"
    "in vec3 vCol;\n"
    "out vec4 FragColor;\n"
    "void main(){ FragColor = vec4(vCol, 0.3); }\n"; // Semi-transparent

static GLuint compile_shader(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);

    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char log[1024];
        glGetShaderInfoLog(s, sizeof(log), NULL, log);
        fprintf(stderr, "[GUI] shader compile failed: %s\n", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

static GLuint create_program(const char *vs_src, const char *fs_src)
{
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_src);
    if (!vs) return 0;

    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fs_src);
    if (!fs)
    {
        glDeleteShader(vs);
        return 0;
    }

    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);

    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char log[1024];
        glGetProgramInfoLog(p, sizeof(log), NULL, log);
        fprintf(stderr, "[GUI] program link failed: %s\n", log);
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

// -------------- color helpers --------------
static void heat_to_color(float heat, float out[3])
{
    if (heat < 40.0f)
    {
        float t = (heat - 20.0f) / 20.0f; // 0..1
        out[0] = 1.0f;
        out[1] = 0.95f + t * 0.05f;
        out[2] = 0.6f;
    }
    else if (heat < 60.0f)
    {
        out[0] = 1.0f;
        out[1] = 0.8f;
        out[2] = 0.3f;
    }
    else if (heat < 80.0f)
    {
        out[0] = 1.0f;
        out[1] = 0.55f;
        out[2] = 0.1f;
    }
    else if (heat < 100.0f)
    {
        out[0] = 0.9f;
        out[1] = 0.25f;
        out[2] = 0.1f;
    }
    else
    {
        out[0] = 0.55f;
        out[1] = 0.0f;
        out[2] = 0.0f;
    }
}

static void cell_color(const Cell *c, float out[3], float time)
{
    if (c->type == CELL_OBSTACLE)
    {
        out[0] = 0.45f;
        out[1] = 0.28f;
        out[2] = 0.12f; // brown
        return;
    }

    heat_to_color(c->heat, out);

    if (c->has_survivor)
    {
        if (c->survivor_helped)
        {
            out[0] = 0.1f;
            out[1] = 0.85f;
            out[2] = 0.85f; // cyan
        }
        else
        {
            float pulse = 0.5f + 0.5f * sinf(time * 3.0f);
            out[0] = 0.15f + pulse * 0.4f;
            out[1] = 0.95f;
            out[2] = 0.15f + pulse * 0.4f;
        }
        return;
    }

    if (c->visited)
    {
        out[0] = fminf(1.0f, out[0] + 0.15f);
        out[1] = fminf(1.0f, out[1] + 0.15f);
        out[2] = fminf(1.0f, out[2] + 0.15f);
    }
}

static void grid_to_world(const Grid *grid, Vec3 p, float *wx, float *wy, float *wz)
{
    const float spacing = 1.00f;
    const float layer_spacing = 2.5f;

    const float origin_x = -(grid->size_x - 1) * spacing * 0.5f;
    const float origin_y = -(grid->size_z - 1) * layer_spacing * 0.5f;
    const float origin_z = -(grid->size_y - 1) * spacing * 0.5f;

    *wx = origin_x + p.x * spacing;
    *wy = origin_y + p.z * layer_spacing;
    *wz = origin_z + p.y * spacing;
}

static void push_sphere(float *cpu, size_t *off,
                        float cx, float cy, float cz,
                        float r, const float col[3],
                        int slices, int stacks)
{
    for (int i = 0; i < stacks; i++)
    {
        float v0 = (float)i / (float)stacks;
        float v1 = (float)(i + 1) / (float)stacks;

        float phi0 = (v0 - 0.5f) * (float)M_PI;
        float phi1 = (v1 - 0.5f) * (float)M_PI;

        float y0 = sinf(phi0), c0 = cosf(phi0);
        float y1 = sinf(phi1), c1 = cosf(phi1);

        for (int j = 0; j < slices; j++)
        {
            float u0 = (float)j / (float)slices;
            float u1 = (float)(j + 1) / (float)slices;

            float th0 = u0 * 2.0f * (float)M_PI;
            float th1 = u1 * 2.0f * (float)M_PI;

            float x00 = c0 * cosf(th0), z00 = c0 * sinf(th0);
            float x01 = c0 * cosf(th1), z01 = c0 * sinf(th1);
            float x10 = c1 * cosf(th0), z10 = c1 * sinf(th0);
            float x11 = c1 * cosf(th1), z11 = c1 * sinf(th1);

            float nx, ny, nz;

            // p00
            nx = x00; ny = y0; nz = z00;
            cpu[(*off)++] = cx + r * nx; cpu[(*off)++] = cy + r * ny; cpu[(*off)++] = cz + r * nz;
            cpu[(*off)++] = col[0]; cpu[(*off)++] = col[1]; cpu[(*off)++] = col[2];
            cpu[(*off)++] = nx; cpu[(*off)++] = ny; cpu[(*off)++] = nz;

            // p10
            nx = x10; ny = y1; nz = z10;
            cpu[(*off)++] = cx + r * nx; cpu[(*off)++] = cy + r * ny; cpu[(*off)++] = cz + r * nz;
            cpu[(*off)++] = col[0]; cpu[(*off)++] = col[1]; cpu[(*off)++] = col[2];
            cpu[(*off)++] = nx; cpu[(*off)++] = ny; cpu[(*off)++] = nz;

            // p11
            nx = x11; ny = y1; nz = z11;
            cpu[(*off)++] = cx + r * nx; cpu[(*off)++] = cy + r * ny; cpu[(*off)++] = cz + r * nz;
            cpu[(*off)++] = col[0]; cpu[(*off)++] = col[1]; cpu[(*off)++] = col[2];
            cpu[(*off)++] = nx; cpu[(*off)++] = ny; cpu[(*off)++] = nz;

            // p00
            nx = x00; ny = y0; nz = z00;
            cpu[(*off)++] = cx + r * nx; cpu[(*off)++] = cy + r * ny; cpu[(*off)++] = cz + r * nz;
            cpu[(*off)++] = col[0]; cpu[(*off)++] = col[1]; cpu[(*off)++] = col[2];
            cpu[(*off)++] = nx; cpu[(*off)++] = ny; cpu[(*off)++] = nz;

            // p11
            nx = x11; ny = y1; nz = z11;
            cpu[(*off)++] = cx + r * nx; cpu[(*off)++] = cy + r * ny; cpu[(*off)++] = cz + r * nz;
            cpu[(*off)++] = col[0]; cpu[(*off)++] = col[1]; cpu[(*off)++] = col[2];
            cpu[(*off)++] = nx; cpu[(*off)++] = ny; cpu[(*off)++] = nz;

            // p01
            nx = x01; ny = y0; nz = z01;
            cpu[(*off)++] = cx + r * nx; cpu[(*off)++] = cy + r * ny; cpu[(*off)++] = cz + r * nz;
            cpu[(*off)++] = col[0]; cpu[(*off)++] = col[1]; cpu[(*off)++] = col[2];
            cpu[(*off)++] = nx; cpu[(*off)++] = ny; cpu[(*off)++] = nz;
        }
    }
}

// -------------- cube mesh with normals --------------
static const float CUBE_DATA[36][6] = {
    {-0.5f, -0.5f, 0.5f, 0, 0, 1},
    {0.5f, -0.5f, 0.5f, 0, 0, 1},
    {0.5f, 0.5f, 0.5f, 0, 0, 1},
    {-0.5f, -0.5f, 0.5f, 0, 0, 1},
    {0.5f, 0.5f, 0.5f, 0, 0, 1},
    {-0.5f, 0.5f, 0.5f, 0, 0, 1},

    {-0.5f, -0.5f, -0.5f, 0, 0, -1},
    {0.5f, 0.5f, -0.5f, 0, 0, -1},
    {0.5f, -0.5f, -0.5f, 0, 0, -1},
    {-0.5f, -0.5f, -0.5f, 0, 0, -1},
    {-0.5f, 0.5f, -0.5f, 0, 0, -1},
    {0.5f, 0.5f, -0.5f, 0, 0, -1},

    {-0.5f, -0.5f, -0.5f, -1, 0, 0},
    {-0.5f, -0.5f, 0.5f, -1, 0, 0},
    {-0.5f, 0.5f, 0.5f, -1, 0, 0},
    {-0.5f, -0.5f, -0.5f, -1, 0, 0},
    {-0.5f, 0.5f, 0.5f, -1, 0, 0},
    {-0.5f, 0.5f, -0.5f, -1, 0, 0},

    {0.5f, -0.5f, -0.5f, 1, 0, 0},
    {0.5f, 0.5f, 0.5f, 1, 0, 0},
    {0.5f, -0.5f, 0.5f, 1, 0, 0},
    {0.5f, -0.5f, -0.5f, 1, 0, 0},
    {0.5f, 0.5f, -0.5f, 1, 0, 0},
    {0.5f, 0.5f, 0.5f, 1, 0, 0},

    {-0.5f, 0.5f, -0.5f, 0, 1, 0},
    {-0.5f, 0.5f, 0.5f, 0, 1, 0},
    {0.5f, 0.5f, 0.5f, 0, 1, 0},
    {-0.5f, 0.5f, -0.5f, 0, 1, 0},
    {0.5f, 0.5f, 0.5f, 0, 1, 0},
    {0.5f, 0.5f, -0.5f, 0, 1, 0},

    {-0.5f, -0.5f, -0.5f, 0, -1, 0},
    {0.5f, -0.5f, 0.5f, 0, -1, 0},
    {-0.5f, -0.5f, 0.5f, 0, -1, 0},
    {-0.5f, -0.5f, -0.5f, 0, -1, 0},
    {0.5f, -0.5f, -0.5f, 0, -1, 0},
    {0.5f, -0.5f, 0.5f, 0, -1, 0},
};

// ---------------- public API ----------------
int renderer_init(GuiRenderer *r)
{
    memset(r, 0, sizeof(*r));

    r->prog = create_program(VS_SRC, FS_SRC);
    if (!r->prog) return -1;

    r->wire_prog = create_program(WIRE_VS_SRC, WIRE_FS_SRC);
    if (!r->wire_prog)
    {
        glDeleteProgram(r->prog);
        return -1;
    }

    r->grid_prog = create_program(GRID_VS_SRC, GRID_FS_SRC);
    if (!r->grid_prog)
    {
        glDeleteProgram(r->prog);
        glDeleteProgram(r->wire_prog);
        return -1;
    }

    // Main VAO/VBO for cubes
    glGenVertexArrays(1, &r->vao);
    glGenBuffers(1, &r->vbo);

    glBindVertexArray(r->vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->vbo);

    r->vtx_capacity = 2048 * 1024; // floats
    glBufferData(GL_ARRAY_BUFFER, r->vtx_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void *)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    // Wireframe VAO/VBO
    glGenVertexArrays(1, &r->wire_vao);
    glGenBuffers(1, &r->wire_vbo);

    glBindVertexArray(r->wire_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->wire_vbo);

    r->wire_capacity = 512 * 1024; // floats
    glBufferData(GL_ARRAY_BUFFER, r->wire_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // Ground grid VAO/VBO
    glGenVertexArrays(1, &r->grid_vao);
    glGenBuffers(1, &r->grid_vbo);

    glBindVertexArray(r->grid_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->grid_vbo);

    r->grid_capacity = 128 * 1024; // floats
    glBufferData(GL_ARRAY_BUFFER, r->grid_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Robot VAO/VBO
    glGenVertexArrays(1, &r->robot_vao);
    glGenBuffers(1, &r->robot_vbo);

    glBindVertexArray(r->robot_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->robot_vbo);

    r->robot_capacity = 256 * 1024; // floats
    glBufferData(GL_ARRAY_BUFFER, r->robot_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void *)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    // Path VAO/VBO
    glGenVertexArrays(1, &r->path_vao);
    glGenBuffers(1, &r->path_vbo);

    glBindVertexArray(r->path_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->path_vbo);

    r->path_capacity = 256 * 1024; // floats
    glBufferData(GL_ARRAY_BUFFER, r->path_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return 0;
}

void renderer_destroy(GuiRenderer *r)
{
    if (!r) return;

    if (r->vbo) glDeleteBuffers(1, &r->vbo);
    if (r->vao) glDeleteVertexArrays(1, &r->vao);

    if (r->wire_vbo) glDeleteBuffers(1, &r->wire_vbo);
    if (r->wire_vao) glDeleteVertexArrays(1, &r->wire_vao);

    if (r->grid_vbo) glDeleteBuffers(1, &r->grid_vbo);
    if (r->grid_vao) glDeleteVertexArrays(1, &r->grid_vao);

    if (r->robot_vbo) glDeleteBuffers(1, &r->robot_vbo);
    if (r->robot_vao) glDeleteVertexArrays(1, &r->robot_vao);

    if (r->path_vbo) glDeleteBuffers(1, &r->path_vbo);
    if (r->path_vao) glDeleteVertexArrays(1, &r->path_vao);

    if (r->prog) glDeleteProgram(r->prog);
    if (r->wire_prog) glDeleteProgram(r->wire_prog);
    if (r->grid_prog) glDeleteProgram(r->grid_prog);

    memset(r, 0, sizeof(*r));
}

void renderer_draw_grid(GuiRenderer *r, const Grid *grid, const float viewproj[16])
{
    if (!r || !grid) return;

    const float spacing = 1.00f;
    const float layer_spacing = 2.5f;

    const float origin_x = -(grid->size_x - 1) * spacing * 0.5f;
    const float origin_y = -(grid->size_z - 1) * layer_spacing * 0.5f;
    const float origin_z = -(grid->size_y - 1) * spacing * 0.5f;

    float time = (float)glfwGetTime();

    size_t cells = (size_t)grid->total_cells;
    size_t floats_per_cell = 36u * 9u;
    size_t needed = cells * floats_per_cell;

    float *cpu = (float *)malloc(needed * sizeof(float));
    if (!cpu) return;

    size_t off = 0;

    for (int z = 0; z < grid->size_z; z++)
    for (int y = 0; y < grid->size_y; y++)
    for (int x = 0; x < grid->size_x; x++)
    {
        Cell *c = get_cell((Grid *)grid, x, y, z);

        float col[3];
        cell_color(c, col, time);

        if (x == (int)grid->entry_point.x && y == (int)grid->entry_point.y && z == (int)grid->entry_point.z)
        {
            col[0] = 0.9f;
            col[1] = 0.2f;
            col[2] = 0.9f; // magenta
        }

        float tx = origin_x + x * spacing;
        float ty = origin_y + z * layer_spacing;
        float tz = origin_z + y * spacing;

        const float cube_scale = 0.95f;

        for (int i = 0; i < 36; i++)
        {
            cpu[off++] = CUBE_DATA[i][0] * cube_scale + tx;
            cpu[off++] = CUBE_DATA[i][1] * cube_scale + ty;
            cpu[off++] = CUBE_DATA[i][2] * cube_scale + tz;

            cpu[off++] = col[0];
            cpu[off++] = col[1];
            cpu[off++] = col[2];

            cpu[off++] = CUBE_DATA[i][3];
            cpu[off++] = CUBE_DATA[i][4];
            cpu[off++] = CUBE_DATA[i][5];
        }
    }

    if (off > r->vtx_capacity)
    {
        r->vtx_capacity = off * 2;
        glBindBuffer(GL_ARRAY_BUFFER, r->vbo);
        glBufferData(GL_ARRAY_BUFFER, r->vtx_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, r->vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, off * sizeof(float), cpu);
    free(cpu);

    glUseProgram(r->prog);
    GLint loc_vp = glGetUniformLocation(r->prog, "uVP");
    GLint loc_time = glGetUniformLocation(r->prog, "uTime");
    GLint loc_light = glGetUniformLocation(r->prog, "uLightDir");

    glUniformMatrix4fv(loc_vp, 1, GL_FALSE, viewproj);
    glUniform1f(loc_time, time);

    float light_dir[3] = {0.3f, 0.8f, 0.5f};
    float len = sqrtf(light_dir[0]*light_dir[0] + light_dir[1]*light_dir[1] + light_dir[2]*light_dir[2]);
    light_dir[0] /= len; light_dir[1] /= len; light_dir[2] /= len;
    glUniform3fv(loc_light, 1, light_dir);

    glBindVertexArray(r->vao);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(off / 9));
}

void renderer_draw_ground_grid(GuiRenderer *r, const Grid *grid, const float viewproj[16])
{
    if (!r || !grid) return;

    const float spacing = 1.20f;
    const float layer_spacing = 2.5f;

    const float origin_x = -(grid->size_x - 1) * spacing * 0.5f;
    const float origin_z = -(grid->size_y - 1) * spacing * 0.5f;

    const float lowest_floor_y = -(grid->size_z - 1) * layer_spacing * 0.5f;
    const float y_level = lowest_floor_y - 0.8f;

    int lines_x = grid->size_y + 1;
    int lines_z = grid->size_x + 1;
    int total_lines = lines_x + lines_z;

    size_t floats_needed = (size_t)total_lines * 2u * 6u;

    float *cpu = (float *)malloc(floats_needed * sizeof(float));
    if (!cpu) return;

    size_t off = 0;
    float col[3] = {0.2f, 0.25f, 0.3f};

    for (int y = 0; y <= grid->size_y; y++)
    {
        float zpos = origin_z + y * spacing;

        cpu[off++] = origin_x - spacing; cpu[off++] = y_level; cpu[off++] = zpos;
        cpu[off++] = col[0]; cpu[off++] = col[1]; cpu[off++] = col[2];

        cpu[off++] = origin_x + (grid->size_x)*spacing; cpu[off++] = y_level; cpu[off++] = zpos;
        cpu[off++] = col[0]; cpu[off++] = col[1]; cpu[off++] = col[2];
    }

    for (int x = 0; x <= grid->size_x; x++)
    {
        float xpos = origin_x + x * spacing;

        cpu[off++] = xpos; cpu[off++] = y_level; cpu[off++] = origin_z - spacing;
        cpu[off++] = col[0]; cpu[off++] = col[1]; cpu[off++] = col[2];

        cpu[off++] = xpos; cpu[off++] = y_level; cpu[off++] = origin_z + (grid->size_y)*spacing;
        cpu[off++] = col[0]; cpu[off++] = col[1]; cpu[off++] = col[2];
    }

    if (off > r->grid_capacity)
    {
        r->grid_capacity = off * 2;
        glBindBuffer(GL_ARRAY_BUFFER, r->grid_vbo);
        glBufferData(GL_ARRAY_BUFFER, r->grid_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, r->grid_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, off * sizeof(float), cpu);
    free(cpu);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(r->grid_prog);
    GLint loc_vp = glGetUniformLocation(r->grid_prog, "uVP");
    glUniformMatrix4fv(loc_vp, 1, GL_FALSE, viewproj);

    glBindVertexArray(r->grid_vao);
    glLineWidth(1.0f);
    glDrawArrays(GL_LINES, 0, (GLsizei)(off / 6));

    glDisable(GL_BLEND);
}

void renderer_draw_robots(GuiRenderer *r, const Grid *grid, const Chromosome *best, int step, const float viewproj[16])
{
    if (!r || !grid || !best || best->num_robots <= 0) return;

    const int slices = 12;
    const int stacks = 8;
    const int verts_per_sphere = stacks * slices * 6;
    size_t floats_needed = (size_t)best->num_robots * (size_t)verts_per_sphere * 9u;

    float *cpu = (float*)malloc(floats_needed * sizeof(float));
    if (!cpu) return;

    size_t off = 0;

    for (int i = 0; i < best->num_robots; i++)
    {
        const Robot *rb = &best->robots[i];

        Vec3 p = rb->start_pos;
        if (rb->path_length > 0)
        {
            int idx = step;
            if (idx > rb->path_length - 1) idx = rb->path_length - 1;
            if (idx < 0) idx = 0;
            p = rb->path[idx];
        }

        float wx, wy, wz;
        grid_to_world(grid, p, &wx, &wy, &wz);

        float col[3]; robot_color(i, col);

        push_sphere(cpu, &off, wx, wy + 0.35f, wz, 0.35f, col, slices, stacks);
    }

    if (off > r->robot_capacity)
    {
        r->robot_capacity = off * 2;
        glBindBuffer(GL_ARRAY_BUFFER, r->robot_vbo);
        glBufferData(GL_ARRAY_BUFFER, r->robot_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, r->robot_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, off * sizeof(float), cpu);
    free(cpu);

    glUseProgram(r->prog);
    GLint loc_vp = glGetUniformLocation(r->prog, "uVP");
    GLint loc_time = glGetUniformLocation(r->prog, "uTime");
    GLint loc_light = glGetUniformLocation(r->prog, "uLightDir");

    float time = (float)glfwGetTime();
    glUniformMatrix4fv(loc_vp, 1, GL_FALSE, viewproj);
    glUniform1f(loc_time, time);

    float light_dir[3] = {0.3f, 0.8f, 0.5f};
    float len = sqrtf(light_dir[0]*light_dir[0] + light_dir[1]*light_dir[1] + light_dir[2]*light_dir[2]);
    light_dir[0]/=len; light_dir[1]/=len; light_dir[2]/=len;
    glUniform3fv(loc_light, 1, light_dir);

    glBindVertexArray(r->robot_vao);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(off / 9));
}

static void robot_color(int i, float col[3])
{
    col[0] = 0.2f; col[1] = 0.6f; col[2] = 1.0f;
    if (i % 3 == 1) { col[0]=1.0f; col[1]=0.6f; col[2]=0.2f; }
    if (i % 3 == 2) { col[0]=0.9f; col[1]=0.2f; col[2]=0.9f; }
}

// -----------------------------------------------------------------------------
// Paths: solid when exploring, dashed when "returning out"
// Dashed triggers:
//   1) after first reaching a survivor cell
//   2) OR when path starts trending toward entry point (3 consecutive decreases)
// -----------------------------------------------------------------------------
void renderer_draw_paths(GuiRenderer *r, const Grid *grid, const Chromosome *best,
                         int step, const float viewproj[16])
{
    if (!r || !grid || !best) return;

    #define CELL_AT(g, X, Y, Z) (&(g)->cells[(Z) * (g)->size_x * (g)->size_y + (Y) * (g)->size_x + (X)])

    int ex = (int)grid->entry_point.x;
    int ey = (int)grid->entry_point.y;
    int ez = (int)grid->entry_point.z;

    // -------- pass 1: count segments actually drawn --------
    size_t total_segments = 0;

    for (int ri = 0; ri < best->num_robots; ri++)
    {
        const Robot *rb = &best->robots[ri];
        if (rb->path_length < 2) continue;

        int last = step;
        if (last > rb->path_length - 1) last = rb->path_length - 1;
        if (last < 1) continue;

        int reached_survivor = 0;
        int return_phase = 0;

        int dec_run = 0;
        int prev_d = 0;

        for (int k = 0; k < last; k++)
        {
            Vec3 p = rb->path[k];
            int x = (int)p.x, y = (int)p.y, z = (int)p.z;

            int d = abs(x - ex) + abs(y - ey) + abs(z - ez);
            if (k == 0) prev_d = d;
            else {
                if (d < prev_d) dec_run++;
                else dec_run = 0;
                prev_d = d;

                if (dec_run >= 3) return_phase = 1;
            }

            if (x >= 0 && x < grid->size_x &&
                y >= 0 && y < grid->size_y &&
                z >= 0 && z < grid->size_z)
            {
                Cell *c = CELL_AT(grid, x, y, z);
                if (c->has_survivor) reached_survivor = 1;
            }

            if (reached_survivor) return_phase = 1;

            if (return_phase && (k % 2 == 0))
                continue;

            total_segments++;
        }
    }

    if (total_segments == 0) return;

    size_t floats_needed = total_segments * 2u * 6u;
    float *cpu = (float *)malloc(floats_needed * sizeof(float));
    if (!cpu) return;

    // -------- pass 2: fill vertices --------
    size_t off = 0;

    for (int ri = 0; ri < best->num_robots; ri++)
    {
        const Robot *rb = &best->robots[ri];
        if (rb->path_length < 2) continue;

        int last = step;
        if (last > rb->path_length - 1) last = rb->path_length - 1;
        if (last < 1) continue;

        float col[3];
        robot_color(ri, col);

        int reached_survivor = 0;
        int return_phase = 0;

        int dec_run = 0;
        int prev_d = 0;

        for (int k = 0; k < last; k++)
        {
            Vec3 p = rb->path[k];
            int x = (int)p.x, y = (int)p.y, z = (int)p.z;

            int d = abs(x - ex) + abs(y - ey) + abs(z - ez);
            if (k == 0) prev_d = d;
            else {
                if (d < prev_d) dec_run++;
                else dec_run = 0;
                prev_d = d;

                if (dec_run >= 3) return_phase = 1;
            }

            if (x >= 0 && x < grid->size_x &&
                y >= 0 && y < grid->size_y &&
                z >= 0 && z < grid->size_z)
            {
                Cell *c = CELL_AT(grid, x, y, z);
                if (c->has_survivor) reached_survivor = 1;
            }

            if (reached_survivor) return_phase = 1;

            if (return_phase && (k % 2 == 0))
                continue;

            float x0, y0, z0, x1, y1, z1;
            grid_to_world(grid, rb->path[k],   &x0, &y0, &z0);
            grid_to_world(grid, rb->path[k+1], &x1, &y1, &z1);

            cpu[off++] = x0; cpu[off++] = y0; cpu[off++] = z0;
            cpu[off++] = col[0]; cpu[off++] = col[1]; cpu[off++] = col[2];

            cpu[off++] = x1; cpu[off++] = y1; cpu[off++] = z1;
            cpu[off++] = col[0]; cpu[off++] = col[1]; cpu[off++] = col[2];
        }
    }

    if (off > r->path_capacity)
    {
        r->path_capacity = off * 2u;
        glBindBuffer(GL_ARRAY_BUFFER, r->path_vbo);
        glBufferData(GL_ARRAY_BUFFER, r->path_capacity * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, r->path_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, off * sizeof(float), cpu);
    free(cpu);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(r->grid_prog);
    GLint loc_vp = glGetUniformLocation(r->grid_prog, "uVP");
    glUniformMatrix4fv(loc_vp, 1, GL_FALSE, viewproj);

    glBindVertexArray(r->path_vao);
    glLineWidth(4.0f);
    glDrawArrays(GL_LINES, 0, (GLsizei)(off / 6));

    glDisable(GL_BLEND);

    #undef CELL_AT
}
