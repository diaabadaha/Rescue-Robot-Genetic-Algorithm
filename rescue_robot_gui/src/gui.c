// rescue_robot_gui/src/gui.c
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glad/glad.h>

#include "../include/gui.h"
#include "../include/gui_camera.h"
#include "../include/gui_renderer.h"
#include "../include/gui_ipc_reader.h"

#include "../../include/grid.h"
#include "../../include/common.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
static OrbitCamera g_cam;

static int g_dragging = 0;
static double g_last_x = 0.0, g_last_y = 0.0;

static int   g_paused = 0;
static float g_speed_steps_per_sec = 6.0f;
static float g_anim_step = 0.0f;
static double g_last_anim_time = 0.0;

static int g_show_paths  = 1;
static int g_show_robots = 1;
static double g_last_title_time = 0.0;
static Chromosome g_best;
static int g_have_best = 0;

// IPC
static GuiIpcReader g_ipc;
static int g_using_ipc = 0;

// Forward
static void key_cb(GLFWwindow *w, int key, int sc, int action, int mods);
static int  *g_surv_reach_step = NULL;
static int   g_surv_n = 0;
static double g_last_fit_seen = -1e300;
static double g_hud_fit = 0.0;
static int    g_hud_surv = 0;
static Grid *g_grid = NULL;

// Helpers
static void reset_grid_runtime_state(Grid *grid)
{
    if (!grid) return;

    for (int i = 0; i < grid->total_cells; i++)
    {
        grid->cells[i].visited = 0;
        grid->cells[i].survivor_helped = 0;
    }
}

static void hud_set_title(GLFWwindow *win,
                          int using_ipc,
                          int have_best,
                          const Chromosome *best,
                          double fit,
                          int surv,
                          int step)
{
    char title[256];

    if (!using_ipc)
    {
        snprintf(title, sizeof(title), "Rescue Robot GUI | IPC: connecting...");
    }
    else if (!have_best)
    {
        snprintf(title, sizeof(title), "Rescue Robot GUI | IPC: connected | waiting best...");
    }
    else
    {
        // You can edit the fields shown, but keep it short
        snprintf(title, sizeof(title),
                 "Rescue Robot GUI | Fit: %.2f | Survivors: %d | Covered: %d | Step: %d",
                 fit,
                 surv,
                 best->cells_covered,
                 step);
    }

    glfwSetWindowTitle(win, title);
}


static void recompute_survivor_reach_steps(const Grid *grid, const Chromosome *best)
{
    if (!grid || !best || grid->num_survivors <= 0) return;

    if (g_surv_n != grid->num_survivors)
    {
        free(g_surv_reach_step);
        g_surv_n = grid->num_survivors;
        g_surv_reach_step = (int*)malloc(sizeof(int) * g_surv_n);
    }

    for (int s = 0; s < g_surv_n; s++)
    {
        Vec3 sp = grid->survivor_positions[s];
        int best_step = -1;

        for (int r = 0; r < best->num_robots; r++)
        {
            const Robot *rb = &best->robots[r];
            int plen = rb->path_length;
            if (plen > MAX_PATH_LENGTH) plen = MAX_PATH_LENGTH;

            for (int i = 0; i < plen; i++)
            {
                if (vec3_equal(rb->path[i], sp))
                {
                    if (best_step < 0 || i < best_step) best_step = i;
                    break; // this robot’s earliest reach found
                }
            }
        }

        g_surv_reach_step[s] = best_step; // -1 if never reached
    }
}

static void apply_survivor_colors_for_step(Grid *grid, int step)
{
    if (!grid || !g_surv_reach_step || g_surv_n != grid->num_survivors) return;

    // Clear all survivor_helped first (CRITICAL)
    for (int s = 0; s < g_surv_n; s++)
    {
        Vec3 sp = grid->survivor_positions[s];
        Cell *c = get_cell_vec(grid, sp);
        if (c)
        {
            c->has_survivor = 1;
            c->survivor_helped = 0;
        }
    }

    // Apply up to current step
    for (int s = 0; s < g_surv_n; s++)
    {
        int rs = g_surv_reach_step[s];
        if (rs >= 0 && step >= rs)
        {
            Vec3 sp = grid->survivor_positions[s];
            Cell *c = get_cell_vec(grid, sp);
            if (c)
                c->survivor_helped = 1;
        }
    }
}


static int compute_max_path_len(const Chromosome *c)
{
    if (!c) return 0;
    int max_len = 0;
    for (int r = 0; r < c->num_robots; r++)
    {
        if (c->robots[r].path_length > max_len)
            max_len = c->robots[r].path_length;
    }
    return max_len;
}

static void mouse_button_cb(GLFWwindow *w, int button, int action, int mods)
{
    (void)mods;
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;

    if (action == GLFW_PRESS)
    {
        g_dragging = 1;
        glfwGetCursorPos(w, &g_last_x, &g_last_y);
    }
    else if (action == GLFW_RELEASE)
    {
        g_dragging = 0;
    }
}

static void cursor_pos_cb(GLFWwindow *w, double x, double y)
{
    (void)w;
    if (!g_dragging) return;

    float dx = (float)(x - g_last_x);
    float dy = (float)(y - g_last_y);
    g_last_x = x;
    g_last_y = y;

    camera_on_mouse_drag(&g_cam, dx, dy);
}

static void scroll_cb(GLFWwindow *w, double sx, double sy)
{
    (void)w; (void)sx;
    camera_on_scroll(&g_cam, (float)sy);
}

// -----------------------------------------------------------------------------
// Main GUI loop
// -----------------------------------------------------------------------------
int gui_run(void)
{
    if (!glfwInit())
    {
        fprintf(stderr, "[GUI] glfwInit failed\n");
        return 1;
    }

    // OpenGL 3.3 core
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *win = glfwCreateWindow(1280, 720, "Rescue Robot GUI", NULL, NULL);
    if (!win)
    {
        fprintf(stderr, "[GUI] glfwCreateWindow failed\n");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(win);
    glfwSwapInterval(1); // vsync

    // Callbacks
    glfwSetKeyCallback(win, key_cb);
    glfwSetMouseButtonCallback(win, mouse_button_cb);
    glfwSetCursorPosCallback(win, cursor_pos_cb);
    glfwSetScrollCallback(win, scroll_cb);

    // Load GL
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        fprintf(stderr, "[GUI] gladLoadGLLoader failed\n");
        glfwDestroyWindow(win);
        glfwTerminate();
        return 1;
    }

    printf("[GUI] OpenGL: %s | Renderer: %s\n",
           glGetString(GL_VERSION), glGetString(GL_RENDERER));
    printf("[GUI] Controls: drag=rotate, scroll=zoom, SPACE=pause, R=reset, P=paths, O=robots, +/- speed\n");

    glEnable(GL_DEPTH_TEST);

    // Camera init
    camera_init(&g_cam, 1280.0f / 720.0f);
    g_cam.distance = 30.0f;
    g_cam.pitch = 0.9f;
    g_cam.yaw   = 0.8f;

    // Renderer init
    GuiRenderer renderer;
    if (renderer_init(&renderer) != 0)
    {
        fprintf(stderr, "[GUI] renderer_init failed\n");
        glfwDestroyWindow(win);
        glfwTerminate();
        return 1;
    }

    // Start in demo grid fast (optional), then auto-connect IPC
    Grid *grid = create_grid(10, 10, 3);
    g_grid = grid;
    if (!grid)
    {
        fprintf(stderr, "[GUI] create_grid failed\n");
        renderer_destroy(&renderer);
        glfwDestroyWindow(win);
        glfwTerminate();
        return 1;
    }
    grid->entry_point = (Vec3){0,0,0};

    // Init IPC reader (auto connect later)
    memset(&g_ipc, 0, sizeof(g_ipc));
    g_using_ipc = 0;

    // Timing
    double last_time = glfwGetTime();
    double fps_acc = 0.0;
    int fps_frames = 0;

    g_last_anim_time = glfwGetTime();

    while (!glfwWindowShouldClose(win))
{
    glfwPollEvents();

    // ------------------------------------------------------------
    // IPC connect (NON-BLOCKING)
    // ------------------------------------------------------------
    if (!g_using_ipc)
    {
        // try_connect is throttled internally (good)
        if (gui_ipc_try_connect(&g_ipc))
        {
            g_using_ipc = 1;

            // Replace demo grid with real IPC grid
            if (grid) { free_grid(grid); grid = NULL; }
                grid = gui_ipc_get_grid(&g_ipc);
                g_grid = grid;


            // reset best state (optional but clean)
            g_have_best = 0;
            g_last_fit_seen = -1.0;

            // optional: reset replay
            g_anim_step = 0.0f;
            g_last_anim_time = glfwGetTime();
        }
    }
    else
    {
        // ------------------------------------------------------------
        // Live grid update (NON-BLOCKING)
        // ------------------------------------------------------------
        if (grid)
{
    gui_ipc_update_grid(&g_ipc, grid);

    // IMPORTANT: GUI controls survivor coloring, NOT shared memory
    for (int i = 0; i < grid->total_cells; i++)
        grid->cells[i].survivor_helped = 0;
}


        // ------------------------------------------------------------
        // Pull best snapshot once per frame (NON-BLOCKING)
        // ------------------------------------------------------------
        Chromosome tmp;
        double fit = 0.0;
        int surv = 0;

        if (gui_ipc_copy_best(&g_ipc, &tmp, &fit, &surv))
        {
            g_best = tmp;        // struct copy
            g_have_best = 1;

            // Only recompute survivor reach steps when fitness changes
            // (this makes reset work + avoids heavy recompute every frame)
            if (grid && fit != g_last_fit_seen)
            {
                recompute_survivor_reach_steps(grid, &g_best);
                g_last_fit_seen = fit;
            }
        }
    }

    // ------------------------------------------------------------
    // Animation timing (always progresses if have best + not paused)
    // ------------------------------------------------------------
    double now = glfwGetTime();
    double dt = now - last_time;
    last_time = now;

    if (!g_paused && g_have_best)
    {
        g_anim_step += (float)(dt * g_speed_steps_per_sec);

        int max_len = compute_max_path_len(&g_best);
        if (max_len > 0 && g_anim_step > (float)max_len)
            g_anim_step = (float)max_len;
    }

    // ------------------------------------------------------------
    // Render
    // ------------------------------------------------------------
    int fbw, fbh;
    glfwGetFramebufferSize(win, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float vp[16];
    g_cam.aspect = (fbh > 0) ? ((float)fbw / (float)fbh) : 1.0f;
    camera_get_viewproj(&g_cam, vp);


    int step = (int)g_anim_step;

// Apply survivor helped state BEFORE drawing cubes
if (grid && g_have_best)
    apply_survivor_colors_for_step(grid, step);

renderer_draw_ground_grid(&renderer, grid, vp);
renderer_draw_grid(&renderer, grid, vp);

if (g_have_best) {
    if (g_show_paths)  renderer_draw_paths(&renderer, grid, &g_best, step, vp);
    if (g_show_robots) renderer_draw_robots(&renderer, grid, &g_best, step, vp);
}


    glfwSwapBuffers(win);

    // ------------------------------------------------------------
    // HUD Phase 1 (window title) — throttled (IMPORTANT)
    // ------------------------------------------------------------
    if (now - g_last_title_time >= 0.20) // 5 updates/sec
    {
        char title[256];

        if (!g_using_ipc)
        {
            snprintf(title, sizeof(title), "Rescue Robot GUI | IPC: connecting...");
        }
        else if (!g_have_best)
        {
            snprintf(title, sizeof(title), "Rescue Robot GUI | IPC connected | waiting best...");
        }
        else
        {
            snprintf(title, sizeof(title),
                     "Rescue Robot GUI | Fit: %.2f | Survivors: %d | Covered: %d | Step: %d",
                     g_last_fit_seen,
                     g_best.survivors_reached,
                     g_best.cells_covered,
                     step);
        }

        glfwSetWindowTitle(win, title);
        g_last_title_time = now;
    }
}

    // Cleanup
    if (g_using_ipc)
        gui_ipc_disconnect(&g_ipc);

    if (grid) {
    free_grid(grid);
    grid = NULL;
}
g_grid = NULL;


    renderer_destroy(&renderer);
    glfwDestroyWindow(win);
    glfwTerminate();

    return 0;
}

// -----------------------------------------------------------------------------
// Keyboard
// -----------------------------------------------------------------------------
static void key_cb(GLFWwindow *w, int key, int sc, int action, int mods)
{
    (void)w; (void)sc; (void)mods;
    if (action != GLFW_PRESS) return;

    if (key == GLFW_KEY_SPACE) g_paused = !g_paused;

   if (key == GLFW_KEY_R)
{
    g_anim_step = 0.0f;
    g_last_anim_time = glfwGetTime();
    g_paused = 1;
}
    if (key == GLFW_KEY_P) g_show_paths = !g_show_paths;
    if (key == GLFW_KEY_O) g_show_robots = !g_show_robots;

    if (key == GLFW_KEY_EQUAL || key == GLFW_KEY_KP_ADD) g_speed_steps_per_sec *= 1.25f;
    if (key == GLFW_KEY_MINUS || key == GLFW_KEY_KP_SUBTRACT) g_speed_steps_per_sec *= 0.80f;

    if (g_speed_steps_per_sec < 0.5f) g_speed_steps_per_sec = 0.5f;
    if (g_speed_steps_per_sec > 60.0f) g_speed_steps_per_sec = 60.0f;
}
