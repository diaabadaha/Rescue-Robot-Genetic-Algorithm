#define _USE_MATH_DEFINES
#include <math.h>
#include "../include/gui_camera.h"
#include <string.h>

static void mat4_identity(float m[16])
{
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}
static void mat4_mul(float out[16], const float a[16], const float b[16])
{
    float r[16];
    for (int col = 0; col < 4; col++)
    {
        for (int row = 0; row < 4; row++)
        {
            r[col * 4 + row] =
                a[0 * 4 + row] * b[col * 4 + 0] +
                a[1 * 4 + row] * b[col * 4 + 1] +
                a[2 * 4 + row] * b[col * 4 + 2] +
                a[3 * 4 + row] * b[col * 4 + 3];
        }
    }
    memcpy(out, r, sizeof(r));
}
static void mat4_perspective(float m[16], float fov_deg, float aspect, float n, float f)
{
    float fov = fov_deg * 0.017453292519943295f;
    float t = tanf(fov * 0.5f);
    mat4_identity(m);
    m[0] = 1.0f / (aspect * t);
    m[5] = 1.0f / t;
    m[10] = -(f + n) / (f - n);
    m[11] = -1.0f;
    m[14] = -(2.0f * f * n) / (f - n);
    m[15] = 0.0f;
}
static void vec3_sub(float o[3], const float a[3], const float b[3])
{
    o[0] = a[0] - b[0];
    o[1] = a[1] - b[1];
    o[2] = a[2] - b[2];
}
static void vec3_cross(float o[3], const float a[3], const float b[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
static float vec3_dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static void vec3_norm(float v[3])
{
    float l = sqrtf(vec3_dot(v, v));
    if (l > 1e-6f)
    {
        v[0] /= l;
        v[1] /= l;
        v[2] /= l;
    }
}
static void mat4_lookat(float m[16], const float eye[3], const float at[3])
{
    float up[3] = {0, 1, 0};
    float f[3];
    vec3_sub(f, at, eye);
    vec3_norm(f);
    float s[3];
    vec3_cross(s, f, up);
    vec3_norm(s);
    float u[3];
    vec3_cross(u, s, f);

    mat4_identity(m);
    m[0] = s[0];
    m[1] = u[0];
    m[2] = -f[0];
    m[4] = s[1];
    m[5] = u[1];
    m[6] = -f[1];
    m[8] = s[2];
    m[9] = u[2];
    m[10] = -f[2];

    m[12] = -vec3_dot(s, eye);
    m[13] = -vec3_dot(u, eye);
    m[14] = vec3_dot(f, eye);
}

void camera_init(OrbitCamera *c, float aspect)
{
    c->target[0] = 0.0f;
    c->target[1] = 0.0f;
    c->target[2] = 0.0f;
    c->yaw = 0.9f;
    c->pitch = 0.7f;
    c->distance = 20.0f;
    c->fov_deg = 55.0f;
    c->aspect = aspect;
    c->near_z = 0.1f;
    c->far_z = 200.0f;
}

void camera_on_mouse_drag(OrbitCamera *c, float dx, float dy)
{
    c->yaw += dx * 0.006f;
    c->pitch += dy * 0.006f;
    if (c->pitch < 0.05f)
        c->pitch = 0.05f;
    if (c->pitch > 1.50f)
        c->pitch = 1.50f;
}

void camera_on_scroll(OrbitCamera *c, float scroll_dy)
{
    c->distance *= (scroll_dy > 0) ? 0.90f : 1.10f;
    if (c->distance < 2.0f)
        c->distance = 2.0f;
    if (c->distance > 120.0f)
        c->distance = 120.0f;
}

void camera_get_viewproj(const OrbitCamera *c, float out16[16])
{
    float eye[3];
    eye[0] = c->target[0] + c->distance * cosf(c->pitch) * cosf(c->yaw);
    eye[1] = c->target[1] + c->distance * sinf(c->pitch);
    eye[2] = c->target[2] + c->distance * cosf(c->pitch) * sinf(c->yaw);

    float view[16], proj[16];
    mat4_lookat(view, eye, c->target);
    mat4_perspective(proj, c->fov_deg, c->aspect, c->near_z, c->far_z);

    // OpenGL convention: clip = (proj * view) * world
    mat4_mul(out16, proj, view);
}
