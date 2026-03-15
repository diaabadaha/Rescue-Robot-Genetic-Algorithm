#ifndef GUI_CAMERA_H
#define GUI_CAMERA_H

typedef struct {
    float target[3];
    float yaw;
    float pitch;
    float distance;

    float fov_deg;
    float aspect;
    float near_z;
    float far_z;
} OrbitCamera;

// init camera
void camera_init(OrbitCamera *c, float aspect);

// input handlers
void camera_on_mouse_drag(OrbitCamera *c, float dx, float dy);
void camera_on_scroll(OrbitCamera *c, float scroll_dy);

// matrix output: out = projection * view
void camera_get_viewproj(const OrbitCamera *c, float out16[16]);

#endif
