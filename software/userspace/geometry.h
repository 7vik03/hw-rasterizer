/*
 * geometry.h
 * Float 3D math + triangle setup → triangle_packet_t.
 * Mirrors demo.cpp exactly; runs on ARM HPS before sending to hardware.
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <stdint.h>
#include "../kernel/avalon_kernel.h"

/* ------------------------------------------------------------------ */
/* Vec3 / Mat4 (float — ARM side only)                                 */
/* ------------------------------------------------------------------ */
typedef struct { float x, y, z; } vec3_t;
typedef struct { float x, y, z, w; } vec4_t;
typedef struct { float m[4][4]; } mat4_t;

/* Screen-space vertex produced by the projection pipeline */
typedef struct { float sx, sy, sz; } screen_vertex_t;

/* ------------------------------------------------------------------ */
/* Matrix construction                                                 */
/* ------------------------------------------------------------------ */
mat4_t mat4_identity(void);
mat4_t mat4_mul(const mat4_t *a, const mat4_t *b);
vec4_t mat4_mul_vec4(const mat4_t *m, vec4_t v);

mat4_t rotation_x(float deg);
mat4_t rotation_y(float deg);
mat4_t rotation_z(float deg);
mat4_t translation(float tx, float ty, float tz);
mat4_t perspective(float fov_deg, float aspect, float near, float far);

/* ------------------------------------------------------------------ */
/* Vec3 helpers                                                         */
/* ------------------------------------------------------------------ */
vec3_t vec3_sub(vec3_t a, vec3_t b);
vec3_t vec3_cross(vec3_t a, vec3_t b);
vec3_t vec3_normalize(vec3_t v);
float  vec3_dot(vec3_t a, vec3_t b);

/* ------------------------------------------------------------------ */
/* Lighting (flat shading, RGB332 output)                              */
/* ------------------------------------------------------------------ */
/*
 * shade_face: given a world-space face normal and a light direction,
 * return an RGB332 byte.
 *   base_r/g/b in [0,1]
 *   light_dir   should be normalized
 */
uint8_t shade_face(vec3_t normal, vec3_t light_dir,
                   float base_r, float base_g, float base_b);

/* ------------------------------------------------------------------ */
/* Triangle setup                                                       */
/* ------------------------------------------------------------------ */
/*
 * setup_triangle: given three screen-space vertices and a color byte,
 * fill in a triangle_packet_t ready to hand to the kernel driver.
 *
 * Returns 0 on success, -1 if the triangle is degenerate or invisible
 * (back-face culled, zero area, entirely off screen).
 *
 * Screen dimensions must match hardware: SCREEN_W=256, SCREEN_H=240.
 * bbox_xmin is snapped to the nearest lower multiple of 16 to satisfy
 * the hardware invariant that each pixel unit owns columns
 * { PU_ID, PU_ID+16, PU_ID+32, … }.
 */
int setup_triangle(const screen_vertex_t *v0,
                   const screen_vertex_t *v1,
                   const screen_vertex_t *v2,
                   uint8_t color,
                   triangle_packet_t *out);

#endif /* GEOMETRY_H */
