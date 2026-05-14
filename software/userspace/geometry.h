// geometry.h

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <stdint.h>
#include "../kernel/avalon_kernel.h"


typedef struct { float x, y, z; } vec3_t;
typedef struct { float x, y, z, w; } vec4_t;
typedef struct { float m[4][4]; } mat4_t;


typedef struct { float sx, sy, sz; } screen_vertex_t;


mat4_t mat4_identity(void);
mat4_t mat4_mul(const mat4_t *a, const mat4_t *b);
vec4_t mat4_mul_vec4(const mat4_t *m, vec4_t v);

mat4_t rotation_x(float deg);
mat4_t rotation_y(float deg);
mat4_t rotation_z(float deg);
mat4_t translation(float tx, float ty, float tz);
mat4_t perspective(float fov_deg, float aspect, float near, float far);


vec3_t vec3_sub(vec3_t a, vec3_t b);
vec3_t vec3_cross(vec3_t a, vec3_t b);
vec3_t vec3_normalize(vec3_t v);
float  vec3_dot(vec3_t a, vec3_t b);


uint8_t shade_face(vec3_t normal, vec3_t light_dir,
                   float base_r, float base_g, float base_b);


int setup_triangle(const screen_vertex_t *v0,
                   const screen_vertex_t *v1,
                   const screen_vertex_t *v2,
                   uint8_t color,
                   triangle_packet_t *out);

#endif
