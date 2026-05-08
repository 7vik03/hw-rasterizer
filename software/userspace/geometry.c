// geometry.c
//
// ARM-side 3D pipeline: matrix math, lighting, and triangle setup.
//
// Everything in this file runs on the HPS (ARM) before each frame.
// All arithmetic is floating-point.  The output of setup_triangle()
// is a triangle_packet_t in Q12.12 fixed-point that the kernel driver
// writes word-by-word to the Avalon-MM slave.
//
// Hardware invariants enforced here:
//   - bbox_xmin is snapped down to the nearest multiple of 16 so each
//     of the 16 pixel units owns a clean column bank.  PU_ID owns
//     columns { PU_ID, PU_ID+16, PU_ID+32, ... }.
//   - Edge initial values (e0/e1/e2_init) are evaluated at pixel centre
//     (bbox_xmin + 0.5, bbox_ymin + 0.5), matching what the hardware
//     expects at stage[9..11].
//   - Depth is scaled to [0, 65535] and stored Q12.12.
//   - Back-facing triangles (signed area <= 0) are culled here;
//     setup_triangle() returns -1 so the caller skips the ioctl.

#include "geometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ---- screen constants -- must match hardware spec ----

#define SCREEN_W   256
#define SCREEN_H   240
#define FRAC_BITS  12
#define FIXED_ONE  (1 << FRAC_BITS)   // 4096
#define DEPTH_MAX  65535

// ---- vec3 helpers ----

vec3_t vec3_sub(vec3_t a, vec3_t b)
{
    return (vec3_t){ a.x - b.x, a.y - b.y, a.z - b.z };
}

vec3_t vec3_cross(vec3_t a, vec3_t b)
{
    return (vec3_t){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float vec3_dot(vec3_t a, vec3_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3_t vec3_normalize(vec3_t v)
{
    float l = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (l < 1e-8f) return (vec3_t){0, 0, 0};
    return (vec3_t){ v.x/l, v.y/l, v.z/l };
}

// ---- mat4 ----

mat4_t mat4_identity(void)
{
    mat4_t r;
    memset(&r, 0, sizeof(r));
    r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
    return r;
}

mat4_t mat4_mul(const mat4_t *a, const mat4_t *b)
{
    mat4_t r;
    memset(&r, 0, sizeof(r));
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                r.m[i][j] += a->m[i][k] * b->m[k][j];
    return r;
}

vec4_t mat4_mul_vec4(const mat4_t *m, vec4_t v)
{
    return (vec4_t){
        m->m[0][0]*v.x + m->m[0][1]*v.y + m->m[0][2]*v.z + m->m[0][3]*v.w,
        m->m[1][0]*v.x + m->m[1][1]*v.y + m->m[1][2]*v.z + m->m[1][3]*v.w,
        m->m[2][0]*v.x + m->m[2][1]*v.y + m->m[2][2]*v.z + m->m[2][3]*v.w,
        m->m[3][0]*v.x + m->m[3][1]*v.y + m->m[3][2]*v.z + m->m[3][3]*v.w
    };
}

// ---- transform constructors ----

mat4_t rotation_x(float deg)
{
    float r = deg * (float)M_PI / 180.0f;
    mat4_t m = mat4_identity();
    m.m[1][1] =  cosf(r); m.m[1][2] = -sinf(r);
    m.m[2][1] =  sinf(r); m.m[2][2] =  cosf(r);
    return m;
}

mat4_t rotation_y(float deg)
{
    float r = deg * (float)M_PI / 180.0f;
    mat4_t m = mat4_identity();
    m.m[0][0] =  cosf(r); m.m[0][2] = sinf(r);
    m.m[2][0] = -sinf(r); m.m[2][2] = cosf(r);
    return m;
}

mat4_t rotation_z(float deg)
{
    float r = deg * (float)M_PI / 180.0f;
    mat4_t m = mat4_identity();
    m.m[0][0] =  cosf(r); m.m[0][1] = -sinf(r);
    m.m[1][0] =  sinf(r); m.m[1][1] =  cosf(r);
    return m;
}

mat4_t translation(float tx, float ty, float tz)
{
    mat4_t m = mat4_identity();
    m.m[0][3] = tx; m.m[1][3] = ty; m.m[2][3] = tz;
    return m;
}

mat4_t perspective(float fov_deg, float aspect, float near, float far)
{
    float f = 1.0f / tanf(fov_deg * (float)M_PI / 360.0f);
    mat4_t m;
    memset(&m, 0, sizeof(m));
    m.m[0][0] = f / aspect;
    m.m[1][1] = f;
    m.m[2][2] = (far + near) / (near - far);
    m.m[2][3] = (2.0f * far * near) / (near - far);
    m.m[3][2] = -1.0f;
    return m;
}

// ---- lighting ----
//
// Flat Phong shading: N dot L with a 0.25 ambient floor, clamped to
// [0,1] and scaled by 1.2 to boost mid-tones.  Result quantised to
// RGB332 (3R 3G 2B) matching the hardware framebuffer format exactly.

static uint8_t float_to_rgb332(float r, float g, float b)
{
    int ri = (int)(r * 7.0f + 0.5f);
    int gi = (int)(g * 7.0f + 0.5f);
    int bi = (int)(b * 3.0f + 0.5f);
    if (ri < 0) ri = 0; if (ri > 7) ri = 7;
    if (gi < 0) gi = 0; if (gi > 7) gi = 7;
    if (bi < 0) bi = 0; if (bi > 3) bi = 3;
    return (uint8_t)((ri << 5) | (gi << 2) | bi);
}

uint8_t shade_face(vec3_t normal, vec3_t light_dir,
                   float base_r, float base_g, float base_b)
{
    float ndotl    = vec3_dot(normal, light_dir);
    if (ndotl < 0.0f) ndotl = 0.0f;
    float intensity = 0.25f + 0.75f * ndotl;
    intensity *= 1.2f;
    if (intensity > 1.0f) intensity = 1.0f;
    return float_to_rgb332(base_r * intensity,
                           base_g * intensity,
                           base_b * intensity);
}

// ---- fixed-point conversion ----
//
// Rounds to nearest Q12.12: multiply by 4096, round half-up in magnitude.

static int32_t to_fixed(float v)
{
    return (int32_t)(v * FIXED_ONE + (v >= 0.0f ? 0.5f : -0.5f));
}

// ---- triangle setup -> triangle_packet_t ----
//
// Computes Pineda edge coefficients and depth interpolation for one
// triangle, then packs the result into the 17-word layout that
// avalon_interface.sv expects in its staging registers.
//
// Word layout (matches avalon_interface.sv stage[0..16]):
//   [0]  a0   [1]  b0   [2]  c0  (c0 accepted by HW but not used)
//   [3]  a1   [4]  b1   [5]  c1
//   [6]  a2   [7]  b2   [8]  c2
//   [9]  e0_init  [10] e1_init  [11] e2_init
//   [12] z_origin [13] z_step_x [14] z_step_y
//   [15] bbox_packed  { ymax[31:24] ymin[23:16] xmax[15:8] xmin[7:0] }
//   [16] flags_color  { ..., front_facing[8], color[7:0] }
//
// bbox_xmin is snapped to a multiple of 16 before computing e?_init.
// Returns 0 on success, -1 if the triangle is back-facing, degenerate,
// or entirely outside the screen.

int setup_triangle(const screen_vertex_t *v0, const screen_vertex_t *v1,
                   const screen_vertex_t *v2, uint8_t color,
                   triangle_packet_t *out)
{
    float a0f = v1->sy - v2->sy, b0f = v2->sx - v1->sx;
    float c0f = v1->sx * v2->sy - v2->sx * v1->sy;

    float a1f = v2->sy - v0->sy, b1f = v0->sx - v2->sx;
    float c1f = v2->sx * v0->sy - v0->sx * v2->sy;

    float a2f = v0->sy - v1->sy, b2f = v1->sx - v0->sx;
    float c2f = v0->sx * v1->sy - v1->sx * v0->sy;

    float area = a0f * v0->sx + b0f * v0->sy + c0f;

    // Reject only truly degenerate triangles.
    if (fabsf(area) <= 1e-6f) return -1;

    // Normalize winding so hardware can always use inside test e0/e1/e2 >= 0.
    if (area < 0.0f) {
        area = -area;

        a0f = -a0f; b0f = -b0f; c0f = -c0f;
        a1f = -a1f; b1f = -b1f; c1f = -c1f;
        a2f = -a2f; b2f = -b2f; c2f = -c2f;
    }

    float xminf = fminf(fminf(v0->sx, v1->sx), v2->sx);
    float yminf = fminf(fminf(v0->sy, v1->sy), v2->sy);
    float xmaxf = fmaxf(fmaxf(v0->sx, v1->sx), v2->sx);
    float ymaxf = fmaxf(fmaxf(v0->sy, v1->sy), v2->sy);

    int bbox_xmin = (int)floorf(xminf);
    int bbox_ymin = (int)floorf(yminf);
    int bbox_xmax = (int)ceilf(xmaxf);
    int bbox_ymax = (int)ceilf(ymaxf);

    if (bbox_xmin < 0) bbox_xmin = 0;
    if (bbox_ymin < 0) bbox_ymin = 0;
    if (bbox_xmax > SCREEN_W - 1) bbox_xmax = SCREEN_W - 1;
    if (bbox_ymax > SCREEN_H - 1) bbox_ymax = SCREEN_H - 1;

    if (bbox_xmin > bbox_xmax || bbox_ymin > bbox_ymax) return -1;

    bbox_xmin = bbox_xmin & ~15;

    float px = (float)bbox_xmin + 0.5f;
    float py = (float)bbox_ymin + 0.5f;

    float e0_initf = a0f * px + b0f * py + c0f;
    float e1_initf = a1f * px + b1f * py + c1f;
    float e2_initf = a2f * px + b2f * py + c2f;

    float zs0 = v0->sz * (float)DEPTH_MAX;
    float zs1 = v1->sz * (float)DEPTH_MAX;
    float zs2 = v2->sz * (float)DEPTH_MAX;

    float z_step_xf = (a0f * zs0 + a1f * zs1 + a2f * zs2) / area;
    float z_step_yf = (b0f * zs0 + b1f * zs1 + b2f * zs2) / area;

    // IMPORTANT: e*_initf are floats, not fixed-point. Do not divide by FIXED_ONE.
    float z_at_originf = (e0_initf * zs0 +
                          e1_initf * zs1 +
                          e2_initf * zs2) / area;

    memset(out, 0, sizeof(*out));

    out->a0 = to_fixed(a0f);
    out->b0 = to_fixed(b0f);
    out->c0 = to_fixed(e0_initf);

    out->a1 = to_fixed(a1f);
    out->b1 = to_fixed(b1f);
    out->c1 = to_fixed(e1_initf);

    out->a2 = to_fixed(a2f);
    out->b2 = to_fixed(b2f);
    out->c2 = to_fixed(e2_initf);

    out->e0_init = to_fixed(e0_initf);
    out->e1_init = to_fixed(e1_initf);
    out->e2_init = to_fixed(e2_initf);

    out->z_origin = to_fixed(z_at_originf);
    out->z_step_x = to_fixed(z_step_xf);
    out->z_step_y = to_fixed(z_step_yf);

    out->bbox_packed =
        ((__u32)(bbox_ymax & 0xFF) << 24) |
        ((__u32)(bbox_ymin & 0xFF) << 16) |
        ((__u32)(bbox_xmax & 0xFF) << 8)  |
        ((__u32)(bbox_xmin & 0xFF));

    out->flags_color = ((__u32)1 << 8) | ((__u32)color & 0xFF);

    return 0;
}
