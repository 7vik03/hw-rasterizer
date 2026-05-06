// model.c
//
// Procedural mesh generators and OBJ loader.
//
// All geometry matches demo.cpp exactly so test vectors from the golden
// reference apply directly to the hardware.  The icosphere subdivision
// uses a midpoint cache to avoid inserting duplicate vertices on shared
// edges -- the same algorithm as the C++ version.

#include "model.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ---- cube ----
//
// 8 vertices, 12 triangles, unit half-size centred at origin.

void model_make_cube(model_t *m)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "cube", sizeof(m->name) - 1);

    static const vec3_t verts[8] = {
        {-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},
        {-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1}
    };
    memcpy(m->verts, verts, sizeof(verts));
    m->num_verts = 8;

    static const face_t faces[12] = {
        {{0,1,2}}, {{0,2,3}},
        {{5,4,7}}, {{5,7,6}},
        {{4,0,3}}, {{4,3,7}},
        {{1,5,6}}, {{1,6,2}},
        {{3,2,6}}, {{3,6,7}},
        {{4,5,1}}, {{4,1,0}}
    };
    memcpy(m->faces, faces, sizeof(faces));
    m->num_faces = 12;
}

// ---- icosphere ----
//
// Starts from a regular icosahedron (20 faces, 12 vertices) and
// subdivides each triangle into 4 by inserting midpoints on every edge.
// Each midpoint is projected back onto the unit sphere so the result
// stays round.  A flat cache of (a, b) -> mid_index pairs avoids
// inserting the same midpoint twice when two triangles share an edge.
//
//   subdivisions=1 -> 80 faces  (sphere_lo)
//   subdivisions=2 -> 320 faces (sphere_med)

#define MIDPT_CACHE_MAX 2048

typedef struct { int a, b, mid; } midpt_entry_t;

static int           g_num_midpt;
static midpt_entry_t g_midpt_cache[MIDPT_CACHE_MAX];

static void midpt_cache_reset(void) { g_num_midpt = 0; }

static int midpt_cache_lookup(int a, int b)
{
    int lo = a < b ? a : b;
    int hi = a < b ? b : a;
    for (int i = 0; i < g_num_midpt; i++)
        if (g_midpt_cache[i].a == lo && g_midpt_cache[i].b == hi)
            return g_midpt_cache[i].mid;
    return -1;
}

static void midpt_cache_insert(int a, int b, int mid)
{
    if (g_num_midpt >= MIDPT_CACHE_MAX) return;
    int lo = a < b ? a : b;
    int hi = a < b ? b : a;
    g_midpt_cache[g_num_midpt++] = (midpt_entry_t){ lo, hi, mid };
}

static int icosphere_midpoint(model_t *m, int a, int b)
{
    int cached = midpt_cache_lookup(a, b);
    if (cached >= 0) return cached;

    if (m->num_verts >= MODEL_MAX_VERTS) return 0;

    vec3_t va  = m->verts[a];
    vec3_t vb  = m->verts[b];
    vec3_t mid = { (va.x+vb.x)*0.5f, (va.y+vb.y)*0.5f, (va.z+vb.z)*0.5f };

    // project onto unit sphere
    float l = sqrtf(mid.x*mid.x + mid.y*mid.y + mid.z*mid.z);
    if (l > 1e-8f) { mid.x /= l; mid.y /= l; mid.z /= l; }

    int idx = m->num_verts;
    m->verts[m->num_verts++] = mid;
    midpt_cache_insert(a, b, idx);
    return idx;
}

void model_make_icosphere(model_t *m, int subdivisions)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "sphere", sizeof(m->name) - 1);

    float t = (1.0f + sqrtf(5.0f)) / 2.0f;

    vec3_t init_v[12] = {
        {-1, t, 0}, { 1, t, 0}, {-1,-t, 0}, { 1,-t, 0},
        { 0,-1, t}, { 0, 1, t}, { 0,-1,-t}, { 0, 1,-t},
        { t, 0,-1}, { t, 0, 1}, {-t, 0,-1}, {-t, 0, 1}
    };
    for (int i = 0; i < 12; i++) {
        float l = sqrtf(init_v[i].x*init_v[i].x +
                        init_v[i].y*init_v[i].y +
                        init_v[i].z*init_v[i].z);
        init_v[i].x /= l; init_v[i].y /= l; init_v[i].z /= l;
    }
    memcpy(m->verts, init_v, sizeof(init_v));
    m->num_verts = 12;

    face_t init_f[20] = {
        {{0,11,5}}, {{0,5,1}},  {{0,1,7}},   {{0,7,10}},  {{0,10,11}},
        {{1,5,9}},  {{5,11,4}}, {{11,10,2}},  {{10,7,6}},  {{7,1,8}},
        {{3,9,4}},  {{3,4,2}},  {{3,2,6}},    {{3,6,8}},   {{3,8,9}},
        {{4,9,5}},  {{2,4,11}}, {{6,2,10}},   {{8,6,7}},   {{9,8,1}}
    };
    memcpy(m->faces, init_f, sizeof(init_f));
    m->num_faces = 20;

    static face_t tmp[MODEL_MAX_FACES];

    for (int s = 0; s < subdivisions; s++) {
        if (m->num_faces * 4 > MODEL_MAX_FACES) break;
        midpt_cache_reset();
        int new_count = 0;

        for (int i = 0; i < m->num_faces; i++) {
            int v0 = m->faces[i].v[0];
            int v1 = m->faces[i].v[1];
            int v2 = m->faces[i].v[2];

            int a = icosphere_midpoint(m, v0, v1);
            int b = icosphere_midpoint(m, v1, v2);
            int c = icosphere_midpoint(m, v2, v0);

            // each original triangle splits into 4
            tmp[new_count++] = (face_t){{ v0, a, c }};
            tmp[new_count++] = (face_t){{ v1, b, a }};
            tmp[new_count++] = (face_t){{ v2, c, b }};
            tmp[new_count++] = (face_t){{  a, b, c }};
        }

        memcpy(m->faces, tmp, new_count * sizeof(face_t));
        m->num_faces = new_count;
    }
}

// ---- torus ----
//
// major_seg rings, each with minor_seg vertices.  Each quad cell is
// split into two triangles.  Default: 12x8 = 192 faces.

void model_make_torus(model_t *m, int major_seg, int minor_seg,
                      float R, float r)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "torus", sizeof(m->name) - 1);

    for (int i = 0; i < major_seg; i++) {
        float theta = 2.0f * (float)M_PI * i / major_seg;
        for (int j = 0; j < minor_seg; j++) {
            float phi = 2.0f * (float)M_PI * j / minor_seg;
            if (m->num_verts >= MODEL_MAX_VERTS) break;
            m->verts[m->num_verts++] = (vec3_t){
                (R + r * cosf(phi)) * cosf(theta),
                r * sinf(phi),
                (R + r * cosf(phi)) * sinf(theta)
            };
        }
    }

    for (int i = 0; i < major_seg; i++) {
        int ni = (i + 1) % major_seg;
        for (int j = 0; j < minor_seg; j++) {
            int nj = (j + 1) % minor_seg;
            int a = i  * minor_seg + j;
            int b = ni * minor_seg + j;
            int c = ni * minor_seg + nj;
            int d = i  * minor_seg + nj;
            if (m->num_faces + 1 >= MODEL_MAX_FACES) break;
            m->faces[m->num_faces++] = (face_t){{ a, b, c }};
            m->faces[m->num_faces++] = (face_t){{ a, c, d }};
        }
    }
}

// ---- OBJ loader ----
//
// Handles "v" (vertex) and "f" (face) lines.  Face tokens may be
// "v", "v/t", or "v/t/n" -- only the vertex index is used.  Quads
// (4 tokens on an f line) are split into two triangles: (0,1,2) and
// (0,2,3).  Vertex indices from the file are 1-based; we subtract 1.
//
// All vertex indices are bounds-checked against the vertices seen so
// far; faces referencing out-of-range indices are silently skipped so
// a malformed file cannot cause an out-of-bounds write.

int model_load_obj(model_t *m, const char *filename)
{
    FILE *f = fopen(filename, "r");
    if (!f) return -1;

    memset(m, 0, sizeof(*m));
    strncpy(m->name, filename, sizeof(m->name) - 1);

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char prefix[8];
        if (sscanf(line, "%7s", prefix) != 1) continue;

        if (strcmp(prefix, "v") == 0) {
            if (m->num_verts >= MODEL_MAX_VERTS) continue;
            vec3_t v;
            if (sscanf(line + 1, "%f %f %f", &v.x, &v.y, &v.z) == 3)
                m->verts[m->num_verts++] = v;

        } else if (strcmp(prefix, "f") == 0) {
            int idx[4] = {0, 0, 0, 0};
            int n = 0;
            const char *p = line + 1;

            while (n < 4) {
                while (*p == ' ' || *p == '\t') p++;
                if (*p == '\0' || *p == '\n' || *p == '\r') break;
                int v_idx = 0;
                if (sscanf(p, "%d", &v_idx) != 1) break;
                idx[n++] = v_idx - 1;  // OBJ indices are 1-based
                while (*p && *p != ' ' && *p != '\t' && *p != '\n') p++;
            }

            // bounds-check all indices; skip face if any are invalid
            int valid = 1;
            for (int vi = 0; vi < n; vi++) {
                if (idx[vi] < 0 || idx[vi] >= m->num_verts) {
                    valid = 0; break;
                }
            }
            if (!valid) continue;

            if (n >= 3 && m->num_faces < MODEL_MAX_FACES)
                m->faces[m->num_faces++] = (face_t){{ idx[0], idx[1], idx[2] }};
            if (n == 4 && m->num_faces < MODEL_MAX_FACES)
                m->faces[m->num_faces++] = (face_t){{ idx[0], idx[2], idx[3] }};
        }
    }

    fclose(f);
    return (m->num_verts > 0 && m->num_faces > 0) ? 0 : -1;
}

// ---- Utah teapot ----
//
// The classic 32-patch bicubic Bezier teapot (Newell, 1975).
// Each patch is tessellated at TEAPOT_DIV x TEAPOT_DIV quads and
// each quad split into 2 triangles.  At DIV=4: 32*4*4*2 = 1024 faces.
// Vertices shared between patches may be duplicated; the face count
// is the hard constraint (MODEL_MAX_FACES=2048).
//
// Control point indices reference the 306 canonical teapot vertices.
// The vertex table and patch index table are the standard Newell data.

#define TEAPOT_DIV 4   // subdivision steps per patch edge

// 306 control point positions (x,y,z), Newell's original coordinates.
// Y is up; teapot sits near y=0 and extends to y~3.15.
static const float teapot_cp[306][3] = {
    {1.4f,2.4f,0.0f},{1.4f,2.4f,-0.784f},{0.784f,2.4f,-1.4f},
    {0.0f,2.4f,-1.4f},{1.3375f,2.53125f,0.0f},{1.3375f,2.53125f,-0.749f},
    {0.749f,2.53125f,-1.3375f},{0.0f,2.53125f,-1.3375f},{1.4375f,2.53125f,0.0f},
    {1.4375f,2.53125f,-0.805f},{0.805f,2.53125f,-1.4375f},{0.0f,2.53125f,-1.4375f},
    {1.5f,2.4f,0.0f},{1.5f,2.4f,-0.84f},{0.84f,2.4f,-1.5f},{0.0f,2.4f,-1.5f},
    {-0.784f,2.4f,-1.4f},{-1.4f,2.4f,-0.784f},{-1.4f,2.4f,0.0f},
    {-0.749f,2.53125f,-1.3375f},{-1.3375f,2.53125f,-0.749f},{-1.3375f,2.53125f,0.0f},
    {-0.805f,2.53125f,-1.4375f},{-1.4375f,2.53125f,-0.805f},{-1.4375f,2.53125f,0.0f},
    {-0.84f,2.4f,-1.5f},{-1.5f,2.4f,-0.84f},{-1.5f,2.4f,0.0f},
    {-1.4f,2.4f,0.784f},{-0.784f,2.4f,1.4f},{0.0f,2.4f,1.4f},
    {-1.3375f,2.53125f,0.749f},{-0.749f,2.53125f,1.3375f},{0.0f,2.53125f,1.3375f},
    {-1.4375f,2.53125f,0.805f},{-0.805f,2.53125f,1.4375f},{0.0f,2.53125f,1.4375f},
    {-1.5f,2.4f,0.84f},{-0.84f,2.4f,1.5f},{0.0f,2.4f,1.5f},
    {0.784f,2.4f,1.4f},{1.4f,2.4f,0.784f},{0.749f,2.53125f,1.3375f},
    {1.3375f,2.53125f,0.749f},{0.805f,2.53125f,1.4375f},{1.4375f,2.53125f,0.805f},
    {0.84f,2.4f,1.5f},{1.5f,2.4f,0.84f},{1.5f,2.25f,0.0f},
    {1.5f,2.25f,-0.84f},{0.84f,2.25f,-1.5f},{0.0f,2.25f,-1.5f},
    {1.75f,1.725f,0.0f},{1.75f,1.725f,-0.98f},{0.98f,1.725f,-1.75f},
    {0.0f,1.725f,-1.75f},{2.0f,1.2f,0.0f},{2.0f,1.2f,-1.12f},
    {1.12f,1.2f,-2.0f},{0.0f,1.2f,-2.0f},{-0.84f,2.25f,-1.5f},
    {-1.5f,2.25f,-0.84f},{-1.5f,2.25f,0.0f},{-0.98f,1.725f,-1.75f},
    {-1.75f,1.725f,-0.98f},{-1.75f,1.725f,0.0f},{-1.12f,1.2f,-2.0f},
    {-2.0f,1.2f,-1.12f},{-2.0f,1.2f,0.0f},{-1.5f,2.25f,0.84f},
    {-0.84f,2.25f,1.5f},{0.0f,2.25f,1.5f},{-1.75f,1.725f,0.98f},
    {-0.98f,1.725f,1.75f},{0.0f,1.725f,1.75f},{-2.0f,1.2f,1.12f},
    {-1.12f,1.2f,2.0f},{0.0f,1.2f,2.0f},{0.84f,2.25f,1.5f},
    {1.5f,2.25f,0.84f},{0.98f,1.725f,1.75f},{1.75f,1.725f,0.98f},
    {1.12f,1.2f,2.0f},{2.0f,1.2f,1.12f},{2.0f,0.9f,0.0f},
    {2.0f,0.9f,-1.12f},{1.12f,0.9f,-2.0f},{0.0f,0.9f,-2.0f},
    {2.0f,0.45f,0.0f},{2.0f,0.45f,-1.12f},{1.12f,0.45f,-2.0f},
    {0.0f,0.45f,-2.0f},{1.5f,0.225f,0.0f},{1.5f,0.225f,-0.84f},
    {0.84f,0.225f,-1.5f},{0.0f,0.225f,-1.5f},{1.5f,0.15f,0.0f},
    {1.5f,0.15f,-0.84f},{0.84f,0.15f,-1.5f},{0.0f,0.15f,-1.5f},
    {-1.12f,0.9f,-2.0f},{-2.0f,0.9f,-1.12f},{-2.0f,0.9f,0.0f},
    {-1.12f,0.45f,-2.0f},{-2.0f,0.45f,-1.12f},{-2.0f,0.45f,0.0f},
    {-0.84f,0.225f,-1.5f},{-1.5f,0.225f,-0.84f},{-1.5f,0.225f,0.0f},
    {-0.84f,0.15f,-1.5f},{-1.5f,0.15f,-0.84f},{-1.5f,0.15f,0.0f},
    {-2.0f,0.9f,1.12f},{-1.12f,0.9f,2.0f},{0.0f,0.9f,2.0f},
    {-2.0f,0.45f,1.12f},{-1.12f,0.45f,2.0f},{0.0f,0.45f,2.0f},
    {-1.5f,0.225f,0.84f},{-0.84f,0.225f,1.5f},{0.0f,0.225f,1.5f},
    {-1.5f,0.15f,0.84f},{-0.84f,0.15f,1.5f},{0.0f,0.15f,1.5f},
    {1.12f,0.9f,2.0f},{2.0f,0.9f,1.12f},{1.12f,0.45f,2.0f},
    {2.0f,0.45f,1.12f},{0.84f,0.225f,1.5f},{1.5f,0.225f,0.84f},
    {0.84f,0.15f,1.5f},{1.5f,0.15f,0.84f},{-1.6f,0.0f,-1.5f},
    {-1.5f,0.15f,-1.5f},{-2.5f,0.0f,-1.5f},{-2.5f,0.15f,-1.5f},
    {-2.5f,0.0f,-1.0f},{-2.5f,0.15f,-1.0f},{-2.5f,0.0f,0.0f},
    {-2.5f,0.15f,0.0f},{-2.5f,0.0f,1.0f},{-2.5f,0.15f,1.0f},
    {-2.5f,0.0f,1.5f},{-2.5f,0.15f,1.5f},{-1.5f,0.15f,1.5f},
    {-1.6f,0.0f,1.5f},{-1.6f,0.0f,0.0f},{-1.6f,0.15f,1.5f},
    {-1.6f,0.0f,-1.5f},{-1.6f,0.15f,-1.5f},{-1.6f,0.0f,0.0f},
    {-1.6f,0.15f,0.0f},{-2.5f,0.0f,0.0f},{-2.5f,0.15f,0.0f},
    {-2.5f,0.0f,-1.0f},{-2.5f,0.15f,-1.0f},{-1.6f,0.0f,-1.5f},
    {-1.6f,0.15f,-1.5f},{-2.5f,0.0f,-1.5f},{-2.5f,0.15f,-1.5f},
    {-2.5f,0.0f,1.0f},{-2.5f,0.15f,1.0f},{-1.6f,0.0f,1.5f},
    {-1.6f,0.15f,1.5f},{-2.5f,0.0f,1.5f},{-2.5f,0.15f,1.5f},
    {-2.5f,0.0f,0.0f},{-2.5f,0.15f,0.0f},{1.7f,1.425f,0.0f},
    {1.7f,1.425f,-0.952f},{0.952f,1.425f,-1.7f},{0.0f,1.425f,-1.7f},
    {1.7f,0.6f,0.0f},{1.7f,0.6f,-0.952f},{0.952f,0.6f,-1.7f},
    {0.0f,0.6f,-1.7f},{-0.952f,1.425f,-1.7f},{-1.7f,1.425f,-0.952f},
    {-1.7f,1.425f,0.0f},{-0.952f,0.6f,-1.7f},{-1.7f,0.6f,-0.952f},
    {-1.7f,0.6f,0.0f},{-1.7f,1.425f,0.952f},{-0.952f,1.425f,1.7f},
    {0.0f,1.425f,1.7f},{-1.7f,0.6f,0.952f},{-0.952f,0.6f,1.7f},
    {0.0f,0.6f,1.7f},{0.952f,1.425f,1.7f},{1.7f,1.425f,0.952f},
    {0.952f,0.6f,1.7f},{1.7f,0.6f,0.952f},{0.0f,0.0f,0.0f},
    {1.425f,0.0f,0.0f},{1.425f,0.0f,-0.798f},{0.798f,0.0f,-1.425f},
    {0.0f,0.0f,-1.425f},{0.0f,0.0f,0.0f},{0.0f,3.15f,0.0f},
    {0.8f,3.15f,0.0f},{0.8f,3.15f,-0.45f},{0.45f,3.15f,-0.8f},
    {0.0f,3.15f,-0.8f},{0.0f,2.85f,0.0f},{0.2f,2.7f,0.0f},
    {0.2f,2.7f,-0.112f},{0.112f,2.7f,-0.2f},{0.0f,2.7f,-0.2f},
    {0.4f,2.55f,0.0f},{0.4f,2.55f,-0.224f},{0.224f,2.55f,-0.4f},
    {0.0f,2.55f,-0.4f},{1.3f,2.55f,0.0f},{1.3f,2.55f,-0.728f},
    {0.728f,2.55f,-1.3f},{0.0f,2.55f,-1.3f},{1.3f,2.4f,0.0f},
    {1.3f,2.4f,-0.728f},{0.728f,2.4f,-1.3f},{0.0f,2.4f,-1.3f},
    {-0.45f,3.15f,-0.8f},{-0.8f,3.15f,-0.45f},{-0.8f,3.15f,0.0f},
    {-0.112f,2.7f,-0.2f},{-0.2f,2.7f,-0.112f},{-0.2f,2.7f,0.0f},
    {-0.224f,2.55f,-0.4f},{-0.4f,2.55f,-0.224f},{-0.4f,2.55f,0.0f},
    {-0.728f,2.55f,-1.3f},{-1.3f,2.55f,-0.728f},{-1.3f,2.55f,0.0f},
    {-0.728f,2.4f,-1.3f},{-1.3f,2.4f,-0.728f},{-1.3f,2.4f,0.0f},
    {-0.8f,3.15f,0.45f},{-0.45f,3.15f,0.8f},{0.0f,3.15f,0.8f},
    {-0.2f,2.7f,0.112f},{-0.112f,2.7f,0.2f},{0.0f,2.7f,0.2f},
    {-0.4f,2.55f,0.224f},{-0.224f,2.55f,0.4f},{0.0f,2.55f,0.4f},
    {-1.3f,2.55f,0.728f},{-0.728f,2.55f,1.3f},{0.0f,2.55f,1.3f},
    {-1.3f,2.4f,0.728f},{-0.728f,2.4f,1.3f},{0.0f,2.4f,1.3f},
    {0.45f,3.15f,0.8f},{0.8f,3.15f,0.45f},{0.112f,2.7f,0.2f},
    {0.2f,2.7f,0.112f},{0.224f,2.55f,0.4f},{0.4f,2.55f,0.224f},
    {0.728f,2.55f,1.3f},{1.3f,2.55f,0.728f},{0.728f,2.4f,1.3f},
    {1.3f,2.4f,0.728f},{0.0f,3.15f,0.0f},{0.8f,3.15f,0.0f},
    {0.0f,0.0f,0.0f},{1.425f,0.0f,0.798f},{0.798f,0.0f,1.425f},
    {0.0f,0.0f,1.425f},{-0.798f,0.0f,1.425f},{-1.425f,0.0f,0.798f},
    {-1.425f,0.0f,0.0f},{-0.798f,0.0f,-1.425f},{0.0f,0.0f,-1.425f}
};

// 32 patches, each referencing 16 control point indices (1-based in Newell).
// Converted here to 0-based.
static const int teapot_patches[32][16] = {
    {  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16},
    {  4,  3, 17, 18,  8,  7, 19, 20, 12, 11, 21, 22, 16, 15, 23, 24},
    { 18, 17,  3,  4, 20, 19,  7,  8, 22, 21, 11, 12, 24, 23, 15, 16},
    {  4,  3,  2,  1,  8,  7,  6,  5, 12, 11, 10,  9, 16, 15, 14, 13},
    { 13, 14, 15, 16, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36},
    { 16, 15, 37, 38, 28, 27, 39, 40, 32, 31, 41, 42, 36, 35, 43, 44},
    { 38, 37, 15, 16, 40, 39, 27, 28, 42, 41, 31, 32, 44, 43, 35, 36},
    { 16, 15, 14, 13, 36, 35, 34, 33, 32, 31, 30, 29, 36, 35, 34, 33},
    { 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64},
    { 52, 51, 65, 66, 56, 55, 67, 68, 60, 59, 69, 70, 64, 63, 71, 72},
    { 66, 65, 51, 52, 68, 67, 55, 56, 70, 69, 59, 60, 72, 71, 63, 64},
    { 64, 63, 62, 61, 64, 63, 62, 61, 64, 63, 62, 61, 64, 63, 62, 61},
    { 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88},
    { 76, 75, 89, 90, 80, 79, 91, 92, 84, 83, 93, 94, 88, 87, 95, 96},
    { 90, 89, 75, 76, 92, 91, 79, 80, 94, 93, 83, 84, 96, 95, 87, 88},
    { 88, 87, 86, 85, 92, 91, 90, 89, 96, 95, 94, 93, 88, 87, 86, 85},
    { 97, 98, 99,100,101,102,103,104,105,106,107,108,109,110,111,112},
    {100, 99,113,114,104,103,115,116,108,107,117,118,112,111,119,120},
    {114,113, 99,100,116,115,103,104,118,117,107,108,120,119,111,112},
    {112,111,110,109,112,111,110,109,112,111,110,109,112,111,110,109},
    {121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136},
    {124,123,137,138,128,127,139,140,132,131,141,142,136,135,143,144},
    {138,137,123,124,140,139,127,128,142,141,131,132,144,143,135,136},
    {136,135,134,133,136,135,134,133,136,135,134,133,136,135,134,133},
    {145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160},
    {148,147,161,162,152,151,163,164,156,155,165,166,160,159,167,168},
    {162,161,147,148,164,163,151,152,166,165,155,156,168,167,159,160},
    {160,159,158,157,160,159,158,157,160,159,158,157,160,159,158,157},
    {169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184},
    {172,171,185,186,176,175,187,188,180,179,189,190,184,183,191,192},
    {186,185,171,172,188,187,175,176,190,189,179,180,192,191,183,184},
    {184,183,182,181,184,183,182,181,184,183,182,181,184,183,182,181}
};

// evaluate a cubic Bezier curve at parameter t
static float bezier1(float p0, float p1, float p2, float p3, float t)
{
    float mt = 1.0f - t;
    return mt*mt*mt*p0 + 3.0f*mt*mt*t*p1 + 3.0f*mt*t*t*p2 + t*t*t*p3;
}

// evaluate a 4x4 bicubic Bezier patch at (u,v)
static vec3_t bezier_patch(const float cp[16][3], float u, float v)
{
    float tmp[4][3];
    for (int i = 0; i < 4; i++) {
        for (int c = 0; c < 3; c++)
            tmp[i][c] = bezier1(cp[i*4+0][c], cp[i*4+1][c],
                                cp[i*4+2][c], cp[i*4+3][c], v);
    }
    return (vec3_t){
        bezier1(tmp[0][0], tmp[1][0], tmp[2][0], tmp[3][0], u),
        bezier1(tmp[0][1], tmp[1][1], tmp[2][1], tmp[3][1], u),
        bezier1(tmp[0][2], tmp[1][2], tmp[2][2], tmp[3][2], u)
    };
}

void model_make_teapot(model_t *m)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "teapot", sizeof(m->name) - 1);

    int div = TEAPOT_DIV;

    for (int p = 0; p < 32; p++) {
        // gather the 16 control points for this patch
        float cp[16][3];
        for (int k = 0; k < 16; k++) {
            int idx = teapot_patches[p][k] - 1;  // convert 1-based to 0-based
            cp[k][0] = teapot_cp[idx][0];
            cp[k][1] = teapot_cp[idx][1];
            cp[k][2] = teapot_cp[idx][2];
        }

        // tessellate into div x div quads
        for (int i = 0; i < div; i++) {
            float u0 = (float)i       / div;
            float u1 = (float)(i + 1) / div;
            for (int j = 0; j < div; j++) {
                float v0 = (float)j       / div;
                float v1 = (float)(j + 1) / div;

                if (m->num_verts + 4 > MODEL_MAX_VERTS) goto done;
                if (m->num_faces + 2 > MODEL_MAX_FACES) goto done;

                vec3_t q00 = bezier_patch(cp, u0, v0);
                vec3_t q10 = bezier_patch(cp, u1, v0);
                vec3_t q01 = bezier_patch(cp, u0, v1);
                vec3_t q11 = bezier_patch(cp, u1, v1);

                // scale down: teapot coords go up to ~2.5; fit in unit cube
                float scale = 0.4f;
                float oy    = -1.2f;  // centre vertically
                q00.x *= scale; q00.y = q00.y * scale + oy; q00.z *= scale;
                q10.x *= scale; q10.y = q10.y * scale + oy; q10.z *= scale;
                q01.x *= scale; q01.y = q01.y * scale + oy; q01.z *= scale;
                q11.x *= scale; q11.y = q11.y * scale + oy; q11.z *= scale;

                int base = m->num_verts;
                m->verts[m->num_verts++] = q00;
                m->verts[m->num_verts++] = q10;
                m->verts[m->num_verts++] = q11;
                m->verts[m->num_verts++] = q01;

                m->faces[m->num_faces++] = (face_t){{ base+0, base+1, base+2 }};
                m->faces[m->num_faces++] = (face_t){{ base+0, base+2, base+3 }};
            }
        }
    }
done:;
}

// ---- diamond gem ----
//
// Classic brilliant-cut diamond: flat table on top, crown of trapezoidal
// facets, a sharp culet at the bottom.  N segments around the girdle.
// At N=16: 16 crown + 16 upper-girdle + 16 lower-girdle + 16 pavilion
//          + 1 table fan (16 tris) = 80 faces total.

void model_make_diamond(model_t *m)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "diamond", sizeof(m->name) - 1);

    int   N        = 16;
    float table_r  = 0.45f;   // radius of flat top face
    float girdle_r = 1.0f;    // widest point
    float table_y  =  0.55f;  // y of table
    float girdle_y =  0.0f;   // y of girdle
    float culet_y  = -1.0f;   // y of bottom point
    float crown_y  =  0.75f;  // not used separately; table_y is crown top

    // vertices:
    //   0          : table centre (top cap)
    //   1..N       : table ring (inner top)
    //   N+1..2N    : girdle ring
    //   2N+1       : culet (bottom point)

    // table centre
    m->verts[m->num_verts++] = (vec3_t){ 0.0f, table_y + 0.05f, 0.0f };

    // table ring
    int table_base = m->num_verts;
    for (int i = 0; i < N; i++) {
        float a = 2.0f * (float)M_PI * i / N;
        m->verts[m->num_verts++] = (vec3_t){
            table_r * cosf(a), table_y, table_r * sinf(a)
        };
    }

    // girdle ring
    int girdle_base = m->num_verts;
    for (int i = 0; i < N; i++) {
        float a = 2.0f * (float)M_PI * i / N;
        m->verts[m->num_verts++] = (vec3_t){
            girdle_r * cosf(a), girdle_y, girdle_r * sinf(a)
        };
    }

    // culet
    int culet_idx = m->num_verts;
    m->verts[m->num_verts++] = (vec3_t){ 0.0f, culet_y, 0.0f };

    // table fan (flat top)
    for (int i = 0; i < N; i++) {
        int a = table_base + i;
        int b = table_base + (i + 1) % N;
        m->faces[m->num_faces++] = (face_t){{ 0, b, a }};
    }

    // crown: table ring -> girdle ring
    for (int i = 0; i < N; i++) {
        int t0 = table_base  + i;
        int t1 = table_base  + (i + 1) % N;
        int g0 = girdle_base + i;
        int g1 = girdle_base + (i + 1) % N;
        m->faces[m->num_faces++] = (face_t){{ t0, g0, t1 }};
        m->faces[m->num_faces++] = (face_t){{ t1, g0, g1 }};
    }

    // pavilion: girdle ring -> culet
    for (int i = 0; i < N; i++) {
        int g0 = girdle_base + i;
        int g1 = girdle_base + (i + 1) % N;
        m->faces[m->num_faces++] = (face_t){{ g0, culet_idx, g1 }};
    }
}

// ---- Lego brick (2x4 stud) ----
//
// A rectangular box body with 8 cylindrical studs on top.
// Body is a simple box; each stud is a cylinder with SEG sides.
// SEG=8 studs=8: 12 (box) + 8*(SEG*2 + SEG) (side+top+bottom cap) faces
// At SEG=8: 12 + 8*24 = 204 faces, well within budget.

static void add_box(model_t *m,
                    float x0, float y0, float z0,
                    float x1, float y1, float z1)
{
    int b = m->num_verts;
    // 8 corners: bottom face then top face
    m->verts[m->num_verts++] = (vec3_t){ x0, y0, z0 };  // b+0
    m->verts[m->num_verts++] = (vec3_t){ x1, y0, z0 };  // b+1
    m->verts[m->num_verts++] = (vec3_t){ x1, y0, z1 };  // b+2
    m->verts[m->num_verts++] = (vec3_t){ x0, y0, z1 };  // b+3
    m->verts[m->num_verts++] = (vec3_t){ x0, y1, z0 };  // b+4
    m->verts[m->num_verts++] = (vec3_t){ x1, y1, z0 };  // b+5
    m->verts[m->num_verts++] = (vec3_t){ x1, y1, z1 };  // b+6
    m->verts[m->num_verts++] = (vec3_t){ x0, y1, z1 };  // b+7

    // bottom
    m->faces[m->num_faces++] = (face_t){{ b+0, b+2, b+1 }};
    m->faces[m->num_faces++] = (face_t){{ b+0, b+3, b+2 }};
    // top
    m->faces[m->num_faces++] = (face_t){{ b+4, b+5, b+6 }};
    m->faces[m->num_faces++] = (face_t){{ b+4, b+6, b+7 }};
    // front (z0)
    m->faces[m->num_faces++] = (face_t){{ b+0, b+1, b+5 }};
    m->faces[m->num_faces++] = (face_t){{ b+0, b+5, b+4 }};
    // back (z1)
    m->faces[m->num_faces++] = (face_t){{ b+2, b+3, b+7 }};
    m->faces[m->num_faces++] = (face_t){{ b+2, b+7, b+6 }};
    // left (x0)
    m->faces[m->num_faces++] = (face_t){{ b+3, b+0, b+4 }};
    m->faces[m->num_faces++] = (face_t){{ b+3, b+4, b+7 }};
    // right (x1)
    m->faces[m->num_faces++] = (face_t){{ b+1, b+2, b+6 }};
    m->faces[m->num_faces++] = (face_t){{ b+1, b+6, b+5 }};
}

static void add_cylinder(model_t *m, float cx, float cy_bot, float cy_top,
                          float cz, float r, int seg)
{
    int bot_centre = m->num_verts;
    m->verts[m->num_verts++] = (vec3_t){ cx, cy_bot, cz };
    int top_centre = m->num_verts;
    m->verts[m->num_verts++] = (vec3_t){ cx, cy_top, cz };

    int ring_bot = m->num_verts;
    for (int i = 0; i < seg; i++) {
        float a = 2.0f * (float)M_PI * i / seg;
        m->verts[m->num_verts++] = (vec3_t){ cx + r*cosf(a), cy_bot, cz + r*sinf(a) };
    }
    int ring_top = m->num_verts;
    for (int i = 0; i < seg; i++) {
        float a = 2.0f * (float)M_PI * i / seg;
        m->verts[m->num_verts++] = (vec3_t){ cx + r*cosf(a), cy_top, cz + r*sinf(a) };
    }

    for (int i = 0; i < seg; i++) {
        int ni = (i + 1) % seg;
        // bottom cap
        m->faces[m->num_faces++] = (face_t){{ bot_centre, ring_bot+i, ring_bot+ni }};
        // top cap
        m->faces[m->num_faces++] = (face_t){{ top_centre, ring_top+ni, ring_top+i }};
        // side
        m->faces[m->num_faces++] = (face_t){{ ring_bot+i, ring_top+i,  ring_top+ni }};
        m->faces[m->num_faces++] = (face_t){{ ring_bot+i, ring_top+ni, ring_bot+ni }};
    }
}

void model_make_lego(model_t *m)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "lego", sizeof(m->name) - 1);

    // brick body: 4 studs wide (x), 2 studs deep (z), standard height
    float bx = 1.6f, by = 0.6f, bz = 0.8f;
    add_box(m, -bx, -by, -bz, bx, by, bz);

    // 8 studs in a 4x2 grid on top
    int   seg     = 8;
    float stud_r  = 0.22f;
    float stud_h  = 0.22f;
    float stud_y0 = by;
    float stud_y1 = by + stud_h;

    float xs[4] = { -1.2f, -0.4f, 0.4f, 1.2f };
    float zs[2] = { -0.4f,  0.4f };
    for (int zi = 0; zi < 2; zi++)
        for (int xi = 0; xi < 4; xi++)
            add_cylinder(m, xs[xi], stud_y0, stud_y1, zs[zi], stud_r, seg);
}

// ---- extruded star ----
//
// 5-pointed star profile extruded along Y.  Each arm of the star has
// an outer tip and an inner notch; profile has 10 vertices.
// Front face fan + back face fan + side quads = 8+8+20*2 = 56 faces.

void model_make_star(model_t *m)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "star", sizeof(m->name) - 1);

    int   N        = 5;
    float r_outer  = 1.0f;
    float r_inner  = 0.42f;
    float half_h   = 0.25f;   // half-thickness of extrusion

    // 10 profile points (alternating outer tip / inner notch)
    // front ring (y = +half_h), back ring (y = -half_h)
    int front_base = 0;
    for (int i = 0; i < 2*N; i++) {
        float a = (float)M_PI / N * i - (float)M_PI / 2.0f;
        float r = (i % 2 == 0) ? r_outer : r_inner;
        m->verts[m->num_verts++] = (vec3_t){ r*cosf(a),  half_h, r*sinf(a) };
    }
    int back_base = m->num_verts;
    for (int i = 0; i < 2*N; i++) {
        float a = (float)M_PI / N * i - (float)M_PI / 2.0f;
        float r = (i % 2 == 0) ? r_outer : r_inner;
        m->verts[m->num_verts++] = (vec3_t){ r*cosf(a), -half_h, r*sinf(a) };
    }
    // front centre, back centre
    int fc = m->num_verts;
    m->verts[m->num_verts++] = (vec3_t){ 0.0f,  half_h, 0.0f };
    int bc = m->num_verts;
    m->verts[m->num_verts++] = (vec3_t){ 0.0f, -half_h, 0.0f };

    // front and back fans
    for (int i = 0; i < 2*N; i++) {
        int ni = (i + 1) % (2*N);
        m->faces[m->num_faces++] = (face_t){{ fc, front_base+i, front_base+ni }};
        m->faces[m->num_faces++] = (face_t){{ bc, back_base+ni, back_base+i  }};
    }
    // side quads (each edge of the profile)
    for (int i = 0; i < 2*N; i++) {
        int ni = (i + 1) % (2*N);
        int f0 = front_base + i,  f1 = front_base + ni;
        int b0 = back_base  + i,  b1 = back_base  + ni;
        m->faces[m->num_faces++] = (face_t){{ f0, b0, f1 }};
        m->faces[m->num_faces++] = (face_t){{ f1, b0, b1 }};
    }
}

// ---- rocket ----
//
// Nose cone (hemisphere, SEG=10 latitudes) + cylindrical body + 4 fins.
// Nose: ~100 faces, body: ~40 faces, fins: 4*2 = 8 faces. Total ~148.

void model_make_rocket(model_t *m)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "rocket", sizeof(m->name) - 1);

    int   seg      = 10;
    float body_r   = 0.3f;
    float body_bot = -1.0f;
    float body_top =  0.4f;
    float nose_top =  1.2f;

    // ---- cylindrical body ----
    int ring_bot = m->num_verts;
    for (int i = 0; i < seg; i++) {
        float a = 2.0f * (float)M_PI * i / seg;
        m->verts[m->num_verts++] = (vec3_t){ body_r*cosf(a), body_bot, body_r*sinf(a) };
    }
    int ring_top = m->num_verts;
    for (int i = 0; i < seg; i++) {
        float a = 2.0f * (float)M_PI * i / seg;
        m->verts[m->num_verts++] = (vec3_t){ body_r*cosf(a), body_top, body_r*sinf(a) };
    }
    // bottom cap centre
    int bot_c = m->num_verts;
    m->verts[m->num_verts++] = (vec3_t){ 0.0f, body_bot, 0.0f };

    for (int i = 0; i < seg; i++) {
        int ni = (i + 1) % seg;
        // body side
        m->faces[m->num_faces++] = (face_t){{ ring_bot+i, ring_top+i,  ring_top+ni }};
        m->faces[m->num_faces++] = (face_t){{ ring_bot+i, ring_top+ni, ring_bot+ni }};
        // bottom cap
        m->faces[m->num_faces++] = (face_t){{ bot_c, ring_bot+ni, ring_bot+i }};
    }

    // ---- hemispherical nose cone ----
    // latitude rings from body_top up to nose_top
    int lat       = 6;
    float nose_h  = nose_top - body_top;
    int prev_ring = ring_top;

    for (int l = 1; l <= lat; l++) {
        float t = (float)l / lat;
        // parametric hemisphere: r shrinks, y rises
        float y = body_top + nose_h * (1.0f - cosf(t * (float)M_PI / 2.0f));
        float r = body_r   * cosf(t * (float)M_PI / 2.0f);

        if (l == lat) {
            // apex vertex
            int apex = m->num_verts;
            m->verts[m->num_verts++] = (vec3_t){ 0.0f, nose_top, 0.0f };
            for (int i = 0; i < seg; i++) {
                int ni = (i + 1) % seg;
                m->faces[m->num_faces++] = (face_t){{ prev_ring+i, apex, prev_ring+ni }};
            }
        } else {
            int cur_ring = m->num_verts;
            for (int i = 0; i < seg; i++) {
                float a = 2.0f * (float)M_PI * i / seg;
                m->verts[m->num_verts++] = (vec3_t){ r*cosf(a), y, r*sinf(a) };
            }
            for (int i = 0; i < seg; i++) {
                int ni = (i + 1) % seg;
                m->faces[m->num_faces++] = (face_t){{ prev_ring+i, cur_ring+i,  cur_ring+ni }};
                m->faces[m->num_faces++] = (face_t){{ prev_ring+i, cur_ring+ni, prev_ring+ni }};
            }
            prev_ring = cur_ring;
        }
    }

    // ---- 4 fins ----
    // thin triangular fins equally spaced around the base
    float fin_angles[4] = { 0.0f, (float)M_PI/2, (float)M_PI, 3.0f*(float)M_PI/2 };
    float fin_out  = 0.85f;  // how far fin extends radially
    float fin_bot  = body_bot;
    float fin_top  = body_bot + 0.55f;
    float fin_thick = 0.04f;

    for (int f = 0; f < 4; f++) {
        float ca = cosf(fin_angles[f]);
        float sa = sinf(fin_angles[f]);

        // tip of fin (outer edge, at body_bot level)
        float tx = (body_r + fin_out) * ca;
        float tz = (body_r + fin_out) * sa;
        // inner-bottom (where fin meets body at bottom)
        float ibx = body_r * ca, ibz = body_r * sa;
        // inner-top (where fin meets body higher up)
        float itx = body_r * ca, itz = body_r * sa;

        // slight thickness offset perpendicular to the fin
        float ox = -sa * fin_thick;
        float oz =  ca * fin_thick;

        int b = m->num_verts;
        m->verts[m->num_verts++] = (vec3_t){ ibx+ox, fin_bot, ibz+oz };  // b+0
        m->verts[m->num_verts++] = (vec3_t){ ibx-ox, fin_bot, ibz-oz };  // b+1
        m->verts[m->num_verts++] = (vec3_t){ tx,     fin_bot, tz      };  // b+2 tip-bot
        m->verts[m->num_verts++] = (vec3_t){ itx+ox, fin_top, itz+oz  };  // b+3
        m->verts[m->num_verts++] = (vec3_t){ itx-ox, fin_top, itz-oz  };  // b+4

        // front face
        m->faces[m->num_faces++] = (face_t){{ b+0, b+2, b+3 }};
        // back face
        m->faces[m->num_faces++] = (face_t){{ b+1, b+4, b+2 }};
        // top edge face
        m->faces[m->num_faces++] = (face_t){{ b+3, b+4, b+0 }};
        m->faces[m->num_faces++] = (face_t){{ b+4, b+1, b+0 }};
    }
}

// ---- Lego minifigure ----
//
// Classic T-pose minifigure assembled from boxes and cylinders.
// Parts (y increases upward, figure centred at origin):
//   legs   : two boxes side by side hanging down
//   hips   : wide short box connecting legs
//   torso  : taller box, slightly narrower than hips
//   arms   : two small boxes out to the sides at shoulder height
//   hands  : small cylinders at arm ends
//   neck   : short cylinder
//   head   : cylinder (the iconic round head)
//   stud   : tiny cylinder on top of head
// Total ~170 faces.

void model_make_minifigure(model_t *m)
{
    memset(m, 0, sizeof(*m));
    strncpy(m->name, "minifig", sizeof(m->name) - 1);

    // ---- legs ----
    float leg_w   = 0.18f, leg_d = 0.18f;
    float leg_bot = -1.0f, leg_top = -0.35f;
    float leg_sep = 0.21f;   // centre-to-centre x offset
    // left leg
    add_box(m, -leg_sep-leg_w, leg_bot, -leg_d,
                -leg_sep+leg_w, leg_top,  leg_d);
    // right leg
    add_box(m,  leg_sep-leg_w, leg_bot, -leg_d,
                leg_sep+leg_w, leg_top,  leg_d);

    // ---- hips ----
    float hip_w = 0.42f, hip_h = 0.18f, hip_d = 0.18f;
    add_box(m, -hip_w, leg_top, -hip_d, hip_w, leg_top+hip_h, hip_d);

    // ---- torso ----
    float tor_w = 0.36f, tor_d = 0.16f;
    float tor_bot = leg_top + hip_h;
    float tor_top = tor_bot + 0.52f;
    add_box(m, -tor_w, tor_bot, -tor_d, tor_w, tor_top, tor_d);

    // ---- arms (horizontal boxes out from shoulders) ----
    float arm_w = 0.22f, arm_h = 0.14f, arm_d = 0.13f;
    float arm_y0 = tor_top - 0.16f;
    float arm_y1 = arm_y0  - arm_h;
    // left arm
    add_box(m, -(tor_w + arm_w*2), arm_y1, -arm_d,
               -(tor_w),           arm_y0,  arm_d);
    // right arm
    add_box(m,  tor_w,             arm_y1, -arm_d,
                tor_w + arm_w*2,   arm_y0,  arm_d);

    // ---- hands (small cylinders at arm ends) ----
    int   hand_seg = 6;
    float hand_r   = 0.09f;
    float hand_cx_l = -(tor_w + arm_w*2 + hand_r);
    float hand_cx_r =  (tor_w + arm_w*2 + hand_r);
    float hand_cy   = (arm_y0 + arm_y1) * 0.5f;
    add_cylinder(m, hand_cx_l, hand_cy - hand_r, hand_cy + hand_r,
                 0.0f, hand_r, hand_seg);
    add_cylinder(m, hand_cx_r, hand_cy - hand_r, hand_cy + hand_r,
                 0.0f, hand_r, hand_seg);

    // ---- neck ----
    float neck_r   = 0.10f;
    float neck_bot = tor_top;
    float neck_top = tor_top + 0.10f;
    add_cylinder(m, 0.0f, neck_bot, neck_top, 0.0f, neck_r, 6);

    // ---- head (wider cylinder) ----
    float head_r   = 0.30f;
    float head_bot = neck_top;
    float head_top = head_bot + 0.38f;
    add_cylinder(m, 0.0f, head_bot, head_top, 0.0f, head_r, 10);

    // ---- stud on top of head ----
    float stud_r   = 0.10f;
    float stud_bot = head_top;
    float stud_top = head_top + 0.08f;
    add_cylinder(m, 0.0f, stud_bot, stud_top, 0.0f, stud_r, 6);
}
