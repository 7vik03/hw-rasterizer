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
