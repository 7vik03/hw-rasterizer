/*
 * model.h
 * Mesh data: vertex + face lists for the built-in procedural models
 * and OBJ loader.  Mirrors the Model struct in demo.cpp.
 */

#ifndef MODEL_H
#define MODEL_H

#include "geometry.h"

/* ------------------------------------------------------------------ */
/* Mesh storage                                                         */
/* ------------------------------------------------------------------ */

#define MODEL_MAX_VERTS  4096
#define MODEL_MAX_FACES  2048

typedef struct {
    int v[3];   /* indices into verts[] */
} face_t;

typedef struct {
    vec3_t  verts[MODEL_MAX_VERTS];
    face_t  faces[MODEL_MAX_FACES];
    int     num_verts;
    int     num_faces;
    char    name[64];
} model_t;

/* ------------------------------------------------------------------ */
/* Procedural generators — same geometry as demo.cpp                  */
/* ------------------------------------------------------------------ */

/* 12-triangle cube, unit half-size */
void model_make_cube(model_t *m);

/*
 * Icosphere with `subdivisions` levels of refinement.
 *   subdivisions=1 → 80 faces (sphere_lo)
 *   subdivisions=2 → 320 faces (sphere_med)
 */
void model_make_icosphere(model_t *m, int subdivisions);

/*
 * Torus.
 *   major_seg, minor_seg — tessellation (12×8 = 192 faces in demo)
 *   R — major radius, r — tube radius
 */
void model_make_torus(model_t *m, int major_seg, int minor_seg,
                      float R, float r);

/*
 * Utah teapot, tessellated from the 32 bicubic Bezier patches at
 * 4x4 samples per patch -- ~1024 triangles, fits in MODEL_MAX_FACES.
 */
void model_make_teapot(model_t *m);

/* Saturn: icosphere body + tilted torus ring -- ~368 faces */
void model_make_saturn(model_t *m);

/* Lego 2x4 brick with 8 cylindrical studs -- ~204 faces */
void model_make_lego(model_t *m);

/* DNA double helix: two helical tubes + rungs -- ~300 faces */
void model_make_dna(model_t *m);

/* Utah teapot mesh baked from 552.obj -- 552 faces */
void model_make_teapot_552(model_t *m);

/* Rocket: nose cone + body + 4 fins -- ~148 faces */
void model_make_rocket(model_t *m);

/* Lego minifigure in T-pose: legs, torso, arms, round head -- ~170 faces */
void model_make_minifigure(model_t *m);

/* ------------------------------------------------------------------ */
/* OBJ loader (triangles only; quads split into two triangles)        */
/* ------------------------------------------------------------------ */

/*
 * Returns 0 on success, -1 on failure (file not found, too large, etc.).
 * Supports "v" and "f" lines; ignores everything else.
 */
int model_load_obj(model_t *m, const char *filename);

#endif /* MODEL_H */
