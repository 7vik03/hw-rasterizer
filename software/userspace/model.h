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

/* ------------------------------------------------------------------ */
/* OBJ loader (triangles only; quads split into two triangles)        */
/* ------------------------------------------------------------------ */

/*
 * Returns 0 on success, -1 on failure (file not found, too large, etc.).
 * Supports "v" and "f" lines; ignores everything else.
 */
int model_load_obj(model_t *m, const char *filename);

#endif /* MODEL_H */
