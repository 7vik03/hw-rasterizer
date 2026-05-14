// model.h

#ifndef MODEL_H
#define MODEL_H

#include "geometry.h"


#define MODEL_MAX_VERTS  131072
#define MODEL_MAX_FACES  262144

typedef struct {
    int v[3];
} face_t;

typedef struct {
    vec3_t  verts[MODEL_MAX_VERTS];
    face_t  faces[MODEL_MAX_FACES];
    int     num_verts;
    int     num_faces;
    char    name[64];
} model_t;


void model_make_cube(model_t *m);


void model_make_icosphere(model_t *m, int subdivisions);


void model_make_torus(model_t *m, int major_seg, int minor_seg,
                      float R, float r);


void model_make_teapot(model_t *m);


void model_make_saturn(model_t *m);


void model_make_lego(model_t *m);


void model_make_dna(model_t *m);


void model_make_teapot_552(model_t *m);


void model_make_rocket(model_t *m);


void model_make_minifigure(model_t *m);


int model_load_obj(model_t *m, const char *filename);

#endif
