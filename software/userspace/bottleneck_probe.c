// bottleneck_probe.c
//
// Standalone timing probe for the hardware rasterizer path. This is kept
// separate from main.c so the interactive demo stays untouched.

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "../kernel/avalon_kernel.h"
#include "geometry.h"
#include "model.h"

#define SCREEN_W 256
#define SCREEN_H 240
#define DEVICE   "/dev/rasterizer"

typedef struct {
    long status_polls;
    long fifo_full_polls;
    long submit_eagain;
    unsigned int max_fifo_level;
} submit_stats_t;

typedef struct {
    int faces_total;
    int triangles_built;
    int triangles_submitted;
    int triangles_culled;
    long bbox_pixels;
    long bbox_cycles_est;
    double setup_sec;
    double submit_sec;
    submit_stats_t submit;
} frame_stats_t;

static screen_vertex_t g_sv[MODEL_MAX_VERTS];
static volatile uint32_t *g_regs = NULL;

static double elapsed_sec(struct timespec a, struct timespec b)
{
    return (double)(b.tv_sec - a.tv_sec) +
           (double)(b.tv_nsec - a.tv_nsec) * 1e-9;
}

static void project_vertices(const model_t *model, const mat4_t *mvp)
{
    for (int i = 0; i < model->num_verts; i++) {
        vec3_t v = model->verts[i];
        vec4_t clip = mat4_mul_vec4(mvp, (vec4_t){v.x, v.y, v.z, 1.0f});

        if (fabsf(clip.w) < 1e-6f) clip.w = 1e-6f;
        float invw = 1.0f / clip.w;
        float ndcx = clip.x * invw;
        float ndcy = clip.y * invw;
        float ndcz = clip.z * invw;

        g_sv[i].sx = (ndcx + 1.0f) * 0.5f * SCREEN_W;
        g_sv[i].sy = (1.0f - ndcy) * 0.5f * SCREEN_H;

        float sz = (ndcz + 1.0f) * 0.5f;
        if (sz < 0.0f) sz = 0.0f;
        if (sz > 1.0f) sz = 1.0f;
        g_sv[i].sz = sz;
    }
}

static int submit_triangle(int fd, const triangle_packet_t *pkt,
                           submit_stats_t *stats)
{
    if (g_regs) {
        const uint32_t *w = (const uint32_t *)pkt;
        for (int i = 0; i < RAST_PACKET_NUM_WORDS; i++)
            g_regs[i] = w[i];
        uint32_t status = g_regs[RAST_STATUS_OFFSET / 4];
        stats->status_polls++;
        unsigned int level = status & RAST_STATUS_LEVEL_MASK;
        if (level > stats->max_fifo_level)
            stats->max_fifo_level = level;
        while (status & RAST_STATUS_FULL_BIT) {
            stats->fifo_full_polls++;
            status = g_regs[RAST_STATUS_OFFSET / 4];
            stats->status_polls++;
        }
        g_regs[RAST_COMMIT_OFFSET / 4] = 1;
        return 0;
    }

    rasterizer_arg_t ra;
    memset(&ra, 0, sizeof(ra));
    ra.packet = *pkt;
    while (ioctl(fd, RASTERIZER_SUBMIT, &ra) != 0) {
        if (errno != EAGAIN) return -1;
        stats->submit_eagain++;
    }
    return 0;
}

static int wait_present(int fd, long *status_polls)
{
    rasterizer_arg_t ra;

    do {
        memset(&ra, 0, sizeof(ra));
        if (ioctl(fd, RASTERIZER_STATUS, &ra) < 0)
            return -1;
        (*status_polls)++;
    } while (!ra.status.swap_busy);

    for (;;) {
        memset(&ra, 0, sizeof(ra));
        if (ioctl(fd, RASTERIZER_STATUS, &ra) < 0)
            return -1;
        (*status_polls)++;
        if (!ra.status.swap_busy)
            return 0;
        //usleep(100);
    }
}

static int render_frame_profile(int fd, int dry_run, const model_t *model,
                                float rot_x, float rot_y, float rot_z,
                                float cam_dist, frame_stats_t *stats)
{
    struct timespec t0, t1;
    float aspect = (float)SCREEN_W / (float)SCREEN_H;

    mat4_t proj = perspective(60.0f, aspect, 0.1f, 100.0f);
    mat4_t view = translation(0.0f, 0.0f, -cam_dist);
    mat4_t rz = rotation_z(rot_z);
    mat4_t ry = rotation_y(rot_y);
    mat4_t rx = rotation_x(rot_x);
    mat4_t ryx = mat4_mul(&ry, &rx);
    mat4_t model_mat = mat4_mul(&rz, &ryx);
    mat4_t vp = mat4_mul(&proj, &view);
    mat4_t mvp = mat4_mul(&vp, &model_mat);

    vec3_t light_dir = vec3_normalize((vec3_t){0.4f, 0.7f, 0.5f});
    float base_r = 0.2f, base_g = 0.7f, base_b = 1.0f;

    memset(stats, 0, sizeof(*stats));
    stats->faces_total = model->num_faces;

    project_vertices(model, &mvp);

    for (int i = 0; i < model->num_faces; i++) {
        const face_t *f = &model->faces[i];

        vec3_t p0 = model->verts[f->v[0]];
        vec3_t p1 = model->verts[f->v[1]];
        vec3_t p2 = model->verts[f->v[2]];
        vec3_t face_n = vec3_normalize(
            vec3_cross(vec3_sub(p1, p0), vec3_sub(p2, p0)));
        vec4_t tn = mat4_mul_vec4(&model_mat,
                                  (vec4_t){face_n.x, face_n.y, face_n.z, 0.0f});
        vec3_t wn = vec3_normalize((vec3_t){tn.x, tn.y, tn.z});
        uint8_t color = shade_face(wn, light_dir, base_r, base_g, base_b);

        triangle_packet_t pkt;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        int ok = setup_triangle(&g_sv[f->v[0]], &g_sv[f->v[1]],
                                &g_sv[f->v[2]], color, &pkt);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        stats->setup_sec += elapsed_sec(t0, t1);

        if (ok < 0) {
            stats->triangles_culled++;
            continue;
        }

        int xmin = pkt.bbox_packed & 0xFF;
        int xmax = (pkt.bbox_packed >> 8) & 0xFF;
        int ymin = (pkt.bbox_packed >> 16) & 0xFF;
        int ymax = (pkt.bbox_packed >> 24) & 0xFF;
        int width = xmax - xmin + 1;
        int height = ymax - ymin + 1;
        int column_banks = (width + 15) / 16;

        stats->triangles_built++;
        if (width > 0 && height > 0) {
            stats->bbox_pixels += (long)width * height;
            stats->bbox_cycles_est += (long)column_banks * height;
        }

        if (!dry_run) {
            clock_gettime(CLOCK_MONOTONIC, &t0);
            if (submit_triangle(fd, &pkt, &stats->submit) < 0)
                return -1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            stats->submit_sec += elapsed_sec(t0, t1);
        }
        stats->triangles_submitted++;
    }

    return 0;
}

static void make_builtin_model(model_t *m, int idx)
{
    switch (idx) {
    case 0: model_make_cube(m); break;
    case 1: model_make_icosphere(m, 1); break;
    case 2: model_make_icosphere(m, 2); break;
    case 3: model_make_torus(m, 12, 8, 0.7f, 0.3f); break;
    case 4: model_make_teapot(m); break;
    case 5: model_make_saturn(m); break;
    case 6: model_make_lego(m); break;
    case 7: model_make_dna(m); break;
    case 8: model_make_teapot_552(m); break;
    case 9: model_make_minifigure(m); break;
    default: model_make_icosphere(m, 2); break;
    }
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [--dry-run] [--frames N] [--model N] [objfile]\n"
            "  --dry-run   run ARM setup math only; do not open /dev/rasterizer\n"
            "  --frames N  number of frames to measure, default 60\n"
            "  --model N   built-in model index 0..9, default 2\n",
            argv0);
}

int main(int argc, char **argv)
{
    int dry_run = 0;
    int frames = 60;
    int model_idx = 2;
    const char *obj_path = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dry-run") == 0) {
            dry_run = 1;
        } else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            frames = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--model") == 0 && i + 1 < argc) {
            model_idx = atoi(argv[++i]);
        } else if (argv[i][0] == '-') {
            usage(argv[0]);
            return 2;
        } else {
            obj_path = argv[i];
        }
    }

    if (frames <= 0) frames = 1;

    model_t model;
    if (obj_path) {
        if (model_load_obj(&model, obj_path) < 0) {
            fprintf(stderr, "could not load OBJ %s\n", obj_path);
            return 1;
        }
    } else {
        make_builtin_model(&model, model_idx);
    }

    int fd = -1;
    if (!dry_run) {
        fd = open(DEVICE, O_RDWR);
        if (fd < 0) {
            fprintf(stderr, "cannot open %s: %s\n", DEVICE, strerror(errno));
            fprintf(stderr, "try --dry-run for software-only profiling\n");
            return 1;
        }
        void *mapped = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, 0);
        if (mapped != MAP_FAILED) {
            g_regs = (volatile uint32_t *)mapped;
            fprintf(stderr, "mmap OK: direct MMIO submission enabled\n");
        } else {
            g_regs = NULL;
            fprintf(stderr, "mmap failed (%s); using ioctl fallback\n",
                    strerror(errno));
        }
    }

    printf("frame,model,faces,built,culled,render_ms,setup_ms,submit_ms,"
           "present_ms,fps,fifo_max,fifo_full_polls,eagain,status_polls,"
           "bbox_avg_px,hw_cycle_est\n");

    for (int frame = 0; frame < frames; frame++) {
        struct timespec t0, t1, tp0, tp1;
        frame_stats_t stats;
        long present_polls = 0;
        double present_ms = 0.0;

        float rot_x = 25.0f;
        float rot_y = 45.0f + (float)frame * 0.5f;
        float rot_z = 0.0f;
        float cam_dist = 4.0f;

        clock_gettime(CLOCK_MONOTONIC, &t0);
        if (render_frame_profile(fd, dry_run, &model, rot_x, rot_y, rot_z,
                                 cam_dist, &stats) < 0) {
            fprintf(stderr, "render failed on frame %d: %s\n",
                    frame, strerror(errno));
            if (fd >= 0) close(fd);
            return 1;
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);

        if (!dry_run) {
            clock_gettime(CLOCK_MONOTONIC, &tp0);
            if (ioctl(fd, RASTERIZER_PRESENT) < 0 ||
                wait_present(fd, &present_polls) < 0) {
                fprintf(stderr, "present failed on frame %d: %s\n",
                        frame, strerror(errno));
                close(fd);
                return 1;
            }
            clock_gettime(CLOCK_MONOTONIC, &tp1);
            present_ms = elapsed_sec(tp0, tp1) * 1000.0;
        }

        double render_ms = elapsed_sec(t0, t1) * 1000.0;
        double setup_ms = stats.setup_sec * 1000.0;
        double submit_ms = stats.submit_sec * 1000.0;
        double fps = render_ms + present_ms > 0.0 ?
                     1000.0 / (render_ms + present_ms) : 0.0;
        double bbox_avg = stats.triangles_built ?
                          (double)stats.bbox_pixels / stats.triangles_built :
                          0.0;

        printf("%d,%s,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.2f,%u,%ld,%ld,%ld,%.1f,%ld\n",
               frame, model.name, stats.faces_total, stats.triangles_built,
               stats.triangles_culled, render_ms, setup_ms, submit_ms,
               present_ms, fps, stats.submit.max_fifo_level,
               stats.submit.fifo_full_polls, stats.submit.submit_eagain,
               stats.submit.status_polls + present_polls, bbox_avg,
               stats.bbox_cycles_est);
        fflush(stdout);
    }

    if (fd >= 0) {
        if (g_regs) munmap((void *)g_regs, 4096);
        close(fd);
    }
    return 0;
}
