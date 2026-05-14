// main.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <libusb-1.0/libusb.h>

#include "../kernel/avalon_kernel.h"
#include "usbkeyboard.h"
#include "geometry.h"
#include "model.h"


#define SCREEN_W    256
#define SCREEN_H    240
#define DEVICE      "/dev/rasterizer"


static volatile uint32_t *g_regs = NULL;

#define STATUS_POLL_PERIOD 32

static inline void mmio_barrier(void)
{
#if defined(__arm__) || defined(__aarch64__)
    __asm__ volatile ("dsb sy" ::: "memory");
#else
    __sync_synchronize();
#endif
}

static int submit_triangle(int fd, const triangle_packet_t *pkt)
{
    if (g_regs) {
        static unsigned int tri_counter = 0;
        const uint32_t *w = (const uint32_t *)pkt;

        if ((tri_counter++ & (STATUS_POLL_PERIOD - 1)) == 0) {
            while (g_regs[RAST_STATUS_OFFSET / 4] & RAST_STATUS_FULL_BIT)
                ;
        }

        for (int i = 0; i < RAST_PACKET_NUM_WORDS; i++)
            g_regs[i] = w[i];

        mmio_barrier();
        g_regs[RAST_COMMIT_OFFSET / 4] = 1;
        return 0;
    }

    rasterizer_arg_t ra;
    memset(&ra, 0, sizeof(ra));
    ra.packet = *pkt;
    while (ioctl(fd, RASTERIZER_SUBMIT, &ra) != 0) {
        if (errno != EAGAIN) return -1;
    }
    return 0;
}


static struct libusb_device_handle *g_keyboard = NULL;
static uint8_t                      g_endpoint  = 0;

static struct usb_keyboard_packet g_pkt_cur;
static struct usb_keyboard_packet g_pkt_prev;

#define SCAN_TABLE_SIZE 256
static int g_keys[SCAN_TABLE_SIZE];
static int g_key_pressed[SCAN_TABLE_SIZE];

static int keyboard_init(void)
{
    g_keyboard = openkeyboard(&g_endpoint);
    if (!g_keyboard) {
        fprintf(stderr, "No USB keyboard found\n");
        return -1;
    }
    memset(&g_pkt_cur,  0, sizeof(g_pkt_cur));
    memset(&g_pkt_prev, 0, sizeof(g_pkt_prev));
    return 0;
}

static void keyboard_close(void)
{
    if (g_keyboard) {
        libusb_release_interface(g_keyboard, 0);
        libusb_close(g_keyboard);
        g_keyboard = NULL;
    }
    libusb_exit(NULL);
}

static void poll_keys(void)
{
    int transferred = 0;

    g_pkt_prev = g_pkt_cur;


    int rc = libusb_interrupt_transfer(g_keyboard, g_endpoint,
                                       (unsigned char *)&g_pkt_cur,
                                       sizeof(g_pkt_cur),
                                       &transferred, 1);
    if (rc != 0 || transferred != (int)sizeof(g_pkt_cur))
        g_pkt_cur = g_pkt_prev;

    memset(g_keys,        0, sizeof(g_keys));
    memset(g_key_pressed, 0, sizeof(g_key_pressed));

    for (int i = 0; i < 6; i++) {
        uint8_t kc = g_pkt_cur.keycode[i];
        if (kc == 0) continue;

        g_keys[kc] = 1;


        int was_held = 0;
        for (int j = 0; j < 6; j++) {
            if (g_pkt_prev.keycode[j] == kc) { was_held = 1; break; }
        }
        if (!was_held)
            g_key_pressed[kc] = 1;
    }
}


static screen_vertex_t g_sv[MODEL_MAX_VERTS];

static void project_vertices(const model_t *model, const mat4_t *mvp)
{
    for (int i = 0; i < model->num_verts; i++) {
        vec3_t v    = model->verts[i];
        vec4_t clip = mat4_mul_vec4(mvp, (vec4_t){ v.x, v.y, v.z, 1.0f });

        if (fabsf(clip.w) < 1e-6f) clip.w = 1e-6f;
        float invw = 1.0f / clip.w;
        float ndcx =  clip.x * invw;
        float ndcy =  clip.y * invw;
        float ndcz =  clip.z * invw;

        g_sv[i].sx = (ndcx + 1.0f) * 0.5f * SCREEN_W;
        g_sv[i].sy = (1.0f - ndcy) * 0.5f * SCREEN_H;


        float sz = (ndcz + 1.0f) * 0.5f;
        if (sz < 0.0f) sz = 0.0f;
        if (sz > 1.0f) sz = 1.0f;
        g_sv[i].sz = sz;
    }
}


static int render_frame(int fd, const model_t *model,
                        float rot_x, float rot_y, float rot_z,
                        float cam_dist)
{
    float aspect = (float)SCREEN_W / (float)SCREEN_H;

    mat4_t proj      = perspective(60.0f, aspect, 0.1f, 100.0f);
    mat4_t view      = translation(0.0f, 0.0f, -cam_dist);
    mat4_t rz        = rotation_z(rot_z);
    mat4_t ry        = rotation_y(rot_y);
    mat4_t rx        = rotation_x(rot_x);
    mat4_t ryx       = mat4_mul(&ry, &rx);
    mat4_t model_mat = mat4_mul(&rz, &ryx);
    mat4_t vp        = mat4_mul(&proj, &view);
    mat4_t mvp       = mat4_mul(&vp, &model_mat);

    vec3_t light_dir = vec3_normalize((vec3_t){ 0.4f, 0.7f, 0.5f });
    float  base_r = 0.2f, base_g = 0.7f, base_b = 1.0f;

    project_vertices(model, &mvp);

    int submitted = 0;
    for (int i = 0; i < model->num_faces; i++) {
        const face_t *f = &model->faces[i];


        vec3_t p0 = model->verts[f->v[0]];
        vec3_t p1 = model->verts[f->v[1]];
        vec3_t p2 = model->verts[f->v[2]];
        vec3_t face_n = vec3_normalize(
                            vec3_cross(vec3_sub(p1, p0), vec3_sub(p2, p0)));

        vec4_t tn = mat4_mul_vec4(&model_mat,
                        (vec4_t){ face_n.x, face_n.y, face_n.z, 0.0f });
        vec3_t wn    = vec3_normalize((vec3_t){ tn.x, tn.y, tn.z });
        uint8_t color = shade_face(wn, light_dir, base_r, base_g, base_b);

        triangle_packet_t pkt;
        if (setup_triangle(&g_sv[f->v[0]], &g_sv[f->v[1]], &g_sv[f->v[2]],
                           color, &pkt) < 0)
            continue;

        if (submit_triangle(fd, &pkt) < 0) {
            fprintf(stderr, "submit_triangle failed: %s\n", strerror(errno));
            return -1;
        }
        submitted++;
    }
    return submitted;
}


static int load_named_obj(model_t *out, const char *stem,
                          char *loaded_path, size_t loaded_path_sz)
{
    char candidates[4][128];
    snprintf(candidates[0], sizeof(candidates[0]), "models/%s", stem);
    snprintf(candidates[1], sizeof(candidates[1]), "models/%s.obj", stem);
    snprintf(candidates[2], sizeof(candidates[2]), "%s", stem);
    snprintf(candidates[3], sizeof(candidates[3]), "%s.obj", stem);

    for (int i = 0; i < 4; i++) {
        if (model_load_obj(out, candidates[i]) == 0) {
            if (loaded_path && loaded_path_sz > 0) {
                snprintf(loaded_path, loaded_path_sz, "%s", candidates[i]);
            }
            return 0;
        }
    }

    if (loaded_path && loaded_path_sz > 0) loaded_path[0] = '\0';
    return -1;
}


int main(int argc, char *argv[])
{


    enum {
        MODEL_CUBE = 0,
        MODEL_SPHERE_LO,
        MODEL_SPHERE_MED,
        MODEL_SPHERE_HI,
        MODEL_SPHERE_ULTRA,
        MODEL_SPHERE_MAX,
        MODEL_TORUS,
        MODEL_SATURN,
        MODEL_LEGO,
        MODEL_DNA,
        MODEL_ARGV_OBJ,
        MODEL_OBJ1,
        MODEL_OBJ2,
        MODEL_OBJ3,
        MODEL_THINKER,
        MODEL_ALMA_MATER,
        MODEL_ALMA_MATER_COMP,
        MODEL_LION,
        MODEL_CROWN,
        MODEL_CROWN_80K,
        MODEL_CROWN_100K,
        MODEL_COUNT
    };

    static model_t models[MODEL_COUNT];
    int num_models = 10;
    int obj1_loaded = 0, obj2_loaded = 0, obj3_loaded = 0;
    int thinker_loaded = 0, alma_loaded = 0, alma_comp_loaded = 0, lion_loaded = 0;
    int crown_loaded = 0, crown_80k_loaded = 0, crown_100k_loaded = 0;
    char obj1_path[128], obj2_path[128], obj3_path[128];
    char thinker_path[128], alma_path[128], alma_comp_path[128], lion_path[128];
    char crown_path[128], crown_80k_path[128], crown_100k_path[128];

    model_make_cube(&models[MODEL_CUBE]);
    model_make_icosphere(&models[MODEL_SPHERE_LO],    1);
    model_make_icosphere(&models[MODEL_SPHERE_MED],   2);
    model_make_icosphere(&models[MODEL_SPHERE_HI],    3);
    model_make_icosphere(&models[MODEL_SPHERE_ULTRA], 4);
    model_make_icosphere(&models[MODEL_SPHERE_MAX],   5);
    model_make_torus(&models[MODEL_TORUS], 12, 8, 0.7f, 0.3f);
    model_make_saturn(&models[MODEL_SATURN]);
    model_make_lego(&models[MODEL_LEGO]);
    model_make_dna(&models[MODEL_DNA]);

    if (argc >= 2) {
        if (model_load_obj(&models[MODEL_ARGV_OBJ], argv[1]) == 0) {
            printf("Loaded %s: %d verts, %d faces\n",
                   argv[1],
                   models[MODEL_ARGV_OBJ].num_verts,
                   models[MODEL_ARGV_OBJ].num_faces);
            num_models = 10;
        } else {
            fprintf(stderr, "Warning: could not load %s\n", argv[1]);
        }
    }


    if (load_named_obj(&models[MODEL_OBJ1], "obj1", obj1_path, sizeof(obj1_path)) == 0) {
        obj1_loaded = 1;
        strncpy(models[MODEL_OBJ1].name, "obj1", sizeof(models[MODEL_OBJ1].name) - 1);
        models[MODEL_OBJ1].name[sizeof(models[MODEL_OBJ1].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_OBJ1]);
        strncpy(models[MODEL_OBJ1].name, "obj1_missing", sizeof(models[MODEL_OBJ1].name) - 1);
    }
    if (load_named_obj(&models[MODEL_OBJ2], "obj2", obj2_path, sizeof(obj2_path)) == 0) {
        obj2_loaded = 1;
        strncpy(models[MODEL_OBJ2].name, "obj2", sizeof(models[MODEL_OBJ2].name) - 1);
        models[MODEL_OBJ2].name[sizeof(models[MODEL_OBJ2].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_OBJ2]);
        strncpy(models[MODEL_OBJ2].name, "obj2_missing", sizeof(models[MODEL_OBJ2].name) - 1);
    }
    if (load_named_obj(&models[MODEL_OBJ3], "obj3", obj3_path, sizeof(obj3_path)) == 0) {
        obj3_loaded = 1;
        strncpy(models[MODEL_OBJ3].name, "obj3", sizeof(models[MODEL_OBJ3].name) - 1);
        models[MODEL_OBJ3].name[sizeof(models[MODEL_OBJ3].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_OBJ3]);
        strncpy(models[MODEL_OBJ3].name, "obj3_missing", sizeof(models[MODEL_OBJ3].name) - 1);
    }
    if (load_named_obj(&models[MODEL_THINKER], "Thinker", thinker_path, sizeof(thinker_path)) == 0) {
        thinker_loaded = 1;
        strncpy(models[MODEL_THINKER].name, "Thinker", sizeof(models[MODEL_THINKER].name) - 1);
        models[MODEL_THINKER].name[sizeof(models[MODEL_THINKER].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_THINKER]);
        strncpy(models[MODEL_THINKER].name, "Thinker_missing", sizeof(models[MODEL_THINKER].name) - 1);
    }
    if (load_named_obj(&models[MODEL_ALMA_MATER], "alma_mater", alma_path, sizeof(alma_path)) == 0) {
        alma_loaded = 1;
        strncpy(models[MODEL_ALMA_MATER].name, "alma_mater", sizeof(models[MODEL_ALMA_MATER].name) - 1);
        models[MODEL_ALMA_MATER].name[sizeof(models[MODEL_ALMA_MATER].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_ALMA_MATER]);
        strncpy(models[MODEL_ALMA_MATER].name, "alma_mater_missing", sizeof(models[MODEL_ALMA_MATER].name) - 1);
    }
    if (load_named_obj(&models[MODEL_ALMA_MATER_COMP], "alma_mater_compressed", alma_comp_path, sizeof(alma_comp_path)) == 0) {
        alma_comp_loaded = 1;
        strncpy(models[MODEL_ALMA_MATER_COMP].name, "alma_mater_comp", sizeof(models[MODEL_ALMA_MATER_COMP].name) - 1);
        models[MODEL_ALMA_MATER_COMP].name[sizeof(models[MODEL_ALMA_MATER_COMP].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_ALMA_MATER_COMP]);
        strncpy(models[MODEL_ALMA_MATER_COMP].name, "alma_comp_missing", sizeof(models[MODEL_ALMA_MATER_COMP].name) - 1);
    }
    if (load_named_obj(&models[MODEL_LION], "wooden_lion_sculpture_derivative", lion_path, sizeof(lion_path)) == 0) {
        lion_loaded = 1;
        strncpy(models[MODEL_LION].name, "lion", sizeof(models[MODEL_LION].name) - 1);
        models[MODEL_LION].name[sizeof(models[MODEL_LION].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_LION]);
        strncpy(models[MODEL_LION].name, "lion_missing", sizeof(models[MODEL_LION].name) - 1);
    }
    if (load_named_obj(&models[MODEL_CROWN], "ColumbiaCrown", crown_path, sizeof(crown_path)) == 0) {
        crown_loaded = 1;
        strncpy(models[MODEL_CROWN].name, "Crown", sizeof(models[MODEL_CROWN].name) - 1);
        models[MODEL_CROWN].name[sizeof(models[MODEL_CROWN].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_CROWN]);
        strncpy(models[MODEL_CROWN].name, "Crown_missing", sizeof(models[MODEL_CROWN].name) - 1);
    }
    if (load_named_obj(&models[MODEL_CROWN_80K], "ColumbiaCrown_80K", crown_80k_path, sizeof(crown_80k_path)) == 0) {
        crown_80k_loaded = 1;
        strncpy(models[MODEL_CROWN_80K].name, "Crown_80K", sizeof(models[MODEL_CROWN_80K].name) - 1);
        models[MODEL_CROWN_80K].name[sizeof(models[MODEL_CROWN_80K].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_CROWN_80K]);
        strncpy(models[MODEL_CROWN_80K].name, "Crown_80K_missing", sizeof(models[MODEL_CROWN_80K].name) - 1);
    }
    if (load_named_obj(&models[MODEL_CROWN_100K], "ColumbiaCrown_100K", crown_100k_path, sizeof(crown_100k_path)) == 0) {
        crown_100k_loaded = 1;
        strncpy(models[MODEL_CROWN_100K].name, "Crown_100K", sizeof(models[MODEL_CROWN_100K].name) - 1);
        models[MODEL_CROWN_100K].name[sizeof(models[MODEL_CROWN_100K].name) - 1] = '\0';
    } else {
        model_make_cube(&models[MODEL_CROWN_100K]);
        strncpy(models[MODEL_CROWN_100K].name, "Crown_100K_missing", sizeof(models[MODEL_CROWN_100K].name) - 1);
    }

    int fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", DEVICE, strerror(errno));
        fprintf(stderr, "Is the kernel module loaded? "
                        "sudo insmod avalon_kernel.ko\n");
        return 1;
    }


    void *mapped = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd, 0);
    if (mapped != MAP_FAILED) {
        g_regs = (volatile uint32_t *)mapped;
        printf("mmap OK: direct MMIO submission enabled\n");
    } else {
        g_regs = NULL;
        fprintf(stderr, "mmap failed (%s); using ioctl fallback\n",
                strerror(errno));
    }


    if (keyboard_init() < 0) {
        close(fd);
        return 1;
    }


    printf("\n=== CSEE 4840 Hardware Rasterizer Demo ===\n");
    printf("Controls (USB keyboard on DE1-SoC):\n");
    printf("  w/s     - tilt up/down\n");
    printf("  a/d     - rotate left/right\n");
    printf("  +/-     - zoom in/out\n");
    printf("  1       - cube\n");
    printf("  2       - sphere lo   (80 tris)\n");
    printf("  3       - sphere med  (320 tris)\n");
    printf("  4       - sphere hi   (1280 tris)\n");
    printf("  5       - sphere max  (5120 tris)\n");
    printf("  0       - sphere stress (20480 tris)\n");
    printf("  6       - torus\n");
    printf("  7       - saturn\n");
    printf("  8       - lego brick\n");
    printf("  9       - dna helix\n");
    printf("  b       - obj1\n");
    printf("  m       - obj2\n");
    printf("  n       - obj3\n");
    printf("  t       - Thinker       (%s)\n", thinker_loaded  ? thinker_path   : "NOT FOUND");
    printf("  f       - alma mater    (%s)\n", alma_loaded      ? alma_path       : "NOT FOUND");
    printf("  c       - alma (compressed) (%s)\n", alma_comp_loaded ? alma_comp_path : "NOT FOUND");
    printf("  l       - wooden lion   (%s)\n", lion_loaded      ? lion_path       : "NOT FOUND");
    printf("  e       - Columbia Crown (%s)\n",      crown_loaded     ? crown_path      : "NOT FOUND");
    printf("  g       - Crown 80K     (%s)\n",       crown_80k_loaded ? crown_80k_path  : "NOT FOUND");
    printf("  h       - Crown 100K    (%s)\n",       crown_100k_loaded? crown_100k_path : "NOT FOUND");
    if (num_models > 10) printf("  (OBJ)   - %s\n", models[MODEL_ARGV_OBJ].name);
    printf("OBJ key mapping:\n");
    printf("  b -> obj1: %s\n", obj1_loaded ? obj1_path : "NOT FOUND");
    printf("  m -> obj2: %s\n", obj2_loaded ? obj2_path : "NOT FOUND");
    printf("  n -> obj3: %s\n", obj3_loaded ? obj3_path : "NOT FOUND");
    printf("  r       - reset rotation\n");
    printf("  SPACE   - toggle auto-rotate\n");
    printf("  ESC/q   - quit\n");
    printf("==========================================\n\n");


    int   current_model = 2;
    float rot_x      = 25.0f;
    float rot_y      = 45.0f;
    float rot_z      =  0.0f;
    float cam_dist   =  4.0f;
    float rot_speed  =  2.0f;
    int   auto_rotate = 0;
    int   running    = 1;
    long  frame      = 0;
    float fps        = 0.0f;

    struct timespec t_prev, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_prev);

    while (running) {


        poll_keys();


        if (g_keys[KEY_W]) rot_x -= rot_speed;
        if (g_keys[KEY_S]) rot_x += rot_speed;
        if (g_keys[KEY_A]) rot_y -= rot_speed;
        if (g_keys[KEY_D]) rot_y += rot_speed;
        if (g_keys[KEY_EQUAL]) {
            cam_dist -= 0.3f;
            if (cam_dist < 1.5f) cam_dist = 1.5f;
        }
        if (g_keys[KEY_MINUS]) {
            cam_dist += 0.3f;
            if (cam_dist > 15.0f) cam_dist = 15.0f;
        }


        if (g_key_pressed[KEY_ESC] || g_key_pressed[KEY_Q]) running = 0;
        if (g_key_pressed[KEY_1]) current_model = MODEL_CUBE;
        if (g_key_pressed[KEY_2]) current_model = MODEL_SPHERE_LO;
        if (g_key_pressed[KEY_3]) current_model = MODEL_SPHERE_MED;
        if (g_key_pressed[KEY_4]) current_model = MODEL_SPHERE_HI;
        if (g_key_pressed[KEY_5]) current_model = MODEL_SPHERE_ULTRA;
        if (g_key_pressed[KEY_0]) current_model = MODEL_SPHERE_MAX;
        if (g_key_pressed[KEY_6]) current_model = MODEL_TORUS;
        if (g_key_pressed[KEY_7]) current_model = MODEL_SATURN;
        if (g_key_pressed[KEY_8]) current_model = MODEL_LEGO;
        if (g_key_pressed[KEY_9]) current_model = MODEL_DNA;
        if (g_key_pressed[KEY_B]) {
            if (obj1_loaded) current_model = MODEL_OBJ1;
            else fprintf(stderr, "\nobj1 not found. Place it at software/models/obj1.obj\n");
        }
        if (g_key_pressed[KEY_M]) {
            if (obj2_loaded) current_model = MODEL_OBJ2;
            else fprintf(stderr, "\nobj2 not found. Place it at software/models/obj2.obj\n");
        }
        if (g_key_pressed[KEY_N]) {
            if (obj3_loaded) current_model = MODEL_OBJ3;
            else fprintf(stderr, "\nobj3 not found. Place it at software/models/obj3.obj\n");
        }
        if (g_key_pressed[KEY_T]) {
            if (thinker_loaded) current_model = MODEL_THINKER;
            else fprintf(stderr, "\nThinker.obj not found in software/models/\n");
        }
        if (g_key_pressed[KEY_F]) {
            if (alma_loaded) current_model = MODEL_ALMA_MATER;
            else fprintf(stderr, "\nalma_mater.obj not found in software/models/\n");
        }
        if (g_key_pressed[KEY_C]) {
            if (alma_comp_loaded) current_model = MODEL_ALMA_MATER_COMP;
            else fprintf(stderr, "\nalma_mater_compressed.obj not found in software/models/\n");
        }
        if (g_key_pressed[KEY_L]) {
            if (lion_loaded) current_model = MODEL_LION;
            else fprintf(stderr, "\nwooden_lion_sculpture_derivative.obj not found in software/models/\n");
        }
        if (g_key_pressed[KEY_E]) {
            if (crown_loaded) current_model = MODEL_CROWN;
            else fprintf(stderr, "\nColumbiaCrown.obj not found in software/models/\n");
        }
        if (g_key_pressed[KEY_G]) {
            if (crown_80k_loaded) current_model = MODEL_CROWN_80K;
            else fprintf(stderr, "\nColumbiaCrown_80K.obj not found in software/models/\n");
        }
        if (g_key_pressed[KEY_H]) {
            if (crown_100k_loaded) current_model = MODEL_CROWN_100K;
            else fprintf(stderr, "\nColumbiaCrown_100K.obj not found in software/models/\n");
        }
        if (g_key_pressed[KEY_R]) {
            rot_x = 25.0f; rot_y = 45.0f; rot_z = 0.0f; cam_dist = 4.0f;
        }
        if (g_key_pressed[KEY_SPACE]) auto_rotate = !auto_rotate;

        if (auto_rotate) rot_y += 0.5f;


        int n = render_frame(fd, &models[current_model],
                             rot_x, rot_y, rot_z, cam_dist);
        if (n < 0) {
            fprintf(stderr, "render_frame error, aborting\n");
            break;
        }

        if (ioctl(fd, RASTERIZER_PRESENT) < 0) {
            fprintf(stderr, "present failed: %s\n", strerror(errno));
            break;
        }


        rasterizer_arg_t ra;
        int present_failed = 0;


        do {
            memset(&ra, 0, sizeof(ra));
            if (ioctl(fd, RASTERIZER_STATUS, &ra) < 0) {
                fprintf(stderr, "status poll (assert) failed: %s\n",
                        strerror(errno));
                present_failed = 1;
                break;
            }
        } while (!ra.status.swap_busy);


        while (!present_failed) {
            memset(&ra, 0, sizeof(ra));
            if (ioctl(fd, RASTERIZER_STATUS, &ra) < 0) {
                fprintf(stderr, "status poll (deassert) failed: %s\n",
                        strerror(errno));
                present_failed = 1;
                break;
            }
            if (!ra.status.swap_busy) break;

        }

        if (present_failed) { running = 0; break; }


        clock_gettime(CLOCK_MONOTONIC, &t_now);
        float dt = (t_now.tv_sec  - t_prev.tv_sec) +
                   (t_now.tv_nsec - t_prev.tv_nsec) * 1e-9f;
        t_prev = t_now;
        if (dt > 1e-6f) fps = 1.0f / dt;


        frame++;
        printf("\rFrame %ld | %s (%d tris) | FPS: %5.1f | "
               "rx=%.1f ry=%.1f dist=%.1f    ",
               frame, models[current_model].name,
               models[current_model].num_faces,
               fps, rot_x, rot_y, cam_dist);
        fflush(stdout);
    }

    printf("\nDone. %ld frames rendered.\n", frame);
    keyboard_close();
    if (g_regs) munmap((void *)g_regs, 4096);
    close(fd);
    return 0;
}
