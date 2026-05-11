// main.c
//
// Userspace demo for the DE1-SoC hardware rasterizer.
//
// Each frame:
//   1. Poll the USB HID keyboard for held/pressed keys.
//   2. Update rotation and camera state.
//   3. Build the MVP matrix, project all model vertices to screen space.
//   4. For each front-facing triangle call setup_triangle() to build a
//      triangle_packet_t, then submit it via ioctl(RASTERIZER_SUBMIT).
//
// The hardware takes over from there:
//   avalon_interface stages the 17-word packet -> triangle_fifo ->
//   triangle_dispatcher -> 16-way systolic pixel_unit chain ->
//   VGA framebuffer.
//
// Build (on DE1-SoC Linux):
//   gcc -Wall -O2 -o demo main.c geometry.c model.c usbkeyboard.c \
//       -I../kernel -lm -lusb-1.0
//
// Run:
//   sudo insmod avalon_kernel.ko
//   sudo ./demo [model.obj]
//
// Controls (USB keyboard plugged directly into DE1-SoC):
//   w/s     - tilt up/down (rotate X)
//   a/d     - rotate left/right (rotate Y)
//   +/-     - zoom in/out
//   1-0     - switch existing numbered models (unchanged mapping)
//   b/m/n   - switch to obj1/obj2/obj3 if present
//   [argv]  - optional OBJ loaded from command-line argument
//   r       - reset rotation and zoom
//   SPACE   - toggle auto-rotate
//   ESC/q   - quit

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
#include <libusb-1.0/libusb.h>

#include "../kernel/avalon_kernel.h"
#include "usbkeyboard.h"
#include "geometry.h"
#include "model.h"

// ---- constants ----

#define SCREEN_W    256
#define SCREEN_H    240
#define DEVICE      "/dev/rasterizer"

// ---- FIFO submission ----
//
// Spins on RASTERIZER_STATUS until the hardware FIFO has a free slot,
// then issues RASTERIZER_SUBMIT.  Uses a flat loop so the stack cannot
// overflow if the FIFO stays full for an extended period (the recursive
// version would overflow on a stalled hardware pipeline).
// Returns 0 on success, -1 on ioctl error.

static int submit_triangle(int fd, const triangle_packet_t *pkt)
{
    rasterizer_arg_t ra;
    memset(&ra, 0, sizeof(ra));
    ra.packet = *pkt;
    while (ioctl(fd, RASTERIZER_SUBMIT, &ra) != 0) {
        if (errno != EAGAIN) return -1;
    }
    return 0;
}

// ---- USB HID keyboard ----
//
// The keyboard is plugged directly into the DE1-SoC USB port and read
// via libusb, bypassing the PC entirely.  This gives raw HID scan codes
// with no key-repeat delay and up to 6 simultaneous keys -- the same
// information SDL_GetKeyboardState() provides.
//
// Each frame poll_keys() does one interrupt transfer to get the current
// HID report (8 bytes: modifiers + reserved + 6 keycodes).  It then
// builds two flat tables indexed by scan code:
//   g_keys[]        -- 1 if the key is held this frame
//   g_key_pressed[] -- 1 if this is the first frame the key appears
//                      (leading edge, equivalent to SDL_KEYDOWN)
//
// g_key_pressed lets one-shot actions (model switch, reset, quit) fire
// exactly once per physical keypress regardless of how long it is held.

static struct libusb_device_handle *g_keyboard = NULL;
static uint8_t                      g_endpoint  = 0;

static struct usb_keyboard_packet g_pkt_cur;   // HID report this frame
static struct usb_keyboard_packet g_pkt_prev;  // HID report last frame

#define SCAN_TABLE_SIZE 256
static int g_keys[SCAN_TABLE_SIZE];         // 1 = held this frame
static int g_key_pressed[SCAN_TABLE_SIZE];  // 1 = leading edge this frame

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

    g_pkt_prev = g_pkt_cur;  // save last frame for edge detection

    // non-blocking transfer: 16 ms timeout matches frame period.
    // only update g_pkt_cur when a full report arrives; on timeout the
    // previous report is kept so held keys remain set in g_keys[].
    int rc = libusb_interrupt_transfer(g_keyboard, g_endpoint,
                                       (unsigned char *)&g_pkt_cur,
                                       sizeof(g_pkt_cur),
                                       &transferred, 1);
    if (rc != 0 || transferred != (int)sizeof(g_pkt_cur))
        g_pkt_cur = g_pkt_prev;  // no new report: preserve last known state

    memset(g_keys,        0, sizeof(g_keys));
    memset(g_key_pressed, 0, sizeof(g_key_pressed));

    for (int i = 0; i < 6; i++) {
        uint8_t kc = g_pkt_cur.keycode[i];
        if (kc == 0) continue;

        g_keys[kc] = 1;

        // leading edge: present now but absent in previous packet
        int was_held = 0;
        for (int j = 0; j < 6; j++) {
            if (g_pkt_prev.keycode[j] == kc) { was_held = 1; break; }
        }
        if (!was_held)
            g_key_pressed[kc] = 1;
    }
}

// ---- vertex projection ----
//
// Transforms every model vertex through the MVP matrix into screen space.
// sx/sy are pixel coordinates; sz is depth in [0,1] (NDC z remapped).
// The w-divide guard prevents divide-by-zero for vertices exactly on the
// near plane.

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

        // NDC z in [-1,1] -> depth in [0,1]
        float sz = (ndcz + 1.0f) * 0.5f;
        if (sz < 0.0f) sz = 0.0f;
        if (sz > 1.0f) sz = 1.0f;
        g_sv[i].sz = sz;
    }
}

// ---- render one frame ----
//
// Builds the MVP matrix, projects vertices, then iterates over every
// face.  For each face: compute the flat world-space normal, shade it,
// call setup_triangle() to produce a triangle_packet_t, and submit it
// to the hardware FIFO.  Back-facing and degenerate triangles are
// skipped by setup_triangle() returning -1.
//
// Returns the number of triangles submitted, or -1 on ioctl error.

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

        // per-face flat normal: cross product of two edges in object space,
        // then rotated into world space (w=0 so translation is ignored)
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
            continue;  // back-face culled or degenerate

        if (submit_triangle(fd, &pkt) < 0) {
            fprintf(stderr, "submit_triangle failed: %s\n", strerror(errno));
            return -1;
        }
        submitted++;
    }
    return submitted;
}

// Try loading a named OBJ from a small set of clear relative paths.
// stem="obj1" will probe: obj1, obj1.obj, ./obj1, ./obj1.obj, ../obj1, ../obj1.obj
// Returns 0 on success and writes the winning path to loaded_path.
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
// OBJ files are expected in software/models/ (e.g. models/obj1.obj).
// Falls back to software/ itself (e.g. obj1.obj) if not found there.

// ---- main ----

int main(int argc, char *argv[])
{
    // ---- build model library ----

    enum {
        MODEL_CUBE = 0,
        MODEL_SPHERE_LO,
        MODEL_SPHERE_MED,
        MODEL_TORUS,
        MODEL_TEAPOT,
        MODEL_SATURN,
        MODEL_LEGO,
        MODEL_DNA,
        MODEL_TEAPOT_552,
        MODEL_MINIFIG,
        MODEL_ARGV_OBJ,
        MODEL_OBJ1,
        MODEL_OBJ2,
        MODEL_OBJ3,
        MODEL_COUNT
    };

    static model_t models[MODEL_COUNT];
    int num_models = 10;
    int obj1_loaded = 0, obj2_loaded = 0, obj3_loaded = 0;
    char obj1_path[128], obj2_path[128], obj3_path[128];

    model_make_cube(&models[MODEL_CUBE]);
    model_make_icosphere(&models[MODEL_SPHERE_LO], 1);       // 80 faces
    model_make_icosphere(&models[MODEL_SPHERE_MED], 2);      // 320 faces
    model_make_torus(&models[MODEL_TORUS], 12, 8, 0.7f, 0.3f);
    model_make_teapot(&models[MODEL_TEAPOT]);                // ~576 faces
    model_make_saturn(&models[MODEL_SATURN]);                // ~368 faces
    model_make_lego(&models[MODEL_LEGO]);                    // ~204 faces
    model_make_dna(&models[MODEL_DNA]);                      // ~300 faces
    model_make_teapot_552(&models[MODEL_TEAPOT_552]);        // 552 faces

    model_make_minifigure(&models[MODEL_MINIFIG]);           // ~170 faces

    if (argc >= 2) {
        if (model_load_obj(&models[MODEL_ARGV_OBJ], argv[1]) == 0) {
            printf("Loaded %s: %d verts, %d faces\n",
                   argv[1],
                   models[MODEL_ARGV_OBJ].num_verts,
                   models[MODEL_ARGV_OBJ].num_faces);
            num_models = 11;
        } else {
            fprintf(stderr, "Warning: could not load %s\n", argv[1]);
        }
    }

    // Extra direct OBJ bindings on letter keys (without touching number mappings).
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

    // ---- open rasterizer device ----

    int fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", DEVICE, strerror(errno));
        fprintf(stderr, "Is the kernel module loaded? "
                        "sudo insmod avalon_kernel.ko\n");
        return 1;
    }

    // ---- open USB keyboard ----

    if (keyboard_init() < 0) {
        close(fd);
        return 1;
    }

    // ---- print controls ----

    printf("\n=== CSEE 4840 Hardware Rasterizer Demo ===\n");
    printf("Controls (USB keyboard on DE1-SoC):\n");
    printf("  w/s     - tilt up/down\n");
    printf("  a/d     - rotate left/right\n");
    printf("  +/-     - zoom in/out\n");
    printf("  1       - cube\n");
    printf("  2       - sphere lo\n");
    printf("  3       - sphere med\n");
    printf("  4       - torus\n");
    printf("  5       - teapot\n");
    printf("  6       - saturn\n");
    printf("  7       - lego brick\n");
    printf("  8       - dna helix\n");
    printf("  9       - teapot_552\n");
    printf("  0       - minifigure\n");
    printf("  b       - obj1\n");
    printf("  m       - obj2\n");
    printf("  n       - obj3\n");
    if (num_models > 10) printf("  (OBJ)   - %s\n", models[MODEL_ARGV_OBJ].name);
    printf("OBJ key mapping:\n");
    printf("  b -> obj1: %s\n", obj1_loaded ? obj1_path : "NOT FOUND");
    printf("  m -> obj2: %s\n", obj2_loaded ? obj2_path : "NOT FOUND");
    printf("  n -> obj3: %s\n", obj3_loaded ? obj3_path : "NOT FOUND");
    printf("  r       - reset rotation\n");
    printf("  SPACE   - toggle auto-rotate\n");
    printf("  ESC/q   - quit\n");
    printf("==========================================\n\n");

    // ---- main loop ----

    int   current_model = 2;  // start with medium sphere
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
        // poll_keys() does one USB interrupt transfer and rebuilds
        // g_keys[] and g_key_pressed[] from the raw HID report
        poll_keys();

        // ---- continuous keys: apply every frame while held ----
        if (g_keys[KEY_W]) rot_x -= rot_speed;
        if (g_keys[KEY_S]) rot_x += rot_speed;
        if (g_keys[KEY_A]) rot_y -= rot_speed;
        if (g_keys[KEY_D]) rot_y += rot_speed;
        if (g_keys[KEY_EQUAL]) {  // = / + key, zoom in
            cam_dist -= 0.3f;
            if (cam_dist < 1.5f) cam_dist = 1.5f;
        }
        if (g_keys[KEY_MINUS]) {  // zoom out
            cam_dist += 0.3f;
            if (cam_dist > 15.0f) cam_dist = 15.0f;
        }

        // ---- one-shot keys: fire only on the leading edge ----
        if (g_key_pressed[KEY_ESC] || g_key_pressed[KEY_Q]) running = 0;
        if (g_key_pressed[KEY_1]) current_model = MODEL_CUBE;
        if (g_key_pressed[KEY_2]) current_model = MODEL_SPHERE_LO;
        if (g_key_pressed[KEY_3]) current_model = MODEL_SPHERE_MED;
        if (g_key_pressed[KEY_4]) current_model = MODEL_TORUS;
        if (g_key_pressed[KEY_5]) current_model = MODEL_TEAPOT;
        if (g_key_pressed[KEY_6]) current_model = MODEL_SATURN;
        if (g_key_pressed[KEY_7]) current_model = MODEL_LEGO;
        if (g_key_pressed[KEY_8]) current_model = MODEL_DNA;
        if (g_key_pressed[KEY_9]) current_model = MODEL_TEAPOT_552;
        if (g_key_pressed[KEY_0]) current_model = MODEL_MINIFIG;
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
        if (g_key_pressed[KEY_R]) {
            rot_x = 25.0f; rot_y = 45.0f; rot_z = 0.0f; cam_dist = 4.0f;
        }
        if (g_key_pressed[KEY_SPACE]) auto_rotate = !auto_rotate;

        if (auto_rotate) rot_y += 0.5f;

        // ---- submit triangles to hardware ----
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

        // ---- wait for hardware to finish swapping + clearing ----
        // STATUS bit [8] (swap_busy) is driven by the FPGA clock, so
        // there are a handful of HPS cycles between iowrite32() of
        // PRESENT returning and swap_busy actually going high. A
        // single-phase "poll until 0" loop would race that rising
        // edge -- the first STATUS read can land before the FPGA has
        // latched present_pending and exit immediately, letting
        // render_frame() of frame N+1 refill the FIFO before the swap
        // fires.
        //
        // Two-phase poll closes that race:
        //   Phase 1: spin until swap_busy == 1 (hardware acknowledged)
        //   Phase 2: spin until swap_busy == 0 (swap + clear complete)
        //
        // Phase 1 typically resolves in microseconds (the FPGA clock
        // is ~50 MHz vs. ioctl round-trip latency), so no sleep needed.
        // Phase 2 waits for a full VGA frame (~16 ms), so //usleep(100)
        // between polls keeps CPU usage in check.
        rasterizer_arg_t ra;
        int present_failed = 0;

        // Phase 1: wait for swap_busy to assert
        do {
            memset(&ra, 0, sizeof(ra));
            if (ioctl(fd, RASTERIZER_STATUS, &ra) < 0) {
                fprintf(stderr, "status poll (assert) failed: %s\n",
                        strerror(errno));
                present_failed = 1;
                break;
            }
        } while (!ra.status.swap_busy);

        // Phase 2: wait for swap_busy to deassert
        while (!present_failed) {
            memset(&ra, 0, sizeof(ra));
            if (ioctl(fd, RASTERIZER_STATUS, &ra) < 0) {
                fprintf(stderr, "status poll (deassert) failed: %s\n",
                        strerror(errno));
                present_failed = 1;
                break;
            }
            if (!ra.status.swap_busy) break;
            //usleep(100);
        }

        if (present_failed) { running = 0; break; }

        // measure frame time and compute FPS
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        float dt = (t_now.tv_sec  - t_prev.tv_sec) +
                   (t_now.tv_nsec - t_prev.tv_nsec) * 1e-9f;
        t_prev = t_now;
        if (dt > 1e-6f) fps = 1.0f / dt;

        // print status every frame (overwrite same line)
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
    close(fd);
    return 0;
}
