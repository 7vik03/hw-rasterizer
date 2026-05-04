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
//   1-4     - switch model: cube / sphere_lo / sphere_med / torus
//   5       - OBJ loaded from command-line argument (if any)
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
#include <sys/ioctl.h>
#include <libusb-1.0/libusb.h>

#include "../kernel/avalon_kernel.h"
#include "usbkeyboard.h"
#include "geometry.h"
#include "model.h"

// ---- constants ----

#define SCREEN_W    320
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

    for (;;) {
        // poll until FIFO has space
        memset(&ra, 0, sizeof(ra));
        if (ioctl(fd, RASTERIZER_STATUS, &ra) < 0) return -1;
        if (ra.status.fifo_full) {
            usleep(100);
            continue;
        }

        // attempt submit
        memset(&ra, 0, sizeof(ra));
        ra.packet = *pkt;
        if (ioctl(fd, RASTERIZER_SUBMIT, &ra) == 0)
            return 0;

        // EAGAIN: race between poll and submit, go around again
        if (errno == EAGAIN)
            continue;

        return -1;
    }
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

    // non-blocking transfer: 1 ms timeout returns whatever is ready now
    libusb_interrupt_transfer(g_keyboard, g_endpoint,
                              (unsigned char *)&g_pkt_cur,
                              sizeof(g_pkt_cur),
                              &transferred, 1);

    // no data this frame -> treat as all keys released
    if (transferred != (int)sizeof(g_pkt_cur))
        memset(&g_pkt_cur, 0, sizeof(g_pkt_cur));

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
    mat4_t model_mat = mat4_mul(&rz, &mat4_mul(&ry, &rx));
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

// ---- main ----

int main(int argc, char *argv[])
{
    // ---- build model library ----

    static model_t models[5];
    int num_models = 4;

    model_make_cube(&models[0]);
    model_make_icosphere(&models[1], 1);         // 80 faces
    model_make_icosphere(&models[2], 2);         // 320 faces
    model_make_torus(&models[3], 12, 8, 0.7f, 0.3f);

    if (argc >= 2) {
        if (model_load_obj(&models[4], argv[1]) == 0) {
            printf("Loaded %s: %d verts, %d faces\n",
                   argv[1], models[4].num_verts, models[4].num_faces);
            num_models = 5;
        } else {
            fprintf(stderr, "Warning: could not load %s\n", argv[1]);
        }
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
    printf("  1-4     - cube / sphere_lo / sphere_med / torus\n");
    if (num_models > 4) printf("  5       - %s\n", models[4].name);
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
        if (g_key_pressed[KEY_1]) current_model = 0;
        if (g_key_pressed[KEY_2]) current_model = 1;
        if (g_key_pressed[KEY_3]) current_model = 2;
        if (g_key_pressed[KEY_4]) current_model = 3;
        if (g_key_pressed[KEY_5] && num_models > 4) current_model = 4;
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

        // print status every 60 frames
        frame++;
        if (frame % 60 == 0) {
            printf("\rFrame %ld | model=%s (%d tris) | "
                   "rx=%.1f ry=%.1f dist=%.1f    ",
                   frame, models[current_model].name,
                   models[current_model].num_faces,
                   rot_x, rot_y, cam_dist);
            fflush(stdout);
        }

        // 16 ms pacing delay (~60 FPS cap); hardware renders the medium
        // sphere in ~1.6 ms so this is purely a throttle
        usleep(16000);
    }

    printf("\nDone. %ld frames rendered.\n", frame);
    keyboard_close();
    close(fd);
    return 0;
}
