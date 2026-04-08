#include <map>
// ============================================================================
// Interactive Golden Reference Rasterizer
// CSEE 4840 - 3D Hardware Rasterizer Project
//
// Real-time interactive demo matching the final hardware output exactly.
// Arrow keys rotate the object. Number keys switch models.
// 320x240 internal resolution, doubled to 640x480 window (same as VGA output).
// Pineda edge equation rasterization, RGB332 color, 16-bit Z-buffer.
//
// Build (macOS):
//   g++ -O2 -std=c++17 -o demo demo.cpp \
//       $(sdl2-config --cflags --libs) -lm
//
// Build (Linux):
//   g++ -O2 -std=c++17 -o demo demo.cpp \
//       $(sdl2-config --cflags --libs) -lm
//
// Run:
//   ./demo [model.obj]
//
// Controls:
//   Arrow keys  - rotate object
//   1,2,3,4     - switch model (cube, sphere_lo, sphere_med, torus)
//   +/-         - zoom in/out
//   R           - reset rotation
//   Q/ESC       - quit
// ============================================================================

#include <SDL.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>

// ============================================================================
// Constants matching hardware spec
// ============================================================================
static const int SCREEN_W = 320;
static const int SCREEN_H = 240;
static const int WINDOW_W = 640;
static const int WINDOW_H = 480;
static const int FRAC_BITS = 12;
static const int FIXED_ONE = (1 << FRAC_BITS);
static const int DEPTH_MAX = 65535;

// ============================================================================
// Vector / Matrix math (floating point — runs on ARM in final system)
// ============================================================================
struct Vec3f {
    float x, y, z;
    Vec3f() : x(0), y(0), z(0) {}
    Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3f operator-(const Vec3f& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3f operator+(const Vec3f& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3f operator*(float s) const { return {x*s, y*s, z*s}; }
    float dot(const Vec3f& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3f cross(const Vec3f& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    Vec3f normalize() const {
        float l = length();
        if (l < 1e-8f) return {0,0,0};
        return {x/l, y/l, z/l};
    }
};

struct Vec4f {
    float x, y, z, w;
    Vec4f() : x(0), y(0), z(0), w(0) {}
    Vec4f(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
};

struct Mat4f {
    float m[4][4] = {};
    static Mat4f identity() {
        Mat4f r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
        return r;
    }
    Mat4f operator*(const Mat4f& b) const {
        Mat4f r;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                r.m[i][j] = 0;
                for (int k = 0; k < 4; k++)
                    r.m[i][j] += m[i][k] * b.m[k][j];
            }
        return r;
    }
    Vec4f operator*(const Vec4f& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
            m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w
        };
    }
};

Mat4f rotation_x(float deg) {
    float r = deg * M_PI / 180.0f;
    Mat4f m = Mat4f::identity();
    m.m[1][1] = cosf(r); m.m[1][2] = -sinf(r);
    m.m[2][1] = sinf(r); m.m[2][2] = cosf(r);
    return m;
}
Mat4f rotation_y(float deg) {
    float r = deg * M_PI / 180.0f;
    Mat4f m = Mat4f::identity();
    m.m[0][0] = cosf(r); m.m[0][2] = sinf(r);
    m.m[2][0] = -sinf(r); m.m[2][2] = cosf(r);
    return m;
}
Mat4f rotation_z(float deg) {
    float r = deg * M_PI / 180.0f;
    Mat4f m = Mat4f::identity();
    m.m[0][0] = cosf(r); m.m[0][1] = -sinf(r);
    m.m[1][0] = sinf(r); m.m[1][1] = cosf(r);
    return m;
}
Mat4f translation(float tx, float ty, float tz) {
    Mat4f m = Mat4f::identity();
    m.m[0][3] = tx; m.m[1][3] = ty; m.m[2][3] = tz;
    return m;
}
Mat4f perspective(float fov_deg, float aspect, float near, float far) {
    float f = 1.0f / tanf(fov_deg * M_PI / 360.0f);
    Mat4f m;
    m.m[0][0] = f / aspect;
    m.m[1][1] = f;
    m.m[2][2] = (far + near) / (near - far);
    m.m[2][3] = (2.0f * far * near) / (near - far);
    m.m[3][2] = -1.0f;
    return m;
}

// ============================================================================
// Model
// ============================================================================
struct Triangle { int v[3]; };

struct Model {
    std::vector<Vec3f> vertices;
    std::vector<Triangle> faces;
    std::string name;

    bool load_obj(const char* filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;
            if (prefix == "v") {
                Vec3f v; iss >> v.x >> v.y >> v.z;
                vertices.push_back(v);
            } else if (prefix == "f") {
                Triangle tri;
                for (int i = 0; i < 3; i++) {
                    std::string token; iss >> token;
                    tri.v[i] = std::stoi(token.substr(0, token.find('/'))) - 1;
                }
                faces.push_back(tri);
                std::string token4;
                if (iss >> token4) {
                    Triangle tri2;
                    tri2.v[0] = tri.v[0];
                    tri2.v[1] = tri.v[2];
                    tri2.v[2] = std::stoi(token4.substr(0, token4.find('/'))) - 1;
                    faces.push_back(tri2);
                }
            }
        }
        name = filename;
        return !vertices.empty() && !faces.empty();
    }

    void make_cube() {
        name = "cube";
        vertices = {
            {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
            {-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}
        };
        faces = {
            {{0,1,2}},{{0,2,3}},{{5,4,7}},{{5,7,6}},
            {{4,0,3}},{{4,3,7}},{{1,5,6}},{{1,6,2}},
            {{3,2,6}},{{3,6,7}},{{4,5,1}},{{4,1,0}}
        };
    }

    // Generate icosphere
    void make_icosphere(int subdivisions) {
        name = "sphere";
        float t = (1.0f + sqrtf(5.0f)) / 2.0f;
        vertices = {
            {-1,t,0},{1,t,0},{-1,-t,0},{1,-t,0},
            {0,-1,t},{0,1,t},{0,-1,-t},{0,1,-t},
            {t,0,-1},{t,0,1},{-t,0,-1},{-t,0,1}
        };
        for (auto& v : vertices) {
            float l = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
            v.x /= l; v.y /= l; v.z /= l;
        }
        faces = {{
            {{0,11,5}},{{0,5,1}},{{0,1,7}},{{0,7,10}},{{0,10,11}},
            {{1,5,9}},{{5,11,4}},{{11,10,2}},{{10,7,6}},{{7,1,8}},
            {{3,9,4}},{{3,4,2}},{{3,2,6}},{{3,6,8}},{{3,8,9}},
            {{4,9,5}},{{2,4,11}},{{6,2,10}},{{8,6,7}},{{9,8,1}}
        }};
        for (int s = 0; s < subdivisions; s++) {
            std::vector<Triangle> new_faces;
            std::map<std::pair<int,int>,int> cache;
            auto midpoint = [&](int a, int b) -> int {
                auto key = std::make_pair(std::min(a,b), std::max(a,b));
                auto it = cache.find(key);
                if (it != cache.end()) return it->second;
                Vec3f mid = {
                    (vertices[a].x+vertices[b].x)*0.5f,
                    (vertices[a].y+vertices[b].y)*0.5f,
                    (vertices[a].z+vertices[b].z)*0.5f
                };
                float l = sqrtf(mid.x*mid.x+mid.y*mid.y+mid.z*mid.z);
                mid.x/=l; mid.y/=l; mid.z/=l;
                int idx = vertices.size();
                vertices.push_back(mid);
                cache[key] = idx;
                return idx;
            };
            for (auto& f : faces) {
                int a = midpoint(f.v[0], f.v[1]);
                int b = midpoint(f.v[1], f.v[2]);
                int c = midpoint(f.v[2], f.v[0]);
                new_faces.push_back({{f.v[0],a,c}});
                new_faces.push_back({{f.v[1],b,a}});
                new_faces.push_back({{f.v[2],c,b}});
                new_faces.push_back({{a,b,c}});
            }
            faces = new_faces;
        }
    }

    void make_torus(int major_seg, int minor_seg, float R, float r) {
        name = "torus";
        for (int i = 0; i < major_seg; i++) {
            float theta = 2.0f * M_PI * i / major_seg;
            for (int j = 0; j < minor_seg; j++) {
                float phi = 2.0f * M_PI * j / minor_seg;
                float x = (R + r*cosf(phi)) * cosf(theta);
                float y = r * sinf(phi);
                float z = (R + r*cosf(phi)) * sinf(theta);
                vertices.push_back({x, y, z});
            }
        }
        for (int i = 0; i < major_seg; i++) {
            int ni = (i+1) % major_seg;
            for (int j = 0; j < minor_seg; j++) {
                int nj = (j+1) % minor_seg;
                int a = i*minor_seg+j, b = ni*minor_seg+j;
                int c = ni*minor_seg+nj, d = i*minor_seg+nj;
                faces.push_back({{a,b,c}});
                faces.push_back({{a,c,d}});
            }
        }
    }
};

// ============================================================================
// RGB332 color (matches hardware framebuffer format exactly)
// ============================================================================
struct Color332 {
    uint8_t val;
    Color332() : val(0) {}
    Color332(uint8_t v) : val(v) {}
    static Color332 from_float(float r, float g, float b) {
        int ri = std::max(0, std::min(7, (int)(r * 7.0f + 0.5f)));
        int gi = std::max(0, std::min(7, (int)(g * 7.0f + 0.5f)));
        int bi = std::max(0, std::min(3, (int)(b * 3.0f + 0.5f)));
        return Color332((ri << 5) | (gi << 2) | bi);
    }
    uint32_t to_argb32() const {
        int ri = (val >> 5) & 0x7;
        int gi = (val >> 2) & 0x7;
        int bi = val & 0x3;
        uint8_t r = (ri * 255) / 7;
        uint8_t g = (gi * 255) / 7;
        uint8_t b = (bi * 255) / 3;
        return 0xFF000000 | (r << 16) | (g << 8) | b;
    }
};

// ============================================================================
// Framebuffer + Z-buffer (matches hardware exactly)
// ============================================================================
struct Framebuffer {
    Color332 color[SCREEN_H][SCREEN_W];
    uint16_t depth[SCREEN_H][SCREEN_W];

    void clear() {
        memset(color, 0, sizeof(color));
        for (int y = 0; y < SCREEN_H; y++)
            for (int x = 0; x < SCREEN_W; x++)
                depth[y][x] = DEPTH_MAX;
    }

    // Blit to SDL surface (2x pixel doubling, same as VGA output)
    void blit_to_surface(SDL_Surface* surface) {
        uint32_t* pixels = (uint32_t*)surface->pixels;
        int pitch = surface->pitch / 4;
        for (int y = 0; y < SCREEN_H; y++) {
            for (int x = 0; x < SCREEN_W; x++) {
                uint32_t c = color[y][x].to_argb32();
                // 2x2 pixel doubling
                int dx = x * 2, dy = y * 2;
                pixels[dy * pitch + dx] = c;
                pixels[dy * pitch + dx + 1] = c;
                pixels[(dy+1) * pitch + dx] = c;
                pixels[(dy+1) * pitch + dx + 1] = c;
            }
        }
    }
};

// ============================================================================
// Screen vertex (output of software transform pipeline)
// ============================================================================
struct ScreenVertex {
    float sx, sy, sz;
};

// ============================================================================
// Triangle packet (what software sends to hardware FIFO)
// ============================================================================
struct TrianglePacket {
    int32_t a0,b0,c0, a1,b1,c1, a2,b2,c2;
    int bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax;
    int32_t z_at_origin, z_step_x, z_step_y;
    uint8_t color;
    bool front_facing;
};

// ============================================================================
// SOFTWARE: Lighting
// ============================================================================
Vec3f face_normal(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2) {
    return (v1-v0).cross(v2-v0).normalize();
}

Color332 shade_face(const Vec3f& normal, const Vec3f& light_dir,
                    float base_r, float base_g, float base_b) {
    float ndotl = std::max(0.0f, normal.dot(light_dir));
    float ambient = 0.25f;
    float intensity = std::min(1.0f, (ambient + (1.0f - ambient) * ndotl) * 1.2f);
    return Color332::from_float(base_r * intensity, base_g * intensity, base_b * intensity);
}

int32_t to_fixed(float v) {
    return (int32_t)(v * FIXED_ONE + (v >= 0 ? 0.5f : -0.5f));
}

// ============================================================================
// SOFTWARE: Triangle setup (computes FIFO packet)
// ============================================================================
TrianglePacket setup_triangle(const ScreenVertex& v0, const ScreenVertex& v1,
                               const ScreenVertex& v2, Color332 color) {
    TrianglePacket pkt;

    float a0f = v1.sy - v2.sy, b0f = v2.sx - v1.sx;
    float c0f = v1.sx*v2.sy - v2.sx*v1.sy;
    float a1f = v2.sy - v0.sy, b1f = v0.sx - v2.sx;
    float c1f = v2.sx*v0.sy - v0.sx*v2.sy;
    float a2f = v0.sy - v1.sy, b2f = v1.sx - v0.sx;
    float c2f = v0.sx*v1.sy - v1.sx*v0.sy;

    float area = a0f*v0.sx + b0f*v0.sy + c0f;
    pkt.front_facing = (area > 0);

    if (area < 0) {
        a0f=-a0f; b0f=-b0f; c0f=-c0f;
        a1f=-a1f; b1f=-b1f; c1f=-c1f;
        a2f=-a2f; b2f=-b2f; c2f=-c2f;
        area = -area;
    }

    pkt.bbox_xmin = std::max(0, (int)floorf(std::min({v0.sx,v1.sx,v2.sx})));
    pkt.bbox_ymin = std::max(0, (int)floorf(std::min({v0.sy,v1.sy,v2.sy})));
    pkt.bbox_xmax = std::min(SCREEN_W-1, (int)ceilf(std::max({v0.sx,v1.sx,v2.sx})));
    pkt.bbox_ymax = std::min(SCREEN_H-1, (int)ceilf(std::max({v0.sy,v1.sy,v2.sy})));

    pkt.a0 = to_fixed(a0f); pkt.b0 = to_fixed(b0f);
    pkt.a1 = to_fixed(a1f); pkt.b1 = to_fixed(b1f);
    pkt.a2 = to_fixed(a2f); pkt.b2 = to_fixed(b2f);

    float px = pkt.bbox_xmin + 0.5f, py = pkt.bbox_ymin + 0.5f;
    pkt.c0 = to_fixed(a0f*px + b0f*py + c0f);
    pkt.c1 = to_fixed(a1f*px + b1f*py + c1f);
    pkt.c2 = to_fixed(a2f*px + b2f*py + c2f);

    float zs0 = v0.sz*65535.0f, zs1 = v1.sz*65535.0f, zs2 = v2.sz*65535.0f;
    if (area > 1e-6f) {
        pkt.z_step_x = to_fixed((a0f*zs0 + a1f*zs1 + a2f*zs2) / area);
        pkt.z_step_y = to_fixed((b0f*zs0 + b1f*zs1 + b2f*zs2) / area);
        pkt.z_at_origin = to_fixed(
            (pkt.c0/(float)FIXED_ONE*zs0 + pkt.c1/(float)FIXED_ONE*zs1 +
             pkt.c2/(float)FIXED_ONE*zs2) / area);
    } else {
        pkt.z_at_origin = pkt.z_step_x = pkt.z_step_y = 0;
    }

    pkt.color = color.val;
    return pkt;
}

// ============================================================================
// HARDWARE: Rasterizer (integer arithmetic only — matches SystemVerilog)
// ============================================================================
void rasterize_triangle(const TrianglePacket& pkt, Framebuffer& fb) {
    if (!pkt.front_facing) return;
    if (pkt.bbox_xmin > pkt.bbox_xmax || pkt.bbox_ymin > pkt.bbox_ymax) return;

    int32_t e0_row = pkt.c0, e1_row = pkt.c1, e2_row = pkt.c2;
    int32_t z_row = pkt.z_at_origin;

    for (int y = pkt.bbox_ymin; y <= pkt.bbox_ymax; y++) {
        int32_t e0 = e0_row, e1 = e1_row, e2 = e2_row;
        int32_t z = z_row;

        for (int x = pkt.bbox_xmin; x <= pkt.bbox_xmax; x++) {
            if (e0 >= 0 && e1 >= 0 && e2 >= 0) {
                uint16_t depth_16 = (uint16_t)std::max(0,
                    std::min(65535, (int)(z >> FRAC_BITS)));
                if (depth_16 < fb.depth[y][x]) {
                    fb.depth[y][x] = depth_16;
                    fb.color[y][x] = Color332(pkt.color);
                }
            }
            e0 += pkt.a0; e1 += pkt.a1; e2 += pkt.a2;
            z += pkt.z_step_x;
        }
        e0_row += pkt.b0; e1_row += pkt.b1; e2_row += pkt.b2;
        z_row += pkt.z_step_y;
    }
}

// ============================================================================
// Render a full frame
// ============================================================================
void render_frame(const Model& model, Framebuffer& fb,
                  float rot_x, float rot_y, float rot_z, float cam_dist) {
    fb.clear();

    float aspect = (float)SCREEN_W / (float)SCREEN_H;
    Mat4f proj = perspective(60.0f, aspect, 0.1f, 100.0f);
    Mat4f view = translation(0.0f, 0.0f, -cam_dist);
    Mat4f model_mat = rotation_z(rot_z) * rotation_y(rot_y) * rotation_x(rot_x);
    Mat4f mvp = proj * view * model_mat;

    Vec3f light_dir = Vec3f(0.4f, 0.7f, 0.5f).normalize();
    float base_r = 0.2f, base_g = 0.7f, base_b = 1.0f;

    // Transform vertices
    std::vector<ScreenVertex> sv(model.vertices.size());
    for (size_t i = 0; i < model.vertices.size(); i++) {
        const Vec3f& v = model.vertices[i];
        Vec4f clip = mvp * Vec4f(v.x, v.y, v.z, 1.0f);
        if (fabsf(clip.w) < 1e-6f) clip.w = 1e-6f;
        float ndcx = clip.x/clip.w, ndcy = clip.y/clip.w, ndcz = clip.z/clip.w;
        sv[i].sx = (ndcx+1.0f) * 0.5f * SCREEN_W;
        sv[i].sy = (1.0f-ndcy) * 0.5f * SCREEN_H;
        sv[i].sz = std::max(0.0f, std::min(1.0f, (ndcz+1.0f)*0.5f));
    }

    // Rasterize each triangle
    for (size_t i = 0; i < model.faces.size(); i++) {
        const Triangle& f = model.faces[i];

        // Per-face lighting (software)
        Vec3f normal = face_normal(model.vertices[f.v[0]],
                                    model.vertices[f.v[1]],
                                    model.vertices[f.v[2]]);
        Vec4f tn = model_mat * Vec4f(normal.x, normal.y, normal.z, 0.0f);
        Vec3f wn = Vec3f(tn.x, tn.y, tn.z).normalize();
        Color332 color = shade_face(wn, light_dir, base_r, base_g, base_b);

        // Triangle setup (software)
        TrianglePacket pkt = setup_triangle(sv[f.v[0]], sv[f.v[1]], sv[f.v[2]], color);

        // Rasterize (hardware)
        rasterize_triangle(pkt, fb);
    }
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    // Load models
    std::vector<Model> models(4);

    // Model 0: cube
    models[0].make_cube();

    // Model 1: low-poly sphere (80 faces)
    models[1].make_icosphere(1);

    // Model 2: medium sphere (320 faces)
    models[2].make_icosphere(2);

    // Model 3: torus (192 faces)
    models[3].make_torus(12, 8, 0.7f, 0.3f);

    // Try to load OBJ from command line as model 4
    if (argc >= 2) {
        Model custom;
        if (custom.load_obj(argv[1])) {
            printf("Loaded %s: %zu verts, %zu faces\n",
                   argv[1], custom.vertices.size(), custom.faces.size());
            models.push_back(custom);
        }
    }

    int current_model = 2; // start with medium sphere

    // Init SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "CSEE 4840 - 3D Hardware Rasterizer Golden Reference",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_W, WINDOW_H, 0);
    if (!window) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Surface* surface = SDL_GetWindowSurface(window);
    Framebuffer fb;

    float rot_x = 25.0f, rot_y = 45.0f, rot_z = 0.0f;
    float cam_dist = 4.0f;
    float rot_speed = 2.0f;
    bool running = true;
    bool auto_rotate = false;

    printf("\n=== CSEE 4840 Rasterizer Golden Reference ===\n");
    printf("Controls:\n");
    printf("  Arrow keys   - rotate object\n");
    printf("  1,2,3,4      - cube, sphere(80), sphere(320), torus\n");
    if (models.size() > 4) printf("  5            - %s\n", models[4].name.c_str());
    printf("  +/-          - zoom in/out\n");
    printf("  Space        - toggle auto-rotate\n");
    printf("  R            - reset rotation\n");
    printf("  Q/ESC        - quit\n");
    printf("=============================================\n\n");

    Uint32 last_time = SDL_GetTicks();
    int frame_count = 0;
    float fps = 0;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE: case SDLK_q: running = false; break;
                    case SDLK_1: current_model = 0; break;
                    case SDLK_2: current_model = 1; break;
                    case SDLK_3: current_model = 2; break;
                    case SDLK_4: current_model = 3; break;
                    case SDLK_5: if (models.size() > 4) current_model = 4; break;
                    case SDLK_r: rot_x = 25; rot_y = 45; rot_z = 0; cam_dist = 4; break;
                    case SDLK_SPACE: auto_rotate = !auto_rotate; break;
                    case SDLK_EQUALS: case SDLK_PLUS: cam_dist = std::max(1.5f, cam_dist - 0.3f); break;
                    case SDLK_MINUS: cam_dist = std::min(15.0f, cam_dist + 0.3f); break;
                }
            }
        }

        // Continuous key input for smooth rotation
        const Uint8* keys = SDL_GetKeyboardState(NULL);
        if (keys[SDL_SCANCODE_UP])    rot_x -= rot_speed;
        if (keys[SDL_SCANCODE_DOWN])  rot_x += rot_speed;
        if (keys[SDL_SCANCODE_LEFT])  rot_y -= rot_speed;
        if (keys[SDL_SCANCODE_RIGHT]) rot_y += rot_speed;

        if (auto_rotate) rot_y += 0.5f;

        // Render
        render_frame(models[current_model], fb, rot_x, rot_y, rot_z, cam_dist);

        // Blit to window (2x doubling, same as VGA output)
        SDL_LockSurface(surface);
        fb.blit_to_surface(surface);
        SDL_UnlockSurface(surface);
        SDL_UpdateWindowSurface(window);

        // FPS counter
        frame_count++;
        Uint32 now = SDL_GetTicks();
        if (now - last_time >= 1000) {
            fps = frame_count * 1000.0f / (now - last_time);
            char title[256];
            snprintf(title, sizeof(title),
                     "CSEE 4840 Rasterizer | %s (%zu tris) | %.0f FPS | RGB332 320x240",
                     models[current_model].name.c_str(),
                     models[current_model].faces.size(), fps);
            SDL_SetWindowTitle(window, title);
            frame_count = 0;
            last_time = now;
        }

        SDL_Delay(16); // ~60 fps cap
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
