// ============================================================================
// Golden Reference Software Rasterizer
// CSEE 4840 - 3D Hardware Rasterizer Project
//
// Implements the exact Pineda edge equation algorithm that the FPGA hardware
// will replicate. Everything after the "SOFTWARE/HARDWARE BOUNDARY" comment
// uses only integer arithmetic (addition + comparison) in the inner loop,
// matching the SystemVerilog implementation.
//
// Output: 320x240 PPM image, RGB332 color format
// Usage:  ./rasterizer [model.obj] [rotation_x] [rotation_y] [rotation_z]
// ============================================================================

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>
#include <cstdint>

// ============================================================================
// Constants matching hardware spec
// ============================================================================
static const int SCREEN_W = 320;
static const int SCREEN_H = 240;

// Fixed-point format: Q12.12 (12 integer bits, 12 fractional bits)
// Range: -2048.0 to +2047.999755859375
// Precision: 1/4096 = 0.000244140625
static const int FRAC_BITS = 12;
static const int FIXED_ONE = (1 << FRAC_BITS);

// Depth buffer: 16-bit unsigned, range [0, 65535]
static const int DEPTH_MAX = 65535;

// ============================================================================
// Vector / Matrix math (floating point — runs on ARM in software)
// ============================================================================
struct Vec3f {
    float x, y, z;
    Vec3f() : x(0), y(0), z(0) {}
    Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3f operator-(const Vec3f& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3f operator+(const Vec3f& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3f operator*(float s) const { return {x * s, y * s, z * s}; }
    float dot(const Vec3f& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3f cross(const Vec3f& v) const {
        return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
    }
    float length() const { return sqrtf(x * x + y * y + z * z); }
    Vec3f normalize() const {
        float l = length();
        if (l < 1e-8f) return {0, 0, 0};
        return {x / l, y / l, z / l};
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

Mat4f rotation_x(float angle_deg) {
    float r = angle_deg * M_PI / 180.0f;
    Mat4f m = Mat4f::identity();
    m.m[1][1] = cosf(r);  m.m[1][2] = -sinf(r);
    m.m[2][1] = sinf(r);  m.m[2][2] =  cosf(r);
    return m;
}

Mat4f rotation_y(float angle_deg) {
    float r = angle_deg * M_PI / 180.0f;
    Mat4f m = Mat4f::identity();
    m.m[0][0] =  cosf(r); m.m[0][2] = sinf(r);
    m.m[2][0] = -sinf(r); m.m[2][2] = cosf(r);
    return m;
}

Mat4f rotation_z(float angle_deg) {
    float r = angle_deg * M_PI / 180.0f;
    Mat4f m = Mat4f::identity();
    m.m[0][0] = cosf(r);  m.m[0][1] = -sinf(r);
    m.m[1][0] = sinf(r);  m.m[1][1] =  cosf(r);
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
// OBJ file loader
// ============================================================================
struct Triangle {
    int v[3]; // vertex indices
};

struct Model {
    std::vector<Vec3f> vertices;
    std::vector<Triangle> faces;

    bool load_obj(const char* filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;
            if (prefix == "v") {
                Vec3f v;
                iss >> v.x >> v.y >> v.z;
                vertices.push_back(v);
            } else if (prefix == "f") {
                // Handle f v, f v/vt, f v/vt/vn, f v//vn
                Triangle tri;
                for (int i = 0; i < 3; i++) {
                    std::string token;
                    iss >> token;
                    // Extract vertex index (before first /)
                    int vi = std::stoi(token.substr(0, token.find('/')));
                    tri.v[i] = vi - 1; // OBJ is 1-indexed
                }
                faces.push_back(tri);
                // Handle quads: if there's a 4th vertex, split into two triangles
                std::string token4;
                if (iss >> token4) {
                    Triangle tri2;
                    int vi4 = std::stoi(token4.substr(0, token4.find('/')));
                    tri2.v[0] = tri.v[0];
                    tri2.v[1] = tri.v[2];
                    tri2.v[2] = vi4 - 1;
                    faces.push_back(tri2);
                }
            }
        }
        return !vertices.empty() && !faces.empty();
    }

    // Generate a default cube if no OBJ file
    void make_cube() {
        vertices = {
            {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1},
            {-1, -1,  1}, { 1, -1,  1}, { 1,  1,  1}, {-1,  1,  1}
        };
        faces = {
            // Front
            {{0, 1, 2}}, {{0, 2, 3}},
            // Back
            {{5, 4, 7}}, {{5, 7, 6}},
            // Left
            {{4, 0, 3}}, {{4, 3, 7}},
            // Right
            {{1, 5, 6}}, {{1, 6, 2}},
            // Top
            {{3, 2, 6}}, {{3, 6, 7}},
            // Bottom
            {{4, 5, 1}}, {{4, 1, 0}}
        };
    }
};

// ============================================================================
// RGB332 color encoding (matches hardware framebuffer format)
// ============================================================================
struct Color332 {
    uint8_t val; // RRRGGGBB

    Color332() : val(0) {}
    Color332(uint8_t v) : val(v) {}

    // Create from float RGB [0,1]
    static Color332 from_float(float r, float g, float b) {
        int ri = std::max(0, std::min(7, (int)(r * 7.0f + 0.5f)));
        int gi = std::max(0, std::min(7, (int)(g * 7.0f + 0.5f)));
        int bi = std::max(0, std::min(3, (int)(b * 3.0f + 0.5f)));
        return Color332((ri << 5) | (gi << 2) | bi);
    }

    // Expand to 24-bit RGB for PPM output
    void to_rgb24(uint8_t& r, uint8_t& g, uint8_t& b) const {
        int ri = (val >> 5) & 0x7;
        int gi = (val >> 2) & 0x7;
        int bi = val & 0x3;
        r = (ri * 255) / 7;
        g = (gi * 255) / 7;
        b = (bi * 255) / 3;
    }
};

// ============================================================================
// Framebuffer and Z-buffer (matches hardware memory layout)
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

    void write_ppm(const char* filename) const {
        FILE* f = fopen(filename, "wb");
        fprintf(f, "P6\n%d %d\n255\n", SCREEN_W, SCREEN_H);
        for (int y = 0; y < SCREEN_H; y++)
            for (int x = 0; x < SCREEN_W; x++) {
                uint8_t r, g, b;
                color[y][x].to_rgb24(r, g, b);
                fputc(r, f); fputc(g, f); fputc(b, f);
            }
        fclose(f);
        printf("Wrote %s\n", filename);
    }
};

// ============================================================================
// Projected vertex (output of software pipeline, input to hardware)
// ============================================================================
struct ScreenVertex {
    float sx, sy; // screen coordinates (floating point, from software)
    float sz;     // depth [0, 1] for z-buffer
};

// ============================================================================
// Triangle packet — this is exactly what software sends to the FIFO
// This struct represents the hardware interface
// ============================================================================
struct TrianglePacket {
    // Edge equation coefficients (Q12.12 fixed point)
    // For edge i: E_i(x,y) = a_i * x + b_i * y + c_i
    // Stepping: E_i(x+1, y) = E_i(x,y) + a_i
    //           E_i(x, y+1) = E_i(x,y) + b_i
    int32_t a0, b0, c0; // Edge 0: v1 -> v2
    int32_t a1, b1, c1; // Edge 1: v2 -> v0
    int32_t a2, b2, c2; // Edge 2: v0 -> v1

    // Bounding box (integer screen coordinates)
    int bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax;

    // Depth interpolation (Q12.12 fixed point)
    int32_t z_at_origin; // Z value at (bbox_xmin, bbox_ymin)
    int32_t z_step_x;    // dZ per pixel step in x
    int32_t z_step_y;    // dZ per pixel step in y

    // Color (RGB332)
    uint8_t color;

    // Triangle area sign (for back-face culling)
    bool front_facing;
};

// ============================================================================
// SOFTWARE SIDE: Transform pipeline (runs on ARM, floating point)
// ============================================================================

// Compute face normal from 3 vertices
Vec3f face_normal(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2) {
    Vec3f e1 = v1 - v0;
    Vec3f e2 = v2 - v0;
    return e1.cross(e2).normalize();
}

// Compute flat shading color from face normal and light direction
Color332 shade_face(const Vec3f& normal, const Vec3f& light_dir, 
                    float base_r, float base_g, float base_b) {
    float ndotl = std::max(0.0f, normal.dot(light_dir));
    // Ambient + diffuse
    float ambient = 0.25f;
    float intensity = ambient + (1.0f - ambient) * ndotl;
    // Boost to use more of the RGB332 range
    intensity = std::min(1.0f, intensity * 1.2f);
    return Color332::from_float(base_r * intensity, 
                                 base_g * intensity, 
                                 base_b * intensity);
}

// Convert floating-point to Q12.12 fixed point
int32_t to_fixed(float v) {
    return (int32_t)(v * FIXED_ONE + (v >= 0 ? 0.5f : -0.5f));
}

// ============================================================================
// SOFTWARE SIDE: Triangle setup (runs on ARM, computes packet for FIFO)
// ============================================================================
TrianglePacket setup_triangle(const ScreenVertex& v0, 
                               const ScreenVertex& v1, 
                               const ScreenVertex& v2,
                               Color332 color) {
    TrianglePacket pkt;

    // Edge equations in floating point first, then convert to fixed point
    // Edge 0: v1 -> v2
    float a0f = v1.sy - v2.sy;
    float b0f = v2.sx - v1.sx;
    float c0f = v1.sx * v2.sy - v2.sx * v1.sy;

    // Edge 1: v2 -> v0
    float a1f = v2.sy - v0.sy;
    float b1f = v0.sx - v2.sx;
    float c1f = v2.sx * v0.sy - v0.sx * v2.sy;

    // Edge 2: v0 -> v1
    float a2f = v0.sy - v1.sy;
    float b2f = v1.sx - v0.sx;
    float c2f = v0.sx * v1.sy - v1.sx * v0.sy;

    // Triangle area (2x signed area) — used for back-face culling and 
    // barycentric normalization
    float area = a0f * v0.sx + b0f * v0.sy + c0f;

    // Back-face culling: if area is negative, triangle faces away
    pkt.front_facing = (area > 0);

    // If back-facing, flip all edge equations so the inside test still works
    // (all three must be >= 0)
    if (area < 0) {
        a0f = -a0f; b0f = -b0f; c0f = -c0f;
        a1f = -a1f; b1f = -b1f; c1f = -c1f;
        a2f = -a2f; b2f = -b2f; c2f = -c2f;
        area = -area;
    }

    // Bounding box (clamped to screen)
    float minx = std::min({v0.sx, v1.sx, v2.sx});
    float miny = std::min({v0.sy, v1.sy, v2.sy});
    float maxx = std::max({v0.sx, v1.sx, v2.sx});
    float maxy = std::max({v0.sy, v1.sy, v2.sy});

    pkt.bbox_xmin = std::max(0, (int)floorf(minx));
    pkt.bbox_ymin = std::max(0, (int)floorf(miny));
    pkt.bbox_xmax = std::min(SCREEN_W - 1, (int)ceilf(maxx));
    pkt.bbox_ymax = std::min(SCREEN_H - 1, (int)ceilf(maxy));

    // Convert edge coefficients to Q12.12 fixed point
    pkt.a0 = to_fixed(a0f); pkt.b0 = to_fixed(b0f);
    pkt.a1 = to_fixed(a1f); pkt.b1 = to_fixed(b1f);
    pkt.a2 = to_fixed(a2f); pkt.b2 = to_fixed(b2f);

    // Compute initial edge values at bounding box origin
    float px = pkt.bbox_xmin + 0.5f; // pixel center
    float py = pkt.bbox_ymin + 0.5f;
    pkt.c0 = to_fixed(a0f * px + b0f * py + c0f);
    pkt.c1 = to_fixed(a1f * px + b1f * py + c1f);
    pkt.c2 = to_fixed(a2f * px + b2f * py + c2f);

    // Depth interpolation using barycentric coordinates
    // z = (w0 * z0 + w1 * z1 + w2 * z2) / area
    // where w0 = E0(x,y), w1 = E1(x,y), w2 = E2(x,y)
    // z = (E0*z0 + E1*z1 + E2*z2) / area
    // We can precompute z_step_x and z_step_y:
    // z_step_x = (a0*z0 + a1*z1 + a2*z2) / area
    // z_step_y = (b0*z0 + b1*z1 + b2*z2) / area
    float z0 = v0.sz, z1 = v1.sz, z2 = v2.sz;

    // Scale depth to 16-bit range [0, 65535]
    float z_scale = 65535.0f;
    float zs0 = z0 * z_scale, zs1 = z1 * z_scale, zs2 = z2 * z_scale;

    if (area > 1e-6f) {
        float z_step_x_f = (a0f * zs0 + a1f * zs1 + a2f * zs2) / area;
        float z_step_y_f = (b0f * zs0 + b1f * zs1 + b2f * zs2) / area;
        float z_at_origin_f = (pkt.c0 / (float)FIXED_ONE * zs0 + 
                               pkt.c1 / (float)FIXED_ONE * zs1 + 
                               pkt.c2 / (float)FIXED_ONE * zs2) / area;

        pkt.z_at_origin = to_fixed(z_at_origin_f);
        pkt.z_step_x = to_fixed(z_step_x_f);
        pkt.z_step_y = to_fixed(z_step_y_f);
    } else {
        pkt.z_at_origin = 0;
        pkt.z_step_x = 0;
        pkt.z_step_y = 0;
    }

    pkt.color = color.val;
    return pkt;
}

// ============================================================================
// HARDWARE SIDE: Rasterizer (this is what the FPGA does)
// ALL ARITHMETIC HERE IS INTEGER ONLY — addition and comparison
// This matches the SystemVerilog implementation exactly
// ============================================================================
struct TestVector {
    int x, y;
    uint16_t depth;
    uint8_t color;
};

void rasterize_triangle(const TrianglePacket& pkt, Framebuffer& fb,
                         std::vector<TestVector>& test_vectors) {
    if (!pkt.front_facing) return; // back-face cull

    // Skip degenerate triangles
    if (pkt.bbox_xmin > pkt.bbox_xmax || pkt.bbox_ymin > pkt.bbox_ymax) return;

    // Initial edge values at start of first row
    int32_t e0_row = pkt.c0;
    int32_t e1_row = pkt.c1;
    int32_t e2_row = pkt.c2;

    // Initial depth at start of first row
    int32_t z_row = pkt.z_at_origin;

    for (int y = pkt.bbox_ymin; y <= pkt.bbox_ymax; y++) {
        // Edge values at start of this row
        int32_t e0 = e0_row;
        int32_t e1 = e1_row;
        int32_t e2 = e2_row;

        // Depth at start of this row
        int32_t z = z_row;

        for (int x = pkt.bbox_xmin; x <= pkt.bbox_xmax; x++) {
            // INSIDE TEST: all three edge functions must be >= 0
            // This is 3 sign checks — the core of Pineda's algorithm
            if (e0 >= 0 && e1 >= 0 && e2 >= 0) {
                // Convert fixed-point depth to 16-bit unsigned
                uint16_t depth_16 = (uint16_t)std::max(0, 
                    std::min(65535, (int)(z >> FRAC_BITS)));

                // EARLY Z TEST: skip if behind existing pixel
                if (depth_16 < fb.depth[y][x]) {
                    fb.depth[y][x] = depth_16;
                    fb.color[y][x] = Color332(pkt.color);

                    // Record test vector for hardware verification
                    test_vectors.push_back({x, y, depth_16, pkt.color});
                }
            }

            // STEP X: increment edge functions by a_i (ONE ADDITION EACH)
            e0 += pkt.a0;
            e1 += pkt.a1;
            e2 += pkt.a2;

            // Step depth in x
            z += pkt.z_step_x;
        }

        // STEP Y: increment row start values by b_i (ONE ADDITION EACH)
        e0_row += pkt.b0;
        e1_row += pkt.b1;
        e2_row += pkt.b2;

        // Step depth in y
        z_row += pkt.z_step_y;
    }
}

// ============================================================================
// Test vector output (for hardware verification)
// ============================================================================
void write_test_vectors(const char* filename,
                        const std::vector<TrianglePacket>& packets,
                        const std::vector<TestVector>& vectors) {
    FILE* f = fopen(filename, "w");

    // Write triangle packets (what software sends to FIFO)
    fprintf(f, "# Triangle packets (FIFO input)\n");
    fprintf(f, "# idx a0 b0 c0 a1 b1 c1 a2 b2 c2 xmin ymin xmax ymax "
               "z_origin z_step_x z_step_y color front\n");
    for (size_t i = 0; i < packets.size(); i++) {
        const auto& p = packets[i];
        fprintf(f, "TRI %zu %d %d %d %d %d %d %d %d %d %d %d %d %d "
                   "%d %d %d %u %d\n",
                i, p.a0, p.b0, p.c0, p.a1, p.b1, p.c1, p.a2, p.b2, p.c2,
                p.bbox_xmin, p.bbox_ymin, p.bbox_xmax, p.bbox_ymax,
                p.z_at_origin, p.z_step_x, p.z_step_y, p.color,
                p.front_facing ? 1 : 0);
    }

    // Write pixel outputs (what hardware should produce)
    fprintf(f, "\n# Pixel outputs (expected hardware output)\n");
    fprintf(f, "# x y depth color\n");
    for (const auto& tv : vectors) {
        fprintf(f, "PIX %d %d %u %u\n", tv.x, tv.y, tv.depth, tv.color);
    }

    fclose(f);
    printf("Wrote %zu triangle packets and %zu pixel test vectors to %s\n",
           packets.size(), vectors.size(), filename);
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    // Parse arguments
    const char* obj_file = nullptr;
    float rot_x = 30.0f, rot_y = 45.0f, rot_z = 0.0f;

    if (argc >= 2) obj_file = argv[1];
    if (argc >= 3) rot_x = atof(argv[2]);
    if (argc >= 4) rot_y = atof(argv[3]);
    if (argc >= 5) rot_z = atof(argv[4]);

    // Load model
    Model model;
    if (obj_file && model.load_obj(obj_file)) {
        printf("Loaded %s: %zu vertices, %zu faces\n",
               obj_file, model.vertices.size(), model.faces.size());
    } else {
        if (obj_file) printf("Failed to load %s, using default cube\n", obj_file);
        else printf("No model specified, using default cube\n");
        model.make_cube();
    }

    // ========================================================================
    // SOFTWARE PIPELINE (runs on ARM, floating point)
    // ========================================================================

    // Build transform matrix: projection * view * model
    float aspect = (float)SCREEN_W / (float)SCREEN_H;
    Mat4f proj = perspective(60.0f, aspect, 0.1f, 100.0f);
    Mat4f view = translation(0.0f, 0.0f, -4.0f);
    Mat4f model_mat = rotation_z(rot_z) * rotation_y(rot_y) * rotation_x(rot_x);
    Mat4f mvp = proj * view * model_mat;

    // Light direction (world space, normalized)
    Vec3f light_dir = Vec3f(0.4f, 0.7f, 0.5f).normalize();

    // Base color for the model (bright teal/cyan)
    float base_r = 0.2f, base_g = 0.7f, base_b = 1.0f;

    // Transform all vertices
    std::vector<ScreenVertex> screen_verts(model.vertices.size());
    for (size_t i = 0; i < model.vertices.size(); i++) {
        Vec3f& v = model.vertices[i];
        Vec4f clip = mvp * Vec4f(v.x, v.y, v.z, 1.0f);

        // Perspective divide
        if (fabsf(clip.w) < 1e-6f) clip.w = 1e-6f;
        float ndcx = clip.x / clip.w;
        float ndcy = clip.y / clip.w;
        float ndcz = clip.z / clip.w;

        // Viewport transform: NDC [-1,1] -> screen [0, W/H]
        screen_verts[i].sx = (ndcx + 1.0f) * 0.5f * SCREEN_W;
        screen_verts[i].sy = (1.0f - ndcy) * 0.5f * SCREEN_H; // flip Y
        // Depth: NDC [-1,1] -> [0,1]
        screen_verts[i].sz = (ndcz + 1.0f) * 0.5f;
        screen_verts[i].sz = std::max(0.0f, std::min(1.0f, screen_verts[i].sz));
    }

    // Set up triangle packets (what software sends to hardware FIFO)
    std::vector<TrianglePacket> packets;
    for (size_t i = 0; i < model.faces.size(); i++) {
        const Triangle& face = model.faces[i];
        const ScreenVertex& sv0 = screen_verts[face.v[0]];
        const ScreenVertex& sv1 = screen_verts[face.v[1]];
        const ScreenVertex& sv2 = screen_verts[face.v[2]];

        // Compute face normal in world space for lighting
        Vec3f wv0 = model.vertices[face.v[0]];
        Vec3f wv1 = model.vertices[face.v[1]];
        Vec3f wv2 = model.vertices[face.v[2]];
        // Transform normals with model matrix (ignoring translation)
        Vec3f normal = face_normal(wv0, wv1, wv2);
        Vec4f tn = model_mat * Vec4f(normal.x, normal.y, normal.z, 0.0f);
        Vec3f world_normal = Vec3f(tn.x, tn.y, tn.z).normalize();

        Color332 color = shade_face(world_normal, light_dir, 
                                     base_r, base_g, base_b);

        TrianglePacket pkt = setup_triangle(sv0, sv1, sv2, color);
        packets.push_back(pkt);
    }

    printf("Generated %zu triangle packets\n", packets.size());

    // ========================================================================
    // HARDWARE SIMULATION (integer arithmetic only from here)
    // ========================================================================

    Framebuffer fb;
    fb.clear();

    std::vector<TestVector> test_vectors;

    int triangles_drawn = 0;
    int pixels_written = 0;
    int pixels_tested = 0;
    int pixels_z_rejected = 0;

    for (size_t i = 0; i < packets.size(); i++) {
        const TrianglePacket& pkt = packets[i];
        if (!pkt.front_facing) continue;
        if (pkt.bbox_xmin > pkt.bbox_xmax || pkt.bbox_ymin > pkt.bbox_ymax) 
            continue;

        size_t before = test_vectors.size();

        // Count pixels in bounding box for stats
        int bbox_pixels = (pkt.bbox_xmax - pkt.bbox_xmin + 1) * 
                          (pkt.bbox_ymax - pkt.bbox_ymin + 1);
        pixels_tested += bbox_pixels;

        rasterize_triangle(pkt, fb, test_vectors);

        int tri_pixels = test_vectors.size() - before;
        pixels_written += tri_pixels;
        triangles_drawn++;
    }

    pixels_z_rejected = pixels_tested - pixels_written;

    // ========================================================================
    // Output
    // ========================================================================

    fb.write_ppm("output.ppm");
    write_test_vectors("test_vectors.txt", packets, test_vectors);

    printf("\n--- Rasterization Statistics ---\n");
    printf("Triangles submitted: %zu\n", packets.size());
    printf("Triangles drawn (front-facing): %d\n", triangles_drawn);
    printf("Pixels tested (bbox area): %d\n", pixels_tested);
    printf("Pixels written: %d\n", pixels_written);
    printf("Pixels rejected (outside + Z-fail): %d\n", pixels_z_rejected);
    printf("Fill efficiency: %.1f%%\n", 
           pixels_tested > 0 ? 100.0f * pixels_written / pixels_tested : 0.0f);

    return 0;
}
