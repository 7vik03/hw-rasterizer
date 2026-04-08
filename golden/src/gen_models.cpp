// gen_models.cpp - Generate low-poly OBJ test models
#include <cstdio>
#include <cmath>
#include <vector>
#include <array>
#include <map>

struct V3 { float x,y,z; };

void write_obj(const char* filename, const std::vector<V3>& verts,
               const std::vector<std::array<int,3>>& faces) {
    FILE* f = fopen(filename, "w");
    fprintf(f, "# Generated low-poly model\n");
    fprintf(f, "# %zu vertices, %zu faces\n", verts.size(), faces.size());
    for (auto& v : verts) fprintf(f, "v %f %f %f\n", v.x, v.y, v.z);
    for (auto& face : faces) fprintf(f, "f %d %d %d\n", face[0]+1, face[1]+1, face[2]+1);
    fclose(f);
    printf("Wrote %s: %zu verts, %zu faces\n", filename, verts.size(), faces.size());
}

// Icosphere: start with icosahedron, subdivide edges
void make_icosphere(const char* filename, int subdivisions) {
    float t = (1.0f + sqrtf(5.0f)) / 2.0f;
    std::vector<V3> verts = {
        {-1, t,0}, {1, t,0}, {-1,-t,0}, {1,-t,0},
        {0,-1, t}, {0, 1, t}, {0,-1,-t}, {0, 1,-t},
        { t,0,-1}, { t,0, 1}, {-t,0,-1}, {-t,0, 1}
    };
    // Normalize to unit sphere
    for (auto& v : verts) {
        float l = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
        v.x /= l; v.y /= l; v.z /= l;
    }
    std::vector<std::array<int,3>> faces = {{
        {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
        {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
        {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
        {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}
    }};

    for (int s = 0; s < subdivisions; s++) {
        std::map<std::pair<int,int>, int> edge_midpoints;
        std::vector<std::array<int,3>> new_faces;

        auto get_mid = [&](int a, int b) -> int {
            auto key = std::make_pair(std::min(a,b), std::max(a,b));
            auto it = edge_midpoints.find(key);
            if (it != edge_midpoints.end()) return it->second;
            V3 mid = {
                (verts[a].x + verts[b].x) * 0.5f,
                (verts[a].y + verts[b].y) * 0.5f,
                (verts[a].z + verts[b].z) * 0.5f
            };
            float l = sqrtf(mid.x*mid.x + mid.y*mid.y + mid.z*mid.z);
            mid.x /= l; mid.y /= l; mid.z /= l;
            int idx = verts.size();
            verts.push_back(mid);
            edge_midpoints[key] = idx;
            return idx;
        };

        for (auto& f : faces) {
            int a = get_mid(f[0], f[1]);
            int b = get_mid(f[1], f[2]);
            int c = get_mid(f[2], f[0]);
            new_faces.push_back({f[0], a, c});
            new_faces.push_back({f[1], b, a});
            new_faces.push_back({f[2], c, b});
            new_faces.push_back({a, b, c});
        }
        faces = new_faces;
    }
    write_obj(filename, verts, faces);
}

// Low-poly torus
void make_torus(const char* filename, int major_seg, int minor_seg,
                float R, float r) {
    std::vector<V3> verts;
    std::vector<std::array<int,3>> faces;

    for (int i = 0; i < major_seg; i++) {
        float theta = 2.0f * M_PI * i / major_seg;
        for (int j = 0; j < minor_seg; j++) {
            float phi = 2.0f * M_PI * j / minor_seg;
            float x = (R + r * cosf(phi)) * cosf(theta);
            float y = r * sinf(phi);
            float z = (R + r * cosf(phi)) * sinf(theta);
            verts.push_back({x, y, z});
        }
    }

    for (int i = 0; i < major_seg; i++) {
        int ni = (i + 1) % major_seg;
        for (int j = 0; j < minor_seg; j++) {
            int nj = (j + 1) % minor_seg;
            int a = i * minor_seg + j;
            int b = ni * minor_seg + j;
            int c = ni * minor_seg + nj;
            int d = i * minor_seg + nj;
            faces.push_back({a, b, c});
            faces.push_back({a, c, d});
        }
    }
    write_obj(filename, verts, faces);
}

// Simple low-poly teapot approximation (body + spout + handle + lid)
void make_simple_teapot(const char* filename) {
    // Use a UV sphere for the body and small shapes for parts
    std::vector<V3> verts;
    std::vector<std::array<int,3>> faces;

    int lat = 10, lon = 16;

    // Body: squashed sphere
    auto add_sphere = [&](float cx, float cy, float cz,
                          float sx, float sy, float sz,
                          int la, int lo) {
        int base = verts.size();
        // Top pole
        verts.push_back({cx, cy + sy, cz});
        // Middle rows
        for (int i = 1; i < la; i++) {
            float theta = M_PI * i / la;
            for (int j = 0; j < lo; j++) {
                float phi = 2.0f * M_PI * j / lo;
                float x = cx + sx * sinf(theta) * cosf(phi);
                float y = cy + sy * cosf(theta);
                float z = cz + sz * sinf(theta) * sinf(phi);
                verts.push_back({x, y, z});
            }
        }
        // Bottom pole
        verts.push_back({cx, cy - sy, cz});

        int top = base;
        int bot = verts.size() - 1;

        // Top cap
        for (int j = 0; j < lo; j++) {
            int next = (j + 1) % lo;
            faces.push_back({top, base + 1 + j, base + 1 + next});
        }
        // Middle strips
        for (int i = 0; i < la - 2; i++) {
            for (int j = 0; j < lo; j++) {
                int next = (j + 1) % lo;
                int a = base + 1 + i * lo + j;
                int b = base + 1 + i * lo + next;
                int c = base + 1 + (i+1) * lo + next;
                int d = base + 1 + (i+1) * lo + j;
                faces.push_back({a, d, c});
                faces.push_back({a, c, b});
            }
        }
        // Bottom cap
        int last_row = base + 1 + (la - 2) * lo;
        for (int j = 0; j < lo; j++) {
            int next = (j + 1) % lo;
            faces.push_back({bot, last_row + next, last_row + j});
        }
    };

    // Body
    add_sphere(0, 0, 0, 1.0f, 0.75f, 1.0f, lat, lon);

    // Lid (smaller sphere on top)
    add_sphere(0, 0.75f, 0, 0.6f, 0.2f, 0.6f, 6, lon);

    // Lid knob
    add_sphere(0, 1.0f, 0, 0.15f, 0.15f, 0.15f, 4, 8);

    // Spout (elongated small sphere)
    add_sphere(1.1f, 0.1f, 0, 0.5f, 0.25f, 0.25f, 6, 8);

    // Handle (torus-like, approximate with small spheres)
    for (int i = 0; i < 6; i++) {
        float angle = -M_PI * 0.4f + M_PI * 0.8f * i / 5.0f;
        float hx = -1.0f - 0.4f * cosf(angle);
        float hy = 0.3f * sinf(angle);
        add_sphere(hx, hy, 0, 0.12f, 0.12f, 0.12f, 4, 6);
    }

    write_obj(filename, verts, faces);
}

int main() {
    make_icosphere("sphere_lo.obj", 1);   // 80 faces
    make_icosphere("sphere_med.obj", 2);  // 320 faces
    make_torus("torus.obj", 12, 8, 0.7f, 0.3f);  // 192 faces
    make_simple_teapot("teapot.obj");
    return 0;
}
