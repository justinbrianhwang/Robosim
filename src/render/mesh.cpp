#ifdef ROBOSIM_HAS_GUI

#include "robosim/render/mesh.h"
#include <cmath>

namespace robosim::render {

Mesh::~Mesh() {
    cleanup();
}

Mesh::Mesh(Mesh&& other) noexcept
    : vao_(other.vao_), vbo_(other.vbo_), ebo_(other.ebo_), index_count_(other.index_count_) {
    other.vao_ = other.vbo_ = other.ebo_ = 0;
    other.index_count_ = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        cleanup();
        vao_ = other.vao_; vbo_ = other.vbo_; ebo_ = other.ebo_;
        index_count_ = other.index_count_;
        other.vao_ = other.vbo_ = other.ebo_ = 0;
        other.index_count_ = 0;
    }
    return *this;
}

void Mesh::cleanup() {
    if (ebo_) glDeleteBuffers(1, &ebo_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    vao_ = vbo_ = ebo_ = 0;
    index_count_ = 0;
}

void Mesh::upload(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices) {
    cleanup();
    index_count_ = static_cast<int>(indices.size());

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);
    // normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // texcoord
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}

void Mesh::draw() const {
    if (!vao_ || index_count_ == 0) return;
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

Mesh Mesh::create_box(float sx, float sy, float sz) {
    float hx = sx / 2, hy = sy / 2, hz = sz / 2;
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;

    auto add_face = [&](Eigen::Vector3f n, Eigen::Vector3f u, Eigen::Vector3f v, Eigen::Vector3f center) {
        unsigned int base = (unsigned int)verts.size();
        Eigen::Vector3f p0 = center - u - v;
        Eigen::Vector3f p1 = center + u - v;
        Eigen::Vector3f p2 = center + u + v;
        Eigen::Vector3f p3 = center - u + v;
        verts.push_back({p0, n, {0,0}});
        verts.push_back({p1, n, {1,0}});
        verts.push_back({p2, n, {1,1}});
        verts.push_back({p3, n, {0,1}});
        indices.insert(indices.end(), {base, base+1, base+2, base, base+2, base+3});
    };

    Eigen::Vector3f X(hx,0,0), Y(0,hy,0), Z(0,0,hz);
    add_face( Eigen::Vector3f::UnitZ(), X, Y,  Z);  // top
    add_face(-Eigen::Vector3f::UnitZ(), Y, X, -Z);  // bottom
    add_face( Eigen::Vector3f::UnitX(), Y, Z,  X);  // right
    add_face(-Eigen::Vector3f::UnitX(), Z, Y, -X);  // left
    add_face( Eigen::Vector3f::UnitY(), Z, X,  Y);  // front
    add_face(-Eigen::Vector3f::UnitY(), X, Z, -Y);  // back

    Mesh m;
    m.upload(verts, indices);
    return m;
}

Mesh Mesh::create_sphere(float radius, int segments, int rings) {
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;

    for (int j = 0; j <= rings; j++) {
        float phi = M_PI * j / rings;
        float sp = std::sin(phi), cp = std::cos(phi);

        for (int i = 0; i <= segments; i++) {
            float theta = 2.0f * M_PI * i / segments;
            float st = std::sin(theta), ct = std::cos(theta);

            Eigen::Vector3f n(sp * ct, sp * st, cp);
            Eigen::Vector3f p = n * radius;
            Eigen::Vector2f uv((float)i / segments, (float)j / rings);
            verts.push_back({p, n, uv});
        }
    }

    for (int j = 0; j < rings; j++) {
        for (int i = 0; i < segments; i++) {
            unsigned int a = j * (segments + 1) + i;
            unsigned int b = a + segments + 1;
            indices.insert(indices.end(), {a, b, a+1, b, b+1, a+1});
        }
    }

    Mesh m;
    m.upload(verts, indices);
    return m;
}

Mesh Mesh::create_cylinder(float radius, float height, int segments) {
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;
    float hh = height / 2;

    // Side
    for (int i = 0; i <= segments; i++) {
        float theta = 2.0f * M_PI * i / segments;
        float ct = std::cos(theta), st = std::sin(theta);
        Eigen::Vector3f n(ct, st, 0);
        verts.push_back({Eigen::Vector3f(radius*ct, radius*st, hh), n, {(float)i/segments, 0}});
        verts.push_back({Eigen::Vector3f(radius*ct, radius*st, -hh), n, {(float)i/segments, 1}});
    }
    for (int i = 0; i < segments; i++) {
        unsigned int a = i * 2;
        indices.insert(indices.end(), {a, a+1, a+2, a+1, a+3, a+2});
    }

    // Top cap
    unsigned int center_top = (unsigned int)verts.size();
    verts.push_back({Eigen::Vector3f(0,0,hh), Eigen::Vector3f::UnitZ(), {0.5f,0.5f}});
    for (int i = 0; i <= segments; i++) {
        float theta = 2.0f * M_PI * i / segments;
        float ct = std::cos(theta), st = std::sin(theta);
        verts.push_back({Eigen::Vector3f(radius*ct, radius*st, hh), Eigen::Vector3f::UnitZ(), {ct*0.5f+0.5f, st*0.5f+0.5f}});
    }
    for (int i = 0; i < segments; i++) {
        indices.insert(indices.end(), {center_top, center_top+1+i, center_top+2+i});
    }

    // Bottom cap
    unsigned int center_bot = (unsigned int)verts.size();
    verts.push_back({Eigen::Vector3f(0,0,-hh), -Eigen::Vector3f::UnitZ(), {0.5f,0.5f}});
    for (int i = 0; i <= segments; i++) {
        float theta = 2.0f * M_PI * i / segments;
        float ct = std::cos(theta), st = std::sin(theta);
        verts.push_back({Eigen::Vector3f(radius*ct, radius*st, -hh), -Eigen::Vector3f::UnitZ(), {ct*0.5f+0.5f, st*0.5f+0.5f}});
    }
    for (int i = 0; i < segments; i++) {
        indices.insert(indices.end(), {center_bot, center_bot+2+i, center_bot+1+i});
    }

    Mesh m;
    m.upload(verts, indices);
    return m;
}

Mesh Mesh::create_capsule(float radius, float height, int segments, int rings) {
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;
    float hh = height / 2;

    // Top hemisphere
    for (int j = 0; j <= rings; j++) {
        float phi = M_PI * 0.5f * j / rings;
        float sp = std::sin(phi), cp = std::cos(phi);
        for (int i = 0; i <= segments; i++) {
            float theta = 2.0f * M_PI * i / segments;
            float ct = std::cos(theta), st = std::sin(theta);
            Eigen::Vector3f n(cp * ct, cp * st, sp);
            Eigen::Vector3f p = Eigen::Vector3f(radius * cp * ct, radius * cp * st, hh + radius * sp);
            verts.push_back({p, n, {(float)i/segments, (float)j/(2*rings)}});
        }
    }

    // Cylinder part
    for (int j = 0; j <= 1; j++) {
        float z = hh - j * height;
        for (int i = 0; i <= segments; i++) {
            float theta = 2.0f * M_PI * i / segments;
            float ct = std::cos(theta), st = std::sin(theta);
            Eigen::Vector3f n(ct, st, 0);
            verts.push_back({Eigen::Vector3f(radius*ct, radius*st, z), n, {(float)i/segments, 0.25f + 0.5f*j}});
        }
    }

    // Bottom hemisphere
    for (int j = 0; j <= rings; j++) {
        float phi = M_PI * 0.5f + M_PI * 0.5f * j / rings;
        float sp = std::sin(phi), cp = std::cos(phi);
        for (int i = 0; i <= segments; i++) {
            float theta = 2.0f * M_PI * i / segments;
            float ct = std::cos(theta), st = std::sin(theta);
            Eigen::Vector3f n(cp * ct, cp * st, sp);
            Eigen::Vector3f p = Eigen::Vector3f(radius * std::abs(cp) * ct, radius * std::abs(cp) * st, -hh + radius * sp);
            verts.push_back({p, n, {(float)i/segments, 0.75f + 0.25f*j/rings}});
        }
    }

    // Generate indices for all strips
    int total_rows = rings + 2 + rings;
    for (int j = 0; j < total_rows; j++) {
        for (int i = 0; i < segments; i++) {
            unsigned int a = j * (segments + 1) + i;
            unsigned int b = a + segments + 1;
            indices.insert(indices.end(), {a, b, a+1, b, b+1, a+1});
        }
    }

    Mesh m;
    m.upload(verts, indices);
    return m;
}

Mesh Mesh::create_plane(float size, int subdivisions) {
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;
    float half = size / 2;
    float step = size / subdivisions;

    for (int j = 0; j <= subdivisions; j++) {
        for (int i = 0; i <= subdivisions; i++) {
            float x = -half + i * step;
            float y = -half + j * step;
            verts.push_back({
                Eigen::Vector3f(x, y, 0),
                Eigen::Vector3f::UnitZ(),
                {(float)i/subdivisions, (float)j/subdivisions}
            });
        }
    }

    for (int j = 0; j < subdivisions; j++) {
        for (int i = 0; i < subdivisions; i++) {
            unsigned int a = j * (subdivisions + 1) + i;
            unsigned int b = a + subdivisions + 1;
            indices.insert(indices.end(), {a, b, a+1, b, b+1, a+1});
        }
    }

    Mesh m;
    m.upload(verts, indices);
    return m;
}

Mesh Mesh::create_arrow(float length, float radius) {
    // Simple arrow: cylinder shaft + cone head
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;
    int seg = 12;
    float shaft_len = length * 0.75f;
    float head_len = length * 0.25f;
    float head_radius = radius * 3;

    // Shaft (cylinder)
    for (int i = 0; i <= seg; i++) {
        float theta = 2.0f * M_PI * i / seg;
        float ct = std::cos(theta), st = std::sin(theta);
        Eigen::Vector3f n(ct, st, 0);
        verts.push_back({Eigen::Vector3f(radius*ct, radius*st, 0), n, {0,0}});
        verts.push_back({Eigen::Vector3f(radius*ct, radius*st, shaft_len), n, {0,1}});
    }
    for (int i = 0; i < seg; i++) {
        unsigned int a = i * 2;
        indices.insert(indices.end(), {a, a+1, a+2, a+1, a+3, a+2});
    }

    // Cone head
    unsigned int tip = (unsigned int)verts.size();
    verts.push_back({Eigen::Vector3f(0, 0, length), Eigen::Vector3f::UnitZ(), {0.5f, 1}});
    for (int i = 0; i <= seg; i++) {
        float theta = 2.0f * M_PI * i / seg;
        float ct = std::cos(theta), st = std::sin(theta);
        Eigen::Vector3f p(head_radius*ct, head_radius*st, shaft_len);
        Eigen::Vector3f n = Eigen::Vector3f(ct, st, head_radius / head_len).normalized();
        verts.push_back({p, n, {(float)i/seg, 0}});
    }
    for (int i = 0; i < seg; i++) {
        indices.insert(indices.end(), {tip, tip+1+i, tip+2+i});
    }

    Mesh m;
    m.upload(verts, indices);
    return m;
}

Mesh Mesh::create_grid(float size, float spacing) {
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;
    float half = size / 2;
    int lines = static_cast<int>(size / spacing) + 1;
    unsigned int idx = 0;

    Eigen::Vector3f n = Eigen::Vector3f::UnitZ();
    for (int i = 0; i < lines; i++) {
        float pos = -half + i * spacing;
        // Horizontal line
        verts.push_back({Eigen::Vector3f(-half, pos, 0), n, {0,0}});
        verts.push_back({Eigen::Vector3f(half, pos, 0), n, {1,0}});
        indices.push_back(idx++); indices.push_back(idx++);
        // Vertical line
        verts.push_back({Eigen::Vector3f(pos, -half, 0), n, {0,0}});
        verts.push_back({Eigen::Vector3f(pos, half, 0), n, {1,0}});
        indices.push_back(idx++); indices.push_back(idx++);
    }

    Mesh m;
    // Grid uses GL_LINES, store as-is and we'll draw with glDrawElements(GL_LINES, ...)
    m.upload(verts, indices);
    return m;
}

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
