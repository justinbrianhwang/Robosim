#pragma once

#ifdef ROBOSIM_HAS_GUI

#include <glad/glad.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace robosim::render {

struct Vertex {
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector2f texcoord;
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh();
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void upload(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
    void draw() const;

    // Primitive factories
    static Mesh create_box(float sx, float sy, float sz);
    static Mesh create_sphere(float radius, int segments = 24, int rings = 16);
    static Mesh create_cylinder(float radius, float height, int segments = 24);
    static Mesh create_capsule(float radius, float height, int segments = 24, int rings = 8);
    static Mesh create_plane(float size, int subdivisions = 10);
    static Mesh create_arrow(float length, float radius);
    static Mesh create_grid(float size, float spacing);

private:
    GLuint vao_ = 0, vbo_ = 0, ebo_ = 0;
    int index_count_ = 0;
    void cleanup();
};

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
