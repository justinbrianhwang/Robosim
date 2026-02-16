#pragma once

#ifdef ROBOSIM_HAS_GUI

#include <glad/glad.h>
#include <string>
#include <Eigen/Dense>

namespace robosim::render {

class Shader {
public:
    Shader() = default;
    ~Shader();

    bool load_from_files(const std::string& vert_path, const std::string& frag_path);
    bool load_from_source(const std::string& vert_src, const std::string& frag_src);
    void use() const;

    void set_bool(const std::string& name, bool value) const;
    void set_int(const std::string& name, int value) const;
    void set_float(const std::string& name, float value) const;
    void set_vec3(const std::string& name, const Eigen::Vector3f& v) const;
    void set_vec4(const std::string& name, const Eigen::Vector4f& v) const;
    void set_mat3(const std::string& name, const Eigen::Matrix3f& m) const;
    void set_mat4(const std::string& name, const Eigen::Matrix4f& m) const;

    GLuint id() const { return program_id_; }

private:
    GLuint program_id_ = 0;
    bool compile_shader(GLuint shader, const std::string& source);
};

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
