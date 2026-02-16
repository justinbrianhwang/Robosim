#ifdef ROBOSIM_HAS_GUI

#include "robosim/render/shader.h"
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>

namespace robosim::render {

Shader::~Shader() {
    if (program_id_) glDeleteProgram(program_id_);
}

bool Shader::load_from_files(const std::string& vert_path, const std::string& frag_path) {
    auto read_file = [](const std::string& path) -> std::string {
        std::ifstream f(path);
        if (!f.is_open()) return "";
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    };

    std::string vert_src = read_file(vert_path);
    std::string frag_src = read_file(frag_path);

    if (vert_src.empty()) {
        spdlog::error("Failed to read vertex shader: {}", vert_path);
        return false;
    }
    if (frag_src.empty()) {
        spdlog::error("Failed to read fragment shader: {}", frag_path);
        return false;
    }

    return load_from_source(vert_src, frag_src);
}

bool Shader::load_from_source(const std::string& vert_src, const std::string& frag_src) {
    GLuint vert = glCreateShader(GL_VERTEX_SHADER);
    GLuint frag = glCreateShader(GL_FRAGMENT_SHADER);

    if (!compile_shader(vert, vert_src)) {
        glDeleteShader(vert);
        return false;
    }
    if (!compile_shader(frag, frag_src)) {
        glDeleteShader(vert);
        glDeleteShader(frag);
        return false;
    }

    program_id_ = glCreateProgram();
    glAttachShader(program_id_, vert);
    glAttachShader(program_id_, frag);
    glLinkProgram(program_id_);

    GLint success;
    glGetProgramiv(program_id_, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(program_id_, 512, nullptr, log);
        spdlog::error("Shader link error: {}", log);
        glDeleteProgram(program_id_);
        program_id_ = 0;
        glDeleteShader(vert);
        glDeleteShader(frag);
        return false;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);
    return true;
}

void Shader::use() const {
    glUseProgram(program_id_);
}

void Shader::set_bool(const std::string& name, bool value) const {
    glUniform1i(glGetUniformLocation(program_id_, name.c_str()), (int)value);
}

void Shader::set_int(const std::string& name, int value) const {
    glUniform1i(glGetUniformLocation(program_id_, name.c_str()), value);
}

void Shader::set_float(const std::string& name, float value) const {
    glUniform1f(glGetUniformLocation(program_id_, name.c_str()), value);
}

void Shader::set_vec3(const std::string& name, const Eigen::Vector3f& v) const {
    glUniform3fv(glGetUniformLocation(program_id_, name.c_str()), 1, v.data());
}

void Shader::set_vec4(const std::string& name, const Eigen::Vector4f& v) const {
    glUniform4fv(glGetUniformLocation(program_id_, name.c_str()), 1, v.data());
}

void Shader::set_mat3(const std::string& name, const Eigen::Matrix3f& m) const {
    glUniformMatrix3fv(glGetUniformLocation(program_id_, name.c_str()), 1, GL_FALSE, m.data());
}

void Shader::set_mat4(const std::string& name, const Eigen::Matrix4f& m) const {
    glUniformMatrix4fv(glGetUniformLocation(program_id_, name.c_str()), 1, GL_FALSE, m.data());
}

bool Shader::compile_shader(GLuint shader, const std::string& source) {
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        spdlog::error("Shader compile error: {}", log);
        return false;
    }
    return true;
}

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
