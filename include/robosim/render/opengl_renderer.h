#pragma once

#ifdef ROBOSIM_HAS_GUI

#include "robosim/render/renderer.h"
#include "robosim/render/shader.h"
#include "robosim/render/camera.h"
#include "robosim/render/mesh.h"
#include <GLFW/glfw3.h>
#include <unordered_map>

namespace robosim::render {

class OpenGLRenderer : public Renderer {
public:
    OpenGLRenderer() = default;
    ~OpenGLRenderer() override;

    bool initialize(const RenderConfig& config) override;
    void begin_frame() override;
    void render_robot(const core::Robot& robot) override;
    void render_ground() override;
    void render_terrain(const std::vector<float>& heights,
                         int rows, int cols, double resolution) override;
    void end_frame() override;
    bool should_close() const override;
    void shutdown() override;

    void set_key_callback(KeyCallback cb) override { key_callback_ = cb; }
    void set_mouse_callback(MouseCallback cb) override { mouse_callback_ = cb; }
    void set_scroll_callback(ScrollCallback cb) override { scroll_callback_ = cb; }
    void set_camera_target(const Eigen::Vector3f& target) override;

    GLFWwindow* window() { return window_; }
    Camera& camera() { return camera_; }

    // ImGui access
    bool want_capture_mouse() const;
    bool want_capture_keyboard() const;

private:
    void setup_lighting();
    void render_shape(const core::CollisionShape& shape,
                      const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori,
                      const Eigen::Vector4d& color);
    void render_axes(float length = 1.0f);
    void render_grid_plane();
    Eigen::Matrix4f make_model_matrix(const Eigen::Vector3d& pos,
                                       const Eigen::Quaterniond& ori,
                                       const Eigen::Vector3d& scale);

    GLFWwindow* window_ = nullptr;
    RenderConfig config_;
    Camera camera_;
    Shader basic_shader_;
    Shader grid_shader_;

    // Cached meshes
    std::unique_ptr<Mesh> box_mesh_;
    std::unique_ptr<Mesh> sphere_mesh_;
    std::unique_ptr<Mesh> cylinder_mesh_;
    std::unique_ptr<Mesh> capsule_mesh_;
    std::unique_ptr<Mesh> plane_mesh_;
    std::unique_ptr<Mesh> grid_mesh_;

    // Input
    KeyCallback key_callback_;
    MouseCallback mouse_callback_;
    ScrollCallback scroll_callback_;
    double last_mouse_x_ = 0, last_mouse_y_ = 0;
    bool mouse_pressed_[3] = {};
    bool first_mouse_ = true;

    // GLFW callbacks
    static void glfw_key_callback(GLFWwindow* w, int key, int scancode, int action, int mods);
    static void glfw_cursor_callback(GLFWwindow* w, double x, double y);
    static void glfw_scroll_callback(GLFWwindow* w, double xoff, double yoff);
    static void glfw_mouse_button_callback(GLFWwindow* w, int button, int action, int mods);
    static void glfw_framebuffer_size_callback(GLFWwindow* w, int width, int height);
};

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
