#pragma once

#include "robosim/core/world.h"
#include "robosim/core/robot.h"
#include <memory>
#include <vector>
#include <string>
#include <functional>

namespace robosim::render {

struct RenderConfig {
    int width = 1280;
    int height = 720;
    bool headless = false;
    bool vsync = true;
    bool shadows = true;
    bool grid = true;
    bool axes = true;
    std::string title = "RoboSim - Robot Simulator";
    std::string shader_path = "shaders/";
};

class Renderer {
public:
    virtual ~Renderer() = default;

    virtual bool initialize(const RenderConfig& config) = 0;
    virtual void begin_frame() = 0;
    virtual void render_robot(const core::Robot& robot) = 0;
    virtual void render_ground() = 0;
    virtual void render_terrain(const std::vector<float>& heights,
                                 int rows, int cols, double resolution) = 0;
    virtual void end_frame() = 0;
    virtual bool should_close() const = 0;
    virtual void shutdown() = 0;

    // Input callbacks
    using KeyCallback = std::function<void(int key, int action)>;
    using MouseCallback = std::function<void(double x, double y)>;
    using ScrollCallback = std::function<void(double yoffset)>;

    virtual void set_key_callback(KeyCallback cb) = 0;
    virtual void set_mouse_callback(MouseCallback cb) = 0;
    virtual void set_scroll_callback(ScrollCallback cb) = 0;

    // Camera control
    virtual void set_camera_target(const Eigen::Vector3f& target) = 0;

    static std::unique_ptr<Renderer> create(const RenderConfig& config);
};

} // namespace robosim::render
