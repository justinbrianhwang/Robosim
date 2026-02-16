#pragma once

#include "robosim/render/renderer.h"

namespace robosim::render {

class HeadlessRenderer : public Renderer {
public:
    bool initialize(const RenderConfig&) override { return true; }
    void begin_frame() override {}
    void render_robot(const core::Robot&) override {}
    void render_ground() override {}
    void render_terrain(const std::vector<float>&, int, int, double) override {}
    void end_frame() override { frame_count_++; }
    bool should_close() const override { return should_close_; }
    void shutdown() override {}

    void set_key_callback(KeyCallback) override {}
    void set_mouse_callback(MouseCallback) override {}
    void set_scroll_callback(ScrollCallback) override {}
    void set_camera_target(const Eigen::Vector3f&) override {}

    void request_close() { should_close_ = true; }
    int frame_count() const { return frame_count_; }

private:
    bool should_close_ = false;
    int frame_count_ = 0;
};

} // namespace robosim::render
