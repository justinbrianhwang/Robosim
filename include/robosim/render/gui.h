#pragma once

#ifdef ROBOSIM_HAS_GUI

#include "robosim/core/robot.h"
#include "robosim/robot/robot_factory.h"
#include <functional>
#include <string>
#include <vector>

struct GLFWwindow;

namespace robosim::render {

struct GuiState {
    // Robot selection
    bool show_robot_selector = true;
    int selected_robot_index = -1;
    std::string selected_robot_id;

    // Simulation control
    bool simulation_running = false;
    bool simulation_paused = false;
    float simulation_speed = 1.0f;
    bool show_contacts = true;
    bool show_com = true;
    bool show_joint_axes = false;

    // Camera
    int camera_mode = 0; // 0=orbit, 1=fps, 2=follow

    // Terrain
    int terrain_type = 0;
    float terrain_friction = 1.0f;
    float terrain_roughness = 0.0f;

    // Physics
    float gravity_z = -9.81f;
    float timestep = 0.002f;

    // Joint control (manual override)
    bool manual_joint_control = false;
    std::vector<float> joint_targets;

    // Performance
    float fps = 0.0f;
    float physics_time_ms = 0.0f;
    float render_time_ms = 0.0f;

    // Domain randomization
    bool domain_rand_enabled = false;
    float friction_range[2] = {0.5f, 1.5f};
    float mass_scale_range[2] = {0.8f, 1.2f};

    // Info
    std::string status_message;
};

class Gui {
public:
    Gui() = default;

    void initialize(GLFWwindow* window);
    void shutdown();

    // Robot selection screen
    bool render_robot_selector(const std::vector<robot::RobotInfo>& robots,
                                GuiState& state);

    // Main simulation panels
    void render_simulation_gui(core::Robot& robot, GuiState& state);

    // Callbacks
    using RobotSelectedCallback = std::function<void(const std::string& robot_id)>;
    void set_robot_selected_callback(RobotSelectedCallback cb) { on_robot_selected_ = cb; }

    using ResetCallback = std::function<void()>;
    void set_reset_callback(ResetCallback cb) { on_reset_ = cb; }

private:
    void render_robot_info_panel(const core::Robot& robot, GuiState& state);
    void render_joint_control_panel(core::Robot& robot, GuiState& state);
    void render_simulation_control_panel(GuiState& state);
    void render_physics_panel(GuiState& state);
    void render_terrain_panel(GuiState& state);
    void render_performance_panel(const GuiState& state);
    void render_domain_rand_panel(GuiState& state);
    void render_logging_panel(GuiState& state);

    RobotSelectedCallback on_robot_selected_;
    ResetCallback on_reset_;
    bool initialized_ = false;
};

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
