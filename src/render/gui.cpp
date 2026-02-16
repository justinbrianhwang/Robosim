#ifdef ROBOSIM_HAS_GUI

#include "robosim/render/gui.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>
#include <cmath>

namespace robosim::render {

void Gui::initialize(GLFWwindow* window) {
    initialized_ = true;
    spdlog::info("GUI initialized");
}

void Gui::shutdown() {
    initialized_ = false;
}

bool Gui::render_robot_selector(const std::vector<robot::RobotInfo>& robots,
                                 GuiState& state) {
    bool selected = false;

    // Full-screen selection overlay
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Begin("RoboSim - Robot Selection", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    // Title
    ImGui::PushFont(nullptr);
    float title_width = ImGui::CalcTextSize("RoboSim - Robot Simulator").x;
    ImGui::SetCursorPosX((io.DisplaySize.x - title_width) / 2);
    ImGui::TextColored(ImVec4(0.4f, 0.7f, 1.0f, 1.0f), "RoboSim - Robot Simulator");
    ImGui::PopFont();

    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Text("Select a robot to simulate:");
    ImGui::Spacing();

    // Category headers
    const char* category_names[] = {"Quadruped", "Humanoid", "Robot Arm", "Drone", "Mobile Robot"};
    const ImVec4 category_colors[] = {
        {0.3f, 0.8f, 0.4f, 1.0f},
        {0.8f, 0.4f, 0.3f, 1.0f},
        {0.3f, 0.5f, 0.9f, 1.0f},
        {0.9f, 0.7f, 0.2f, 1.0f},
        {0.7f, 0.3f, 0.8f, 1.0f}
    };

    for (int cat = 0; cat < 5; cat++) {
        auto category = static_cast<core::RobotCategory>(cat);

        // Find robots in this category
        std::vector<const robot::RobotInfo*> cat_robots;
        for (auto& r : robots) {
            if (r.category == category) cat_robots.push_back(&r);
        }
        if (cat_robots.empty()) continue;

        ImGui::Spacing();
        ImGui::TextColored(category_colors[cat], "%s", category_names[cat]);
        ImGui::Separator();

        float card_width = 280.0f;
        float card_height = 120.0f;
        float available = ImGui::GetContentRegionAvail().x;
        int cards_per_row = std::max(1, (int)(available / (card_width + 10)));

        for (size_t i = 0; i < cat_robots.size(); i++) {
            if (i > 0 && (i % cards_per_row) != 0)
                ImGui::SameLine();

            auto* info = cat_robots[i];
            ImGui::PushID(info->id.c_str());

            bool is_selected = (state.selected_robot_id == info->id);
            if (is_selected) {
                ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.2f, 0.3f, 0.5f, 0.8f));
            }

            ImGui::BeginChild("card", ImVec2(card_width, card_height), true,
                              ImGuiWindowFlags_NoScrollbar);
            {
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "%s",
                                   info->display_name.c_str());
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "%s",
                                   info->manufacturer.c_str());
                ImGui::Spacing();
                ImGui::TextWrapped("%s", info->description.c_str());
                ImGui::Spacing();
                ImGui::Text("DOF: %d | Weight: %.1f kg", info->num_joints, info->weight_kg);

                // Detect click
                if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0)) {
                    state.selected_robot_id = info->id;
                    state.selected_robot_index = static_cast<int>(i);
                }
            }
            ImGui::EndChild();

            if (is_selected) {
                ImGui::PopStyleColor();
            }
            ImGui::PopID();
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Start button
    bool can_start = !state.selected_robot_id.empty();
    if (!can_start) ImGui::BeginDisabled();

    float btn_width = 200;
    ImGui::SetCursorPosX((io.DisplaySize.x - btn_width) / 2);
    if (ImGui::Button("Start Simulation", ImVec2(btn_width, 40))) {
        selected = true;
        state.show_robot_selector = false;
        if (on_robot_selected_) {
            on_robot_selected_(state.selected_robot_id);
        }
    }

    if (!can_start) ImGui::EndDisabled();

    ImGui::End();
    return selected;
}

void Gui::render_simulation_gui(core::Robot& robot, GuiState& state) {
    render_robot_info_panel(robot, state);
    render_joint_control_panel(robot, state);
    render_simulation_control_panel(state);
    render_physics_panel(state);
    render_terrain_panel(state);
    render_performance_panel(state);
    render_domain_rand_panel(state);
}

void Gui::render_robot_info_panel(const core::Robot& robot, GuiState& state) {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 180), ImGuiCond_FirstUseEver);
    ImGui::Begin("Robot Info");

    ImGui::Text("Name: %s", robot.name().c_str());
    ImGui::Text("Joints: %d", robot.num_joints());
    ImGui::Text("Links: %d", robot.num_links());
    ImGui::Separator();

    auto pos = robot.base_position();
    auto ori = robot.base_orientation();
    Eigen::Vector3d euler = ori.toRotationMatrix().eulerAngles(0, 1, 2);
    ImGui::Text("Position: (%.3f, %.3f, %.3f)", pos.x(), pos.y(), pos.z());
    ImGui::Text("Height: %.3f m", robot.base_height());
    ImGui::Text("Roll/Pitch/Yaw: (%.1f, %.1f, %.1f)",
                euler.x() * 180/M_PI, euler.y() * 180/M_PI, euler.z() * 180/M_PI);

    auto vel = robot.base_linear_velocity();
    ImGui::Text("Velocity: (%.2f, %.2f, %.2f)", vel.x(), vel.y(), vel.z());
    ImGui::Text("Speed: %.2f m/s", vel.norm());

    if (!state.status_message.empty()) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(1, 0.8f, 0, 1), "%s", state.status_message.c_str());
    }

    ImGui::End();
}

void Gui::render_joint_control_panel(core::Robot& robot, GuiState& state) {
    ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Joint Control");

    ImGui::Checkbox("Manual Control", &state.manual_joint_control);

    if (state.joint_targets.size() != static_cast<size_t>(robot.num_joints())) {
        state.joint_targets.resize(robot.num_joints(), 0.0f);
    }

    ImGui::BeginChild("JointList", ImVec2(0, 0), false);
    const auto& def = robot.definition();

    int joint_idx = 0;
    for (auto& jdef : def.joints) {
        if (jdef.type == core::JointType::Fixed) continue;
        if (joint_idx >= robot.num_joints()) break;

        ImGui::PushID(joint_idx);

        const auto& js = robot.joint_state(joint_idx);
        ImGui::Text("%s", jdef.name.c_str());
        ImGui::SameLine(180);
        ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "%.2f", js.position);

        if (state.manual_joint_control) {
            float lower = static_cast<float>(jdef.lower_limit);
            float upper = static_cast<float>(jdef.upper_limit);
            ImGui::SliderFloat("##target", &state.joint_targets[joint_idx],
                               lower, upper, "%.2f");
        }

        ImGui::PopID();
        joint_idx++;
    }
    ImGui::EndChild();

    ImGui::End();
}

void Gui::render_simulation_control_panel(GuiState& state) {
    ImGui::SetNextWindowPos(ImVec2(10, 610), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 120), ImGuiCond_FirstUseEver);
    ImGui::Begin("Simulation Control");

    if (ImGui::Button(state.simulation_paused ? "Resume" : "Pause", ImVec2(80, 30))) {
        state.simulation_paused = !state.simulation_paused;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset", ImVec2(80, 30))) {
        if (on_reset_) on_reset_();
    }

    ImGui::SliderFloat("Speed", &state.simulation_speed, 0.1f, 5.0f, "%.1fx");

    ImGui::Checkbox("Show Contacts", &state.show_contacts);
    ImGui::SameLine();
    ImGui::Checkbox("Show CoM", &state.show_com);

    const char* cam_modes[] = {"Orbit", "FPS", "Follow"};
    ImGui::Combo("Camera", &state.camera_mode, cam_modes, 3);

    ImGui::End();
}

void Gui::render_physics_panel(GuiState& state) {
    ImGui::SetNextWindowPos(ImVec2(300, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 130), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Physics Settings")) {
        ImGui::SliderFloat("Gravity Z", &state.gravity_z, -15.0f, 0.0f, "%.2f m/s^2");
        ImGui::SliderFloat("Timestep", &state.timestep, 0.0005f, 0.01f, "%.4f s");
    }
    ImGui::End();
}

void Gui::render_terrain_panel(GuiState& state) {
    ImGui::SetNextWindowPos(ImVec2(300, 150), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 150), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Terrain")) {
        const char* terrain_types[] = {"Flat", "Stairs", "Slope", "Rough", "Stepping Stones"};
        ImGui::Combo("Type", &state.terrain_type, terrain_types, 5);
        ImGui::SliderFloat("Friction", &state.terrain_friction, 0.1f, 3.0f);
        ImGui::SliderFloat("Roughness", &state.terrain_roughness, 0.0f, 0.2f);
    }
    ImGui::End();
}

void Gui::render_performance_panel(const GuiState& state) {
    ImGui::SetNextWindowPos(ImVec2(300, 310), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 100), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Performance")) {
        ImGui::Text("FPS: %.1f", state.fps);
        ImGui::Text("Physics: %.2f ms", state.physics_time_ms);
        ImGui::Text("Render: %.2f ms", state.render_time_ms);

        // FPS history bar
        static float fps_history[120] = {};
        static int fps_idx = 0;
        fps_history[fps_idx] = state.fps;
        fps_idx = (fps_idx + 1) % 120;
        ImGui::PlotLines("##fps", fps_history, 120, fps_idx, nullptr, 0, 120, ImVec2(0, 30));
    }
    ImGui::End();
}

void Gui::render_domain_rand_panel(GuiState& state) {
    ImGui::SetNextWindowPos(ImVec2(300, 420), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 150), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Domain Randomization")) {
        ImGui::Checkbox("Enabled", &state.domain_rand_enabled);
        if (state.domain_rand_enabled) {
            ImGui::SliderFloat2("Friction Range", state.friction_range, 0.1f, 3.0f);
            ImGui::SliderFloat2("Mass Scale", state.mass_scale_range, 0.5f, 2.0f);
        }
    }
    ImGui::End();
}

void Gui::render_logging_panel(GuiState& state) {
    // Placeholder for logging controls
}

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
