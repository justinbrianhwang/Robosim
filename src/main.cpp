#include "robosim/robot/robot_factory.h"
#include "robosim/core/simulator.h"
#include "robosim/env/environment.h"
#include "robosim/terrain/terrain.h"
#include "robosim/task/task.h"
#include "robosim/config/config_manager.h"
#include <spdlog/spdlog.h>
#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <atomic>

#ifdef ROBOSIM_HAS_GUI
#include "robosim/render/opengl_renderer.h"
#include "robosim/render/gui.h"
#endif

static std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

void print_banner() {
    std::cout << R"(
 ╔══════════════════════════════════════════════════════════════╗
 ║                    RoboSim v1.0.0                           ║
 ║          Research-Grade Robot Simulation Engine              ║
 ║                                                              ║
 ║  Bullet Physics | OpenGL | ONNX Runtime | Gymnasium API     ║
 ╚══════════════════════════════════════════════════════════════╝
)" << std::endl;
}

void print_robot_list() {
    auto& factory = robosim::robot::RobotFactory::instance();
    auto robots = factory.available_robots();

    std::cout << "\n Available Robots (" << robots.size() << " total):\n";
    std::cout << " ─────────────────────────────────────────────────\n";

    auto print_category = [&](robosim::core::RobotCategory cat, const std::string& label) {
        std::cout << "\n  [" << label << "]\n";
        for (const auto& r : robots) {
            if (r.category == cat) {
                int pad = static_cast<int>(25) - static_cast<int>(r.id.size());
                std::cout << "    " << r.id << std::string(std::max(pad, 1), ' ')
                          << r.display_name << " (" << r.num_joints << " DOF, "
                          << r.weight_kg << " kg)\n";
            }
        }
    };

    print_category(robosim::core::RobotCategory::Quadruped, "Quadrupeds");
    print_category(robosim::core::RobotCategory::Humanoid, "Humanoids");
    print_category(robosim::core::RobotCategory::Arm, "Robot Arms");
    print_category(robosim::core::RobotCategory::Drone, "Drones");
    print_category(robosim::core::RobotCategory::MobileRobot, "Mobile Robots");
    std::cout << "\n";
}

int run_headless(const std::string& robot_name, const std::string& task_name, int max_steps) {
    spdlog::info("Starting headless simulation: robot='{}', task='{}'", robot_name, task_name);

    robosim::env::Environment env(robot_name, task_name, true);
    auto [obs, info] = env.reset(42);

    spdlog::info("Observation dim: {}, Action dim: {}", env.observation_dim(), env.action_dim());

    double total_reward = 0.0;
    int episodes = 0;

    for (int step = 0; step < max_steps && g_running; step++) {
        // Random action
        Eigen::VectorXd action = Eigen::VectorXd::Random(env.action_dim()) * 0.5;
        auto result = env.step(action);
        total_reward += result.reward;

        if (result.terminated || result.truncated) {
            episodes++;
            spdlog::info("Episode {} ended at step {} | Reward: {:.3f}",
                         episodes, step, total_reward);
            total_reward = 0.0;
            auto reset_result = env.reset();
        }
    }

    spdlog::info("Headless simulation complete. {} episodes.", episodes);
    return 0;
}

#ifdef ROBOSIM_HAS_GUI
int run_gui(const std::string& initial_robot) {
    using namespace robosim;

    render::RenderConfig render_cfg;
    render_cfg.title = "RoboSim - Robot Simulator";
    render_cfg.width = 1280;
    render_cfg.height = 720;

    auto renderer = std::make_unique<render::OpenGLRenderer>();
    if (!renderer->initialize(render_cfg)) {
        spdlog::error("Failed to initialize renderer");
        return 1;
    }

    render::Gui gui;
    gui.initialize(renderer->window());

    render::GuiState gui_state;
    auto& factory = robot::RobotFactory::instance();
    auto robot_list = factory.available_robots();

    std::unique_ptr<env::Environment> environment;
    bool in_simulation = false;

    if (!initial_robot.empty()) {
        environment = std::make_unique<env::Environment>(initial_robot, "walk_forward", false);
        environment->reset(42);
        in_simulation = true;
        gui_state.show_robot_selector = false;
        gui_state.simulation_running = true;
    }

    Eigen::VectorXd current_action;
    if (environment) {
        current_action = Eigen::VectorXd::Zero(environment->action_dim());
        gui_state.joint_targets.resize(environment->robot().num_joints(), 0.0f);
    }
    double sim_time_accumulator = 0.0;

    while (!renderer->should_close() && g_running) {
        renderer->begin_frame();

        if (gui_state.show_robot_selector) {
            if (gui.render_robot_selector(robot_list, gui_state)) {
                // Robot selected
                std::string robot_id = gui_state.selected_robot_id;
                spdlog::info("Selected robot: {}", robot_id);
                environment = std::make_unique<env::Environment>(robot_id, "walk_forward", false);
                environment->reset(42);
                current_action = Eigen::VectorXd::Zero(environment->action_dim());
                gui_state.show_robot_selector = false;
                gui_state.simulation_running = true;
                gui_state.joint_targets.resize(environment->robot().num_joints(), 0.0f);
                in_simulation = true;
            }
        }

        if (in_simulation && environment) {
            auto& robot = environment->robot();

            // Run physics if not paused
            if (gui_state.simulation_running && !gui_state.simulation_paused) {
                if (gui_state.manual_joint_control) {
                    // Use GUI sliders as action
                    for (size_t i = 0; i < gui_state.joint_targets.size(); i++) {
                        current_action(i) = gui_state.joint_targets[i];
                    }
                }
                auto result = environment->step(current_action);
                if (result.terminated || result.truncated) {
                    environment->reset();
                }
            }

            // Render robot
            renderer->set_camera_target(robot.base_position().cast<float>());
            renderer->render_robot(robot);
            renderer->render_ground();

            // Render GUI panels
            gui.render_simulation_gui(robot, gui_state);
        }

        renderer->end_frame();
    }

    gui.shutdown();
    renderer->shutdown();
    return 0;
}
#endif

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    spdlog::set_level(spdlog::level::info);

    print_banner();

    // Register all robots
    robosim::robot::RobotFactory::register_all_robots();

    // Parse command line arguments
    std::string robot_name;
    std::string task_name = "walk_forward";
    bool headless = false;
    int max_steps = 10000;
    std::string config_file;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--headless") {
            headless = true;
        } else if (arg == "--robot" && i + 1 < argc) {
            robot_name = argv[++i];
        } else if (arg == "--task" && i + 1 < argc) {
            task_name = argv[++i];
        } else if (arg == "--steps" && i + 1 < argc) {
            max_steps = std::stoi(argv[++i]);
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--list") {
            print_robot_list();
            return 0;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: robosim [options]\n"
                      << "  --robot <name>    Select robot (e.g., unitree_go2)\n"
                      << "  --task <name>     Select task (walk_forward, follow_path, reach_target, pick_and_place)\n"
                      << "  --headless        Run without GUI\n"
                      << "  --steps <N>       Max simulation steps (headless mode)\n"
                      << "  --config <file>   Load YAML configuration\n"
                      << "  --list            List available robots\n"
                      << "  --help            Show this help\n";
            return 0;
        }
    }

    if (headless) {
        if (robot_name.empty()) {
            std::cout << "Error: --robot required for headless mode\n";
            print_robot_list();
            return 1;
        }
        return run_headless(robot_name, task_name, max_steps);
    }

#ifdef ROBOSIM_HAS_GUI
    return run_gui(robot_name);
#else
    spdlog::warn("GUI not available. Running in headless mode.");
    if (robot_name.empty()) {
        robot_name = "unitree_go2";
    }
    return run_headless(robot_name, task_name, max_steps);
#endif
}
