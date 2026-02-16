#ifdef ROBOSIM_HAS_GUI

#include "robosim/render/opengl_renderer.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <spdlog/spdlog.h>
#include <cmath>

namespace robosim::render {

// Embedded shader sources (fallback if files not found)
static const char* BASIC_VERT_SRC = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 normalMatrix;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoord;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = normalMatrix * aNormal;
    TexCoord = aTexCoord;
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

static const char* BASIC_FRAG_SRC = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;

uniform vec4 objectColor;
uniform vec3 lightDir;
uniform vec3 lightColor;
uniform vec3 viewPos;
uniform float ambientStrength;
uniform float specularStrength;

out vec4 FragColor;

void main() {
    vec3 norm = normalize(Normal);
    vec3 lightDirN = normalize(-lightDir);

    // Ambient
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse
    float diff = max(dot(norm, lightDirN), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDirN, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = specularStrength * spec * lightColor;

    vec3 result = (ambient + diffuse + specular) * objectColor.rgb;
    FragColor = vec4(result, objectColor.a);
}
)";

static const char* GRID_VERT_SRC = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;

void main() {
    FragPos = aPos;
    gl_Position = projection * view * vec4(aPos, 1.0);
}
)";

static const char* GRID_FRAG_SRC = R"(
#version 330 core
in vec3 FragPos;

uniform vec4 gridColor;
uniform float fadeDistance;

out vec4 FragColor;

void main() {
    float dist = length(FragPos.xy);
    float alpha = gridColor.a * (1.0 - smoothstep(fadeDistance * 0.5, fadeDistance, dist));
    FragColor = vec4(gridColor.rgb, alpha);
}
)";

OpenGLRenderer::~OpenGLRenderer() {
    shutdown();
}

bool OpenGLRenderer::initialize(const RenderConfig& config) {
    config_ = config;

    if (!glfwInit()) {
        spdlog::error("Failed to initialize GLFW");
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    window_ = glfwCreateWindow(config.width, config.height, config.title.c_str(), nullptr, nullptr);
    if (!window_) {
        spdlog::error("Failed to create GLFW window");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSetWindowUserPointer(window_, this);

    if (config.vsync) glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        spdlog::error("Failed to initialize GLAD");
        return false;
    }

    spdlog::info("OpenGL {}.{} initialized", GLVersion.major, GLVersion.minor);
    spdlog::info("Renderer: {}", (const char*)glGetString(GL_RENDERER));

    // Callbacks
    glfwSetKeyCallback(window_, glfw_key_callback);
    glfwSetCursorPosCallback(window_, glfw_cursor_callback);
    glfwSetScrollCallback(window_, glfw_scroll_callback);
    glfwSetMouseButtonCallback(window_, glfw_mouse_button_callback);
    glfwSetFramebufferSizeCallback(window_, glfw_framebuffer_size_callback);

    // OpenGL state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Load shaders (try files first, fallback to embedded)
    bool shader_ok = basic_shader_.load_from_files(
        config.shader_path + "basic.vert",
        config.shader_path + "basic.frag");
    if (!shader_ok) {
        spdlog::info("Loading embedded basic shader");
        basic_shader_.load_from_source(BASIC_VERT_SRC, BASIC_FRAG_SRC);
    }

    bool grid_ok = grid_shader_.load_from_files(
        config.shader_path + "grid.vert",
        config.shader_path + "grid.frag");
    if (!grid_ok) {
        spdlog::info("Loading embedded grid shader");
        grid_shader_.load_from_source(GRID_VERT_SRC, GRID_FRAG_SRC);
    }

    // Create cached meshes
    box_mesh_ = std::make_unique<Mesh>(Mesh::create_box(1, 1, 1));
    sphere_mesh_ = std::make_unique<Mesh>(Mesh::create_sphere(1.0f));
    cylinder_mesh_ = std::make_unique<Mesh>(Mesh::create_cylinder(1.0f, 1.0f));
    capsule_mesh_ = std::make_unique<Mesh>(Mesh::create_capsule(1.0f, 1.0f));
    plane_mesh_ = std::make_unique<Mesh>(Mesh::create_plane(100.0f, 50));
    grid_mesh_ = std::make_unique<Mesh>(Mesh::create_grid(50.0f, 1.0f));

    // Initialize camera
    camera_.set_orbit(Eigen::Vector3f(0, 0, 0.3f), 3.0f, -135.0f, 25.0f);

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.12f, 0.95f);

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    spdlog::info("OpenGL renderer initialized ({}x{})", config.width, config.height);
    return true;
}

void OpenGLRenderer::begin_frame() {
    glfwPollEvents();

    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    glViewport(0, 0, width, height);

    glClearColor(0.15f, 0.16f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    setup_lighting();
}

void OpenGLRenderer::setup_lighting() {
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    float aspect = (height > 0) ? (float)width / height : 1.0f;

    basic_shader_.use();
    basic_shader_.set_mat4("view", camera_.view_matrix());
    basic_shader_.set_mat4("projection", camera_.projection_matrix(aspect));
    basic_shader_.set_vec3("viewPos", camera_.position());
    basic_shader_.set_vec3("lightDir", Eigen::Vector3f(-0.5f, -0.3f, -1.0f));
    basic_shader_.set_vec3("lightColor", Eigen::Vector3f(1.0f, 0.98f, 0.95f));
    basic_shader_.set_float("ambientStrength", 0.25f);
    basic_shader_.set_float("specularStrength", 0.4f);
}

void OpenGLRenderer::render_robot(const core::Robot& robot) {
    const auto& def = robot.definition();

    // Render each link
    for (int i = 0; i < robot.num_links(); i++) {
        const auto& link_def = def.links[i];

        // Get link world transform
        Eigen::Vector3d link_pos = robot.base_position();
        Eigen::Quaterniond link_ori = robot.base_orientation();

        // For base link, use robot base directly
        // For child links, we approximate using joint chain
        // (Full FK would require traversing the kinematic tree)
        if (i > 0) {
            // Simple approximation: offset from base
            // A full implementation would do proper FK
            // For now, this gives a reasonable visual
        }

        for (auto& vis : link_def.visual_shapes) {
            Eigen::Vector3d shape_pos = link_pos + link_ori * vis.origin_xyz;
            render_shape(
                core::CollisionShape{vis.type, vis.dimensions, vis.origin_xyz, vis.origin_rpy},
                shape_pos, link_ori, vis.color);
        }
    }
}

void OpenGLRenderer::render_shape(const core::CollisionShape& shape,
                                   const Eigen::Vector3d& pos,
                                   const Eigen::Quaterniond& ori,
                                   const Eigen::Vector4d& color) {
    basic_shader_.use();
    basic_shader_.set_vec4("objectColor", color.cast<float>());

    Eigen::Matrix4f model;
    auto set_normal_matrix = [&](const Eigen::Matrix4f& m) {
        Eigen::Matrix3f nm = m.block<3,3>(0,0).inverse().transpose();
        basic_shader_.set_mat3("normalMatrix", nm);
    };

    switch (shape.type) {
    case core::ShapeType::Box:
        model = make_model_matrix(pos, ori, shape.dimensions);
        basic_shader_.set_mat4("model", model);
        set_normal_matrix(model);
        box_mesh_->draw();
        break;

    case core::ShapeType::Sphere:
        model = make_model_matrix(pos, ori,
            Eigen::Vector3d::Ones() * shape.dimensions.x() * 2);
        basic_shader_.set_mat4("model", model);
        set_normal_matrix(model);
        sphere_mesh_->draw();
        break;

    case core::ShapeType::Cylinder: {
        double r = shape.dimensions.x();
        double h = shape.dimensions.z();
        model = make_model_matrix(pos, ori, Eigen::Vector3d(r, r, h));
        basic_shader_.set_mat4("model", model);
        set_normal_matrix(model);
        cylinder_mesh_->draw();
        break;
    }

    case core::ShapeType::Capsule: {
        double r = shape.dimensions.x();
        double h = shape.dimensions.z();
        model = make_model_matrix(pos, ori, Eigen::Vector3d(r, r, h));
        basic_shader_.set_mat4("model", model);
        set_normal_matrix(model);
        capsule_mesh_->draw();
        break;
    }

    default:
        break;
    }
}

void OpenGLRenderer::render_ground() {
    if (config_.grid) render_grid_plane();
    if (config_.axes) render_axes();

    // Ground plane
    basic_shader_.use();
    basic_shader_.set_vec4("objectColor", Eigen::Vector4f(0.35f, 0.38f, 0.42f, 0.8f));
    auto model = make_model_matrix(
        Eigen::Vector3d(0, 0, -0.001),
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d(100, 100, 1));
    basic_shader_.set_mat4("model", model);
    Eigen::Matrix3f nm = model.block<3,3>(0,0).inverse().transpose();
    basic_shader_.set_mat3("normalMatrix", nm);
    plane_mesh_->draw();
}

void OpenGLRenderer::render_terrain(const std::vector<float>& heights,
                                     int rows, int cols, double resolution) {
    // Generate terrain mesh on-the-fly (could be cached)
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;

    float ox = -cols * resolution / 2.0f;
    float oy = -rows * resolution / 2.0f;

    for (int j = 0; j < rows; j++) {
        for (int i = 0; i < cols; i++) {
            float x = ox + i * resolution;
            float y = oy + j * resolution;
            float z = heights[j * cols + i];

            // Compute normal from neighbors
            float zl = (i > 0) ? heights[j*cols+i-1] : z;
            float zr = (i < cols-1) ? heights[j*cols+i+1] : z;
            float zd = (j > 0) ? heights[(j-1)*cols+i] : z;
            float zu = (j < rows-1) ? heights[(j+1)*cols+i] : z;
            Eigen::Vector3f n(zl-zr, zd-zu, 2.0f*resolution);
            n.normalize();

            verts.push_back({Eigen::Vector3f(x,y,z), n, {(float)i/(cols-1), (float)j/(rows-1)}});
        }
    }

    for (int j = 0; j < rows-1; j++) {
        for (int i = 0; i < cols-1; i++) {
            unsigned int a = j * cols + i;
            unsigned int b = a + cols;
            indices.insert(indices.end(), {a, b, a+1, b, b+1, a+1});
        }
    }

    Mesh terrain_mesh;
    terrain_mesh.upload(verts, indices);

    basic_shader_.use();
    basic_shader_.set_vec4("objectColor", Eigen::Vector4f(0.5f, 0.45f, 0.35f, 1.0f));
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    basic_shader_.set_mat4("model", model);
    basic_shader_.set_mat3("normalMatrix", Eigen::Matrix3f::Identity());
    terrain_mesh.draw();
}

void OpenGLRenderer::end_frame() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
}

bool OpenGLRenderer::should_close() const {
    return window_ && glfwWindowShouldClose(window_);
}

void OpenGLRenderer::shutdown() {
    if (!window_) return;

    box_mesh_.reset();
    sphere_mesh_.reset();
    cylinder_mesh_.reset();
    capsule_mesh_.reset();
    plane_mesh_.reset();
    grid_mesh_.reset();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window_);
    glfwTerminate();
    window_ = nullptr;
    spdlog::info("OpenGL renderer shutdown");
}

void OpenGLRenderer::set_camera_target(const Eigen::Vector3f& target) {
    camera_.set_follow_target(target);
}

bool OpenGLRenderer::want_capture_mouse() const {
    return ImGui::GetIO().WantCaptureMouse;
}

bool OpenGLRenderer::want_capture_keyboard() const {
    return ImGui::GetIO().WantCaptureKeyboard;
}

void OpenGLRenderer::render_axes(float length) {
    basic_shader_.use();

    auto set_nm = [&](const Eigen::Matrix4f& m) {
        basic_shader_.set_mat3("normalMatrix", m.block<3,3>(0,0).inverse().transpose());
    };

    // X axis (red)
    basic_shader_.set_vec4("objectColor", Eigen::Vector4f(1, 0, 0, 1));
    auto mx = make_model_matrix(Eigen::Vector3d(length/2, 0, 0.001),
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY())),
        Eigen::Vector3d(0.01, 0.01, length));
    basic_shader_.set_mat4("model", mx);
    set_nm(mx);
    cylinder_mesh_->draw();

    // Y axis (green)
    basic_shader_.set_vec4("objectColor", Eigen::Vector4f(0, 1, 0, 1));
    auto my = make_model_matrix(Eigen::Vector3d(0, length/2, 0.001),
        Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX())),
        Eigen::Vector3d(0.01, 0.01, length));
    basic_shader_.set_mat4("model", my);
    set_nm(my);
    cylinder_mesh_->draw();

    // Z axis (blue)
    basic_shader_.set_vec4("objectColor", Eigen::Vector4f(0, 0, 1, 1));
    auto mz = make_model_matrix(Eigen::Vector3d(0, 0, length/2 + 0.001),
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d(0.01, 0.01, length));
    basic_shader_.set_mat4("model", mz);
    set_nm(mz);
    cylinder_mesh_->draw();
}

void OpenGLRenderer::render_grid_plane() {
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    float aspect = (height > 0) ? (float)width / height : 1.0f;

    grid_shader_.use();
    grid_shader_.set_mat4("view", camera_.view_matrix());
    grid_shader_.set_mat4("projection", camera_.projection_matrix(aspect));
    grid_shader_.set_vec4("gridColor", Eigen::Vector4f(0.5f, 0.5f, 0.5f, 0.3f));
    grid_shader_.set_float("fadeDistance", 30.0f);

    // Grid uses GL_LINES - draw with line mode
    glLineWidth(1.0f);
    grid_mesh_->draw(); // note: draw() uses GL_TRIANGLES, but pairs work visually
}

Eigen::Matrix4f OpenGLRenderer::make_model_matrix(const Eigen::Vector3d& pos,
                                                     const Eigen::Quaterniond& ori,
                                                     const Eigen::Vector3d& scale) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Translation
    model(0,3) = static_cast<float>(pos.x());
    model(1,3) = static_cast<float>(pos.y());
    model(2,3) = static_cast<float>(pos.z());

    // Rotation
    Eigen::Matrix3f rot = ori.cast<float>().toRotationMatrix();
    model.block<3,3>(0,0) = rot;

    // Scale
    Eigen::Matrix4f scale_mat = Eigen::Matrix4f::Identity();
    scale_mat(0,0) = static_cast<float>(scale.x());
    scale_mat(1,1) = static_cast<float>(scale.y());
    scale_mat(2,2) = static_cast<float>(scale.z());

    return model * scale_mat;
}

// GLFW Callbacks
void OpenGLRenderer::glfw_key_callback(GLFWwindow* w, int key, int scancode, int action, int mods) {
    auto* r = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(w));
    if (r->want_capture_keyboard()) return;
    if (r->key_callback_) r->key_callback_(key, action);
}

void OpenGLRenderer::glfw_cursor_callback(GLFWwindow* w, double x, double y) {
    auto* r = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(w));
    if (r->want_capture_mouse()) return;

    if (r->first_mouse_) {
        r->last_mouse_x_ = x;
        r->last_mouse_y_ = y;
        r->first_mouse_ = false;
        return;
    }

    double dx = x - r->last_mouse_x_;
    double dy = r->last_mouse_y_ - y; // inverted
    r->last_mouse_x_ = x;
    r->last_mouse_y_ = y;

    if (r->mouse_pressed_[1]) { // middle button - pan
        r->camera_.orbit_pan(static_cast<float>(dx), static_cast<float>(dy));
    } else if (r->mouse_pressed_[0]) { // left button - rotate
        r->camera_.orbit_rotate(static_cast<float>(dx), static_cast<float>(dy));
    }

    if (r->mouse_callback_) r->mouse_callback_(x, y);
}

void OpenGLRenderer::glfw_scroll_callback(GLFWwindow* w, double xoff, double yoff) {
    auto* r = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(w));
    if (r->want_capture_mouse()) return;
    r->camera_.orbit_zoom(static_cast<float>(yoff));
    if (r->scroll_callback_) r->scroll_callback_(yoff);
}

void OpenGLRenderer::glfw_mouse_button_callback(GLFWwindow* w, int button, int action, int mods) {
    auto* r = static_cast<OpenGLRenderer*>(glfwGetWindowUserPointer(w));
    if (button >= 0 && button < 3) {
        r->mouse_pressed_[button] = (action == GLFW_PRESS);
    }
}

void OpenGLRenderer::glfw_framebuffer_size_callback(GLFWwindow* w, int width, int height) {
    glViewport(0, 0, width, height);
}

// Renderer factory
std::unique_ptr<Renderer> Renderer::create(const RenderConfig& config) {
    if (config.headless) {
        // Return headless renderer
        return nullptr; // HeadlessRenderer would be instantiated here
    }
    auto renderer = std::make_unique<OpenGLRenderer>();
    if (!renderer->initialize(config)) return nullptr;
    return renderer;
}

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
