#pragma once

#ifdef ROBOSIM_HAS_GUI

#include <Eigen/Dense>

namespace robosim::render {

enum class CameraMode { Orbit, FPS, Follow };

class Camera {
public:
    Camera();

    void set_orbit(const Eigen::Vector3f& target, float distance, float yaw, float pitch);
    void set_position(const Eigen::Vector3f& pos);
    void set_target(const Eigen::Vector3f& target);
    void set_follow_target(const Eigen::Vector3f& target);

    void orbit_rotate(float dyaw, float dpitch);
    void orbit_zoom(float delta);
    void orbit_pan(float dx, float dy);

    void fps_rotate(float dyaw, float dpitch);
    void fps_move(float forward, float right, float up);

    Eigen::Matrix4f view_matrix() const;
    Eigen::Matrix4f projection_matrix(float aspect) const;
    Eigen::Vector3f position() const { return position_; }
    Eigen::Vector3f forward() const;
    Eigen::Vector3f right() const;
    Eigen::Vector3f up() const { return up_; }

    void set_fov(float fov) { fov_ = fov; }
    void set_near_far(float near, float far) { near_ = near; far_ = far; }
    void set_mode(CameraMode mode) { mode_ = mode; }
    CameraMode mode() const { return mode_; }

private:
    void update_from_orbit();

    CameraMode mode_ = CameraMode::Orbit;
    Eigen::Vector3f position_ = Eigen::Vector3f(3, 3, 2);
    Eigen::Vector3f target_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f up_ = Eigen::Vector3f::UnitZ();

    float orbit_distance_ = 5.0f;
    float orbit_yaw_ = -135.0f;
    float orbit_pitch_ = 30.0f;

    float fov_ = 45.0f;
    float near_ = 0.01f;
    float far_ = 500.0f;
    float move_speed_ = 3.0f;
    float mouse_sensitivity_ = 0.3f;
};

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
