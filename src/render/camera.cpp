#ifdef ROBOSIM_HAS_GUI

#include "robosim/render/camera.h"
#include <cmath>
#include <algorithm>

namespace robosim::render {

Camera::Camera() {
    update_from_orbit();
}

void Camera::set_orbit(const Eigen::Vector3f& target, float distance, float yaw, float pitch) {
    target_ = target;
    orbit_distance_ = distance;
    orbit_yaw_ = yaw;
    orbit_pitch_ = pitch;
    mode_ = CameraMode::Orbit;
    update_from_orbit();
}

void Camera::set_position(const Eigen::Vector3f& pos) {
    position_ = pos;
}

void Camera::set_target(const Eigen::Vector3f& target) {
    target_ = target;
    if (mode_ == CameraMode::Orbit) {
        Eigen::Vector3f diff = position_ - target_;
        orbit_distance_ = diff.norm();
        update_from_orbit();
    }
}

void Camera::set_follow_target(const Eigen::Vector3f& target) {
    if (mode_ == CameraMode::Follow) {
        target_ = target;
        update_from_orbit();
    }
}

void Camera::orbit_rotate(float dyaw, float dpitch) {
    orbit_yaw_ += dyaw * mouse_sensitivity_;
    orbit_pitch_ += dpitch * mouse_sensitivity_;
    orbit_pitch_ = std::clamp(orbit_pitch_, -89.0f, 89.0f);
    update_from_orbit();
}

void Camera::orbit_zoom(float delta) {
    orbit_distance_ -= delta * 0.5f;
    orbit_distance_ = std::clamp(orbit_distance_, 0.5f, 100.0f);
    update_from_orbit();
}

void Camera::orbit_pan(float dx, float dy) {
    Eigen::Vector3f r = right();
    Eigen::Vector3f u = up_;
    float pan_speed = orbit_distance_ * 0.002f;
    target_ += r * (-dx * pan_speed) + u * (dy * pan_speed);
    update_from_orbit();
}

void Camera::fps_rotate(float dyaw, float dpitch) {
    orbit_yaw_ += dyaw * mouse_sensitivity_;
    orbit_pitch_ += dpitch * mouse_sensitivity_;
    orbit_pitch_ = std::clamp(orbit_pitch_, -89.0f, 89.0f);
}

void Camera::fps_move(float fwd, float rt, float up) {
    float speed = move_speed_ * 0.016f; // assume ~60fps
    position_ += forward() * (fwd * speed) + right() * (rt * speed) +
                 Eigen::Vector3f::UnitZ() * (up * speed);
}

Eigen::Vector3f Camera::forward() const {
    float yaw_rad = orbit_yaw_ * M_PI / 180.0f;
    float pitch_rad = orbit_pitch_ * M_PI / 180.0f;
    return Eigen::Vector3f(
        std::cos(pitch_rad) * std::cos(yaw_rad),
        std::cos(pitch_rad) * std::sin(yaw_rad),
        std::sin(pitch_rad)
    ).normalized();
}

Eigen::Vector3f Camera::right() const {
    return forward().cross(up_).normalized();
}

Eigen::Matrix4f Camera::view_matrix() const {
    Eigen::Vector3f f, r, u;

    if (mode_ == CameraMode::FPS) {
        f = forward();
        r = f.cross(up_).normalized();
        u = r.cross(f).normalized();
    } else {
        f = (target_ - position_).normalized();
        r = f.cross(up_).normalized();
        u = r.cross(f).normalized();
    }

    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view(0,0) = r.x(); view(0,1) = r.y(); view(0,2) = r.z();
    view(1,0) = u.x(); view(1,1) = u.y(); view(1,2) = u.z();
    view(2,0) = -f.x(); view(2,1) = -f.y(); view(2,2) = -f.z();
    view(0,3) = -r.dot(position_);
    view(1,3) = -u.dot(position_);
    view(2,3) = f.dot(position_);
    return view;
}

Eigen::Matrix4f Camera::projection_matrix(float aspect) const {
    float fov_rad = fov_ * M_PI / 180.0f;
    float t = std::tan(fov_rad / 2.0f);

    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
    proj(0,0) = 1.0f / (aspect * t);
    proj(1,1) = 1.0f / t;
    proj(2,2) = -(far_ + near_) / (far_ - near_);
    proj(2,3) = -(2.0f * far_ * near_) / (far_ - near_);
    proj(3,2) = -1.0f;
    return proj;
}

void Camera::update_from_orbit() {
    float yaw_rad = orbit_yaw_ * M_PI / 180.0f;
    float pitch_rad = orbit_pitch_ * M_PI / 180.0f;

    position_.x() = target_.x() + orbit_distance_ * std::cos(pitch_rad) * std::cos(yaw_rad);
    position_.y() = target_.y() + orbit_distance_ * std::cos(pitch_rad) * std::sin(yaw_rad);
    position_.z() = target_.z() + orbit_distance_ * std::sin(pitch_rad);
}

} // namespace robosim::render

#endif // ROBOSIM_HAS_GUI
