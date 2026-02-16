#pragma once

#include "robosim/core/world.h"
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <memory>
#include <unordered_map>

namespace robosim::physics {

class BulletWorld : public core::World {
public:
    explicit BulletWorld(const core::WorldConfig& config);
    ~BulletWorld() override;

    void initialize() override;
    void step() override;
    void add_robot(std::shared_ptr<core::Robot> robot) override;
    void remove_robot(const std::string& name) override;
    void set_terrain_heightfield(const std::vector<float>& heights,
                                  int rows, int cols,
                                  double resolution,
                                  double min_h, double max_h) override;
    std::vector<core::ContactInfo> get_contacts() const override;
    bool raycast(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                 Eigen::Vector3d& hit_point, Eigen::Vector3d& hit_normal) const override;

    btMultiBodyDynamicsWorld* dynamics_world() { return dynamics_world_.get(); }

private:
    void create_ground_plane();
    void sync_robot_state(std::shared_ptr<core::Robot>& robot);
    btMultiBody* create_multibody(const core::RobotDef& def);
    btCollisionShape* create_collision_shape(const core::CollisionShape& shape);

    // Bullet components
    std::unique_ptr<btBroadphaseInterface> broadphase_;
    std::unique_ptr<btCollisionConfiguration> collision_config_;
    std::unique_ptr<btCollisionDispatcher> dispatcher_;
    std::unique_ptr<btMultiBodyConstraintSolver> solver_;
    std::unique_ptr<btMultiBodyDynamicsWorld> dynamics_world_;

    // Ground
    std::unique_ptr<btCollisionShape> ground_shape_;
    std::unique_ptr<btRigidBody> ground_body_;

    // Terrain heightfield
    std::unique_ptr<btHeightfieldTerrainShape> terrain_shape_;
    std::unique_ptr<btRigidBody> terrain_body_;
    std::vector<float> terrain_data_;

    // Robot multibodies
    struct RobotPhysics {
        btMultiBody* multibody = nullptr;
        std::vector<btMultiBodyLinkCollider*> colliders;
        std::vector<btMultiBodyJointMotor*> motors;
        std::vector<btCollisionShape*> shapes;
    };
    std::unordered_map<std::string, RobotPhysics> robot_physics_;
};

} // namespace robosim::physics
