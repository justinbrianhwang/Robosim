#include "robosim/physics/bullet_world.h"
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <spdlog/spdlog.h>

namespace robosim::physics {

BulletWorld::BulletWorld(const core::WorldConfig& config)
    : core::World(config) {}

BulletWorld::~BulletWorld() {
    // Cleanup robot physics
    for (auto& [name, rp] : robot_physics_) {
        for (auto* motor : rp.motors) delete motor;
        for (auto* collider : rp.colliders) {
            dynamics_world_->removeCollisionObject(collider);
            delete collider;
        }
        for (auto* shape : rp.shapes) delete shape;
        if (rp.multibody) {
            dynamics_world_->removeMultiBody(rp.multibody);
            delete rp.multibody;
        }
    }

    // Cleanup terrain motion state
    if (terrain_body_ && terrain_body_->getMotionState()) {
        delete terrain_body_->getMotionState();
    }
    // Cleanup ground motion state
    if (ground_body_ && ground_body_->getMotionState()) {
        delete ground_body_->getMotionState();
    }
}

void BulletWorld::initialize() {
    broadphase_ = std::make_unique<btDbvtBroadphase>();
    collision_config_ = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher_ = std::make_unique<btCollisionDispatcher>(collision_config_.get());
    solver_ = std::make_unique<btMultiBodyConstraintSolver>();

    dynamics_world_ = std::make_unique<btMultiBodyDynamicsWorld>(
        dispatcher_.get(), broadphase_.get(), solver_.get(), collision_config_.get());

    dynamics_world_->setGravity(btVector3(
        config_.gravity.x(), config_.gravity.y(), config_.gravity.z()));

    create_ground_plane();
    spdlog::info("Bullet physics world initialized");
}

void BulletWorld::create_ground_plane() {
    ground_shape_ = std::make_unique<btStaticPlaneShape>(btVector3(0, 0, 1), 0);
    btTransform transform;
    transform.setIdentity();
    btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo info(0.0, motion_state, ground_shape_.get());
    info.m_friction = config_.ground_friction;
    info.m_restitution = config_.ground_restitution;
    ground_body_ = std::make_unique<btRigidBody>(info);
    dynamics_world_->addRigidBody(ground_body_.get());
}

void BulletWorld::step() {
    dynamics_world_->stepSimulation(config_.timestep, config_.max_substeps, config_.timestep);
    sim_time_ += config_.timestep;

    // Sync robot states
    for (auto& robot : robots_) {
        sync_robot_state(robot);
    }
}

btCollisionShape* BulletWorld::create_collision_shape(const core::CollisionShape& shape) {
    switch (shape.type) {
    case core::ShapeType::Box:
        return new btBoxShape(btVector3(
            shape.dimensions.x() / 2.0,
            shape.dimensions.y() / 2.0,
            shape.dimensions.z() / 2.0));
    case core::ShapeType::Sphere:
        return new btSphereShape(shape.dimensions.x());
    case core::ShapeType::Cylinder:
        return new btCylinderShapeZ(btVector3(
            shape.dimensions.x(),
            shape.dimensions.x(),
            shape.dimensions.z() / 2.0));
    case core::ShapeType::Capsule:
        return new btCapsuleShapeZ(shape.dimensions.x(), shape.dimensions.z());
    default:
        return new btSphereShape(0.01);
    }
}

btMultiBody* BulletWorld::create_multibody(const core::RobotDef& def) {
    // Count movable joints
    int num_links = 0;
    for (const auto& j : def.joints) {
        if (j.type != core::JointType::Fixed) num_links++;
    }

    bool floating_base = true; // Most robots have floating base
    bool can_sleep = false;

    btMultiBody* mb = new btMultiBody(num_links, def.total_mass > 0 ? def.total_mass : 10.0,
                                        btVector3(0, 0, 0), floating_base, can_sleep);

    // Set base position
    btTransform base_transform;
    base_transform.setIdentity();
    base_transform.setOrigin(btVector3(
        def.spawn_position.x(), def.spawn_position.y(), def.spawn_position.z()));
    mb->setBaseWorldTransform(base_transform);

    // Setup links
    int link_idx = 0;
    for (size_t ji = 0; ji < def.joints.size(); ji++) {
        const auto& joint = def.joints[ji];
        if (joint.type == core::JointType::Fixed) continue;

        // Find link mass
        double link_mass = 1.0;
        btVector3 link_inertia(0.001, 0.001, 0.001);
        for (const auto& link : def.links) {
            if (link.name == joint.child_link) {
                link_mass = link.inertial.mass;
                auto& I = link.inertial.inertia;
                link_inertia = btVector3(I(0,0), I(1,1), I(2,2));
                break;
            }
        }

        int parent_idx = link_idx > 0 ? link_idx - 1 : -1;
        // For simplicity, use chain topology. Advanced: build tree topology.
        btVector3 joint_axis(joint.axis.x(), joint.axis.y(), joint.axis.z());
        btVector3 parent_to_pivot(joint.origin_xyz.x(), joint.origin_xyz.y(), joint.origin_xyz.z());
        btVector3 pivot_to_com(0, 0, 0); // COM at joint

        if (joint.type == core::JointType::Revolute || joint.type == core::JointType::Continuous) {
            mb->setupRevolute(link_idx, link_mass, link_inertia,
                              parent_idx, btQuaternion::getIdentity(),
                              joint_axis, parent_to_pivot, pivot_to_com, !can_sleep);
        } else if (joint.type == core::JointType::Prismatic) {
            mb->setupPrismatic(link_idx, link_mass, link_inertia,
                               parent_idx, btQuaternion::getIdentity(),
                               joint_axis, parent_to_pivot, !can_sleep);
        }

        link_idx++;
    }

    mb->finalizeMultiDof();
    mb->setHasSelfCollision(false);
    mb->setLinearDamping(0.0);
    mb->setAngularDamping(0.0);

    return mb;
}

void BulletWorld::add_robot(std::shared_ptr<core::Robot> robot) {
    const auto& def = robot->definition();
    robots_.push_back(robot);

    RobotPhysics rp;
    rp.multibody = create_multibody(def);
    dynamics_world_->addMultiBody(rp.multibody);

    // Create colliders for base
    if (!def.links.empty()) {
        const auto& base_link = def.links[0];
        for (const auto& cs : base_link.collision_shapes) {
            auto* shape = create_collision_shape(cs);
            rp.shapes.push_back(shape);

            auto* collider = new btMultiBodyLinkCollider(rp.multibody, -1);
            collider->setCollisionShape(shape);
            collider->setFriction(config_.ground_friction);
            dynamics_world_->addCollisionObject(collider,
                btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
            rp.multibody->setBaseCollider(collider);
            rp.colliders.push_back(collider);
        }
    }

    // Create colliders and motors for links
    int link_idx = 0;
    for (size_t ji = 0; ji < def.joints.size(); ji++) {
        const auto& joint = def.joints[ji];
        if (joint.type == core::JointType::Fixed) continue;

        // Find child link
        for (const auto& link : def.links) {
            if (link.name == joint.child_link) {
                for (const auto& cs : link.collision_shapes) {
                    auto* shape = create_collision_shape(cs);
                    rp.shapes.push_back(shape);

                    auto* collider = new btMultiBodyLinkCollider(rp.multibody, link_idx);
                    collider->setCollisionShape(shape);
                    collider->setFriction(config_.ground_friction);
                    dynamics_world_->addCollisionObject(collider,
                        btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
                    rp.multibody->getLink(link_idx).m_collider = collider;
                    rp.colliders.push_back(collider);
                }
                break;
            }
        }

        // Add joint motor
        auto* motor = new btMultiBodyJointMotor(rp.multibody, link_idx, 0.0, 100.0);
        dynamics_world_->addMultiBodyConstraint(motor);
        rp.motors.push_back(motor);

        link_idx++;
    }

    robot_physics_[def.name] = std::move(rp);
    spdlog::info("Added robot '{}' to physics world ({} links)", def.name, link_idx);
}

void BulletWorld::remove_robot(const std::string& name) {
    auto it = robot_physics_.find(name);
    if (it == robot_physics_.end()) return;

    auto& rp = it->second;
    for (auto* motor : rp.motors) {
        dynamics_world_->removeMultiBodyConstraint(motor);
        delete motor;
    }
    for (auto* collider : rp.colliders) {
        dynamics_world_->removeCollisionObject(collider);
        delete collider;
    }
    for (auto* shape : rp.shapes) delete shape;
    if (rp.multibody) {
        dynamics_world_->removeMultiBody(rp.multibody);
        delete rp.multibody;
    }
    robot_physics_.erase(it);

    // Remove from robots_ vector
    robots_.erase(std::remove_if(robots_.begin(), robots_.end(),
        [&name](const auto& r) { return r->name() == name; }), robots_.end());
}

void BulletWorld::sync_robot_state(std::shared_ptr<core::Robot>& robot) {
    auto it = robot_physics_.find(robot->name());
    if (it == robot_physics_.end()) return;

    auto* mb = it->second.multibody;
    auto& motors = it->second.motors;

    // Sync base state
    btTransform base_tr = mb->getBaseWorldTransform();
    Eigen::Vector3d pos(base_tr.getOrigin().x(), base_tr.getOrigin().y(), base_tr.getOrigin().z());
    btQuaternion q = base_tr.getRotation();
    Eigen::Quaterniond ori(q.w(), q.x(), q.y(), q.z());

    btVector3 base_vel = mb->getBaseVel();
    btVector3 base_omega = mb->getBaseOmega();
    Eigen::Vector3d lin_vel(base_vel.x(), base_vel.y(), base_vel.z());
    Eigen::Vector3d ang_vel(base_omega.x(), base_omega.y(), base_omega.z());

    robot->set_base_state(pos, ori, lin_vel, ang_vel);

    // Sync joint states
    for (int i = 0; i < robot->num_joints() && i < mb->getNumLinks(); i++) {
        auto& js = robot->joint_state(i);
        js.position = mb->getJointPos(i);
        js.velocity = mb->getJointVel(i);

        // Apply computed torque directly to physics
        if (i < static_cast<int>(motors.size())) {
            // Disable motor velocity target (set max force to 0)
            motors[i]->setVelocityTarget(0.0, 0.0);
            // Apply PD-computed torque directly
            mb->addJointTorque(i, js.torque);
        }
    }
}

void BulletWorld::set_terrain_heightfield(const std::vector<float>& heights,
                                            int rows, int cols,
                                            double resolution,
                                            double min_h, double max_h) {
    // Remove old terrain and its motion state
    if (terrain_body_) {
        dynamics_world_->removeRigidBody(terrain_body_.get());
        delete terrain_body_->getMotionState();
        terrain_body_.reset();
        terrain_shape_.reset();
    }

    terrain_data_ = heights;
    terrain_shape_ = std::make_unique<btHeightfieldTerrainShape>(
        cols, rows, terrain_data_.data(), 1.0f,
        min_h, max_h, 2 /* z-up */, PHY_FLOAT, false);

    terrain_shape_->setLocalScaling(btVector3(resolution, resolution, 1.0));

    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, 0, (min_h + max_h) / 2.0));

    btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo info(0.0, motion_state, terrain_shape_.get());
    info.m_friction = config_.ground_friction;
    terrain_body_ = std::make_unique<btRigidBody>(info);
    dynamics_world_->addRigidBody(terrain_body_.get());
}

std::vector<core::ContactInfo> BulletWorld::get_contacts() const {
    std::vector<core::ContactInfo> contacts;
    int num_manifolds = dynamics_world_->getDispatcher()->getNumManifolds();
    contacts.reserve(num_manifolds * 4);
    for (int i = 0; i < num_manifolds; i++) {
        btPersistentManifold* manifold = dynamics_world_->getDispatcher()->getManifoldByIndexInternal(i);
        int num_contacts = manifold->getNumContacts();
        for (int j = 0; j < num_contacts; j++) {
            btManifoldPoint& pt = manifold->getContactPoint(j);
            if (pt.getDistance() < 0.01) {
                core::ContactInfo ci;
                btVector3 p = pt.getPositionWorldOnA();
                btVector3 n = pt.m_normalWorldOnB;
                ci.position = Eigen::Vector3d(p.x(), p.y(), p.z());
                ci.normal = Eigen::Vector3d(n.x(), n.y(), n.z());
                ci.depth = -pt.getDistance();
                ci.normal_force = pt.getAppliedImpulse() / config_.timestep;
                contacts.push_back(ci);
            }
        }
    }
    return contacts;
}

bool BulletWorld::raycast(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                           Eigen::Vector3d& hit_point, Eigen::Vector3d& hit_normal) const {
    btVector3 bt_from(from.x(), from.y(), from.z());
    btVector3 bt_to(to.x(), to.y(), to.z());
    btCollisionWorld::ClosestRayResultCallback callback(bt_from, bt_to);
    dynamics_world_->rayTest(bt_from, bt_to, callback);
    if (callback.hasHit()) {
        hit_point = Eigen::Vector3d(callback.m_hitPointWorld.x(),
                                     callback.m_hitPointWorld.y(),
                                     callback.m_hitPointWorld.z());
        hit_normal = Eigen::Vector3d(callback.m_hitNormalWorld.x(),
                                      callback.m_hitNormalWorld.y(),
                                      callback.m_hitNormalWorld.z());
        return true;
    }
    return false;
}

} // namespace robosim::physics
