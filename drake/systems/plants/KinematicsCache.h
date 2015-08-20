#ifndef DRAKE_KINEMATICSCACHE_H
#define DRAKE_KINEMATICSCACHE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <vector>
#include <cassert>
#include "GradientVar.h"
#include "drakeGeometryUtil.h"
#include "RigidBody.h"
#include <stdexcept>
#include <utility>

template <typename Scalar>
class KinematicsCacheElement
{
public:
  /*
   * Configuration dependent
   */
  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> transform_to_world;
  typename Gradient<typename decltype(transform_to_world)::MatrixType, Eigen::Dynamic>::type dtransform_to_world_dq;
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> motion_subspace_in_body; // gradient w.r.t. q_i only
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> motion_subspace_in_world; // gradient w.r.t. q
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> qdot_to_v; // gradient w.r.t. q
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> v_to_qdot; // gradient w.r.t. q
  GradientVar<Scalar, TWIST_SIZE, TWIST_SIZE> inertia_in_world;
  GradientVar<Scalar, TWIST_SIZE, TWIST_SIZE> crb_in_world;

  /*
   * Configuration and velocity dependent
   */
  GradientVar<Scalar, TWIST_SIZE, 1> twist_in_world; // gradient w.r.t. q only; gradient w.r.t. v is motion_subspace_in_world
  GradientVar<Scalar, TWIST_SIZE, 1> motion_subspace_in_body_dot_times_v; // gradient w.r.t. q_i and v_i only
  GradientVar<Scalar, TWIST_SIZE, 1> motion_subspace_in_world_dot_times_v; // gradient w.r.t. q and v

public:
  KinematicsCacheElement(int num_positions_robot, int num_velocities_robot, int num_positions_joint, int num_velocities_joint, int gradient_order) :
      dtransform_to_world_dq(HOMOGENEOUS_TRANSFORM_SIZE, num_positions_robot),
      motion_subspace_in_body(TWIST_SIZE, num_velocities_joint, num_positions_joint, gradient_order),
      motion_subspace_in_world(TWIST_SIZE, num_velocities_joint, num_positions_robot, gradient_order),
      qdot_to_v(num_velocities_joint, num_positions_joint, num_positions_robot, gradient_order),
      v_to_qdot(num_positions_joint, num_velocities_joint, num_positions_robot, gradient_order),
      inertia_in_world(TWIST_SIZE, TWIST_SIZE, num_positions_robot, gradient_order),
      crb_in_world(TWIST_SIZE, TWIST_SIZE, num_positions_robot, gradient_order),
      twist_in_world(TWIST_SIZE, 1, num_positions_robot, gradient_order),
      motion_subspace_in_body_dot_times_v(TWIST_SIZE, 1, num_positions_joint + num_velocities_joint, gradient_order),
      motion_subspace_in_world_dot_times_v(TWIST_SIZE, 1, num_positions_robot + num_velocities_robot, gradient_order)
  {
    // empty
  }
};

template <typename Scalar>
class KinematicsCache
{
private:
  std::unordered_map<RigidBody const *, KinematicsCacheElement<Scalar>> elements;

public:
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> v;
  int gradient_order;
  bool position_kinematics_cached;
  bool gradients_cached;
  bool velocity_kinematics_cached;
  bool jdotV_cached;
  int cached_inertia_gradients_order;

public:
  KinematicsCache(const std::vector<std::shared_ptr<RigidBody>>& bodies, int gradient_order) :
      q(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(getNumPositions(bodies), 1)),
      v(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(0, 1)), // initialize to zero length; doKinematics must resize to getNumVelocities(bodies) if v is passed in
      gradient_order(gradient_order),
      position_kinematics_cached(false),
      gradients_cached(false),
      velocity_kinematics_cached(false),
      jdotV_cached(false),
      cached_inertia_gradients_order(-1)
  {
    assert(gradient_order == 0 || gradient_order == 1);

    for (const auto& body_shared_ptr : bodies) {
      const RigidBody& body = *body_shared_ptr;
      int num_positions_joint = body.hasParent() ? body.getJoint().getNumPositions() : 0;
      int num_velocities_joint = body.hasParent() ? body.getJoint().getNumVelocities() : 0;
      elements.insert(std::make_pair(&body, KinematicsCacheElement<Scalar>(q.size(), v.size(), num_positions_joint, num_velocities_joint, gradient_order)));
    }
  }

  KinematicsCacheElement<Scalar>& getElement(const RigidBody& body)
  {
    return elements.at(&body);
  }

  const KinematicsCacheElement<Scalar>& getElement(const RigidBody& body) const
  {
    return elements.at(&body);
  }

  void checkCachedKinematicsSettings(bool kinematics_gradients_required, bool velocity_kinematics_required, bool jdot_times_v_required, const std::string& method_name) const
  {
    std::string message;
    if (!position_kinematics_cached) {
      message = method_name + " requires position kinematics, which have not been cached. Please call doKinematics.";
    }
    if (kinematics_gradients_required && ! gradients_cached) {
      message = method_name + " requires kinematics gradients, which have not been cached. Please call doKinematics with compute_gradients set to true.";
    }
    if (velocity_kinematics_required && ! velocity_kinematics_cached) {
      message = method_name + " requires velocity kinematics, which have not been cached. Please call doKinematics with a velocity vector.";
    }
    if (jdot_times_v_required && !jdotV_cached) {
      message = method_name + " requires Jdot times v, which has not been cached. Please call doKinematics with a velocity vector and compute_JdotV set to true.";
    }
    if (message.length() > 0) {
      throw std::runtime_error(message.c_str());
    }
  }

  static int getNumPositions(const std::vector<std::shared_ptr<RigidBody>>& bodies) {
    int ret = 0;
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
      RigidBody& body = **it;
      if (body.hasParent())
        ret += body.getJoint().getNumPositions();
    }
    return ret;
  }

  static int getNumVelocities(const std::vector<std::shared_ptr<RigidBody>>& bodies) {
    int ret = 0;
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
      RigidBody& body = **it;
      if (body.hasParent())
        ret += body.getJoint().getNumVelocities();
    }
    return ret;
  }
};


#endif //DRAKE_KINEMATICSCACHE_H
