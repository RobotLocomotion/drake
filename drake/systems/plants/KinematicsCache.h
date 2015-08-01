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
  GradientVar<Scalar, TWIST_SIZE, 1> motion_subspace_in_body_dot_times_v; // gradient w.r.t. q and v
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
      motion_subspace_in_body_dot_times_v(TWIST_SIZE, 1, num_positions_robot + num_velocities_robot, gradient_order),
      motion_subspace_in_world_dot_times_v(TWIST_SIZE, 1, num_positions_robot + num_velocities_robot, gradient_order)
  {
    // empty
  }
};

template <typename Scalar>
class KinematicsCache
{
private:
  int gradient_order;
  std::unordered_map<RigidBody*, KinematicsCacheElement<Scalar>> elements;

public:
  KinematicsCache(const std::vector<std::shared_ptr<RigidBody>>& bodies, int gradient_order) :
      gradient_order(gradient_order)
  {
    assert(gradient_order == 0 || gradient_order == 1);

    int num_positions_robot = 0;
    int num_velocities_robot = 0;
    for (const auto& body_shared_ptr : bodies) {
      if (body_shared_ptr->hasParent()) {
        num_positions_robot += body_shared_ptr->getJoint().getNumPositions();
        num_positions_robot += body_shared_ptr->getJoint().getNumPositions();
      }
    }

    for (const auto& body_shared_ptr : bodies) {
      const RigidBody& body = *body_shared_ptr;
      int num_positions_joint = body.hasParent() ? body.getJoint().getNumPositions() : 0;
      int num_velocities_joint = body.hasParent() ? body.getJoint().getNumVelocities() : 0;
      elements[&body] = KinematicsCacheElement<Scalar>(num_positions_robot, num_velocities_robot, num_positions_joint, num_velocities_joint, gradient_order);
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
};


#endif //DRAKE_KINEMATICSCACHE_H
