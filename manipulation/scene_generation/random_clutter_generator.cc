#include "drake/manipulation/scene_generation/random_clutter_generator.h"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/math/random_rotation.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace manipulation {
namespace scene_generation {
using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::map;
using std::string;
using std::stringstream;
using std::vector;
using std::default_random_engine;
using std::uniform_real_distribution;

namespace {
// TODO(naveenoid) : Replace with VectorXi::LinSpaced.
Eigen::VectorXi GenerateIndices(int num_elements) {
  Eigen::VectorXi cumsum = Eigen::VectorXi::Constant(num_elements, 0);
  int i = 0;
  for (int it = 0; it < cumsum.size(); ++it) {
    cumsum(it) = i++;
  }
  return cumsum;
}

VectorX<double> GenerateBoundedRandomSample(
    std::default_random_engine* generator, VectorX<double> min,
    VectorX<double> max) {
  DRAKE_DEMAND(min.size() == max.size());
  VectorX<double> return_vector = VectorX<double>::Zero(min.size());
  for (int i = 0; i < min.size(); ++i) {
    uniform_real_distribution<double> distribution(min(i), max(i));
    return_vector(i) = distribution(*generator);
  }
  return return_vector;
}

}  // namespace

RandomClutterGenerator::RandomClutterGenerator(
    RigidBodyTree<double>* scene_tree,
    const std::set<int>& clutter_model_instances,
    const Vector3<double>& clutter_center, const Vector3<double>& clutter_size,
    double min_inter_object_distance)
    : scene_tree_ptr_(scene_tree),
      clutter_model_instances_(clutter_model_instances),
      clutter_center_(clutter_center),
      clutter_lb_(-0.5 * clutter_size),
      clutter_ub_(0.5 * clutter_size),
      inter_object_distance_(min_inter_object_distance) {
  // Checks that the number of requested instances > 0 & < the total number
  // of instances in the tree.
  DRAKE_DEMAND(clutter_model_instances.size() >= 1);
  for (auto& it : clutter_model_instances) {
    // check that the tree contains the model instance in question. i.e
    // atleast one body exists for each model instance listed in
    // clutter_model_instance.
    DRAKE_DEMAND(scene_tree_ptr_->FindModelInstanceBodies(it).size() > 0);
  }
}

VectorX<double> RandomClutterGenerator::GenerateFloatingClutter(
    const VectorX<double>& q_nominal, std::default_random_engine* generator,
    optional<double> z_height_cost) {
  DRAKE_DEMAND(scene_tree_ptr_->get_num_positions() == q_nominal.size());

  VectorX<double> q_nominal_candidate = q_nominal;
  VectorX<double> q_ik_result = q_nominal;

  int ik_result_code = 100;
  // Keep running the IK until a feasible solution is found.
  while (ik_result_code > 1) {
    drake::log()->debug("IK new run initiated on tree of size {}.",
                        scene_tree_ptr_->get_num_positions());

    // Setup constraint array.
    std::vector<RigidBodyConstraint*> constraint_array;

    // set MinDistanceConstraint
    drake::log()->debug("Adding MinDistanceConstraint.");
    std::vector<int> active_bodies_idx;
    std::set<std::string> active_group_names;
    MinDistanceConstraint min_distance_constraint(
        scene_tree_ptr_, inter_object_distance_ /* min_distance */,
        active_bodies_idx, active_group_names);
    constraint_array.push_back(&min_distance_constraint);

    Eigen::VectorXi linear_posture_iAfun = GenerateIndices(q_nominal.size()),
                    linear_posture_jAvar = linear_posture_iAfun;
    VectorX<double> linear_posture_A = VectorX<double>::Ones(q_nominal.size()),
                    linear_posture_lb = q_nominal,
                    linear_posture_ub = q_nominal;
    VectorX<double> q_initial = q_nominal;
    std::vector<int> z_indices;

    Vector3<double> bounded_position;
    // Iterate through each of the model instances of the clutter and add
    // elements
    // to the linear posture_constraint.
    for (auto& instance : clutter_model_instances_) {
      auto model_instance_bodies =
          scene_tree_ptr_->FindModelInstanceBodies(instance);
      for (size_t i = 0; i < model_instance_bodies.size(); ++i) {
        auto body = model_instance_bodies[i];
        if (body->has_joint()) {
          const DrakeJoint* joint = &body->getJoint();
          if (!joint->is_fixed()) {
            int joint_dofs = joint->get_num_positions();
            // Enforces checks only for Floating quaternion joints.
            DRAKE_DEMAND(joint_dofs == 1 || joint_dofs == 7);
            VectorX<double> joint_lb = VectorX<double>::Zero(joint_dofs);
            VectorX<double> joint_ub = joint_lb;
            VectorX<double> joint_initial = joint_ub;
            VectorX<double> joint_nominal = joint_ub;
            if (joint_dofs == 7) {
              // If the num dofs of the joint is 7 its a floating quaternion
              // joint.The position part to be bounded by the clutter bounding
              // box, the orientation part to be a random quaternion.
              // Position
              joint_lb.head(3) = clutter_lb_;
              joint_ub.head(3) = clutter_ub_;
              auto temp_out = GenerateBoundedRandomSample(
                  generator, clutter_lb_, clutter_ub_);
              joint_initial.head(3) = temp_out;
              z_indices.push_back(body->get_position_start_index() + 2);
              joint_nominal.head(3) = Vector3<double>::Zero(3);
              // Orientation
              Eigen::Quaterniond quat =
                  drake::math::UniformlyRandomQuaternion(generator);
              joint_lb[3] = quat.w();
              joint_lb[4] = quat.x();
              joint_lb[5] = quat.y();
              joint_lb[6] = quat.z();
              joint_ub.tail(4) = joint_lb.tail(4);
              joint_initial.tail(4) = joint_ub.tail(4);
              joint_nominal.tail(4) = joint_ub.tail(4);
            } else if (joint_dofs == 1) {
              // If the num dofs of the joint is 1, set lb, ub to joint_lim_min,
              // joint_lim_max.
              joint_lb[0] = joint->getJointLimitMin()[0];
              joint_ub[0] = joint->getJointLimitMin()[0];
              joint_initial =
                  GenerateBoundedRandomSample(generator, joint_lb, joint_ub);
              joint_nominal = joint_initial;
            }
            // The indexing has to be computed directly here since we are using
            // InverseKinematics' API.
            // TODO(naveenoid) : Consider moving to a MathematicalProgram
            // formulation of the problem to use that API directly and use
            // symbolic mapping.
            linear_posture_lb.segment(body->get_position_start_index(),
                                      joint_dofs) = joint_lb;
            linear_posture_ub.segment(body->get_position_start_index(),
                                      joint_dofs) = joint_ub;
            q_initial.segment(body->get_position_start_index(), joint_dofs) =
                joint_initial;
            q_nominal_candidate.segment(body->get_position_start_index(),
                                        joint_dofs) = joint_nominal;
          }
        }
      }
    }

    linear_posture_A = VectorX<double>::Ones(linear_posture_iAfun.size());

    drake::log()->debug("Adding SingleTimeLinearPostureConstraint");

    SingleTimeLinearPostureConstraint linear_posture_constraint(
        scene_tree_ptr_, linear_posture_iAfun, linear_posture_jAvar,
        linear_posture_A, linear_posture_lb, linear_posture_ub);

    constraint_array.push_back(&linear_posture_constraint);
    drake::log()->debug("Constraint array size {}", constraint_array.size());

    Eigen::MatrixXd Q =
        Eigen::MatrixXd::Zero(q_initial.size(), q_initial.size());

    IKoptions ikoptions(scene_tree_ptr_);
    if (z_height_cost) {
      DRAKE_DEMAND(*z_height_cost > 0);
      for (auto& it : z_indices) {
        Q(it, it) = *z_height_cost;
      }
      ikoptions.setQ(Q);
    }

    ikoptions.setDebug(true);

    // setup IK problem and run.
    IKResults ik_results =
        inverseKinSimple(scene_tree_ptr_, q_initial, q_nominal_candidate,
                         constraint_array, ikoptions);

    for (auto it : ik_results.info) {
      drake::log()->info("IK Result code : {}", it);
      ik_result_code = it;
      if (ik_result_code > 1) {
        drake::log()->debug("IK failure, recomputing IK");
      }
    }

    if (!ik_results.q_sol.empty()) {
      q_ik_result = ik_results.q_sol.back();
    }
  }
  return q_ik_result;
}

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake
