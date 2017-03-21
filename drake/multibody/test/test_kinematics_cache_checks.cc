/* clang-format off */
#include "drake/multibody/rigid_body_tree.h"
/* clang-format on */

#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::VectorXd;
using std::cerr;
using std::default_random_engine;
using std::endl;
using std::runtime_error;

struct CheckSettings {
  bool expect_error_on_configuration_methods;
  bool expect_error_on_velocity_methods;
  bool expect_error_on_jdot_times_v_methods;
};

template <typename O, typename F, typename... Args>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void checkForErrors(bool expect_error, O& object, F function,
                    Args&&... arguments) {
  try {
    (object.*function)(std::forward<Args>(arguments)...);
  } catch (runtime_error& e) {
    if (expect_error)
      return;
    else
      throw std::runtime_error(
          std::string("Caught an unexpected runtime error.\n") + e.what());
  }
  if (expect_error)
    throw std::runtime_error(
        "Expected a runtime error, but did not catch one.");
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void performChecks(RigidBodyTree<double>& model, KinematicsCache<double>& cache,
                   const CheckSettings& settings) {
  auto points = drake::Matrix3X<double>::Random(3, 5).eval();
  typedef decltype(points) PointsType;
  int body_or_frame_ind = 8;
  int base_or_frame_ind = 0;
  int expressed_in_frame_ind = body_or_frame_ind;
  int old_expressed_in_body_or_frame_ind = 7;
  int new_expressed_in_body_or_frame_ind = 3;
  bool in_terms_of_qdot = false;
  std::vector<int> v_or_qdot_indices;
  int npoints = 3;
  drake::TwistVector<double> spatial_acceleration;
  spatial_acceleration.setRandom();
  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;

  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::centerOfMass<double>, cache,
                 RigidBodyTreeConstants::default_model_instance_id_set);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::transformPoints<double, PointsType>,
                 cache, points, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::relativeQuaternion<double>, cache,
                 body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::relativeRollPitchYaw<double>, cache,
                 body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::worldMomentumMatrix<double>, cache,
                 RigidBodyTreeConstants::default_model_instance_id_set,
                 in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::centroidalMomentumMatrix<double>,
                 cache, RigidBodyTreeConstants::default_model_instance_id_set,
                 in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::centerOfMassJacobian<double>, cache,
                 RigidBodyTreeConstants::default_model_instance_id_set,
                 in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::geometricJacobian<double>, cache,
                 base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind,
                 in_terms_of_qdot, &v_or_qdot_indices);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::relativeTransform<double>, cache,
                 base_or_frame_ind, body_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::massMatrix<double>, cache);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::forwardKinPositionGradient<double>,
                 cache, npoints, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(
      settings.expect_error_on_configuration_methods, model,
      &RigidBodyTree<double>::transformPointsJacobian<double, PointsType>,
      cache, points, body_or_frame_ind, base_or_frame_ind, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::relativeQuaternionJacobian<double>,
                 cache, body_or_frame_ind, base_or_frame_ind, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::relativeRollPitchYawJacobian<double>,
                 cache, body_or_frame_ind, base_or_frame_ind, in_terms_of_qdot);

  checkForErrors(settings.expect_error_on_velocity_methods, model,
                 &RigidBodyTree<double>::relativeTwist<double>, cache,
                 base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind);

  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree<double>::geometricJacobianDotTimesV<double>,
                 cache, base_or_frame_ind, body_or_frame_ind,
                 expressed_in_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree<double>::worldMomentumMatrixDotTimesV<double>,
                 cache, RigidBodyTreeConstants::default_model_instance_id_set);
  checkForErrors(
      settings.expect_error_on_jdot_times_v_methods, model,
      &RigidBodyTree<double>::centroidalMomentumMatrixDotTimesV<double>, cache,
      RigidBodyTreeConstants::default_model_instance_id_set);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree<double>::centerOfMassJacobianDotTimesV<double>,
                 cache, RigidBodyTreeConstants::default_model_instance_id_set);
  checkForErrors(
      settings.expect_error_on_jdot_times_v_methods, model,
      &RigidBodyTree<double>::transformPointsJacobianDotTimesV<double,
                                                               PointsType>,
      cache, points, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(
      settings.expect_error_on_jdot_times_v_methods, model,
      &RigidBodyTree<double>::relativeQuaternionJacobianDotTimesV<double>,
      cache, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(
      settings.expect_error_on_jdot_times_v_methods, model,
      &RigidBodyTree<double>::relativeRollPitchYawJacobianDotTimesV<double>,
      cache, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree<double>::transformSpatialAcceleration<double>,
                 cache, spatial_acceleration, base_or_frame_ind,
                 body_or_frame_ind, old_expressed_in_body_or_frame_ind,
                 new_expressed_in_body_or_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree<double>::dynamicsBiasTerm<double>, cache,
                 no_external_wrenches, true);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree<double>::dynamicsBiasTerm<double>, cache,
                 no_external_wrenches, false);
}

int main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf",
      drake::multibody::joints::kRollPitchYaw, tree.get());

  CheckSettings settings;

  default_random_engine generator;
  VectorXd q = tree->getRandomConfiguration(generator);
  VectorXd v = VectorXd::Random(tree->get_num_velocities());

  // check before calling doKinematics
  {
    auto cache = tree->CreateKinematicsCache();
    settings.expect_error_on_configuration_methods = true;
    settings.expect_error_on_velocity_methods = true;
    settings.expect_error_on_jdot_times_v_methods = true;
    performChecks(*tree, cache, settings);
  }

  // q only, no gradients
  {
    KinematicsCache<double> cache = tree->doKinematics(q);
    settings.expect_error_on_configuration_methods = false;
    performChecks(*tree, cache, settings);
  }

  // q and v, no gradients, no jdot_times_v
  {
    KinematicsCache<double> cache = tree->doKinematics(q, v, false);
    settings.expect_error_on_jdot_times_v_methods = true;
    settings.expect_error_on_configuration_methods = false;
    settings.expect_error_on_velocity_methods = false;
    performChecks(*tree, cache, settings);
  }

  // q and v, no gradients, with jdot_times_v
  {
    KinematicsCache<double> cache = tree->doKinematics(q, v, true);
    settings.expect_error_on_configuration_methods = false;
    settings.expect_error_on_velocity_methods = false;
    settings.expect_error_on_jdot_times_v_methods = false;
    performChecks(*tree, cache, settings);
  }

  return 0;
}
