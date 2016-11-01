#include "drake/systems/plants/RigidBodyTree.h"

#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
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
void performChecks(RigidBodyTree& model, KinematicsCache<double>& cache,
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
  const RigidBodyTree::BodyToWrenchMap<double> no_external_wrenches;

  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::centerOfMass<double>, cache,
                 RigidBodyTree::default_model_instance_id_set);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::transformPoints<double, PointsType>, cache,
                 points, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::relativeQuaternion<double>, cache,
                 body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::relativeRollPitchYaw<double>, cache,
                 body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::worldMomentumMatrix<double>, cache,
                 RigidBodyTree::default_model_instance_id_set,
                 in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::centroidalMomentumMatrix<double>, cache,
                 RigidBodyTree::default_model_instance_id_set,
                 in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::centerOfMassJacobian<double>, cache,
                 RigidBodyTree::default_model_instance_id_set,
                 in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::geometricJacobian<double>, cache,
                 base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind,
                 in_terms_of_qdot, &v_or_qdot_indices);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::relativeTransform<double>, cache,
                 base_or_frame_ind, body_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::massMatrix<double>, cache);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::forwardKinPositionGradient<double>, cache,
                 npoints, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::transformPointsJacobian<double, PointsType>,
                 cache, points, body_or_frame_ind, base_or_frame_ind,
                 in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::relativeQuaternionJacobian<double>, cache,
                 body_or_frame_ind, base_or_frame_ind, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::relativeRollPitchYawJacobian<double>, cache,
                 body_or_frame_ind, base_or_frame_ind, in_terms_of_qdot);

  checkForErrors(settings.expect_error_on_velocity_methods, model,
                 &RigidBodyTree::relativeTwist<double>, cache,
                 base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind);

  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree::geometricJacobianDotTimesV<double>, cache,
                 base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree::worldMomentumMatrixDotTimesV<double>, cache,
                 RigidBodyTree::default_model_instance_id_set);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree::centroidalMomentumMatrixDotTimesV<double>,
                 cache, RigidBodyTree::default_model_instance_id_set);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree::centerOfMassJacobianDotTimesV<double>, cache,
                 RigidBodyTree::default_model_instance_id_set);
  checkForErrors(
      settings.expect_error_on_jdot_times_v_methods, model,
      &RigidBodyTree::transformPointsJacobianDotTimesV<double, PointsType>,
      cache, points, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree::relativeQuaternionJacobianDotTimesV<double>,
                 cache, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree::relativeRollPitchYawJacobianDotTimesV<double>,
                 cache, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree::transformSpatialAcceleration<double>, cache,
                 spatial_acceleration, base_or_frame_ind, body_or_frame_ind,
                 old_expressed_in_body_or_frame_ind,
                 new_expressed_in_body_or_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model,
                 &RigidBodyTree::dynamicsBiasTerm<double>, cache,
                 no_external_wrenches, true);
  checkForErrors(settings.expect_error_on_configuration_methods, model,
                 &RigidBodyTree::dynamicsBiasTerm<double>, cache,
                 no_external_wrenches, false);
}

int main() {
  std::unique_ptr<RigidBodyTree> model(
      new RigidBodyTree("examples/Atlas/urdf/atlas_minimal_contact.urdf"));
  if (model == nullptr) {
    cerr << "ERROR: Failed to load model" << endl;
  }
  CheckSettings settings;

  default_random_engine generator;
  VectorXd q = model->getRandomConfiguration(generator);
  VectorXd v = VectorXd::Random(model->get_num_velocities());

  // check before calling doKinematics
  {
    KinematicsCache<double> cache(model->bodies);
    settings.expect_error_on_configuration_methods = true;
    settings.expect_error_on_velocity_methods = true;
    settings.expect_error_on_jdot_times_v_methods = true;
    performChecks(*model, cache, settings);
  }

  // q only, no gradients
  {
    KinematicsCache<double> cache = model->doKinematics(q);
    settings.expect_error_on_configuration_methods = false;
    performChecks(*model, cache, settings);
  }

  // q and v, no gradients, no jdot_times_v
  {
    KinematicsCache<double> cache = model->doKinematics(q, v, false);
    settings.expect_error_on_jdot_times_v_methods = true;
    settings.expect_error_on_configuration_methods = false;
    settings.expect_error_on_velocity_methods = false;
    performChecks(*model, cache, settings);
  }

  // q and v, no gradients, with jdot_times_v
  {
    KinematicsCache<double> cache = model->doKinematics(q, v, true);
    settings.expect_error_on_configuration_methods = false;
    settings.expect_error_on_velocity_methods = false;
    settings.expect_error_on_jdot_times_v_methods = false;
    performChecks(*model, cache, settings);
  }

  return 0;
}
