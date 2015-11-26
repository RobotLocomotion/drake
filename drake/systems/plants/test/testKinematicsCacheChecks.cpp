#include "RigidBodyManipulator.h"
#include "RigidBodyManipulator.h"
#include "drakeGeometryUtil.h"
#include "GradientVar.h"
#include <iostream>
#include <cstdlib>
#include <random>
#include <memory>
#include <stdexcept>
#include <map>

using namespace std;
using namespace Eigen;

struct CheckSettings {
  bool expect_error_on_configuration_methods;
  bool expect_error_on_velocity_methods;
  bool expect_error_on_jdot_times_v_methods;
};

template<typename O, typename F, typename ...Args>
void checkForErrors(bool expect_error, O &object, F function, Args &&... arguments) {
  try {
    (object.*function)(std::forward<Args>(arguments)...);
  }
  catch (runtime_error &e) {
    if (expect_error)
      return;
    else
      throw std::runtime_error(std::string("Caught an unexpected runtime error.\n") + e.what());
  }
  if (expect_error)
    throw std::runtime_error("Expected a runtime error, but did not catch one.");
}

void performChecks(RigidBodyManipulator &model, KinematicsCache<double> &cache, const CheckSettings &settings) {
  auto points = Matrix<double, 3, Eigen::Dynamic>::Random(3, 5).eval();
  typedef decltype(points) PointsType;
  int body_or_frame_ind = 8;
  int base_or_frame_ind = 0;
  int expressed_in_frame_ind = body_or_frame_ind;
  int old_expressed_in_body_or_frame_ind = 7;
  int new_expressed_in_body_or_frame_ind = 3;
  int rotation_type = 0;
  bool in_terms_of_qdot = false;
  std::vector<int> v_or_qdot_indices;
  int npoints = 3;
  Matrix<double, TWIST_SIZE, 1> spatial_acceleration;
  spatial_acceleration.setRandom();
  eigen_aligned_unordered_map<RigidBody const *, Matrix<double, TWIST_SIZE, 1> > f_ext;

  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::centerOfMass<double>, cache, RigidBodyManipulator::default_robot_num_set);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::forwardKin<double, PointsType>, cache, points, body_or_frame_ind, base_or_frame_ind, rotation_type);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::worldMomentumMatrix<double>, cache, RigidBodyManipulator::default_robot_num_set, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::centroidalMomentumMatrix<double>, cache, RigidBodyManipulator::default_robot_num_set, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::centerOfMassJacobian<double>, cache, RigidBodyManipulator::default_robot_num_set, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::geometricJacobian<double>, cache, base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind,
                 in_terms_of_qdot, &v_or_qdot_indices);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::relativeTransform<double>, cache, base_or_frame_ind, body_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::massMatrix<double>, cache);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::forwardKinPositionGradient<double>, cache, npoints, body_or_frame_ind, base_or_frame_ind);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::forwardKinJacobian<double, PointsType>, cache, points, body_or_frame_ind, base_or_frame_ind,
                 rotation_type, in_terms_of_qdot);

  checkForErrors(settings.expect_error_on_velocity_methods, model, &RigidBodyManipulator::relativeTwist<double>, cache, base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind);

  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::geometricJacobianDotTimesV<double>, cache, base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::worldMomentumMatrixDotTimesV<double>, cache, RigidBodyManipulator::default_robot_num_set);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<double>, cache, RigidBodyManipulator::default_robot_num_set);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::centerOfMassJacobianDotTimesV<double>, cache, RigidBodyManipulator::default_robot_num_set);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::forwardJacDotTimesV<double, PointsType>, cache, points, body_or_frame_ind, base_or_frame_ind,
                 rotation_type);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::transformSpatialAcceleration<double>, cache, spatial_acceleration, base_or_frame_ind, body_or_frame_ind,
                 old_expressed_in_body_or_frame_ind, new_expressed_in_body_or_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::dynamicsBiasTerm<double>, cache, f_ext);
}

int main() {
  std::unique_ptr<RigidBodyManipulator> model(new RigidBodyManipulator("examples/Atlas/urdf/atlas_minimal_contact.urdf"));
  if (model == nullptr) {
    cerr << "ERROR: Failed to load model" << endl;
  }
  CheckSettings settings;

  VectorXd q = VectorXd::Random(model->num_positions);
  VectorXd v = VectorXd::Random(model->num_velocities);

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
