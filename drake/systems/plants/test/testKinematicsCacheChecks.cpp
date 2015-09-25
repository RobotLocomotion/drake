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
void checkForErrors(bool expect_error, O& object, F function, Args&&... arguments)
{
  try {
    (object.*function)(std::forward<Args>(arguments)...);
  }
  catch (runtime_error& e) {
    if (expect_error)
      return;
    else
      throw std::runtime_error(std::string("Caught an unexpected runtime error.\n") + e.what());
  }
  if (expect_error)
    throw std::runtime_error("Expected a runtime error, but did not catch one.");
}

void performChecks(RigidBodyManipulator& model, KinematicsCache<double>& cache, int gradient_order, const CheckSettings& settings)
{
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
  int nq = model.num_positions;
  GradientVar<double, TWIST_SIZE, 1> spatial_acceleration(TWIST_SIZE, 1, nq, gradient_order);
  spatial_acceleration.value().setRandom();
  if (gradient_order > 0)
    spatial_acceleration.gradient().value().setRandom();
  std::map<int, std::unique_ptr<GradientVar<double, TWIST_SIZE, 1> > > f_ext;

  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::worldMomentumMatrix<double>, cache, gradient_order, RigidBody::defaultRobotNumSet, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::centroidalMomentumMatrix<double>, cache, gradient_order, RigidBody::defaultRobotNumSet, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::centerOfMass<double>, cache, gradient_order + 1, RigidBody::defaultRobotNumSet);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::centerOfMassJacobian<double>, cache, gradient_order, RigidBody::defaultRobotNumSet, in_terms_of_qdot);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::geometricJacobian<double>, cache, base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind, gradient_order, in_terms_of_qdot, &v_or_qdot_indices);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::relativeTransform<double>, cache, base_or_frame_ind, body_or_frame_ind, gradient_order);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::massMatrix<double>, cache, gradient_order);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::forwardKin<PointsType>, cache, points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order + 1);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::forwardKinPositionGradient<double>, cache, npoints, body_or_frame_ind, base_or_frame_ind, gradient_order);
  checkForErrors(settings.expect_error_on_configuration_methods, model, &RigidBodyManipulator::forwardKinJacobian<PointsType>, cache, points, body_or_frame_ind, base_or_frame_ind, rotation_type, in_terms_of_qdot,
                 gradient_order);

  checkForErrors(settings.expect_error_on_velocity_methods, model, &RigidBodyManipulator::relativeTwist<double>, cache, base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind, gradient_order);

  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::geometricJacobianDotTimesV<double>, cache, base_or_frame_ind, body_or_frame_ind, expressed_in_frame_ind, gradient_order);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::worldMomentumMatrixDotTimesV<double>, cache, gradient_order, RigidBody::defaultRobotNumSet);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<double>, cache, gradient_order, RigidBody::defaultRobotNumSet);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::centerOfMassJacobianDotTimesV<double>, cache, gradient_order, RigidBody::defaultRobotNumSet);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::forwardJacDotTimesV<PointsType>, cache, points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::transformSpatialAcceleration<double>, cache, spatial_acceleration, base_or_frame_ind, body_or_frame_ind, old_expressed_in_body_or_frame_ind, new_expressed_in_body_or_frame_ind);
  checkForErrors(settings.expect_error_on_jdot_times_v_methods, model, &RigidBodyManipulator::inverseDynamics<double>, cache, f_ext, nullptr, gradient_order);
}

int main()
{
  std::unique_ptr<RigidBodyManipulator> model(new RigidBodyManipulator("examples/Atlas/urdf/atlas_minimal_contact.urdf"));
  if (model == nullptr)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }
  CheckSettings settings;
  int max_gradient_order = 2;

  VectorXd q = VectorXd::Random(model->num_positions);
  VectorXd v = VectorXd::Random(model->num_velocities);

  // check before calling doKinematics
  {
    KinematicsCache<double> cache(model->bodies, 1);
    settings.expect_error_on_configuration_methods = true;
    settings.expect_error_on_velocity_methods = true;
    settings.expect_error_on_jdot_times_v_methods = true;
    for (int gradient_order = 0; gradient_order < max_gradient_order; gradient_order++)
      performChecks(*model, cache, gradient_order, settings);
  }

  // q only, no gradients
  {
    KinematicsCache<double> cache = model->doKinematics(q, 0);
    for (int gradient_order = 1; gradient_order < max_gradient_order; gradient_order++)
      performChecks(*model, cache, gradient_order, settings); // still expect everything to fail for gradient_order > 0
    settings.expect_error_on_configuration_methods = false;
    performChecks(*model, cache, 0, settings);
  }

  // q only, with gradients
  {
    KinematicsCache<double> cache = model->doKinematics(q, 1);
    for (int gradient_order = 0; gradient_order < max_gradient_order; gradient_order++)
      performChecks(*model, cache, gradient_order, settings);
  }

  // q and v, no gradients, no jdot_times_v
  {
    KinematicsCache<double> cache = model->doKinematics(q, v, 0, false);
    settings.expect_error_on_configuration_methods = true;
    settings.expect_error_on_velocity_methods = true;
    settings.expect_error_on_jdot_times_v_methods = true;
    for (int gradient_order = 1; gradient_order < max_gradient_order; gradient_order++)
      performChecks(*model, cache, gradient_order, settings); // still expect everything to fail for gradient_order > 0
    settings.expect_error_on_configuration_methods = false;
    settings.expect_error_on_velocity_methods = false;
    performChecks(*model, cache, 0, settings);
  }

  // q and v, with gradients, no jdot_times_v
  {
    KinematicsCache<double> cache = model->doKinematics(q, v, 1, false);
    for (int gradient_order = 0; gradient_order < max_gradient_order; gradient_order++)
      performChecks(*model, cache, gradient_order, settings);
  }

  // q and v, no gradients, with jdot_times_v
  {
    KinematicsCache<double> cache = model->doKinematics(q, v, 0, true);
    settings.expect_error_on_configuration_methods = true;
    settings.expect_error_on_velocity_methods = true;
    settings.expect_error_on_jdot_times_v_methods = true;
    for (int gradient_order = 1; gradient_order < max_gradient_order; gradient_order++)
      performChecks(*model, cache, gradient_order, settings); // still expect everything to fail for gradient_order > 0
    settings.expect_error_on_configuration_methods = false;
    settings.expect_error_on_velocity_methods = false;
    settings.expect_error_on_jdot_times_v_methods = false;
    performChecks(*model, cache, 0, settings);
  }

  // q and v, with gradients, with jdot_times_v
  {
    KinematicsCache<double> cache = model->doKinematics(q, v, 1, true);
    for (int gradient_order = 0; gradient_order < max_gradient_order; gradient_order++)
      performChecks(*model, cache, gradient_order, settings);
  }

  return 0;
}
