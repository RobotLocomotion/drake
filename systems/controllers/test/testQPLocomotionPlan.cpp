#include "QPLocomotionPlan.h"
#include <iostream>
#include <random>

using namespace std;
using namespace Eigen;

default_random_engine generator;

// possible tests:
// * assert error when no pelvis passed in
// *

int main(int argc, char **argv) {
  RigidBodyManipulator robot("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  QPLocomotionPlanSettings settings;
  RigidBodySupportState support_state;
  RigidBodySupportStateElement support_state_element;
  support_state_element.body = robot.findLinkId("r_foot");
  support_state_element.contact_groups.push_back("toe");
  support_state_element.contact_points = Matrix3Xd::Random(3, 4);
  support_state_element.contact_surface = 0;
  support_state.push_back(support_state_element);
  QPLocomotionPlanSettings::ContactGroupNameToContactPointsMap contact_group;
  contact_group["toe"] = Matrix3Xd::Random(3, 2);
  settings.addSupport(support_state, contact_group, 1.0);
  settings.duration = settings.support_times[settings.support_times.size() - 1];

//  BodyMotionData body_motion;
//  settings.body_motions.push_back(body_motion);   // TODO
  settings.zmp_trajectory = PiecewisePolynomial<double>::random(2, 1, 3, PiecewiseFunction::randomSegmentTimes(4, generator));
  settings.zmp_final = Vector2d::Random();
  settings.lipm_height = 0.9;

  int num_segments = 3;
  Matrix4d S = Matrix4d::Random();
  S = S * S.transpose();
  MatrixXd K = MatrixXd::Random(4, 4);
  MatrixXd A = MatrixXd::Random(4, 4);
  std::vector<VectorXd> alpha;
  for (int i = 0; i < num_segments; ++i) {
    alpha.push_back(VectorXd::Random(4));
  }
  PiecewisePolynomial<double> polynomial_part = PiecewisePolynomial<double>::random(4, 1, 5, PiecewiseFunction::randomSegmentTimes(num_segments, generator));
  ExponentialPlusPiecewisePolynomial<double> s1(K, A, alpha, polynomial_part);
  settings.V = QuadraticLyapunovFunction(S, s1);
  settings.q_traj = PiecewisePolynomial<double>::random(robot.num_positions, 1, 3, PiecewiseFunction::randomSegmentTimes(4, generator));

//  settings.com_traj;   // TODO
  settings.default_qp_input.be_silent = false;

  QPLocomotionPlan plan(robot, settings, "qp_controller_input");
  VectorXd q = VectorXd::Random(robot.num_positions);
  VectorXd v = VectorXd::Random(robot.num_positions);
  vector<bool> contact_force_detected;
  bernoulli_distribution distribution(0.5);
  for (int i = 0; i < robot.num_bodies; ++i) {
    contact_force_detected.push_back(distribution(generator));
  }

  plan.publishQPControllerInput(0.1, q, v, contact_force_detected);

  cout << "test passed" << endl;
  return 0;
}
