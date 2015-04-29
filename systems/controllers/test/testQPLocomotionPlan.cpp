#include "QPLocomotionPlan.h"
#include <iostream>
#include <random>

using namespace std;
using namespace Eigen;

default_random_engine generator;

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
  contact_group["toe"] = Matrix3Xd::Random();
  settings.addSupport(support_state, contact_group, 1.0);
  settings.duration = settings.support_times[settings.support_times.size() - 1];

//  BodyMotionData body_motion;
//  settings.body_motions.push_back(body_motion);   // TODO
  settings.zmp_trajectory = PiecewisePolynomial<double>::random(2, 1, 3, PiecewiseFunction::randomSegmentTimes(4, generator));
  settings.zmp_final = Vector2d::Random();
  settings.lipm_height = 0.9;
  settings.V.S = Matrix4d::Random();
  settings.V.S = settings.V.S.transpose() * settings.V.S;
  settings.q_traj = PiecewisePolynomial<double>::random(robot.num_positions, 1, 3, PiecewiseFunction::randomSegmentTimes(4, generator));

//  settings.com_traj;   // TODO
  settings.default_qp_input.be_silent = false;

  QPLocomotionPlan plan(robot, settings, "qp_controller_input");

  cout << "test passed" << endl;
  return 0;
}
