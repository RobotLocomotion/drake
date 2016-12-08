#include "drake/multibody/rbt_with_alternates/rigid_body_tree_with_alternates.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/parser_urdf.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <utility>

using drake::AutoDiffXd;
using drake::math::initializeAutoDiffGivenGradientMatrix;
using drake::RigidBodyTreeWithAlternates;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using std::complex;
using std::cout;
using std::endl;
using std::unique_ptr;
using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

int main() {
  // Fill in the "MultibodyTree" first.
  auto tree = make_unique<RigidBodyTree<double>>();

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());

#if 0
  tree->AddJoint(make_unique<PinJoint<double>>(1.1));
  tree->AddJoint(make_unique<SliderJoint<double>>(2.2));
  tree->AddJoint(make_unique<PinJoint<double>>(3.3));
  tree->AddJoint(make_unique<SliderJoint<double>>(4.4));
#endif

  // Create the fundamental MBSystem (that is, type double).
  RigidBodyTreeWithAlternates<double> tree_with_alternates(std::move(tree));

  // Create some alternate instantiations of MBSystem (kept within the
  // fundamental system).
  // Adds double alternate so that I can request it below.
  RigidBodyTreeWithAlternates<double>::AddAlternate(tree_with_alternates);
  RigidBodyTreeWithAlternates<AutoDiffXd>::AddAlternate(tree_with_alternates);

  cout << "num alternates=" << tree_with_alternates.get_num_alternates() << endl;

  //const auto& tree_double = tree_with_alternates.get_rigid_body_tree();
  const auto& tree_double =
      tree_with_alternates.get_alternate<double>().get_rigid_body_tree();
  const auto& tree_dynamics_autodiff =
      tree_with_alternates.get_alternate<AutoDiffXd>().get_rigid_body_tree();

  PRINT_VAR(tree_with_alternates.get_alternate<double>().type());
  PRINT_VAR(tree_with_alternates.get_alternate<AutoDiffXd>().type());

  PRINT_VAR(tree_double.get_num_bodies());
  PRINT_VAR(tree_double.get_num_positions());
  PRINT_VAR(tree_double.get_num_velocities());

  PRINT_VAR(tree_dynamics_autodiff.get_num_bodies());
  PRINT_VAR(tree_dynamics_autodiff.get_num_positions());
  PRINT_VAR(tree_dynamics_autodiff.get_num_velocities());

  const int num_positions = tree_with_alternates.get_num_positions();
  const int num_velocities = tree_with_alternates.get_num_velocities();
  PRINT_VAR(num_positions);
  PRINT_VAR(num_velocities);

  auto q = VectorXd::Random(num_positions).eval();
  auto qd = VectorXd::Random(num_positions).eval();
  auto qdd = VectorXd::Zero(num_positions).eval();

  // First convert qd to MatrixXd to make another explicit instantiation of
  // mass_matrix unnecessary.
  auto qd_dynamic_num_rows = MatrixXd(qd);
  auto q_time_autodiff =
      initializeAutoDiffGivenGradientMatrix(q, qd_dynamic_num_rows);
  typedef decltype(q_time_autodiff)::Scalar TimeADScalar;
  auto qd_time_autodiff = qd.cast<TimeADScalar>();


  auto cache_double = tree_double.doKinematics(q, qd);

  // Notice I call here doKinematics on tree_double instead given that the
  // explicit instantiations are done on RigidBodyTree<double> only.
  // Once doKinematics is templated on <T> we will be able to call it on
  // tree_dynamics_autodiff

  auto cache_dynamics_autodiff =
      tree_dynamics_autodiff.CreateKinematicsCache();
  cache_dynamics_autodiff.initialize(q_time_autodiff, qd_time_autodiff);
  tree_double.doKinematics(cache_dynamics_autodiff);

  //auto cache_dynamics_autodiff =
  //    tree_double.doKinematics(q_time_autodiff, qd_time_autodiff);

  auto com_double = tree_with_alternates.centerOfMass(cache_double);

  PRINT_VAR(com_double.transpose());

  auto com_dynamics_autodiff =
      tree_with_alternates.centerOfMass(cache_dynamics_autodiff);

  for (int i = 0; i < com_dynamics_autodiff.size(); ++i) {
    PRINT_VAR(com_dynamics_autodiff(i).value());
    PRINT_VAR(com_dynamics_autodiff(i).derivatives().transpose());
  }

}
