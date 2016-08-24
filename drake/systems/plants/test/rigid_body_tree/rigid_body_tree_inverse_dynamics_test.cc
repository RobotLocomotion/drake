#include <iostream> // TODO: remove

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"


namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using drake::parsers::ModelInstanceIdTable;
using drake::math::initializeAutoDiff;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiffGivenGradientMatrix;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::CompareMatrices;

using drake::math::initializeAutoDiffTuple;

class RigidBodyTreeInverseDynamicsTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree_.reset(new RigidBodyTree());
    std::string kAtlasUrdf =
        drake::GetDrakePath() + "/examples/Atlas/urdf/atlas_convex_hull.urdf";
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        kAtlasUrdf, DrakeJoint::ROLLPITCHYAW,
        nullptr /* weld_to_frame */, tree_.get());
  }

 public:
  // TODO(amcastro-tri): A stack object here (preferable to a pointer)
  // generates build issues on Windows platforms. See git-hub issue #1854.
  std::unique_ptr<RigidBodyTree> tree_;
};



// Check that mass matrix derivative minus two times Coriolis matrix is skew
// symmetric (see e.g. Murray, Li, Sastry. "A Mathematical Introduction to
// Robotic Manipulation" (1994). Lemma 4.2).
// Note: property only holds true if qd = v
TEST_F(RigidBodyTreeInverseDynamicsTest, TestSkewSymmetryProperty) {

  int num_velocities = tree_->number_of_velocities();

  //  std::default_random_engine generator;
//  auto q = tree_->getRandomConfiguration(generator);
  auto q = tree_->getZeroConfiguration();
  auto qd = VectorXd::Constant(num_velocities, 1.);
  auto qdd = VectorXd::Zero(tree_->number_of_velocities());

  // compute mass matrix time derivative

  // convert qd to MatrixXd to make another explicit instantiation of
  // mass_matrix unnecessary
  auto qd_dynamic_num_rows = MatrixXd(qd);
  auto q_time_autodiff = initializeAutoDiffGivenGradientMatrix(q, qd_dynamic_num_rows);
  typedef typename decltype(q_time_autodiff)::Scalar TimeADScalar;
  auto qd_time_autodiff = qd.cast<TimeADScalar>();
  KinematicsCache<TimeADScalar> kinematics_cache_time_autodiff(tree_->bodies);
  kinematics_cache_time_autodiff.initialize(q_time_autodiff, qd_time_autodiff);
  tree_->doKinematics(kinematics_cache_time_autodiff);
  auto mass_matrix_time_autodiff = tree_->massMatrix(
      kinematics_cache_time_autodiff);
  auto mass_matrix_dot_vectorized = autoDiffToGradientMatrix(
      mass_matrix_time_autodiff);
  auto mass_matrix_dot = Eigen::Map<Eigen::MatrixXd>(
      mass_matrix_dot_vectorized.data(), num_velocities, num_velocities);

  // compute Coriolis matrix
  auto qd_qd_autodiff = initializeAutoDiff(qd);
  typedef typename decltype(qd_qd_autodiff)::Scalar QdADScalar;
  auto q_qd_autodiff = q.cast<QdADScalar>().eval();
  auto qdd_qd_autodiff = qdd.cast<QdADScalar>().eval();
  KinematicsCache<QdADScalar> kinematics_cache_qd_autodiff(tree_->bodies);
  kinematics_cache_qd_autodiff.initialize(q_qd_autodiff, qd_qd_autodiff);
  tree_->doKinematics(kinematics_cache_qd_autodiff, true);
  eigen_aligned_unordered_map<const RigidBody *, TwistVector<QdADScalar>> f_ext;
  auto tau_autodiff = tree_->inverseDynamics(
      kinematics_cache_qd_autodiff, f_ext, qdd_qd_autodiff, true);
  tau_autodiff -= tree_->frictionTorques(qd_qd_autodiff);
  auto coriolis_matrix = (autoDiffToGradientMatrix(tau_autodiff) / 2.).eval();

  // assert that property holds
  auto skew = (mass_matrix_dot - 2 * coriolis_matrix).eval();
  EXPECT_TRUE(CompareMatrices((skew + skew.transpose()).eval(),
                              MatrixXd::Zero(num_velocities, num_velocities),
                              1e-10, MatrixCompareType::absolute));
}


// Check that the gradient of the output of inverseDynamics with respect to
// the joint acceleration vector is the mass matrix.
TEST_F(RigidBodyTreeInverseDynamicsTest, TestAccelerationJacobianIsMassMatrix) {
  std::default_random_engine generator;

  auto vd = VectorXd::Random(tree_->number_of_velocities());
  auto v = VectorXd::Random(tree_->number_of_velocities());
  auto q = tree_->getRandomConfiguration(generator);
  KinematicsCache<double> kinematics_cache(tree_->bodies);
  kinematics_cache.initialize(q, v);
  tree_->doKinematics(kinematics_cache);
  auto mass_matrix = tree_->massMatrix(kinematics_cache);

  auto vd_autodiff = initializeAutoDiff(vd);
  typedef typename decltype(vd_autodiff)::Scalar ADScalar;
  auto v_autodiff = v.cast<ADScalar>().eval();
  auto q_autodiff = v.cast<ADScalar>().eval();
  KinematicsCache<ADScalar> kinematics_cache_autodiff(tree_->bodies);
  kinematics_cache_autodiff.initialize(q_autodiff, v_autodiff);
  tree_->doKinematics(kinematics_cache_autodiff, true);
  eigen_aligned_unordered_map<const RigidBody *, TwistVector<ADScalar>> f_ext;
  auto tau_autodiff = tree_->inverseDynamics(kinematics_cache_autodiff, f_ext,
      vd_autodiff);
  auto mass_matrix_from_inverse_dynamics = autoDiffToGradientMatrix(
      tau_autodiff);

  autoDiffToGradientMatrix(tree_->frictionTorques(v_autodiff));

  EXPECT_TRUE(CompareMatrices(mass_matrix,
                              mass_matrix_from_inverse_dynamics,
                              1e-10, MatrixCompareType::absolute));
}

// Check that the gradient transpose of potential energy matches the vector of
// generalized gravitational forces
TEST_F(RigidBodyTreeInverseDynamicsTest, TestGeneralizeGravitationalForces) {
  std::default_random_engine generator;

  // compute vector of generalized gravitational forces
  auto q = tree_->getRandomConfiguration(generator);
  KinematicsCache<double> kinematics_cache(tree_->bodies);
  kinematics_cache.initialize(q);
  tree_->doKinematics(kinematics_cache);
  eigen_aligned_unordered_map<const RigidBody *, TwistVector<double>> f_ext;
  auto gravitational_forces = tree_->dynamicsBiasTerm(
      kinematics_cache, f_ext,false);

  auto q_autodiff = initializeAutoDiff(q);
  typedef typename decltype(q_autodiff)::Scalar ADScalar;
  KinematicsCache<ADScalar> kinematics_cache_autodiff(tree_->bodies);
  kinematics_cache_autodiff.initialize(q_autodiff);
  tree_->doKinematics(kinematics_cache_autodiff);
  auto gravitational_acceleration = tree_->a_grav.tail<3>();
  auto gravitational_force =
      (gravitational_acceleration.cast<ADScalar>() * tree_->getMass()).eval();
  auto center_of_mass = tree_->centerOfMass(kinematics_cache_autodiff);
  auto potential_energy = -center_of_mass.dot(gravitational_force);

  EXPECT_TRUE(CompareMatrices(gravitational_forces,
                              potential_energy.derivatives(),
                              1e-10, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
