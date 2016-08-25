#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/drake_path.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
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

class RigidBodyTreeInverseDynamicsTest: public ::testing::Test {
 protected:
  virtual void SetUp() {
    trees_.clear();

    std::string kAtlasUrdf =
        drake::GetDrakePath() + "/examples/Atlas/urdf/atlas_convex_hull.urdf";

    tree_rpy_.reset(new RigidBodyTree());
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        kAtlasUrdf, DrakeJoint::ROLLPITCHYAW,
        nullptr /* weld_to_frame */, tree_rpy_.get());
    trees_.push_back(tree_rpy_.get());

    tree_quaternion_.reset(new RigidBodyTree());
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        kAtlasUrdf, DrakeJoint::QUATERNION,
        nullptr /* weld_to_frame */, tree_quaternion_.get());
    trees_.push_back(tree_quaternion_.get());
  }

 public:
  // TODO(amcastro-tri): A stack object here (preferable to a pointer)
  // generates build issues on Windows platforms. See git-hub issue #1854.
  std::unique_ptr<RigidBodyTree> tree_rpy_;
  std::unique_ptr<RigidBodyTree> tree_quaternion_;
  std::vector<RigidBodyTree *> trees_;
};



// Check that mass matrix derivative minus two times Coriolis matrix is skew
// symmetric (see e.g. Murray, Li, Sastry. "A Mathematical Introduction to
// Robotic Manipulation" (1994). Lemma 4.2).
// Note: property only holds true if qd = v
TEST_F(RigidBodyTreeInverseDynamicsTest, TestSkewSymmetryProperty) {
  int num_velocities = tree_rpy_->number_of_velocities();

  std::default_random_engine generator;
  auto q = tree_rpy_->getRandomConfiguration(generator);
  auto qd = VectorXd::Constant(num_velocities, 1.).eval();
  auto qdd = VectorXd::Zero(tree_rpy_->number_of_velocities()).eval();

  // compute mass matrix time derivative

  // convert qd to MatrixXd to make another explicit instantiation of
  // mass_matrix unnecessary
  auto qd_dynamic_num_rows = MatrixXd(qd);
  auto q_time_autodiff = initializeAutoDiffGivenGradientMatrix(
      q, qd_dynamic_num_rows);
  typedef typename decltype(q_time_autodiff)::Scalar TimeADScalar;
  auto qd_time_autodiff = qd.cast<TimeADScalar>();
  KinematicsCache<TimeADScalar> kinematics_cache_time_autodiff(
      tree_rpy_->bodies);
  kinematics_cache_time_autodiff.initialize(q_time_autodiff, qd_time_autodiff);
  tree_rpy_->doKinematics(kinematics_cache_time_autodiff);
  auto mass_matrix_time_autodiff = tree_rpy_->massMatrix(
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
  KinematicsCache<QdADScalar> kinematics_cache_qd_autodiff(tree_rpy_->bodies);
  kinematics_cache_qd_autodiff.initialize(q_qd_autodiff, qd_qd_autodiff);
  tree_rpy_->doKinematics(kinematics_cache_qd_autodiff, true);
  eigen_aligned_unordered_map<const RigidBody *, TwistVector<QdADScalar>> f_ext;
  auto tau_autodiff = tree_rpy_->inverseDynamics(
      kinematics_cache_qd_autodiff, f_ext, qdd_qd_autodiff, true);
  tau_autodiff -= tree_rpy_->frictionTorques(qd_qd_autodiff);
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

  for (RigidBodyTree *tree : trees_) {
    auto q = tree->getRandomConfiguration(generator);
    auto v = VectorXd::Random(tree->number_of_velocities()).eval();
    auto vd = VectorXd::Random(tree->number_of_velocities()).eval();
    KinematicsCache<double> kinematics_cache(tree->bodies);
    kinematics_cache.initialize(q, v);
    tree->doKinematics(kinematics_cache);
    auto mass_matrix = tree->massMatrix(kinematics_cache);

    auto vd_autodiff = initializeAutoDiff(vd);
    typedef typename decltype(vd_autodiff)::Scalar ADScalar;
    auto q_autodiff = q.cast<ADScalar>().eval();
    auto v_autodiff = v.cast<ADScalar>().eval();
    KinematicsCache<ADScalar> kinematics_cache_autodiff(tree->bodies);
    kinematics_cache_autodiff.initialize(q_autodiff, v_autodiff);
    tree->doKinematics(kinematics_cache_autodiff, true);
    eigen_aligned_unordered_map<const RigidBody *, TwistVector<ADScalar>> f_ext;
    auto tau_autodiff = tree->inverseDynamics(kinematics_cache_autodiff, f_ext,
                                              vd_autodiff);
    auto mass_matrix_from_inverse_dynamics = autoDiffToGradientMatrix(
        tau_autodiff);

    autoDiffToGradientMatrix(tree->frictionTorques(v_autodiff));

    EXPECT_TRUE(CompareMatrices(mass_matrix,
                                mass_matrix_from_inverse_dynamics,
                                1e-10, MatrixCompareType::absolute));
  }
}

// Check that the derivatives of potential energy with respect to q match the
// vector of generalized gravitational forces for rpy tree.
TEST_F(RigidBodyTreeInverseDynamicsTest, TestGeneralizedGravitationalForces) {
  std::default_random_engine generator;

  // compute vector of generalized gravitational forces
  auto &tree = tree_rpy_;
  auto q = tree->getRandomConfiguration(generator);
  KinematicsCache<double> kinematics_cache(tree->bodies);
  kinematics_cache.initialize(q);
  tree->doKinematics(kinematics_cache);
  eigen_aligned_unordered_map<const RigidBody *, TwistVector<double>> f_ext;
  auto gravitational_forces = tree->dynamicsBiasTerm(
      kinematics_cache, f_ext, false);

  // compute derivatives of potential energy
  auto q_autodiff = initializeAutoDiff(q);
  typedef typename decltype(q_autodiff)::Scalar ADScalar;
  KinematicsCache<ADScalar> kinematics_cache_autodiff(tree->bodies);
  kinematics_cache_autodiff.initialize(q_autodiff);
  tree->doKinematics(kinematics_cache_autodiff);
  auto gravitational_acceleration = tree->a_grav.tail<3>().eval();
  auto gravitational_force =
      (gravitational_acceleration.cast<ADScalar>()
          * tree->getMass()).eval();
  auto center_of_mass = tree->centerOfMass(kinematics_cache_autodiff);
  ADScalar potential_energy = -center_of_mass.dot(gravitational_force);

  EXPECT_TRUE(CompareMatrices(gravitational_forces,
                              potential_energy.derivatives(),
                              1e-10, MatrixCompareType::absolute));
}

// Check that momentum rate of change is equal to the sum of all external
// wrenches exerted upon the mechanism
TEST_F(RigidBodyTreeInverseDynamicsTest, TestMomentumRateOfChange) {
  std::default_random_engine generator;
  int world_index = 0;

  // use a quaternion-parameterized floating joint, so that torque vector for
  // this joint is the same as the floating joint wrench
  RigidBodyTree& tree = *tree_quaternion_;
  auto q = tree.getRandomConfiguration(generator);
  auto v = VectorXd::Random(tree.number_of_velocities()).eval();
  auto vd = VectorXd::Random(tree.number_of_velocities()).eval();
  eigen_aligned_unordered_map<const RigidBody *, TwistVector<double>> f_ext;

  KinematicsCache<double> kinematics_cache(tree.bodies);
  kinematics_cache.initialize(q, v);
  tree.doKinematics(kinematics_cache, true);

  // compute gravitational wrench
  auto gravitational_wrench_centroidal = (tree.a_grav *
      tree.getMass()).eval();
  Eigen::Isometry3d centroidal_to_world;
  centroidal_to_world.setIdentity();
  centroidal_to_world.translation() =
      tree.centerOfMass(kinematics_cache);
  auto gravitational_wrench_world = transformSpatialForce(
      centroidal_to_world, gravitational_wrench_centroidal);

  // set external wrenches, keep track of total wrench
  Vector6<double> total_wrench_world = gravitational_wrench_world;
  for (const auto &body_ptr : tree.bodies) {
    if (body_ptr->hasParent()) {
      auto wrench_body = Vector6<double>::Random().eval();
      f_ext[body_ptr.get()] = wrench_body;
      auto body_to_world = tree.relativeTransform(
          kinematics_cache, world_index, body_ptr->get_body_index());
      auto wrench_world = transformSpatialForce(
          body_to_world, wrench_body);
      total_wrench_world += wrench_world;
    }
  }

  // compute wrench across floating joint, add to total wrench
  auto tau = tree.inverseDynamics(kinematics_cache, f_ext, vd);
  auto &floating_body_ptr = tree.bodies[1];
  int floating_joint_start_index = floating_body_ptr->
      get_velocity_start_index();
  Vector6<double> floating_joint_wrench_body = tau.segment<kTwistSize>(
      floating_joint_start_index);
  auto body_to_world = tree.relativeTransform(
      kinematics_cache, world_index, floating_body_ptr->get_body_index());
  auto floating_joint_wrench_world = transformSpatialForce(
      body_to_world, floating_joint_wrench_body);
  total_wrench_world += floating_joint_wrench_world;

  // compute rate of change of momentum
  auto identity = MatrixXd::Identity(q.size(), q.size());
  auto v_to_qd = kinematics_cache.transformPositionDotMappingToVelocityMapping(
      identity);
  auto qd = v_to_qd * v;
  // convert to MatrixXd to make another explicit instantiation unnecessary
  auto q_autodiff = initializeAutoDiffGivenGradientMatrix(q, MatrixXd(qd));
  auto v_autodiff = initializeAutoDiffGivenGradientMatrix(v, MatrixXd(vd));
  typedef typename decltype(q_autodiff)::Scalar ADScalar;
  KinematicsCache<ADScalar> kinematics_cache_autodiff(tree.bodies);
  kinematics_cache_autodiff.initialize(q_autodiff, v_autodiff);
  tree.doKinematics(kinematics_cache_autodiff);
  auto momentum_matrix_autodiff = tree.worldMomentumMatrix(
      kinematics_cache_autodiff);
  auto momentum_world_autodiff = momentum_matrix_autodiff * v_autodiff;
  auto momentum_rate = autoDiffToGradientMatrix(momentum_world_autodiff);

  // Newton-Euler: total wrench should equal momentum rate of change
  EXPECT_TRUE(CompareMatrices(momentum_rate,
                              total_wrench_world,
                              1e-10, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
