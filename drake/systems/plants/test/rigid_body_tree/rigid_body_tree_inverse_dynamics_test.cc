#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/jacobian.h"
#include "drake/systems/plants/joints/floating_base_types.h"
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
using drake::math::jacobian;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::CompareMatrices;
using drake::systems::plants::joints::kRollPitchYaw;
using drake::systems::plants::joints::kQuaternion;

constexpr const int kChunkSize =
    drake::AutoDiffUpTo73d::DerType::MaxRowsAtCompileTime;

class RigidBodyTreeInverseDynamicsTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    trees_.clear();

    const std::string kAtlasUrdf =
        drake::GetDrakePath() + "/examples/Atlas/urdf/atlas_convex_hull.urdf";

    tree_rpy_ = std::make_unique<RigidBodyTree>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        kAtlasUrdf, kRollPitchYaw, nullptr /* weld_to_frame */,
        tree_rpy_.get());
    trees_.push_back(tree_rpy_.get());

    tree_quaternion_ = std::make_unique<RigidBodyTree>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        kAtlasUrdf, kQuaternion, nullptr /* weld_to_frame */,
        tree_quaternion_.get());
    trees_.push_back(tree_quaternion_.get());
  }

 public:
  // TODO(amcastro-tri): A stack object here (preferable to a pointer)
  // generates build issues on Windows platforms. See git-hub issue #1854.
  std::unique_ptr<RigidBodyTree> tree_rpy_;
  std::unique_ptr<RigidBodyTree> tree_quaternion_;
  std::vector<RigidBodyTree*> trees_;
};

// Check that mass matrix time derivative minus two times Coriolis matrix is
// skew symmetric (see e.g. Murray, Li, Sastry. "A Mathematical Introduction to
// Robotic Manipulation" (1994). Lemma 4.2).
// http://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-complete.pdf
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.671.7040&rep=rep1&type=pdf
// Note: property only holds true if qd = v.
// The Coriolis matrix is the matrix C(q, qd) when the equations of motion
// are written as
//
//    H(q) qdd + C(q, qd) qd + g(q) = tau - tau_friction(q, qd)
//
// with H(q) the mass matrix, g(q) the gravitational terms, tau the
// joint torques, and tau_friction(q, qd) the torques due to friction.
TEST_F(RigidBodyTreeInverseDynamicsTest, TestSkewSymmetryProperty) {
  int num_velocities = tree_rpy_->get_num_velocities();

  std::default_random_engine generator;
  auto q = tree_rpy_->getRandomConfiguration(generator);
  auto qd = VectorXd::Random(num_velocities).eval();
  auto qdd = VectorXd::Zero(num_velocities).eval();

  // Compute mass matrix time derivative

  // First convert qd to MatrixXd to make another explicit instantiation of
  // mass_matrix unnecessary.
  auto qd_dynamic_num_rows = MatrixXd(qd);
  auto q_time_autodiff =
      initializeAutoDiffGivenGradientMatrix(q, qd_dynamic_num_rows);
  typedef decltype(q_time_autodiff)::Scalar TimeADScalar;
  auto qd_time_autodiff = qd.cast<TimeADScalar>();
  KinematicsCache<TimeADScalar> kinematics_cache_time_autodiff(
      tree_rpy_->bodies);
  kinematics_cache_time_autodiff.initialize(q_time_autodiff, qd_time_autodiff);
  tree_rpy_->doKinematics(kinematics_cache_time_autodiff);
  auto mass_matrix_time_autodiff =
      tree_rpy_->massMatrix(kinematics_cache_time_autodiff);

  Eigen::MatrixXd mass_matrix_dot(qd.size(), qd.size());
  for (int i = 0; i < qd.size(); ++i) {
    for (int j = 0; j < qd.size(); ++j) {
      // Derivatives only contain one component with the time derivative.
      // Note that the derivative vector may not be initialized (has zero
      // length) if the component is constant in time.
      auto& derivatives = mass_matrix_time_autodiff(i, j).derivatives();
      mass_matrix_dot(i, j) = derivatives.size() == 1 ? derivatives[0] : 0.;
    }
  }

  // Compute Coriolis matrix (see Murray et al., eq. (4.23)).
  auto qd_to_coriolis_term = [&](const auto& qd_arg) {
    using Scalar =
        typename std::remove_reference<decltype(qd_arg)>::type::Scalar;
    KinematicsCache<Scalar> kinematics_cache_coriolis(tree_rpy_->bodies);
    kinematics_cache_coriolis.initialize(q.cast<Scalar>(), qd_arg);
    tree_rpy_->doKinematics(kinematics_cache_coriolis, true);

    const RigidBodyTree::BodyToWrenchMap<Scalar> no_external_wrenches;
    auto coriolis_term = tree_rpy_->inverseDynamics(
        kinematics_cache_coriolis, no_external_wrenches,
        qdd.cast<Scalar>().eval(), true);
    coriolis_term -= tree_rpy_->frictionTorques(qd_arg);
    return coriolis_term;
  };
  auto coriolis_term_qd_jacobian =
      autoDiffToGradientMatrix(jacobian<kChunkSize>(qd_to_coriolis_term, qd));
  auto coriolis_matrix = (coriolis_term_qd_jacobian / 2.).eval();

  // Asserts that the the Coriolis matrix is a square matrix with a dimension
  // equal to the number of DOF's in the rigid body tree.
  ASSERT_EQ(qd.size(), coriolis_matrix.rows());
  ASSERT_EQ(qd.size(), coriolis_matrix.cols());

  // Assert that property holds.
  auto skew = (mass_matrix_dot - 2 * coriolis_matrix).eval();
  EXPECT_TRUE(CompareMatrices((skew + skew.transpose()).eval(),
                              MatrixXd::Zero(num_velocities, num_velocities),
                              1e-10, MatrixCompareType::absolute));
}

// Check that the gradient of the output of inverseDynamics with respect to
// the joint acceleration vector is the mass matrix.
// The equations of motion can be written as
//
//    H(q) vd + c(q, v) = tau
//
// Since inverseDynamics computes tau given q, v, and vd, the gradient of tau
// with respect to vd should be the mass matrix H(q).
TEST_F(RigidBodyTreeInverseDynamicsTest, TestAccelerationJacobianIsMassMatrix) {
  std::default_random_engine generator;

  for (RigidBodyTree* tree : trees_) {
    auto q = tree->getRandomConfiguration(generator);
    auto v = VectorXd::Random(tree->get_num_velocities()).eval();
    auto vd = VectorXd::Random(tree->get_num_velocities()).eval();
    KinematicsCache<double> kinematics_cache(tree->bodies);
    kinematics_cache.initialize(q, v);
    tree->doKinematics(kinematics_cache);
    auto mass_matrix = tree->massMatrix(kinematics_cache);

    auto vd_to_mass_matrix = [&](const auto& vd_arg) {
      using Scalar =
          typename std::remove_reference<decltype(vd_arg)>::type::Scalar;
      KinematicsCache<Scalar> kinematics_cache_2(tree->bodies);
      kinematics_cache_2.initialize(q.cast<Scalar>(), v.cast<Scalar>());
      tree->doKinematics(kinematics_cache_2, true);
      const RigidBodyTree::BodyToWrenchMap<Scalar> no_external_wrenches;
      return tree->inverseDynamics(kinematics_cache_2, no_external_wrenches,
                                   vd_arg);
    };
    auto mass_matrix_from_inverse_dynamics =
        autoDiffToGradientMatrix(jacobian<kChunkSize>(vd_to_mass_matrix, vd));

    ASSERT_EQ(tree->get_num_velocities(),
              mass_matrix_from_inverse_dynamics.rows());
    ASSERT_EQ(tree->get_num_velocities(),
              mass_matrix_from_inverse_dynamics.cols());

    EXPECT_TRUE(CompareMatrices(mass_matrix, mass_matrix_from_inverse_dynamics,
                                1e-10, MatrixCompareType::absolute));
  }
}

// Check that the derivatives of gravitational potential energy with respect
// to q match the vector of generalized gravitational forces for an rpy tree.
// When qd = v, the equations of motion can be written as
//
//    H(q) qdd + C(q, qd) qd + g(q) = tau
//
// When qd = 0 and qdd = 0, tau = g(q), so we can compute the gravitational
// term using the inverse dynamics algorithm.
// Now let V(q) denote gravitational potential energy. Then, by definition,
// g(q) = dV(q)/dq. Assert that that is indeed the case.
TEST_F(RigidBodyTreeInverseDynamicsTest, TestGeneralizedGravitationalForces) {
  std::default_random_engine generator;

  // Compute vector of generalized gravitational forces. Note that
  // dynamicsBiasTerm simply calls inverseDynamics with vd = 0.
  auto& tree = tree_rpy_;
  auto q = tree->getRandomConfiguration(generator);
  KinematicsCache<double> kinematics_cache(tree->bodies);
  kinematics_cache.initialize(q);
  tree->doKinematics(kinematics_cache);
  const RigidBodyTree::BodyToWrenchMap<double> no_external_wrenches;
  auto gravitational_forces =
      tree->dynamicsBiasTerm(kinematics_cache, no_external_wrenches, false);

  // Compute the vector gradient of potential energy.

  auto gravitational_acceleration = tree->a_grav.tail<3>();
  auto gravitational_force =
      (gravitational_acceleration * tree->getMass()).eval();
  auto q_to_gravitational_potential_energy = [&](const auto& q_arg) {
    using Scalar =
        typename std::remove_reference<decltype(q_arg)>::type::Scalar;
    KinematicsCache<Scalar> kinematics_cache_2(tree->bodies);
    kinematics_cache_2.initialize(q_arg);
    tree->doKinematics(kinematics_cache_2);
    auto center_of_mass = tree->centerOfMass(kinematics_cache_2);
    Scalar gravitational_potential_energy =
        -center_of_mass.dot(gravitational_force.cast<Scalar>().eval());
    // The jacobian function expects a Matrix as output, so:
    return Eigen::Matrix<Scalar, 1, 1>(gravitational_potential_energy);
  };
  auto gravitational_forces_from_potential_energy =
      jacobian<kChunkSize>(q_to_gravitational_potential_energy, q)
          .value()
          .derivatives();

  EXPECT_TRUE(CompareMatrices(gravitational_forces,
                              gravitational_forces_from_potential_energy, 1e-10,
                              MatrixCompareType::absolute));
}

// Check that momentum rate of change is equal to the sum of all external
// wrenches exerted upon the mechanism.
//
// The total (spatial) momentum of a mechanism can be written as
//
//    h(q, v) = A(q) v
//
// where A is the momentum matrix (see Orin, Goswami. "Centroidal momentum
// matrix of a humanoid robot: Structure and properties" (2008)
// http://www.ambarish
// .com/paper/20080707_Orin_Goswami_momentum_matrix_IROS2008.pdf
//
// The rate of change of momentum, hdot, can be computed using automatic
// differentiation, given q, v, and vdot.
//
// By the Newton-Euler equations, the rate of change of momentum must be
// equal to the sum of wrenches acting externally on the mechanism:
//
//    hdot = W_f + W_g + sum(W_i)
//
// where:
// * W_g is the wrench due to gravity
// * W_f is the wrench across the floating joint (treating it as having an
// actuator)
// * the W_i are other external wrenches acting upon the bodies of the
// mechanism.
//
// Given q, v, vdot, and W_i, the inverse dynamics algorithm can be used to
// compute the required torques. For a floating mechanism with a
// QuaternionFloatingJoint, the torque vector associated with the floating
// joint should exactly be the wrench W_f. All other joint torques are
// internal to the mechanism.
// Given hdot, W_f, W_g and the W_i's, assert that Newton-Euler holds.
TEST_F(RigidBodyTreeInverseDynamicsTest, TestMomentumRateOfChange) {
  std::default_random_engine generator;
  int world_index = 0;

  // Use a quaternion-parameterized floating joint, so that torque vector for
  // this joint is the same as the floating joint wrench.
  RigidBodyTree& tree = *tree_quaternion_;
  auto q = tree.getRandomConfiguration(generator);
  auto v = VectorXd::Random(tree.get_num_velocities()).eval();
  auto vd = VectorXd::Random(tree.get_num_velocities()).eval();
  RigidBodyTree::BodyToWrenchMap<double> external_wrenches;

  KinematicsCache<double> kinematics_cache(tree.bodies);
  kinematics_cache.initialize(q, v);
  tree.doKinematics(kinematics_cache, true);

  // Compute gravitational wrench W_g.
  // Gravitational force can be thought of as acting at center of mass. Change
  // to world frame.
  auto gravitational_wrench_centroidal = (tree.a_grav * tree.getMass()).eval();
  Eigen::Isometry3d centroidal_to_world;
  centroidal_to_world.setIdentity();
  centroidal_to_world.translation() = tree.centerOfMass(kinematics_cache);
  auto gravitational_wrench_world = transformSpatialForce(
      centroidal_to_world, gravitational_wrench_centroidal);

  // Set external wrenches, keep track of total wrench.
  Vector6<double> total_wrench_world = gravitational_wrench_world;
  for (const auto& body_ptr : tree.bodies) {
    if (body_ptr->has_parent_body()) {
      auto wrench_body = Vector6<double>::Random().eval();
      external_wrenches[body_ptr.get()] = wrench_body;
      auto body_to_world = tree.relativeTransform(kinematics_cache, world_index,
                                                  body_ptr->get_body_index());
      auto wrench_world = transformSpatialForce(body_to_world, wrench_body);
      total_wrench_world += wrench_world;
    }
  }

  // Compute wrench across floating joint W_f, add to total wrench.
  auto tau = tree.inverseDynamics(kinematics_cache, external_wrenches, vd);
  auto& floating_body_ptr = tree.bodies[1];
  int floating_joint_start_index =
      floating_body_ptr->get_velocity_start_index();
  Vector6<double> floating_joint_wrench_body =
      tau.segment<kTwistSize>(floating_joint_start_index);
  auto body_to_world = tree.relativeTransform(
      kinematics_cache, world_index, floating_body_ptr->get_body_index());
  auto floating_joint_wrench_world =
      transformSpatialForce(body_to_world, floating_joint_wrench_body);
  total_wrench_world += floating_joint_wrench_world;

  // Compute rate of change of momentum hdot.
  auto identity = MatrixXd::Identity(q.size(), q.size());
  auto v_to_qd =
      kinematics_cache.transformPositionDotMappingToVelocityMapping(identity);
  auto qd = v_to_qd * v;
  // Convert to MatrixXd to make another explicit instantiation unnecessary.
  auto q_time_autodiff = initializeAutoDiffGivenGradientMatrix(q, MatrixXd(qd));
  auto v_time_autodiff = initializeAutoDiffGivenGradientMatrix(v, MatrixXd(vd));
  typedef decltype(q_time_autodiff)::Scalar ADScalar;
  KinematicsCache<ADScalar> kinematics_cache_autodiff(tree.bodies);
  kinematics_cache_autodiff.initialize(q_time_autodiff, v_time_autodiff);
  tree.doKinematics(kinematics_cache_autodiff);
  auto momentum_matrix_time_autodiff =
      tree.worldMomentumMatrix(kinematics_cache_autodiff);
  auto momentum_world_time_autodiff =
      momentum_matrix_time_autodiff * v_time_autodiff;
  auto momentum_rate = autoDiffToGradientMatrix(momentum_world_time_autodiff);

  // Newton-Euler: total wrench should equal momentum rate of change.
  EXPECT_TRUE(CompareMatrices(momentum_rate, total_wrench_world, 1e-10,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
