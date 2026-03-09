#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/multibody/tree/rpy_ball_mobilizer.h"

/* @file This file provides testing for the SapDriver's limited support for
joint limits on joints with multiple degrees of freedom.

    Constraints are only supported by the SAP solver. Therefore, to exercise the
  relevant code paths, we arbitrarily choose one contact approximation that uses
  the SAP solver. More precisely, in the unit tests below we call
  set_discrete_contact_approximation(DiscreteContactApproximation::kSap) on the
  MultibodyPlant used for testing, before constraints are added. */

using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace internal {

// Friend class used to provide access to a selection of private functions in
// SapDriver for testing purposes.
class SapDriverTest {
 public:
  static void AddLimitConstraints(const SapDriver<double>& driver,
                                  const Context<double>& context,
                                  const VectorXd& v_star,
                                  SapContactProblem<double>* problem) {
    driver.AddLimitConstraints(context, v_star, problem);
  }
};

// This joint is used to verify the support of multi-DOF joints with/without
// joint limits. In particular, SapDriver does not support limits
// for constraints with more than 1 DOF and therefore we expect the driver to
// throw an exception when building the problem.
// The implementation for this joint is incomplete. Only the strictly necessary
// overrides for the unit tests in this file are implemented.
template <typename T>
class MultiDofJointWithLimits final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultiDofJointWithLimits);

  // Arbitrary number of DOFs, though larger than one for these tests.
  static constexpr int kNumDofs = 3;

  // The constructor allows to specify finite joint limits.
  MultiDofJointWithLimits(const Frame<T>& frame_on_parent,
                          const Frame<T>& frame_on_child,
                          double pos_lower_limit, double pos_upper_limit)
      : Joint<T>("MultiDofJointWithLimits", frame_on_parent, frame_on_child,
                 VectorX<double>::Zero(kNumDofs),
                 VectorX<double>::Constant(kNumDofs, pos_lower_limit),
                 VectorX<double>::Constant(kNumDofs, pos_upper_limit),
                 VectorX<double>::Constant(
                     kNumDofs, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     kNumDofs, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     kNumDofs, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     kNumDofs, std::numeric_limits<double>::infinity())) {
    DRAKE_DEMAND(pos_lower_limit <= pos_upper_limit);
  }

  const std::string& type_name() const override {
    static const never_destroyed<std::string> name{"MultiDofJointWithLimits"};
    return name.access();
  }

 private:
  // Make MultiDofJointWithLimits templated on every other scalar type a friend
  // of MultiDofJointWithLimits<T> so that CloneToScalar<ToAnyOtherScalar>() can
  // access private members of MultiDofJointWithLimits<T>.
  template <typename>
  friend class MultiDofJointWithLimits;

  int do_get_num_velocities() const override { return kNumDofs; }
  int do_get_num_positions() const override { return kNumDofs; }
  // Dummy implementation, knowing our unit tests below have a single joint of
  // this type.
  int do_get_velocity_start() const override { return 0; }
  int do_get_position_start() const override { return 0; }

  std::unique_ptr<Mobilizer<T>> MakeMobilizerForJoint(
      const SpanningForest::Mobod& mobod,
      MultibodyTree<T>* tree) const override {
    DRAKE_DEMAND(tree != nullptr);  // Just a sanity check; we don't need it.
    const auto [inboard_frame, outboard_frame] =
        this->tree_frames(mobod.is_reversed());
    // TODO(sherm1) The mobilizer needs to be reversed, not just the frames.
    //  The only restriction here relevant for these tests is that we provide a
    //  mobilizer with kNumDofs positions and velocities, so that indexes are
    //  consistent during MultibodyPlant::Finalize().
    auto revolute_mobilizer = std::make_unique<internal::RpyBallMobilizer<T>>(
        mobod, *inboard_frame, *outboard_frame);
    return revolute_mobilizer;
  }

  // We do not need an implementation for the methods below since the unit tests
  // do not exercise them. We mark them as "unreachable".

  void DoAddInOneForce(const Context<T>&, int, const T&,
                       MultibodyForces<T>*) const override {
    DRAKE_UNREACHABLE();
  }
  void DoAddInDamping(const Context<T>&, MultibodyForces<T>*) const override {
    DRAKE_UNREACHABLE();
  }
  std::string do_get_position_suffix(int) const override {
    DRAKE_UNREACHABLE();
  }
  std::string do_get_velocity_suffix(int) const override {
    DRAKE_UNREACHABLE();
  }
  void do_set_default_positions(const VectorX<double>&) override {
    DRAKE_UNREACHABLE();
  }
  const T& DoGetOnePosition(const Context<T>&) const override {
    DRAKE_UNREACHABLE();
  }
  const T& DoGetOneVelocity(const Context<T>&) const override {
    DRAKE_UNREACHABLE();
  }
  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override {
    DRAKE_UNREACHABLE();
  }
  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override {
    DRAKE_UNREACHABLE();
  }
  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override {
    DRAKE_UNREACHABLE();
  }
  std::unique_ptr<Joint<T>> DoShallowClone() const override {
    DRAKE_UNREACHABLE();
  }
};

// Verify that SapDriver throws when the model contains multi-DOF
// joints with finite limits.
GTEST_TEST(MultiDofJointWithLimitsTest, ThrowForUnsupportedJoints) {
  MultibodyPlant<double> plant(1.0e-3);
  // N.B. Currently only SAP goes through the manager.
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant.AddRigidBody("DummyBody", SpatialInertia<double>::MakeUnitary());
  plant.AddJoint(std::make_unique<MultiDofJointWithLimits<double>>(
      plant.world_frame(), body.body_frame(), -1.0, 2.0));
  plant.Finalize();
  auto owned_contact_manager =
      std::make_unique<CompliantContactManager<double>>();
  CompliantContactManager<double>* contact_manager =
      owned_contact_manager.get();
  plant.SetDiscreteUpdateManager(std::move(owned_contact_manager));
  auto context = plant.CreateDefaultContext();

  // Dummy A, v* and problem.
  const std::vector<MatrixX<double>> A(1, Matrix3<double>::Ones());
  const VectorXd v_star = Vector3d::Zero();
  SapContactProblem<double> problem(plant.time_step(), A, v_star);

  const SapDriver<double>& driver =
      CompliantContactManagerTester::sap_driver(*contact_manager);

  // We verify the driver throws for the right reasons.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SapDriverTest::AddLimitConstraints(driver, *context, v_star, &problem),
      "Limits for joints with more than one degree of "
      "freedom are not supported(.|\n)*");
}

// Verify that SapDriver allows multi-DOF joints whenever these do
// not specify finite limits.
GTEST_TEST(MultiDofJointWithLimitsTest,
           VerifyMultiDofJointsWithoutLimitsAreSupported) {
  MultibodyPlant<double> plant(1.0e-3);
  // N.B. Currently only SAP goes through the manager.
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant.AddRigidBody("DummyBody", SpatialInertia<double>::MakeUnitary());
  const double kInf = std::numeric_limits<double>::infinity();
  plant.AddJoint(std::make_unique<MultiDofJointWithLimits<double>>(
      plant.world_frame(), body.body_frame(), -kInf, kInf));
  plant.Finalize();
  auto owned_contact_manager =
      std::make_unique<CompliantContactManager<double>>();
  CompliantContactManager<double>* contact_manager =
      owned_contact_manager.get();
  plant.SetDiscreteUpdateManager(std::move(owned_contact_manager));
  auto context = plant.CreateDefaultContext();

  // Arbitrary A and v*.
  const std::vector<MatrixX<double>> A(1, Matrix3<double>::Ones());
  const VectorXd v_star = Vector3d::Zero();
  SapContactProblem<double> problem(plant.time_step(), A, v_star);

  const SapDriver<double>& driver =
      CompliantContactManagerTester::sap_driver(*contact_manager);

  EXPECT_NO_THROW(
      SapDriverTest::AddLimitConstraints(driver, *context, v_star, &problem));

  // No limit constraints are added since the only one joint in the model has
  // no limits.
  EXPECT_EQ(problem.num_constraints(), 0);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
