#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/controllers/plan_eval/test/test_common.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {

using systems::controllers::qp_inverse_dynamics::ConstraintType;
using systems::controllers::qp_inverse_dynamics::ParamSet;
using systems::controllers::qp_inverse_dynamics::QpInput;
using systems::controllers::qp_inverse_dynamics::RobotKinematicState;

// This is a derived class from GenericPlan with no additional features.
// It sets up a plan that does not have any Cartesian tracking objectives,
// no contacts, and holds the current generalized position forever.
template <typename T>
class DummyPlan : public GenericPlan<T> {
 public:
  DummyPlan() {}

  void set_all_to_compatible() {
    set_model_compatible(true);
    set_param_compatible(true);
    set_alias_compatible(true);
  }

  void set_model_compatible(bool model_compatible) {
    is_model_compatible_ = model_compatible;
  }

  void set_param_compatible(bool param_compatible) {
    is_param_compatible_ = param_compatible;
  }

  void set_alias_compatible(bool alias_compatible) {
    is_alias_compatible_ = alias_compatible;
  }

  bool IsRigidBodyTreeCompatible(const RigidBodyTree<T>&) const override {
    return is_model_compatible_;
  }

  bool IsRigidBodyTreeAliasGroupsCompatible(
      const RigidBodyTreeAliasGroups<T>&) const override {
    return is_alias_compatible_;
  }

  bool IsParamSetCompatible(const ParamSet&) const override {
    return is_param_compatible_;
  }

 private:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyPlan)

  GenericPlan<T>* CloneGenericPlanDerived() const override {
    return new DummyPlan<T>(*this);
  }

  void InitializeGenericPlanDerived(
      const RobotKinematicState<T>& robot_status,
      const ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups) override {}

  void ModifyPlanGenericPlanDerived(
      const RobotKinematicState<T>& robot_stauts,
      const ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups) override {}

  void HandlePlanGenericPlanDerived(
      const RobotKinematicState<T>& robot_stauts,
      const ParamSet& paramset, const RigidBodyTreeAliasGroups<T>& alias_groups,
      const AbstractValue& plan) override {}

  void UpdateQpInputGenericPlanDerived(
      const RobotKinematicState<T>& robot_status,
      const ParamSet& paramset, const RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const override {}

  bool is_model_compatible_{true};
  bool is_param_compatible_{true};
  bool is_alias_compatible_{true};
};

class DummyPlanTest : public GenericPlanTest {
 protected:
  void SetUp() override {
    const std::string kModelPath = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf");

    const std::string kAliasGroupsPath = FindResourceOrThrow(
        "drake/systems/controllers/qp_inverse_dynamics/"
        "test/iiwa.alias_groups");

    const std::string kControlConfigPath = FindResourceOrThrow(
        "drake/systems/controllers/qp_inverse_dynamics/"
        "test/iiwa.id_controller_config");

    std::default_random_engine generator(123);
    AllocateResources(kModelPath, kAliasGroupsPath, kControlConfigPath);
    SetRandomConfiguration(&generator);
    dut_ = std::unique_ptr<GenericPlan<double>>(new DummyPlan<double>());
  }

  void CheckIncompatibleAndThrow() {
    EXPECT_THROW(dut_->Initialize(*robot_status_, *params_, *alias_groups_),
                 std::logic_error);
    EXPECT_THROW(dut_->ModifyPlan(*robot_status_, *params_, *alias_groups_),
                 std::logic_error);
    EXPECT_THROW(dut_->HandlePlan(*robot_status_, *params_, *alias_groups_,
                                  Value<int>(10)),
                 std::logic_error);
    QpInput qp_input;
    EXPECT_THROW(dut_->UpdateQpInput(*robot_status_, *params_, *alias_groups_,
                                     &qp_input),
                 std::logic_error);
  }
};

// Tests GenericPlan's Initialize(). It should generate a plan with no
// contacts, no tracked bodies, and holds all dof at where the plan was
// initialized.
TEST_F(DummyPlanTest, TestInitialize) {
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  EXPECT_TRUE(dut_->get_planned_contact_state().empty());
  EXPECT_TRUE(dut_->get_body_trajectories().empty());

  // The desired position interpolated at any time should be equal to the
  // current posture.
  // The desired velocity and acceleration should be zero.
  std::vector<double> test_times = {robot_status_->get_time() - 0.5,
                                    robot_status_->get_time(),
                                    robot_status_->get_time() + 3};

  for (double time : test_times) {
    EXPECT_TRUE(drake::CompareMatrices(
        robot_status_->get_cache().getQ(),
        dut_->get_dof_trajectory().get_position(time), kSmallTolerance,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        VectorX<double>::Zero(robot_->get_num_velocities()),
        dut_->get_dof_trajectory().get_velocity(time), kSmallTolerance,
        drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(
        VectorX<double>::Zero(robot_->get_num_velocities()),
        dut_->get_dof_trajectory().get_acceleration(time), kSmallTolerance,
        drake::MatrixCompareType::absolute));
  }
}

// Tests if the cloned fields are the same as the original.
TEST_F(DummyPlanTest, TestClone) {
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);
  TestGenericClone();
}

// Checks the generated QpInput vs expected.
TEST_F(DummyPlanTest, TestUpdateQpInput) {
  dut_->Initialize(*robot_status_, *params_, *alias_groups_);

  QpInput qp_input;

  // Desired joint position and velocity.
  VectorX<double> q_d = robot_status_->get_cache().getQ();
  VectorX<double> v_d = VectorX<double>::Zero(robot_->get_num_velocities());
  VectorX<double> vd_d = VectorX<double>::Zero(robot_->get_num_velocities());

  // Changes the current state. The choice of position and velocity is
  // arbitrary.
  robot_status_->UpdateKinematics(0.66, 0.3 * robot_status_->get_cache().getQ(),
                                  0.4 * robot_status_->get_cache().getV());

  // Computes QpInput.
  dut_->UpdateQpInput(*robot_status_, *params_, *alias_groups_, &qp_input);

  // The expected dof acceleration should only contain the position and
  // velocity terms.
  VectorX<double> expected_vd = ComputeExpectedDoFAcceleration(q_d, v_d, vd_d);

  // The weights / constraint types are hard coded in this test. They need to
  // match the numbers specified in
  // "config/iiwa.id_controller_config".

  // Checks dof acceleration.
  EXPECT_TRUE(drake::CompareMatrices(
      expected_vd, qp_input.desired_dof_motions().values(), kSmallTolerance,
      drake::MatrixCompareType::absolute));
  for (int i = 0; i < robot_->get_num_positions(); ++i) {
    // Checks dof constraint type.
    EXPECT_EQ(qp_input.desired_dof_motions().constraint_type(i),
              ConstraintType::Soft);
    // Checks dof weight.
    EXPECT_EQ(qp_input.desired_dof_motions().weight(i), 1e-1);
  }

  // Checks force basis regularization weight.
  EXPECT_EQ(qp_input.w_basis_reg(), 1e-6);

  // No body motion objectives.
  EXPECT_TRUE(qp_input.desired_body_motions().empty());

  // No contacts.
  EXPECT_TRUE(qp_input.contact_information().empty());

  // No center of mass or angular momentum objective.
  for (int i = 0; i < qp_input.desired_centroidal_momentum_dot().size(); ++i) {
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().value(i), 0);
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().weight(i), 0);
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().constraint_type(i),
              ConstraintType::Skip);
  }
}

TEST_F(DummyPlanTest, TestIncompatbileAndThrow) {
  DummyPlan<double>* dummy_plan = dynamic_cast<DummyPlan<double>*>(dut_.get());
  dummy_plan->set_all_to_compatible();
  dummy_plan->set_model_compatible(false);
  CheckIncompatibleAndThrow();

  dummy_plan->set_all_to_compatible();
  dummy_plan->set_param_compatible(false);
  CheckIncompatibleAndThrow();

  dummy_plan->set_all_to_compatible();
  dummy_plan->set_alias_compatible(false);
  CheckIncompatibleAndThrow();

  dummy_plan->set_all_to_compatible();
  dummy_plan->set_model_compatible(false);
  dummy_plan->set_param_compatible(false);
  CheckIncompatibleAndThrow();

  dummy_plan->set_all_to_compatible();
  dummy_plan->set_model_compatible(false);
  dummy_plan->set_alias_compatible(false);
  CheckIncompatibleAndThrow();

  dummy_plan->set_all_to_compatible();
  dummy_plan->set_alias_compatible(false);
  dummy_plan->set_param_compatible(false);
  CheckIncompatibleAndThrow();

  dummy_plan->set_model_compatible(false);
  dummy_plan->set_alias_compatible(false);
  dummy_plan->set_param_compatible(false);
  CheckIncompatibleAndThrow();
}

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake
