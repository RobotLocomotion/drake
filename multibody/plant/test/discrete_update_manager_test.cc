#include "drake/multibody/plant/discrete_update_manager.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/dummy_model.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace internal {
namespace test {
using contact_solvers::internal::ContactSolverResults;
using Eigen::VectorXd;
using systems::BasicVector;
using systems::Context;
using systems::DiscreteStateIndex;
using systems::DiscreteValues;
using systems::OutputPortIndex;
// Dummy state data.
constexpr int kNumRigidDofs = 6;
constexpr int kNumAdditionalDofs = 9;
constexpr double kDummyStateValue = 3.15;
// Dummy contact data.
constexpr int kNumContacts = 4;
constexpr double kDummyVNext = 1.0;
constexpr double kDummyFn = 2.0;
constexpr double kDummyFt = 3.0;
constexpr double kDummyVn = 4.0;
constexpr double kDummyVt = 5.0;
constexpr double kDummyTau = 6.0;
constexpr double kDummyVdot = 7.0;
constexpr double kDt = 0.1;

/* A dummy manager class derived from DiscreteUpdateManager for testing
 purpose. It implements the interface in DiscreteUpdateManager by filling in
 dummy data. */
class DummyDiscreteUpdateManager : public DiscreteUpdateManager<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyDiscreteUpdateManager);

  DummyDiscreteUpdateManager() = default;

  ~DummyDiscreteUpdateManager() = default;

  /* Returns the number of times CalcContactSolverResults() is called. */
  int num_calls_to_calc_contact_solver_results() const {
    return num_calls_to_calc_contact_solver_results_;
  }

 private:
  /* Extracts information about the additional discrete state that
   DummyModel declares if one exists in the owning MultibodyPlant. */
  void ExtractModelInfo() final {
    /* For unit testing we verify there is a single physical model of type
     DummyModel. */
    DRAKE_DEMAND(plant().physical_models().size() == 1);
    const auto* dummy_model =
        dynamic_cast<const DummyModel*>(plant().physical_models()[0].get());
    DRAKE_DEMAND(dummy_model != nullptr);
    additional_state_index_ = dummy_model->discrete_state_index();
  }

  /* Increments the number of times CalcContactSolverResults() is called for
   testing. */
  void DoCalcContactSolverResults(
      const Context<double>&,
      ContactSolverResults<double>* results) const final {
    ++num_calls_to_calc_contact_solver_results_;
    results->Resize(kNumRigidDofs, kNumContacts);
    results->v_next = VectorXd::Ones(kNumRigidDofs) * kDummyVNext;
    results->fn = VectorXd::Ones(kNumContacts) * kDummyFn;
    results->ft = VectorXd::Ones(2 * kNumContacts) * kDummyFt;
    results->vn = VectorXd::Ones(kNumContacts) * kDummyVn;
    results->vt = VectorXd::Ones(2 * kNumContacts) * kDummyVt;
    results->tau_contact = VectorXd::Ones(kNumRigidDofs) * kDummyTau;
  }

  // TODO(xuchenhan-tri): Currently AccelerationKinematicsCache only caches
  // acceleration for rigid dofs. Modify the dummy manager to test for
  // deformable acclerations when they are supported.
  /* Assigns dummy values to an AccelerationKinematicsCache. */
  void DoCalcAccelerationKinematicsCache(
      const Context<double>& context,
      internal::AccelerationKinematicsCache<double>* ac) const final {
    VectorXd& vdot = ac->get_mutable_vdot();
    vdot = VectorXd::Ones(vdot.size()) * kDummyVdot;
  }

  /* Increments the discrete rigid dofs by 1 and additional discrete state by 2
   if there is any. */
  void DoCalcDiscreteValues(const Context<double>& context,
                            DiscreteValues<double>* updates) const final {
    auto multibody_data = updates->get_mutable_vector(multibody_state_index())
                              .get_mutable_value();
    multibody_data += VectorXd::Ones(multibody_data.size());
    if (additional_state_index_.is_valid()) {
      auto additional_data =
          updates->get_mutable_vector(additional_state_index_)
              .get_mutable_value();
      additional_data += 2.0 * VectorXd::Ones(additional_data.size());
    }
  }

 private:
  systems::DiscreteStateIndex additional_state_index_;
  mutable int num_calls_to_calc_contact_solver_results_{0};
};

namespace {
class DiscreteUpdateManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_.AddRigidBody("rigid body", SpatialInertia<double>());
    dummy_model_ = &plant_.AddPhysicalModel(std::make_unique<DummyModel>());
    dummy_model_->AppendDiscreteState(dummy_discrete_state());
    plant_.Finalize();
    // MultibodyPlant::num_velocities() only reports the number of rigid
    // generalized velocities for the rigid model.
    EXPECT_EQ(plant_.num_velocities(), kNumRigidDofs);
    discrete_update_manager_ = &plant_.set_discrete_update_manager(
        std::make_unique<DummyDiscreteUpdateManager>());
  }

  static VectorXd dummy_discrete_state() {
    return VectorXd::Ones(kNumAdditionalDofs) * kDummyStateValue;
  }

  // A discrete MultibodyPlant.
  MultibodyPlant<double> plant_{kDt};
  // A PhysicalModel to illustrate how physical models and discrete update
  // managers interact.
  DummyModel* dummy_model_{nullptr};
  // The discrete update manager under test.
  DummyDiscreteUpdateManager* discrete_update_manager_{nullptr};
};

/* Tests that the CalcDiscrete() method is correctly wired to MultibodyPlant. */
TEST_F(DiscreteUpdateManagerTest, CalcDiscreteState) {
  auto context = plant_.CreateDefaultContext();
  auto simulator = systems::Simulator<double>(plant_, std::move(context));
  const int time_steps = 2;
  simulator.AdvanceTo(time_steps * kDt);
  const VectorXd final_additional_state =
      dummy_model_->get_vector_output_port().Eval(simulator.get_context());
  EXPECT_EQ(final_additional_state.size(), kNumAdditionalDofs);
  EXPECT_TRUE(
      CompareMatrices(final_additional_state,
                      dummy_discrete_state() +
                          2.0 * VectorXd::Ones(kNumAdditionalDofs) * time_steps,
                      std::numeric_limits<double>::epsilon()));
}

/* Tests that the CalcContactSolverResults() method is correctly wired to
 MultibodyPlant. */
TEST_F(DiscreteUpdateManagerTest, CalcContactSolverResults) {
  auto context = plant_.CreateDefaultContext();
  context->DisableCaching();
  // Evaluates an output port whose Calc function invokes
  // CalcContactSolverResults().
  const auto& port = plant_.get_generalized_contact_forces_output_port(
      default_model_instance());
  port.Eval(*context);
  EXPECT_EQ(
      discrete_update_manager_->num_calls_to_calc_contact_solver_results(), 1);
}

/* Tests that the CalcAccelerationKinematicsCache() method is correctly wired
 to MultibodyPlant. */
TEST_F(DiscreteUpdateManagerTest, CalcAccelerationKinematicsCache) {
  auto context = plant_.CreateDefaultContext();
  const auto generalized_acceleration =
      plant_.get_generalized_acceleration_output_port().Eval(*context);
  EXPECT_TRUE(CompareMatrices(generalized_acceleration,
                              VectorXd::Ones(kNumRigidDofs) * kDummyVdot));
}
}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace multibody
}  // namespace drake
