#include "drake/multibody/plant/discrete_update_manager.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/dummy_model_manager.h"
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
constexpr int kNumTotalDofs = kNumRigidDofs + kNumAdditionalDofs;
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

 private:
  /* Extracts information about the additional discrete state that
   DummyModelManager declares if one exists in the owning MultibodyPlant. */
  void DoExtractModelInfo() final {
    bool extracted_dummy_model = false;
    for (const auto& model_manager : plant().model_managers()) {
      const auto* dummy_model =
          dynamic_cast<const DummyModelManager*>(model_manager.get());
      if (dummy_model) {
        if (extracted_dummy_model) {
          throw std::runtime_error(
              "Found more than one DummyModelManager. Please only register one "
              "DummyModelManager to a MultibodyPlant when testing");
        }
        additional_state_index_ = dummy_model->discrete_state_index();
        extracted_dummy_model = true;
      } else {
        throw std::runtime_error(
            "DummyDiscreteUpdateManager can only handle multibody models and a "
            "single dummy model but not models from any other "
            "PhysicalModelManager.");
      }
    }
  }

  /* Assigns dummy values to the output ContactSolverResults. */
  void DoCalcContactSolverResults(
      const Context<double>&,
      ContactSolverResults<double>* results) const final {
    results->Resize(kNumTotalDofs, kNumContacts);
    results->v_next = VectorXd::Ones(kNumTotalDofs) * kDummyVNext;
    results->fn = VectorXd::Ones(kNumContacts) * kDummyFn;
    results->ft = VectorXd::Ones(2 * kNumContacts) * kDummyFt;
    results->vn = VectorXd::Ones(kNumContacts) * kDummyVn;
    results->vt = VectorXd::Ones(2 * kNumContacts) * kDummyVt;
    results->tau_contact = VectorXd::Ones(kNumTotalDofs) * kDummyTau;
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
};

namespace {
class DiscreteUpdateManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_.AddRigidBody("rigid body", SpatialInertia<double>());
    model_manager_ =
        &plant_.AddModelManager(std::make_unique<DummyModelManager>(&plant_));
    model_manager_->AppendDiscreteState(dummy_discrete_state());
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

  // A discrete MbP.
  MultibodyPlant<double> plant_{kDt};
  // A PhysicalModelManager to illustrate how model managers and discrete update
  // managers interact.
  DummyModelManager* model_manager_{nullptr};
  // The discrete update manager under test.
  DummyDiscreteUpdateManager* discrete_update_manager_{nullptr};
};

/* Tests that the CalcDiscrete() method is correctly wired to MultibodyPlant. */
TEST_F(DiscreteUpdateManagerTest, CalcDiscreteState) {
  auto context = plant_.CreateDefaultContext();
  auto simulator = systems::Simulator<double>(plant_, std::move(context));
  const int time_steps = 10;
  simulator.AdvanceTo(time_steps * kDt);
  const VectorXd final_additional_state =
      model_manager_->get_vector_output_port().Eval(simulator.get_context());
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
  const ContactSolverResults<double>& results =
      discrete_update_manager_->EvalContactSolverResults(*context);
  EXPECT_TRUE(CompareMatrices(results.v_next,
                              VectorXd::Ones(kNumTotalDofs) * kDummyVNext));
  EXPECT_TRUE(
      CompareMatrices(results.fn, VectorXd::Ones(kNumContacts) * kDummyFn));
  EXPECT_TRUE(
      CompareMatrices(results.ft, VectorXd::Ones(2 * kNumContacts) * kDummyFt));
  EXPECT_TRUE(
      CompareMatrices(results.vn, VectorXd::Ones(kNumContacts) * kDummyVn));
  EXPECT_TRUE(
      CompareMatrices(results.vt, VectorXd::Ones(2 * kNumContacts) * kDummyVt));
  EXPECT_TRUE(CompareMatrices(results.tau_contact,
                              VectorXd::Ones(kNumTotalDofs) * kDummyTau));
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
