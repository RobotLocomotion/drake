#include "drake/multibody/plant/discrete_update_manager.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/dummy_model.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/abstract_value_cloner.h"

namespace drake {
namespace multibody {
namespace internal {
namespace test {
using contact_solvers::internal::ContactSolverResults;
using Eigen::VectorXd;
using systems::BasicVector;
using systems::Context;
using systems::ContextBase;
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
 dummy data.
 @tparam_nonsymbolic_scalar */
template <typename T>
class DummyDiscreteUpdateManager : public DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyDiscreteUpdateManager);

  DummyDiscreteUpdateManager() = default;

  ~DummyDiscreteUpdateManager() final = default;

  /* Returns the number of times CalcContactSolverResults() is called. */
  int num_calls_to_calc_contact_solver_results() const {
    return num_calls_to_calc_contact_solver_results_;
  }

  const VectorX<T>& EvalCacheEntry(const systems::Context<T>& context) const {
    const auto& data = this->plant()
                           .get_cache_entry(cache_index_)
                           .template Eval<VectorX<T>>(context);
    return data;
  }

 private:
  /* Allow different specializations to access each other's private data for
   cloning to a different scalar type. */
  template <typename U>
  friend class DummyDiscreteUpdateManager;

  std::unique_ptr<DiscreteUpdateManager<double>> CloneToDouble() const final {
    return CloneToScalar<double>();
  }

  std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>> CloneToAutoDiffXd()
      const final {
    return CloneToScalar<AutoDiffXd>();
  }

  bool is_cloneable_to_double() const final { return true; }

  bool is_cloneable_to_autodiff() const final { return true; }

  template <typename ScalarType>
  std::unique_ptr<DiscreteUpdateManager<ScalarType>> CloneToScalar() const {
    auto clone = std::make_unique<DummyDiscreteUpdateManager<ScalarType>>();
    return clone;
  }

  /* Extracts information about the additional discrete state that
   DummyModel declares if one exists in the owning MultibodyPlant. */
  void ExtractModelInfo() final {
    /* For unit testing we verify there is a single physical model of type
     DummyModel. */
    DRAKE_DEMAND(this->plant().physical_models().size() == 1);
    const auto* dummy_model = dynamic_cast<const DummyModel<T>*>(
        this->plant().physical_models()[0].get());
    DRAKE_DEMAND(dummy_model != nullptr);
    additional_state_index_ = dummy_model->discrete_state_index();
  }

  /* Declares a cache entry that stores twice the additional state value. */
  void DeclareCacheEntries() final {
    cache_index_ = this->DeclareCacheEntry(
                           "Twice the additional_state value",
                           systems::ValueProducer(
                               this, VectorX<T>(kNumAdditionalDofs),
                               &DummyDiscreteUpdateManager::CalcTwiceState),
                           {systems::System<double>::xd_ticket()})
                       .cache_index();
  }

  void CalcTwiceState(const Context<T>& context, VectorX<T>* data) const {
    *data =
        2.0 * context.get_discrete_state(additional_state_index_).get_value();
  }

  /* Increments the number of times CalcContactSolverResults() is called for
   testing. */
  void DoCalcContactSolverResults(
      const Context<T>&, ContactSolverResults<T>* results) const final {
    ++num_calls_to_calc_contact_solver_results_;
    results->Resize(kNumRigidDofs, kNumContacts);
    results->v_next = VectorX<T>::Ones(kNumRigidDofs) * kDummyVNext;
    results->fn = VectorX<T>::Ones(kNumContacts) * kDummyFn;
    results->ft = VectorX<T>::Ones(2 * kNumContacts) * kDummyFt;
    results->vn = VectorX<T>::Ones(kNumContacts) * kDummyVn;
    results->vt = VectorX<T>::Ones(2 * kNumContacts) * kDummyVt;
    results->tau_contact = VectorX<T>::Ones(kNumRigidDofs) * kDummyTau;
  }

  // TODO(xuchenhan-tri): Currently AccelerationKinematicsCache only caches
  // acceleration for rigid dofs. Modify the dummy manager to test for
  // deformable acclerations when they are supported.
  /* Assigns dummy values to an AccelerationKinematicsCache. */
  void DoCalcAccelerationKinematicsCache(
      const Context<T>& context,
      internal::AccelerationKinematicsCache<T>* ac) const final {
    VectorX<T>& vdot = ac->get_mutable_vdot();
    vdot = VectorX<T>::Ones(vdot.size()) * kDummyVdot;
  }

  /* Increments the discrete rigid dofs by 1 and additional discrete state by 2
   if there is any. */
  void DoCalcDiscreteValues(const Context<T>& context,
                            DiscreteValues<T>* updates) const final {
    auto multibody_data =
        updates->get_mutable_value(this->multibody_state_index());
    multibody_data += VectorX<T>::Ones(multibody_data.size());
    if (additional_state_index_.is_valid()) {
      auto additional_data =
          updates->get_mutable_value(additional_state_index_);
      additional_data += 2.0 * VectorX<T>::Ones(additional_data.size());
    }
  }

 private:
  systems::DiscreteStateIndex additional_state_index_;
  systems::CacheIndex cache_index_;
  mutable int num_calls_to_calc_contact_solver_results_{0};
};

namespace {
class DiscreteUpdateManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_.AddRigidBody("rigid body", SpatialInertia<double>());
    auto dummy_model = std::make_unique<DummyModel<double>>();
    dummy_model_ = dummy_model.get();
    plant_.AddPhysicalModel(std::move(dummy_model));
    dummy_model_->AppendDiscreteState(dummy_discrete_state());
    plant_.Finalize();
    // MultibodyPlant::num_velocities() only reports the number of rigid
    // generalized velocities for the rigid model.
    EXPECT_EQ(plant_.num_velocities(), kNumRigidDofs);
    auto dummy_manager = std::make_unique<DummyDiscreteUpdateManager<double>>();
    dummy_manager_ = dummy_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(dummy_manager));
  }

  static VectorXd dummy_discrete_state() {
    return VectorXd::Ones(kNumAdditionalDofs) * kDummyStateValue;
  }

  // A discrete MultibodyPlant.
  MultibodyPlant<double> plant_{kDt};
  // A PhysicalModel to illustrate how physical models and discrete update
  // managers interact.
  DummyModel<double>* dummy_model_{nullptr};
  // The discrete update manager under test.
  DummyDiscreteUpdateManager<double>* dummy_manager_{nullptr};
};

/* Tests that the CalcDiscrete() method is correctly wired to MultibodyPlant and
 that the CacheEntry is properly evaluated. */
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
  /* Verifies that the cache value is twice the addition state value. */
  const VectorXd& cache_value =
      dummy_manager_->EvalCacheEntry(simulator.get_context());
  EXPECT_TRUE(CompareMatrices(2.0 * final_additional_state, cache_value,
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
  EXPECT_EQ(dummy_manager_->num_calls_to_calc_contact_solver_results(), 1);
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

/* Verifies that the DummyDiscreteUpdateManager survives scalar conversion from
 double to AutoDiffXd. */
TEST_F(DiscreteUpdateManagerTest, ScalarConversion) {
  auto autodiff_plant = systems::System<double>::ToAutoDiffXd(plant_);
  auto context = autodiff_plant->CreateDefaultContext();
  auto simulator =
      systems::Simulator<AutoDiffXd>(*autodiff_plant, std::move(context));
  ASSERT_EQ(autodiff_plant->physical_models().size(), 1);
  const DummyModel<AutoDiffXd>* model =
      dynamic_cast<const DummyModel<AutoDiffXd>*>(
          autodiff_plant->physical_models()[0].get());
  ASSERT_NE(model, nullptr);

  const int time_steps = 2;
  simulator.AdvanceTo(time_steps * kDt);

  /* Verify that the discrete state is updated as expected by the
   DummyDiscreteUpdateManager. */
  const VectorX<AutoDiffXd> final_additional_state_autodiff =
      model->get_vector_output_port().Eval(simulator.get_context());
  const VectorXd final_additional_state =
      math::ExtractValue(final_additional_state_autodiff);
  EXPECT_EQ(final_additional_state.size(), kNumAdditionalDofs);
  EXPECT_TRUE(
      CompareMatrices(final_additional_state,
                      dummy_discrete_state() +
                          2.0 * VectorXd::Ones(kNumAdditionalDofs) * time_steps,
                      std::numeric_limits<double>::epsilon()));
}
}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace multibody
}  // namespace drake
