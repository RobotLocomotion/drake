#include "drake/multibody/plant/contact_computation_manager.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_access.h"

using Eigen::VectorXd;

namespace drake {
using systems::BasicVector;
using systems::Context;
using systems::DiscreteStateIndex;
using systems::DiscreteValues;
using systems::LeafSystem;
using systems::OutputPortIndex;
namespace multibody {
using contact_solvers::internal::ContactSolverResults;

constexpr double kDummyStateValue = 3.1415;
constexpr int kNumDofs = 3;
constexpr int kNumContacts = 4;
constexpr double kDummyVNext = 1.0;
constexpr double kDummyFn = 2.0;
constexpr double kDummyFt = 3.0;
constexpr double kDummyVn = 4.0;
constexpr double kDummyVt = 5.0;
constexpr double kDummyTau = 6.0;
constexpr double kDummyVdot = 7.0;

class DummyContactComputationManager
    : public internal::ContactComputationManager<double> {
 public:
  ~DummyContactComputationManager() = default;

  //   void DeclareStateCacheAndPorts(LeafSystem<double>* system) final {
  // discrete_state_index_ =
  //     leaf_system->DeclareDiscreteState(VectorXd::Zero(kNumDofs));
  // output_port_index_ = leaf_system
  //                          ->DeclareAbstractOutputPort(
  //                              "dummy_output_port",
  //                              &DummyContactComputationManager::CalcOutput)
  //                          .get_index();
  //   }

  // Assign dummy values to the output ContactSolverResults.
  void CalcContactSolverResults(
      const Context<double>&,
      ContactSolverResults<double>* results) const final {
    results->Resize(kNumDofs, kNumContacts);
    results->v_next = VectorXd::Ones(kNumDofs) * kDummyVNext;
    results->fn = VectorXd::Ones(kNumContacts) * kDummyFn;
    results->ft = VectorXd::Ones(2 * kNumContacts) * kDummyFt;
    results->vn = VectorXd::Ones(kNumContacts) * kDummyVn;
    results->vt = VectorXd::Ones(2 * kNumContacts) * kDummyVt;
    results->tau_contact = VectorXd::Ones(kNumDofs) * kDummyTau;
  }

  // Assign dummy values to the output AccelerationKinematicCache.
  void CalcAccelerationKinematicsCache(
      const Context<double>& context,
      internal::AccelerationKinematicsCache<double>* ac) const final {
    VectorXd& vdot = ac->get_mutable_vdot();
    vdot = VectorXd::Ones(kNumDofs) * kDummyVdot;
  }

  // Assign dummy values to the output DiscreteValues.
  void CalcDiscreteValues(const Context<double>& context0,
                          DiscreteValues<double>* updates) const final {
    auto discrete_data =
        updates->get_mutable_vector(discrete_state_index_).get_mutable_value();
    discrete_data.head(kNumDofs) = VectorXd::Ones(kNumDofs) * kDummyStateValue;
  }

 private:
  void CalcOutput(const Context<double>&, std::string* output) const {
    *output = "dummy output";
  }

  // Sets the cache entry value to be the same as the discrete state values.
  void CalcCacheEntry(const Context<double>& context,
                      AbstractValue* cache_value) const {
    VectorXd& dummy_value = cache_value->get_mutable_value<VectorXd>();
    dummy_value = context.get_discrete_state()
                      .get_vector(discrete_state_index_)
                      .get_value();
  }

  DiscreteStateIndex discrete_state_index_;
  OutputPortIndex output_port_index_;
};

class ContactComputationManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_.Finalize();
    manager_ = &plant_.set_contact_manager(
        std::make_unique<DummyContactComputationManager>());
  }

  MultibodyPlant<double> plant_{0.1};  // A discrete MbP.
  DummyContactComputationManager* manager_{nullptr};
  internal::MultibodyPlantAccess<double> plant_accessor_{&plant_};
};

// TEST_F(ContactComputationManagerTest, CalcDiscreteValues) {
//   auto context = plant_.CreateDefaultContext();
//   EXPECT_EQ(context->get_discrete_state().num_groups(), 1);
//   EXPECT_EQ(context->get_discrete_state_vector().size(), kNumDofs);
//   DiscreteValues<double> discrete_values(
//       std::make_unique<BasicVector<double>>(kNumDofs));
//   manager_->CalcDiscreteValues(*context, &discrete_values);
//   const VectorXd discrete_data = discrete_values.get_vector().get_value();
//   EXPECT_TRUE(CompareMatrices(discrete_data,
//                               VectorXd::Ones(kNumDofs) * kDummyStateValue));
// }

TEST_F(ContactComputationManagerTest, CalcContactSolverResults) {
  auto context = plant_.CreateDefaultContext();
  ContactSolverResults<double> results;
  manager_->CalcContactSolverResults(*context, &results);
}

TEST_F(ContactComputationManagerTest, CalcAccelerationKinematicsCache) {
  auto context = plant_.CreateDefaultContext();
  internal::AccelerationKinematicsCache<double> ac(
      plant_accessor_.internal_tree().get_topology());
  manager_->CalcAccelerationKinematicsCache(*context, &ac);
  EXPECT_TRUE(
      CompareMatrices(ac.get_vdot(), VectorXd::Ones(kNumDofs) * kDummyVdot));
}

}  // namespace multibody
}  // namespace drake
