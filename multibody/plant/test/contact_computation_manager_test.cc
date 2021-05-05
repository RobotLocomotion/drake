#include "drake/multibody/plant/contact_computation_manager.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_access.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace internal {
using contact_solvers::internal::ContactSolverResults;
using Eigen::VectorXd;
using systems::BasicVector;
using systems::Context;
using systems::DiscreteStateIndex;
using systems::DiscreteValues;
using systems::OutputPortIndex;
constexpr int kNumRigidDofs = 6;
constexpr int kNumAdditionalDofs = 9;
constexpr int kNumTotalDofs = kNumRigidDofs + kNumAdditionalDofs;
constexpr int kNumContacts = 4;
constexpr double kDummyVNext = 1.0;
constexpr double kDummyFn = 2.0;
constexpr double kDummyFt = 3.0;
constexpr double kDummyVn = 4.0;
constexpr double kDummyVt = 5.0;
constexpr double kDummyTau = 6.0;
constexpr double kDummyVdot = 7.0;
constexpr double kDt = 0.1;

/* A dummy manager class derived from ContactComputationManager for testing
 purpose. This dummy manager declares a discrete state in addition to the MbP's
 discrete state. The additional state is of size `kNumAdditionalDofs` and is
 intialized to zeros. The discrete update event increments each entry in the
 additional discrete state vector by 1. This class has a vector output port that
 reports this additional state and an abstract output port that reports the the
 same state. */
class DummyContactComputationManager : public ContactComputationManager<double>,
                                       private MultibodyPlantAccess<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyContactComputationManager);

  ~DummyContactComputationManager() = default;

  explicit DummyContactComputationManager(MultibodyPlant<double>* plant)
      : MultibodyPlantAccess<double>(plant) {}

  const systems::OutputPort<double>& get_abstract_output_port() const {
    return this->get_output_port(abstract_output_port_index_);
  }
  const systems::OutputPort<double>& get_vector_output_port() const {
    return this->get_output_port(vector_output_port_index_);
  }

  const ContactSolverResults<double>& EvalContactSolverResults(
      const Context<double>& context) {
    return MultibodyPlantAccess<double>::EvalContactSolverResults(context);
  }

 private:
  /* Declares a dummy discrete state of size `kNumAdditionalDofs` along with two
   output ports that reports the value of the dummy discrete state: one abstract
   output port with underlying value type VectorXd and one plain-old vector
   port. */
  void DeclareStateCacheAndPorts(MultibodyPlant<double>* plant) final {
    discrete_state_index_ =
        this->DeclareDiscreteState(VectorXd::Zero(kNumAdditionalDofs));
    abstract_output_port_index_ =
        this->DeclareAbstractOutputPort(
                "dummy_abstract_output_port",
                []() {
                  VectorXd model_value = VectorXd::Zero(kNumAdditionalDofs);
                  return AbstractValue::Make(model_value);
                },
                [this](const Context<double>& context, AbstractValue* output) {
                  VectorXd& data = output->get_mutable_value<VectorXd>();
                  data = context.get_discrete_state(discrete_state_index_)
                             .get_value();
                },
                // TODO(xuchenhan_tri): It may be worthwhile to expose the
                // discrete_state_ticket() method from MbP so that more
                // fine-grained ticket can be created.
                {systems::System<double>::xd_ticket()})
            .get_index();
    vector_output_port_index_ =
        this->DeclareVectorOutputPort(
                "dummy_vector_output_port",
                BasicVector<double>(kNumAdditionalDofs),
                [this](const Context<double>& context,
                       BasicVector<double>* output) {
                  auto data = output->get_mutable_value();
                  data = context.get_discrete_state(discrete_state_index_)
                             .get_value();
                },
                // TODO(xuchenhan_tri): It may be worthwhile to expose the
                // discrete_state_ticket() method from MbP so that more
                // fine-grained ticket can be created.
                {systems::System<double>::xd_ticket()})
            .get_index();
  }

  /* Assigns dummy values to the output ContactSolverResults. */
  void CalcContactSolverResults(
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
  void CalcAccelerationKinematicsCache(
      const Context<double>& context,
      internal::AccelerationKinematicsCache<double>* ac) const final {
    VectorXd& vdot = ac->get_mutable_vdot();
    vdot = VectorXd::Ones(vdot.size()) * kDummyVdot;
  }

  /* Increments the additional discrete state by 1. */
  void CalcDiscreteValues(const Context<double>& context0,
                          DiscreteValues<double>* updates) const final {
    auto discrete_data =
        updates->get_mutable_vector(discrete_state_index_).get_mutable_value();
    discrete_data += VectorXd::Ones(discrete_data.size());
  }

  OutputPortIndex abstract_output_port_index_;
  OutputPortIndex vector_output_port_index_;
  DiscreteStateIndex discrete_state_index_;
};

class ContactComputationManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_.AddRigidBody("rigid body", SpatialInertia<double>());
    plant_.Finalize();
    EXPECT_EQ(plant_.num_velocities(), kNumRigidDofs);
    manager_ = &plant_.set_contact_manager(
        std::make_unique<DummyContactComputationManager>(&plant_));
  }

  MultibodyPlant<double> plant_{kDt};                 // A discrete MbP.
  DummyContactComputationManager* manager_{nullptr};  // The manager under test.
};

/* Tests that the CalcDiscrete() method is correctly wired to MultibodyPlant. */
TEST_F(ContactComputationManagerTest, CalcDiscreteState) {
  auto context = plant_.CreateDefaultContext();
  auto simulator = systems::Simulator<double>(plant_, std::move(context));
  const int time_steps = 10;
  simulator.AdvanceTo(time_steps * kDt);
  const VectorXd final_state =
      manager_->get_vector_output_port().Eval(simulator.get_context());
  EXPECT_EQ(final_state.size(), kNumAdditionalDofs);
  EXPECT_TRUE(CompareMatrices(final_state,
                              VectorXd::Ones(kNumAdditionalDofs) * time_steps));

  // Verifies that the vector and abstract output reports the same state.
  const VectorXd final_state_through_abstract_port =
      manager_->get_abstract_output_port().Eval<VectorXd>(
          simulator.get_context());
  EXPECT_TRUE(CompareMatrices(final_state, final_state_through_abstract_port));
}

/* Tests that the CalcContactSolverResults() method is correctly wired to
 MultibodyPlant. */
TEST_F(ContactComputationManagerTest, CalcContactSolverResults) {
  auto context = plant_.CreateDefaultContext();
  const ContactSolverResults<double>& results =
      manager_->EvalContactSolverResults(*context);
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
TEST_F(ContactComputationManagerTest, CalcAccelerationKinematicsCache) {
  auto context = plant_.CreateDefaultContext();
  const auto generalized_acceleration =
      plant_.get_generalized_acceleration_output_port().Eval(*context);
  EXPECT_TRUE(CompareMatrices(generalized_acceleration,
                              VectorXd::Ones(kNumRigidDofs) * kDummyVdot));
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
