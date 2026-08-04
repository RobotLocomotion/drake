#include "drake/multibody/plant/discrete_update_manager.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/kinematics_vector.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/dummy_physical_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/plant/test_utilities/multibody_plant_remodeling.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/abstract_value_cloner.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  // Returns the manager for the given plant.
  // @pre The plant must be discrete time and already finalized.
  template <typename T>
  static internal::DiscreteUpdateManager<T>& discrete_update_manager(
      const MultibodyPlant<T>& plant) {
    auto* manager = plant.discrete_update_manager_.get();
    DRAKE_DEMAND(manager != nullptr);
    return *manager;
  }
};

namespace internal {
namespace test {
using contact_solvers::internal::ContactSolverResults;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::FramePoseVector;
using geometry::Sphere;
using math::RigidTransformd;
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

/* Returns a pointer to the DummyPhysicalModel owned by the given
 MultibodyPlant. Returns the nullptr if no DummyPhysicalModel exists. */
template <typename T>
const DummyPhysicalModel<T>* GetDummyModel(const MultibodyPlant<T>& plant) {
  for (const auto* model : plant.physical_models()) {
    const auto* dummy_model = dynamic_cast<const DummyPhysicalModel<T>*>(model);
    if (dummy_model != nullptr) {
      return dummy_model;
    }
  }
  return nullptr;
}

/* A dummy manager class derived from DiscreteUpdateManager for testing
 purpose. It implements the interface in DiscreteUpdateManager by filling in
 dummy data.
 @tparam_nonsymbolic_scalar */
template <typename T>
class DummyDiscreteUpdateManager final : public DiscreteUpdateManager<T> {
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
   DummyPhysicalModel declares if one exists in the owning MultibodyPlant. */
  void DoExtractModelInfo() final {
    const DummyPhysicalModel<T>* dummy_model = GetDummyModel(this->plant());
    DRAKE_DEMAND(dummy_model != nullptr);
    additional_state_index_ = dummy_model->discrete_state_index();
  }

  /* Declares a cache entry that stores twice the additional state value. */
  void DoDeclareCacheEntries() final {
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

  void DoCalcDiscreteUpdateMultibodyForces(
      const systems::Context<T>&, MultibodyForces<T>* forces) const final {
    DRAKE_DEMAND(forces != nullptr);
    forces->SetZero();
  }

  void DoCalcActuation(const systems::Context<T>&,
                       VectorX<T>* tau) const final {
    DRAKE_DEMAND(tau != nullptr);
    tau->setZero();
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
    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    plant_.AddRigidBody("rigid body", SpatialInertia<double>::MakeUnitary());
    auto dummy_model = std::make_unique<DummyPhysicalModel<double>>(&plant_);
    dummy_model_ = dummy_model.get();
    plant_.AddDummyModel(std::move(dummy_model));
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
  DummyPhysicalModel<double>* dummy_model_{nullptr};
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
  plant_.ExecuteForcedEvents(context.get());  // Force a time step.
  EXPECT_EQ(dummy_manager_->num_calls_to_calc_contact_solver_results(), 1);
}

/* Tests that the CalcAccelerationKinematicsCache() method is correctly wired
 to MultibodyPlant. */
TEST_F(DiscreteUpdateManagerTest, CalcAccelerationKinematicsCache) {
  auto context = plant_.CreateDefaultContext();
  plant_.ExecuteForcedEvents(context.get());  // Force a time step.
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
  const DummyPhysicalModel<AutoDiffXd>* model = GetDummyModel(*autodiff_plant);
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

/* Tests that the contact solver results correctly depends on the actuation
 inputs. Guards against regression in #18682. */
GTEST_TEST(DiscreteUpdateManagerCacheEntry, ContactSolverResults) {
  double dt = 0.25;
  systems::DiagramBuilder<double> builder;
  MultibodyPlantConfig config{.time_step = dt};
  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
  const std::string sdf_model = R"""(
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="object">
    <joint name="z_axis" type="prismatic">
      <parent>world</parent>
      <child>object</child>
      <axis>0 0 1</axis>
    </joint>
    <link name="object">
      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
)""";
  const std::vector<ModelInstanceIndex> models =
      Parser(&plant).AddModelsFromString(sdf_model, "sdf");
  /* Disable gravity for easier algebra. */
  plant.mutable_gravity_field().set_gravity_vector(Vector3<double>::Zero());
  plant.Finalize();
  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&context);

  const ModelInstanceIndex only_model = models[0];
  const systems::InputPort<double>& u_input =
      plant.get_actuation_input_port(only_model);
  /* When there's zero actuation, the velocity should stay zero due to the lack
   of gravity. */
  u_input.FixValue(&plant_context, Vector1<double>(0.0));
  simulator.AdvanceTo(dt);
  VectorX<double> v = plant.GetVelocities(plant_context, only_model);
  ASSERT_EQ(v.size(), 1);
  EXPECT_TRUE(CompareMatrices(v, Vector1<double>(0.0)));

  /* We are not interested in the actual result of the contact force. Instead,
   we pull on this output port to force a ContactSolverResult calculation which
   depends on MbP's external force input ports. We will switch the values in
   these input ports immediately below, and the test ensures that the discrete
   updates below resamples the input ports upon updates.  */
  plant.get_generalized_contact_forces_output_port(only_model)
      .Eval(plant_context);
  /* Then switch to a non-zero actuation and advance another step. */
  const double nonzero_actuation = 1.2;
  u_input.FixValue(&plant_context, Vector1<double>(nonzero_actuation));
  /* Advance another time step. */
  simulator.AdvanceTo(2.0 * dt);
  v = plant.GetVelocities(plant_context, only_model);
  ASSERT_EQ(v.size(), 1);
  /* The velocity update in this time step should reflect the change in
   actuation. */
  EXPECT_TRUE(CompareMatrices(v, Vector1<double>(nonzero_actuation * dt)));
}

/* Tests that actuation forces are accumulated using the correct indexing from
 JointActuaton::input_start() */
TEST_F(MultibodyPlantRemodelingDiscrete, RemoveJointActuator) {
  BuildModel();
  DoRemoval(true /* remove actuator */, false /* do not remove joint */);
  // Set gravity vector to zero so there is no force element contribution.
  plant_->mutable_gravity_field().set_gravity_vector(Vector3d::Zero());

  FinalizeAndBuild();

  const systems::InputPort<double>& u_input =
      plant_->get_actuation_input_port();
  u_input.FixValue(plant_context_, Vector2d(0.25, 0.5));

  DiscreteUpdateManager<double>& manager =
      MultibodyPlantTester::discrete_update_manager(*plant_);

  // CalcNonContactForces includes:
  //   - Force elements
  //   - Externally applied general/spatial forces
  //   - Feed forward actuation
  //   - PD controlled actuation
  //   - Joint limits penalty forces
  // By construction of the model above, all of these are zero except for the
  // feed forward actuation. Thus
  // DiscreteUpdateManager::CalcJointActuationForces() is the only function that
  // contributes to the accumulated forces. This tests that the indexing in
  // CalcJointActuationForces() correctly uses JointActuaton::input_start().
  MultibodyForces<double> forces(*plant_);
  manager.CalcNonContactForces(
      *plant_context_, false /* no joint limit penalty forces */,
      false /* no pd controlled actuator forces */, &forces);

  const Vector3d expected_actuation_wo_pd(0.25, 0.0, 0.5);
  EXPECT_TRUE(
      CompareMatrices(forces.generalized_forces(), expected_actuation_wo_pd));
}

/* Helper class that sets the pose of geometries in SceneGraph. */
class PoseSource : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSource);
  PoseSource() {
    this->DeclareAbstractOutputPort(systems::kUseDefaultName,
                                    FramePoseVector<double>(),
                                    &PoseSource::ReadPoses);
  }

  void SetPoses(FramePoseVector<double> poses) { poses_ = std::move(poses); }

 private:
  void ReadPoses(const Context<double>&, FramePoseVector<double>* poses) const {
    *poses = poses_;
  }

  FramePoseVector<double> poses_;
};

/* Tests that the discrete contact computation doesn't choke on contact pairs
 that involve geometries outside of MbP. */
GTEST_TEST(DiscreteUpdateManagerGeometryTest, NonMbPContactPair) {
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.01;
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  Parser parser(&plant, &scene_graph);
  parser.AddModelsFromUrl("package://drake/multibody/models/box.urdf");
  plant.Finalize();

  /* Add another proximity geometry (that doesn't belong to the existing plant)
   to SceneGraph. */
  MultibodyPlant<double> other_plant(0.0);
  Parser other_parser(&other_plant, &scene_graph);
  const auto model_instance_indices = other_parser.AddModelsFromUrl(
      "package://drake/multibody/models/box.urdf");
  other_plant.Finalize();
  const auto body_indices =
      other_plant.GetBodyIndices(model_instance_indices[0]);
  ASSERT_EQ(body_indices.size(), 1);
  const BodyIndex body_index = body_indices[0];
  const auto frame_id = other_plant.GetBodyFrameIdOrThrow(body_index);
  const auto source_id = other_plant.get_source_id().value();
  auto pose_source = builder.template AddSystem<PoseSource>();
  /* Set the pose of the new geometry so that it's in contact with the geometry
   in the plant. */
  pose_source->SetPoses({{frame_id, RigidTransformd()}});
  builder.Connect(pose_source->get_output_port(0),
                  scene_graph.get_source_pose_port(source_id));

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  DRAKE_EXPECT_THROWS_MESSAGE(simulator.AdvanceTo(0.1),
                              ".*contact results.*ID is not known.*");
}

}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace multibody
}  // namespace drake
