#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"
#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace fem {

/* Deformable body parameters.  */
constexpr double kYoungsModulus = 1e5;      // unit: N/m²
constexpr double kPoissonRatio = 0.4;       // unitless.
constexpr double kDensity = 1e3;            // unit: kg/m³
constexpr double kMassDamping = 0;          // unit: 1/s
constexpr double kStiffnessDamping = 0.01;  // unit: s
constexpr double kSideLength = 0.1;         // unit: m
/* Time step (seconds). */
constexpr double kDt = 5e-3;
/* Contact parameters. */
const CoulombFriction kFriction{1.0, 1.0};
/* PGS solver parameters. */
/* Absolute tolerance (m/s) *and* relative tolerance (unitless). */
constexpr double kTolerance = 1e-6;
constexpr int kMaxIterations = 200;
constexpr double kRelaxationFactor = 1.0;

using drake::systems::Simulator;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::Box;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using math::RigidTransformd;
using multibody::contact_solvers::internal::ContactSolverResults;
using multibody::contact_solvers::internal::PgsSolver;
using systems::Context;

class DeformableRigidContactSolverTest : public ::testing::Test {
 protected:
  /* Set up a scene with a deformable cube sitting on the ground. */
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, kDt);

    /* Size of the deformable box. */
    const Box box = Box::MakeCube(kSideLength);

    /* Set up the deformable box. */
    /* Set the position of the box so that the bottom face aligns with the
     xy-plane. */
    const RigidTransformd X_WB(Vector3<double>(0, 0, kSideLength / 2.0));
    DeformableBodyConfig<double> box_config;
    box_config.set_youngs_modulus(kYoungsModulus);
    box_config.set_poisson_ratio(kPoissonRatio);
    box_config.set_mass_damping_coefficient(kMassDamping);
    box_config.set_stiffness_damping_coefficient(kStiffnessDamping);
    box_config.set_mass_density(kDensity);
    box_config.set_material_model(MaterialModel::kCorotated);
    /* Discretize the box into 2x2x2 grid of boxes (3x3x3 vertices). */
    constexpr int kNumBlocks = 2;
    const internal::ReferenceDeformableGeometry<double> deformable_box =
        MakeDiamondCubicBoxDeformableGeometry<double>(
            box, kSideLength / kNumBlocks, X_WB);
    geometry::ProximityProperties proximity_props;
    /* Use default stiffness and dissipation. */
    geometry::AddContactMaterial({}, {}, {}, kFriction, &proximity_props);
    /* Use the Corotated constitutive model. */
    auto deformable_model = std::make_unique<DeformableModel<double>>(plant_);
    deformable_index_ = deformable_model->RegisterDeformableBody(
        deformable_box, "deformable_box", box_config, proximity_props);
    deformable_model.get();
    plant_->AddPhysicalModel(std::move(deformable_model));

    constexpr double kRigidSize = 0.5;
    /* Set the position of the ground so that the top face aligns with the
     xy-plane. */
    const RigidTransformd X_WG(Vector3<double>(0, 0, -kRigidSize / 2));
    plant_->RegisterCollisionGeometry(plant_->world_body(), X_WG,
                                      Box::MakeCube(kRigidSize), "ground",
                                      proximity_props);
    plant_->Finalize();

    auto pgs_solver = std::make_unique<PgsSolver<double>>();
    pgs_solver->set_parameters(
        {kRelaxationFactor, kTolerance, kTolerance, kMaxIterations});
    auto owned_deformable_rigid_manager =
        std::make_unique<DeformableRigidManager<double>>(std::move(pgs_solver));
    deformable_rigid_manager_ = owned_deformable_rigid_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_deformable_rigid_manager));
    deformable_rigid_manager_->RegisterCollisionObjects(*scene_graph_);

    diagram_ = builder.Build();
  }

  /* Calls DeformableRigidManager::EvalFemStateBase(). */
  const FemStateBase<double>& EvalFemStateBase(
      const Context<double>& context, DeformableBodyIndex index) const {
    return deformable_rigid_manager_->EvalFemStateBase(context, index);
  }

  /* Calls DeformableRigidManager::EvalDeformableContactSolverResults(). */
  const ContactSolverResults<double>& EvalDeformableContactSolverResults(
      const Context<double>& context) const {
    return deformable_rigid_manager_->EvalDeformableContactSolverResults(
        context);
  }

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  const DeformableRigidManager<double>* deformable_rigid_manager_{nullptr};
  DeformableBodyIndex deformable_index_;
  std::unique_ptr<systems::Diagram<double>> diagram_{nullptr};
};

namespace {

/* Verify the contact impulse applied by the ground to the deformable box is as
 expected at steady state. */
TEST_F(DeformableRigidContactSolverTest, SteadyState) {
  Simulator<double> simulator(*diagram_);
  const auto& diagram_context = simulator.get_context();
  const auto& plant_context = plant_->GetMyContextFromRoot(diagram_context);
  /* Run simulation for long enough to reach steady state. */
  simulator.AdvanceTo(0.5);

  /* Verify the system has reached steady state. */
  const FemStateBase<double>& fem_state =
      EvalFemStateBase(plant_context, deformable_index_);
  constexpr double kSteadyStateThreshold = 4e-5;
  const VectorXd& v = fem_state.qdot();
  EXPECT_TRUE(
      CompareMatrices(v, VectorXd::Zero(v.size()), kSteadyStateThreshold));
  const VectorXd& a = fem_state.qddot();
  EXPECT_TRUE(CompareMatrices(a, VectorXd::Zero(a.size()),
                              kSteadyStateThreshold / kDt));

  /* Verify the contact impulse is equal to the contact force times the time
   step and that the contact force counterbalances gravity. */
  const ContactSolverResults<double>& deformable_contact_solver_results =
      EvalDeformableContactSolverResults(plant_context);
  const VectorXd& tau_contact = deformable_contact_solver_results.tau_contact;
  Vector3d contact_impulse = Vector3d::Zero();
  for (int i = 0; i < tau_contact.size() / 3; ++i) {
    contact_impulse += tau_contact.segment<3>(3 * i);
  }
  constexpr double kVolume = kSideLength * kSideLength * kSideLength;
  const Vector3d expected_contact_force =
      kVolume * kDensity * (-plant_->gravity_field().gravity_vector());
  const Vector3d expected_contact_impulse = expected_contact_force * kDt;
  EXPECT_TRUE(
      CompareMatrices(expected_contact_impulse, contact_impulse, kTolerance));
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
