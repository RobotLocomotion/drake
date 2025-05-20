#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/contact_solvers/contact_configuration.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_builder.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"
#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::FrameId;
using drake::geometry::SceneGraphInspector;
using drake::math::RigidTransformd;
using drake::multibody::contact_solvers::internal::ContactConfiguration;
using drake::multibody::contact_solvers::internal::SapConstraintJacobian;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapHuntCrossleyApproximation;
using drake::multibody::contact_solvers::internal::SapHuntCrossleyConstraint;
using drake::multibody::contact_solvers::internal::SapModel;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using drake::multibody::internal::CompliantContactManager;
using drake::multibody::internal::ContactProblemCache;
using drake::multibody::internal::SapDriver;
using drake::systems::Context;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = default;
  template <typename T>
  static VectorX<T> AssembleActuationInput(const MultibodyPlant<T>& plant,
                                           const Context<T>& context) {
    return plant.AssembleActuationInput(context);
  }
};

namespace internal {

class CompliantContactManagerTester {
 public:
  template <typename T>
  static const SapDriver<T>& sap_driver(
      const CompliantContactManager<T>& manager) {
    DRAKE_DEMAND(manager.sap_driver_ != nullptr);
    return *manager.sap_driver_;
  }
};

class SapDriverTest {
 public:
  static const ContactProblemCache<double>& EvalContactProblemCache(
      const SapDriver<double>& driver, const Context<double>& context) {
    return driver.EvalContactProblemCache(context);
  }
};

}  // namespace internal

namespace contact_solvers {
namespace pooled_sap {

class TwoSpheres : public testing::Test {
 public:
  TwoSpheres() {
    systems::DiagramBuilder<double> builder{};

    multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

    geometry::SceneGraphConfig scene_graph_config{
        .default_proximity_properties = {.compliance_type = "compliant"}};

    std::tie(plant_, scene_graph_) = multibody::AddMultibodyPlant(
        plant_config, scene_graph_config, &builder);

    // Discrete clone.
    plant_config.time_step = discrete_.time_step;
    systems::DiagramBuilder<double> discrete_builder{};
    std::tie(discrete_.plant, discrete_.scene_graph) =
        multibody::AddMultibodyPlant(plant_config, scene_graph_config,
                                     &discrete_builder);

    // Add two spheres with arbitrary mass/inertia.
    std::string xml = fmt::format(R"""(
    <mujoco model="twospheres">
      <worldbody>
         <body name="sphere1"> <geom type="sphere" size="{}"/><freejoint name="freejoint1"/></body>
         <body name="sphere2"> <geom type="sphere" size="{}"/><freejoint name="freejoint2"/></body>
      </worldbody>
    </mujoco>
    )""",
                                  kRadius1, kRadius2);
    {  // continuous
      multibody::Parser parser(plant_);
      parser.AddModelsFromString(xml, "xml");
      sphere1_ = &plant_->GetBodyByName("sphere1");
      sphere2_ = &plant_->GetBodyByName("sphere2");
      sphere1_index_ = sphere1_->index();
      sphere2_index_ = sphere2_->index();
    }

    {  // Discrete
      multibody::Parser parser(discrete_.plant);
      parser.AddModelsFromString(xml, "xml");
    }

    // plant_->SetUseSampledOutputPorts(false);

    // TODO(amcastro-tri): parameterize test.
    plant_->set_contact_model(ContactModel::kHydroelastic);
    discrete_.plant->set_contact_model(ContactModel::kHydroelastic);

    plant_->Finalize();
    discrete_.plant->Finalize();
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    discrete_.contact_manager = owned_contact_manager.get();
    discrete_.plant->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    fmt::print("nb: {}, nv: {}\n", plant_->num_bodies(),
               plant_->num_velocities());

    diagram_ = builder.Build();
    discrete_.diagram = discrete_builder.Build();

    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());

    discrete_.diagram_context = discrete_.diagram->CreateDefaultContext();
    discrete_.plant_context = &discrete_.plant->GetMyMutableContextFromRoot(
        discrete_.diagram_context.get());
  }

  void SetInContact(double penetration) {
    plant_->SetFreeBodyPoseInWorldFrame(plant_context_, *sphere1_,
                                        RigidTransformd::Identity());
    plant_->SetFreeBodyPoseInWorldFrame(
        plant_context_, *sphere2_,
        RigidTransformd(Vector3d(kRadius1 + kRadius2 - penetration, 0, 0)));

    // And discrete.
    discrete_.plant->SetPositionsAndVelocities(
        discrete_.plant_context,
        plant_->GetPositionsAndVelocities(*plant_context_));
  }

  const SapDriver<double>& sap_driver() const {
    return drake::multibody::internal::CompliantContactManagerTester::
        sap_driver(*discrete_.contact_manager);
  }
  const ContactProblemCache<double>& EvalContactProblemCache(
      const Context<double>& context) const {
    return drake::multibody::internal::SapDriverTest::EvalContactProblemCache(
        sap_driver(), context);
  }
  const SapContactProblem<double>& EvalSapProblem() const {
    const ContactProblemCache<double>& cache =
        EvalContactProblemCache(*discrete_.plant_context);
    return *cache.sap_problem;
  }

 protected:
  // Parameters (in sync with xml)
  double kRadius1{0.1};
  double kRadius2{0.2};

  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  const multibody::RigidBody<double>* sphere1_;
  const multibody::RigidBody<double>* sphere2_;
  multibody::BodyIndex sphere1_index_;
  multibody::BodyIndex sphere2_index_;
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_;

  struct DiscreteModel {
    double time_step{0.01};
    std::unique_ptr<systems::Diagram<double>> diagram;
    multibody::MultibodyPlant<double>* plant;
    geometry::SceneGraph<double>* scene_graph;
    CompliantContactManager<double>* contact_manager{nullptr};
    std::unique_ptr<systems::Context<double>> diagram_context;
    systems::Context<double>* plant_context;
  } discrete_;
};

template <typename T>
std::vector<geometry::ContactSurface<T>> CalcContactSurfaces(
    const MultibodyPlant<T>& plant, const Context<T>& context) {
  auto& query_object = plant.get_geometry_query_input_port()
                           .template Eval<geometry::QueryObject<T>>(context);

  return query_object.ComputeContactSurfaces(
      geometry::HydroelasticContactRepresentation::kPolygon);
}

TEST_F(TwoSpheres, GetContact) {
  SetInContact(0.001);

  const double time_step = 0.01;

  const std::vector<geometry::ContactSurface<double>> surfaces =
      CalcContactSurfaces(*plant_, *plant_context_);

  fmt::print("Num surfaces: {}\n", ssize(surfaces));
  fmt::print("Num pairs: {}\n", surfaces[0].num_faces());

  const SapContactProblem<double>& problem = EvalSapProblem();
  fmt::print("Problem:\n");
  fmt::print("  cliques    : {}\n", problem.num_cliques());
  fmt::print("  velocities : {}\n", problem.num_velocities());
  fmt::print("  constraints: {}\n", problem.num_constraints());

  const VectorXd v0 = plant_->GetVelocities(*plant_context_);
  SapSolver<double> sap;
  SapSolverResults<double> results;
  const SapSolverStatus status = sap.SolveWithGuess(problem, v0, &results);
  ASSERT_EQ(status, SapSolverStatus::kSuccess);

  const double accel_ratio = results.v(3) / results.v(9);
  const double mass_ratio = sphere2_->default_mass() / sphere1_->default_mass();

  fmt::print("v0: {}\n", fmt_eigen(v0.transpose()));
  fmt::print("v : {}\n", fmt_eigen(results.v.transpose()));
  fmt::print("j : {}\n", fmt_eigen(results.j.transpose()));

  EXPECT_NEAR(accel_ratio, -mass_ratio,
              20. * std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(results.j(3), -results.j(9),
              std::numeric_limits<double>::epsilon());

  fmt::print("Acc. ratio : {}\n", accel_ratio);
  fmt::print("Mass ratio : {}\n", mass_ratio);

  PooledSapBuilder<double> builder(*plant_);
  PooledSapModel<double> model;
  builder.UpdateModel(*plant_context_, time_step, &model);
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), plant_->num_velocities());
  EXPECT_EQ(model.num_patch_constraints(), 1);
}

TEST_F(TwoSpheres, MakeData) {
  const double penetration = 0.002;
  const double time_step = 0.01;
  SetInContact(penetration);
  const int nv = plant_->num_velocities();

  const int num_pairs =
      plant_->get_contact_model() == ContactModel::kPoint ? 1 : 4;

  PooledSapBuilder<double> builder(*plant_);
  PooledSapModel<double> model;
  builder.UpdateModel(*plant_context_, time_step, &model);
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(model.num_patch_constraints(), 1);
  EXPECT_EQ(model.clique_sizes(), std::vector<int>({nv}));

  PooledSapModel<double>::PatchConstraintsPool& patch_constraints =
      model.patch_constraints_pool();
  EXPECT_EQ(patch_constraints.num_patches(), 1);
  EXPECT_EQ(patch_constraints.total_num_pairs(), num_pairs);
  EXPECT_EQ(patch_constraints.patch_sizes(), std::vector<int>({num_pairs}));

  SapData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.num_patches(), 1);

  // Clear patch constraints and verify resizing data does not allocate.
  patch_constraints.Clear();
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(model.num_patch_constraints(), 0);
  {
    drake::test::LimitMalloc guard;
    model.ResizeData(&data);
  }
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(data.num_patches(), 0);

  // Update problem. There should be no allocations for the same problem size.
  // TODO(amcastro-tri): Move this function within the guard. You'll need a
  // pre-allocated workspace for this function.
  builder.UpdateModel(*plant_context_, time_step, &model);
  {
    drake::test::LimitMalloc guard;
    model.ResizeData(&data);
  }
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(data.num_patches(), 1);
}

// Fix bug in this test, related to evaling SapProblem with a discrete plant.
#if 0

const double kEps = std::numeric_limits<double>::epsilon();

TEST_F(TwoSpheres, CalcData) {
  const double penetration = 0.002;
  const double time_step = 0.01;
  SetInContact(penetration);
  const int nv = plant_->num_velocities();
  const int num_pairs =
      plant_->get_contact_model() == ContactModel::kPoint ? 1 : 4;

  PooledSapBuilder<double> builder(*plant_);
  PooledSapModel<double> model;
  builder.UpdateModel(*plant_context_, time_step, &model);
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), nv);
  // EXPECT_EQ(model.num_patch_constraints(), 1);

  SapData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  // EXPECT_EQ(data.num_patches(), 1);

  const VectorXd v = VectorXd::LinSpaced(nv, -1.5, 1.5);
  // VectorXd v(nv);
  // v << Vector3d::Zero(), Vector3d(1.0, 0.0, 0.0), Vector3d::Zero(),
  //     Vector3d(-1.0, 0.0, 0.0);
  model.CalcData(v, &data);

  // Compute a reference solution.
  const SapContactProblem<double>& problem = EvalSapProblem();
  SapModel<double> sap_model(&problem);
  auto context = sap_model.MakeContext();
  sap_model.GetMutableVelocities(context.get()) = v;
  double reference_cost = sap_model.EvalCost(*context);
  // N.B. momentum_cost = 1/2⋅(v-v*)ᵀ⋅A⋅(v-v*)
  const double ref_mom_cost = sap_model.EvalMomentumCost(*context);
  fmt::print("Ref. Cost: {}\n", reference_cost);
  fmt::print("Ref. Mom. Cost: {}\n", ref_mom_cost);

  // N.B. momentum_cost = 1/2⋅vᵀ⋅A⋅v - rᵀ⋅v
  fmt::print("Cost: {}\n", data.cache().cost);
  // For this problem, v* = 0, and then r = 0 and both sap_model and model
  // momentum costs match.
  EXPECT_NEAR(data.cache().momentum_cost, ref_mom_cost, kEps);
  EXPECT_NEAR(data.cache().cost, reference_cost, kEps);

  fmt::print("∇ℓ: {}\n", fmt_eigen(data.cache().gradient.transpose()));

  const PatchConstraintsDataPool<double> patch_data =
      data.cache().patch_constraints_data;

  EXPECT_EQ(patch_data.num_patches(), 1);
  EXPECT_EQ(patch_data.num_pairs(), num_pairs);

  // For this case we expect the body Jacobians to be identity.
  // The model convention places body B first always.
  ASSERT_EQ(data.cache().spatial_velocities.size(), 3);  // 2 bodies + world.
  EXPECT_TRUE(CompareMatrices(data.cache().spatial_velocities[0],
                              Vector6<double>::Zero(), kEps));  // World.
  EXPECT_TRUE(
      CompareMatrices(data.cache().spatial_velocities[1], v.head<6>(), kEps));
  EXPECT_TRUE(
      CompareMatrices(data.cache().spatial_velocities[2], v.tail<6>(), kEps));

  ASSERT_EQ(patch_data.Gamma_Bo_W_pool().size(), 1);  // One patch.
  fmt::print("Gmma_B_W: {}\n",
             fmt_eigen(patch_data.Gamma_Bo_W_pool()[0].transpose()));

  // Use SAP as a reference.
  const VectorX<double> reference_gradient =
      sap_model.EvalCostGradient(*context);
  EXPECT_TRUE(CompareMatrices(data.cache().gradient, reference_gradient, kEps,
                              MatrixCompareType::relative));

  // Hessian.
  const auto& H_factorization =
      sap_model.EvalHessianFactorizationCache(*context);
  ASSERT_FALSE(H_factorization.is_empty());
  MatrixXd Hinv = MatrixXd::Identity(nv, nv);
  H_factorization.SolveInPlace(&Hinv);
  const MatrixXd H_reference = Hinv.inverse();

  fmt::print("|H-Href| = {}\n", (H_reference - data.cache().hessian).norm());

  EXPECT_TRUE(CompareMatrices(data.cache().hessian, H_reference, 3.0e-13));
}
#endif

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
