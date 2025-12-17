#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/contact_solvers/icf/icf_builder.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/plant/sap_driver.h"

using drake::math::RigidTransformd;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using drake::multibody::internal::CompliantContactManager;
using drake::multibody::internal::ContactProblemCache;
using drake::multibody::internal::SapDriver;
using drake::systems::Context;
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
namespace icf {
namespace internal {

class TwoSpheres : public testing::TestWithParam<ContactModel> {
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

    plant_->set_contact_model(GetParam());
    discrete_.plant->set_contact_model(GetParam());

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
    plant_->SetFloatingBaseBodyPoseInWorldFrame(plant_context_, *sphere1_,
                                                RigidTransformd::Identity());
    plant_->SetFloatingBaseBodyPoseInWorldFrame(
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

TEST_P(TwoSpheres, GetContact) {
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

  IcfBuilder<double> builder(*plant_);
  IcfModel<double> model;
  IcfLinearFeedbackGains<double> no_feedback;
  no_feedback.K.setZero(plant_->num_velocities());
  no_feedback.b.setZero(plant_->num_velocities());
  builder.UpdateModel(*plant_context_, time_step, &no_feedback, &no_feedback,
                      &model);
  EXPECT_EQ(model.num_cliques(), 2);
  EXPECT_EQ(model.num_velocities(), plant_->num_velocities());
  EXPECT_EQ(model.num_patch_constraints(), 1);
}

TEST_P(TwoSpheres, MakeData) {
  const double penetration = 0.002;
  const double time_step = 0.01;
  SetInContact(penetration);
  const int nv = plant_->num_velocities();

  const int num_pairs =
      plant_->get_contact_model() == ContactModel::kPoint ? 1 : 4;

  IcfBuilder<double> builder(*plant_);
  IcfModel<double> model;
  builder.UpdateModel(*plant_context_, time_step, &model);
  EXPECT_EQ(model.num_cliques(), 2);
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(model.num_patch_constraints(), 1);
  EXPECT_EQ(model.clique_size(0), 6);
  EXPECT_EQ(model.clique_size(1), 6);

  PatchConstraintsPool<double>& patch_constraints =
      model.patch_constraints_pool();
  EXPECT_EQ(patch_constraints.num_patches(), 1);
  EXPECT_EQ(patch_constraints.total_num_pairs(), num_pairs);
  EXPECT_THAT(patch_constraints.patch_sizes(), testing::ElementsAre(num_pairs));

  IcfData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.patch_constraints_data().num_constraints(), 1);

  // Clear patch constraints and verify resizing data does not allocate.
  patch_constraints.Resize({});
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(model.num_patch_constraints(), 0);
  {
    drake::test::LimitMalloc guard;
    model.ResizeData(&data);
  }
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(data.patch_constraints_data().num_constraints(), 0);

  // Update problem. There should be no allocations for the same problem size.
  {
    // TODO(amcastro-tri): Fix this function to not allocate. You'll need a
    // pre-allocated workspace for this function.
    drake::test::LimitMalloc guard({
        .max_num_allocations = 352,
        .min_num_allocations = 0,
        .ignore_realloc_noops = true,
    });
    builder.UpdateModel(*plant_context_, time_step, &model);
  }
  {
    drake::test::LimitMalloc guard;
    model.ResizeData(&data);
  }
  EXPECT_EQ(model.num_velocities(), nv);
  EXPECT_EQ(data.patch_constraints_data().num_constraints(), 1);
}

INSTANTIATE_TEST_SUITE_P(
    TestContactModels, TwoSpheres,
    testing::Values(ContactModel::kHydroelastic, ContactModel::kPoint,
                    ContactModel::kHydroelasticWithFallback),
    [](const testing::TestParamInfo<TwoSpheres::ParamType>& stuff) {
      return multibody::internal::GetStringFromContactModel(stuff.param);
    });

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
