#include "drake/multibody/plant/deformable_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::Sphere;
using math::RigidTransformd;
using std::make_unique;
using systems::BasicVector;

class DeformableModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    constexpr double kDt = 0.01;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder_, kDt);
    auto deformable_model = make_unique<DeformableModel<double>>(plant_);
    deformable_model_ptr_ = deformable_model.get();
    plant_->AddPhysicalModel(std::move(deformable_model));
  }

  systems::DiagramBuilder<double> builder_;
  const fem::DeformableBodyConfig<double> default_body_config_{};
  DeformableModel<double>* deformable_model_ptr_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};

  /* Registers a deformable sphere with the given initial pose in world. */
  DeformableBodyId RegisterSphere(double resolution_hint,
                                  RigidTransformd X_WS = RigidTransformd()) {
    auto geometry =
        make_unique<GeometryInstance>(X_WS, make_unique<Sphere>(1), "sphere");
    DeformableBodyId body_id = deformable_model_ptr_->RegisterDeformableBody(
        std::move(geometry), default_body_config_, resolution_hint);
    return body_id;
  }
};

/* Verifies that a DeformableModel has been successfully created. */
TEST_F(DeformableModelTest, Constructor) {
  ASSERT_NE(deformable_model_ptr_, nullptr);
  EXPECT_EQ(deformable_model_ptr_->num_bodies(), 0);
}

TEST_F(DeformableModelTest, RegisterDeformableBody) {
  constexpr double kRezHint = 0.5;
  DeformableBodyId body_id = RegisterSphere(kRezHint);
  EXPECT_EQ(deformable_model_ptr_->num_bodies(), 1);
  /* Verify that a corresponding FemModel has been built. */
  EXPECT_NO_THROW(deformable_model_ptr_->GetFemModel(body_id));

  /* Registering deformable bodies after Finalize() is prohibited. */
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->RegisterDeformableBody(
          make_unique<GeometryInstance>(RigidTransformd(),
                                        make_unique<Sphere>(1), "sphere"),
          default_body_config_, kRezHint),
      ".*RegisterDeformableBody.*after system resources have been declared.*");
}

/* Coarsely tests that SetWallBoundaryCondition adds some sort of boundary
 condition. Showing that boundary conditions only get conditionally added (based
 on location of the boundary wall) is sufficient evidence to infer that the
 right dofs are being given the right constraints. See
 deformable_boundary_condition_test.cc for a more complete test on the effect of
 calling this function. */
TEST_F(DeformableModelTest, SetWallBoundaryCondition) {
  constexpr double kRezHint = 0.5;
  DeformableBodyId body_id = RegisterSphere(kRezHint);
  const auto& fem_model = deformable_model_ptr_->GetFemModel(body_id);
  const auto& dirichlet_bc = fem_model.dirichlet_boundary_condition();
  /* No boundary condition has been added yet. */
  EXPECT_TRUE(dirichlet_bc.index_to_boundary_state().empty());
  /* Put the wall just far away enough so that no boundary condition is added.
   */
  const Eigen::Vector3d p_WQ1(0, 0, -1.0 - 1e-10);
  const Eigen::Vector3d n_W(0, 0, 1);
  deformable_model_ptr_->SetWallBoundaryCondition(body_id, p_WQ1, n_W);
  EXPECT_TRUE(dirichlet_bc.index_to_boundary_state().empty());
  /* Put the wall just close enough so that some boundary conditions are added.
   */
  const Eigen::Vector3d p_WQ2(0, 0, -1.0 + 1e-10);
  deformable_model_ptr_->SetWallBoundaryCondition(body_id, p_WQ2, n_W);
  EXPECT_FALSE(dirichlet_bc.index_to_boundary_state().empty());

  /* Throws when called on unregistered id. */
  const DeformableBodyId fake_body_id = DeformableBodyId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->SetWallBoundaryCondition(fake_body_id, p_WQ2, n_W),
      fmt::format(".*No.*id.*{}.*registered.*", fake_body_id));

  /* Setting boundary condition must be done pre-finalize. */
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->SetWallBoundaryCondition(body_id, p_WQ2, n_W),
      ".*SetWallBoundaryCondition.*after system resources have been "
      "declared.*");
}

TEST_F(DeformableModelTest, DiscreteStateIndexAndReferencePositions) {
  constexpr double kRezHint = 0.5;
  Sphere sphere(1.0);
  auto geometry = make_unique<GeometryInstance>(
      RigidTransformd(), make_unique<Sphere>(sphere), "sphere");
  const DeformableBodyId body_id =
      deformable_model_ptr_->RegisterDeformableBody(
          std::move(geometry), default_body_config_, kRezHint);

  /* Getting state index before Finalize() is prohibited. */
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetDiscreteStateIndex(body_id),
      ".*GetDiscreteStateIndex.*before system resources have been declared.*");

  /* Verify that the position values in  default discrete state is given by the
   reference positions. */
  plant_->Finalize();
  systems::DiscreteStateIndex state_index =
      deformable_model_ptr_->GetDiscreteStateIndex(body_id);
  auto context = plant_->CreateDefaultContext();
  const VectorX<double>& discrete_state =
      context->get_discrete_state(state_index).value();
  const int num_dofs = deformable_model_ptr_->GetFemModel(body_id).num_dofs();
  EXPECT_EQ(discrete_state.head(num_dofs),
            deformable_model_ptr_->GetReferencePositions(body_id));
  /* Verify that the velocity and acceleration values in default discrete state
   is given by the reference positions. */
  EXPECT_EQ(discrete_state.tail(2 * num_dofs),
            VectorX<double>::Zero(2 * num_dofs));
}

/* Verifies that calling any member function with an invalid body id throws,
 even if everything else was done correctly. */
TEST_F(DeformableModelTest, InvalidBodyId) {
  DeformableBodyId fake_id = DeformableBodyId::get_new_id();
  /* Pre-finalize calls. */
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetFemModel(fake_id),
                              "GetFemModel.*No deformable body with id.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetReferencePositions(fake_id),
      "GetReferencePositions.*No deformable body with id.*");

  plant_->Finalize();
  /* Post-finalize calls. */
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetDiscreteStateIndex(fake_id),
      "GetDiscreteStateIndex.*No deformable body with id.*");
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetFemModel(fake_id),
                              "GetFemModel.*No deformable body with id.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetReferencePositions(fake_id),
      "GetReferencePositions.*No deformable body with id.*");
}

TEST_F(DeformableModelTest, GetBodyIdFromBodyIndex) {
  constexpr double kRezHint = 0.5;
  const DeformableBodyId body_id = RegisterSphere(kRezHint);
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetBodyId(DeformableBodyIndex(0)),
      ".*GetBodyId.*before system resources have been declared.*");
  plant_->Finalize();
  EXPECT_EQ(deformable_model_ptr_->GetBodyId(DeformableBodyIndex(0)), body_id);
  // Throws for invalid indexes.
  EXPECT_THROW(deformable_model_ptr_->GetBodyId(DeformableBodyIndex(1)),
               std::exception);
  EXPECT_THROW(deformable_model_ptr_->GetBodyId(DeformableBodyIndex()),
               std::exception);
}

TEST_F(DeformableModelTest, GetBodyIndex) {
  constexpr double kRezHint = 0.5;
  const DeformableBodyId body_id = RegisterSphere(kRezHint);
  /* Throws for pre-finalize call. */
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetBodyIndex(body_id),
                              ".*before system resources.*declared.*");
  plant_->Finalize();
  EXPECT_EQ(deformable_model_ptr_->GetBodyIndex(body_id),
            DeformableBodyIndex(0));
  /* Throws for unregistered body id. */
  const DeformableBodyId fake_body_id = DeformableBodyId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetBodyIndex(fake_body_id),
      fmt::format(".*No.*id.*{}.*registered.*", fake_body_id));
}

TEST_F(DeformableModelTest, GetGeometryId) {
  constexpr double kRezHint = 0.5;
  const DeformableBodyId body_id = RegisterSphere(kRezHint);
  geometry::GeometryId geometry_id =
      deformable_model_ptr_->GetGeometryId(body_id);
  const SceneGraphInspector<double>& inspector =
      scene_graph_->model_inspector();
  EXPECT_TRUE(inspector.IsDeformableGeometry(geometry_id));
  DeformableBodyId fake_id = DeformableBodyId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetGeometryId(fake_id),
                              "GetGeometryId.*No deformable body with id.*");
}

TEST_F(DeformableModelTest, GetBodyIdFromGeometryId) {
  constexpr double kRezHint = 0.5;
  const DeformableBodyId body_id = RegisterSphere(kRezHint);
  geometry::GeometryId geometry_id =
      deformable_model_ptr_->GetGeometryId(body_id);
  /* Test pre-finalize call. */
  EXPECT_EQ(deformable_model_ptr_->GetBodyId(geometry_id), body_id);
  plant_->Finalize();
  /* Test post-finalize call. */
  EXPECT_EQ(deformable_model_ptr_->GetBodyId(geometry_id), body_id);
  /* Throws for unregistered geometry id. */
  const geometry::GeometryId fake_geometry_id =
      geometry::GeometryId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetBodyId(fake_geometry_id),
      ".*GeometryId.*not.*registered.*");
}

TEST_F(DeformableModelTest, ToPhysicalModelPointerVariant) {
  PhysicalModelPointerVariant<double> variant =
      deformable_model_ptr_->ToPhysicalModelPointerVariant();
  EXPECT_TRUE(std::holds_alternative<const DeformableModel<double>*>(variant));
}

TEST_F(DeformableModelTest, VertexPositionsOutputPort) {
  Sphere sphere(1.0);
  auto geometry = make_unique<GeometryInstance>(
      RigidTransformd(), make_unique<Sphere>(sphere), "sphere");
  constexpr double kRezHint = 0.5;
  DeformableBodyId body_id = deformable_model_ptr_->RegisterDeformableBody(
      std::move(geometry), default_body_config_, kRezHint);
  plant_->Finalize();

  std::unique_ptr<systems::Context<double>> context =
      plant_->CreateDefaultContext();
  std::unique_ptr<AbstractValue> output_value =
      deformable_model_ptr_->vertex_positions_port().Allocate();
  /* Compute the configuration for each geometry in the model. */
  deformable_model_ptr_->vertex_positions_port().Calc(*context,
                                                      output_value.get());
  const geometry::GeometryConfigurationVector<double>& configurations =
      output_value->get_value<geometry::GeometryConfigurationVector<double>>();

  /* There's only one body and one geometry. */
  EXPECT_EQ(configurations.size(), 1);
  const geometry::GeometryId geometry_id =
      deformable_model_ptr_->GetGeometryId(body_id);
  ASSERT_TRUE(configurations.has_id(geometry_id));
  /* Verify that the vertex positions port returns expected values, which in
   this case should be the reference positions. */
  EXPECT_EQ(configurations.value(geometry_id),
            deformable_model_ptr_->GetReferencePositions(body_id));
}

TEST_F(DeformableModelTest, AddFixedConstraint) {
  RigidTransformd X_WA(math::RollPitchYawd(1, 2, 3),
                       Vector3d(1.23, 4.56, 7.89));
  /* Register a deformable sphere with radius 1.0 and a large resolution hint so
   that it is discretized as an octahedron. Give it an arbitrary initial pose.
  */
  DeformableBodyId deformable_id = RegisterSphere(100, X_WA);
  ASSERT_EQ(deformable_model_ptr_->GetFemModel(deformable_id).num_nodes(), 7);
  const RigidBody<double>& rigid_body =
      plant_->AddRigidBody("box", SpatialInertia<double>());
  geometry::Box box(1.0, 1.0, 1.0);
  const RigidTransformd X_BA(Vector3d(-2, 0, 0));
  const RigidTransformd X_BG(Vector3d(-1, 0, 0));
  const MultibodyConstraintId constraint_id =
      deformable_model_ptr_->AddFixedConstraint(deformable_id, rigid_body, X_BA,
                                                box, X_BG);

  EXPECT_TRUE(deformable_model_ptr_->HasConstraint(deformable_id));
  EXPECT_EQ(deformable_model_ptr_->fixed_constraint_ids(deformable_id).size(),
            1);
  const DeformableRigidFixedConstraintSpec& spec =
      deformable_model_ptr_->fixed_constraint_spec(constraint_id);
  EXPECT_EQ(spec.body_A, deformable_id);
  EXPECT_EQ(spec.body_B, rigid_body.index());
  /* Only the right-most deformable body vertex (with world position (1, 0, 0))
   is under constraint. */
  ASSERT_EQ(spec.vertices.size(), 1);
  const int vertex_index = spec.vertices[0];
  const Vector3d p_WPi =
      deformable_model_ptr_->GetReferencePositions(deformable_id)
          .segment<3>(3 * vertex_index);
  const Vector3d p_APi(1, 0, 0);
  EXPECT_EQ(p_WPi, X_WA * p_APi);
  /* Qi should be coincident with Pi. */
  ASSERT_EQ(spec.p_BQs.size(), 1);
  const Vector3d p_BQi = spec.p_BQs[0];
  EXPECT_EQ(X_BA * p_APi, p_BQi);

  /* Throw conditions */
  /* Non-existant deformable body. */
  DeformableBodyId fake_deformable_id = DeformableBodyId::get_new_id();
  EXPECT_THROW(deformable_model_ptr_->AddFixedConstraint(
                   fake_deformable_id, rigid_body, X_BA, box, X_BG),
               std::exception);
  /* Non-existant rigid body (registered with a different MbP). */
  MultibodyPlant<double> other_plant(0.0);
  const RigidBody<double>& wrong_rigid_body =
      other_plant.AddRigidBody("wrong body", SpatialInertia<double>());
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->AddFixedConstraint(deformable_id, wrong_rigid_body,
                                                X_BA, box, X_BG),
      ".*rigid body.*not registered.*");
  /* Unsupported shape. */
  const geometry::Mesh mesh("fake_mesh.vtk");
  EXPECT_THROW(deformable_model_ptr_->AddFixedConstraint(
                   deformable_id, rigid_body, X_BA, mesh, X_BG),
               std::exception);
  /* No constraint is added . */
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->AddFixedConstraint(
                                  deformable_id, rigid_body, X_BA, box,
                                  RigidTransformd(Vector3d(100, 100, 100))),
                              "No constraint has been added.*box.*");
  /* Adding constraint after finalize. */
  plant_->Finalize();
  EXPECT_THROW(deformable_model_ptr_->AddFixedConstraint(
                   deformable_id, rigid_body, X_BA, box, X_BG),
               std::exception);
}

TEST_F(DeformableModelTest, ExternalForces) {
  /* A user defined force density field. */
  class ConstantForceDensityField final : public ForceDensityField<double> {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstantForceDensityField)

    /* Constructs an force density field that has the functional form given by
     input `field` which is then scaled by a scalar value via input port. */
    explicit ConstantForceDensityField(
        std::function<Vector3<double>(const Vector3<double>&)> field)
        : field_(std::move(field)) {}

    /* Gets the double-valued input port that determines whether the force field
     is turned on or off. 0 for off and anything else for on.  */
    const systems::InputPort<double>& get_input_port() const {
      return parent_system_or_throw().get_input_port(scale_port_index_);
    }

   private:
    Vector3<double> DoEvaluateAt(const systems::Context<double>& context,
                                 const Vector3<double>& p_WQ) const final {
      return get_input_port().Eval(context)(0) * field_(p_WQ);
    };

    std::unique_ptr<ForceDensityField<double>> DoClone() const final {
      return std::make_unique<ConstantForceDensityField>(*this);
    }

    void DoDeclareInputPorts(multibody::MultibodyPlant<double>* plant) final {
      scale_port_index_ = this->DeclareVectorInputPort(
                                  plant, "on/off signal for the force field",
                                  BasicVector<double>(1.0))
                              .get_index();
    }

    std::function<Vector3<double>(const Vector3<double>&)> field_;
    systems::InputPortIndex scale_port_index_;
  };

  auto force_field = [](const Vector3d& x) {
    return 3.14 * x;
  };
  auto constant_force =
      std::make_unique<ConstantForceDensityField>(force_field);
  const ConstantForceDensityField* constant_force_ptr = constant_force.get();
  deformable_model_ptr_->AddExternalForce(std::move(constant_force));
  constexpr double kRezHint = 0.5;
  DeformableBodyId body_id = RegisterSphere(kRezHint);
  /* Pre-finalize calls to GetExternalForces are not allowed. */
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetExternalForces(body_id),
                              ".*before system resources.*declared.*");
  /* We modify the gravity to deviate from the default to verify that it is
   reflected in the external gravity applied to the deformable body. */
  const Vector3d gravity_vector(0.0, 0.0, -10.0);
  plant_->mutable_gravity_field().set_gravity_vector(gravity_vector);
  plant_->Finalize();
  auto plant_context = plant_->CreateDefaultContext();

  /* Post-finalize calls to AddExternalForce are not allowed. */
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->AddExternalForce(
          std::make_unique<ConstantForceDensityField>(force_field)),
      ".*AddExternalForce.*after system resources have been declared.*");

  DeformableBodyId fake_id = DeformableBodyId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetExternalForces(fake_id),
      fmt::format(".*No.*id.*{}.*registered.*", fake_id));

  const std::vector<const ForceDensityField<double>*>& external_forces =
      deformable_model_ptr_->GetExternalForces(body_id);
  for (const auto* force : external_forces) {
    const auto* g = dynamic_cast<const GravityForceField<double>*>(force);
    const auto* f = dynamic_cast<const ConstantForceDensityField*>(force);
    /* We know two external forces are added to the deformable body, one is
     gravity (added automatically), the other is the explicit force defined
     above. */
    ASSERT_TRUE((g != nullptr) ^ (f != nullptr));
    const Vector3d p_WQ(1, 2, 3);
    if (g != nullptr) {
      EXPECT_EQ(g->density_type(), ForceDensityType::kPerReferenceVolume);
      EXPECT_EQ(force->EvaluateAt(*plant_context, p_WQ),
                gravity_vector * default_body_config_.mass_density());
    } else {
      EXPECT_EQ(f->density_type(), ForceDensityType::kPerCurrentVolume);
      const double scale = 2.71;
      constant_force_ptr->get_input_port().FixValue(plant_context.get(),
                                                    Vector1d(scale));
      EXPECT_EQ(force->EvaluateAt(*plant_context, p_WQ), scale * 3.14 * p_WQ);
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
