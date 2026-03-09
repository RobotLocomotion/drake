#include "drake/multibody/plant/deformable_model.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/force_density_field.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::VectorBlock;
using Eigen::VectorXd;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::Sphere;
using math::RigidTransform;
using math::RigidTransformd;
using std::make_unique;
using systems::BasicVector;

class DeformableModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    MultibodyPlantConfig plant_config;
    plant_config.time_step = 0.01;
    plant_config.discrete_contact_approximation = "sap";
    std::tie(plant_, scene_graph_) = AddMultibodyPlant(plant_config, &builder_);
    deformable_model_ptr_ = &plant_->mutable_deformable_model();
  }

  systems::DiagramBuilder<double> builder_;
  const fem::DeformableBodyConfig<double> default_body_config_{};
  DeformableModel<double>* deformable_model_ptr_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};

  /* Registers a deformable sphere with the given initial pose in world. */
  DeformableBodyId RegisterSphere(double resolution_hint,
                                  RigidTransformd X_WS = RigidTransformd()) {
    return RegisterSphere(deformable_model_ptr_, resolution_hint, X_WS);
  }

  template <typename T>
  DeformableBodyId RegisterSphere(
      DeformableModel<T>* model, double resolution_hint,
      RigidTransformd X_WS = RigidTransformd(),
      ModelInstanceIndex model_instance = default_model_instance()) {
    auto geometry =
        make_unique<GeometryInstance>(X_WS, make_unique<Sphere>(1), "sphere");
    geometry::ProximityProperties deformable_proximity_props;
    geometry->set_proximity_properties(deformable_proximity_props);
    DeformableBodyId body_id = model->RegisterDeformableBody(
        std::move(geometry), model_instance, fem::DeformableBodyConfig<T>{},
        resolution_hint);
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

  /* Registering a deformable body within a continuous plant throws. */
  MultibodyPlant<double> continuous_plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      continuous_plant.mutable_deformable_model().RegisterDeformableBody(
          make_unique<GeometryInstance>(RigidTransformd(),
                                        make_unique<Sphere>(1), "sphere"),
          default_body_config_, kRezHint),
      ".*RegisterDeformableBody.*continuous MultibodyPlant.*not allowed.*");
}

/* Tests that deformable models can be added with all supported element
 subdivision counts. */
TEST_F(DeformableModelTest, RegisterWithSubdivision) {
  for (int subd = 0; subd <= 4; subd++) {
    Sphere sphere(1.0);
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd(), make_unique<Sphere>(sphere),
        fmt::format("sphere_subd_{}", subd));
    constexpr double kRezHint = 0.5;

    /* Copy default config with desired element subdivision count. */
    fem::DeformableBodyConfig<double> body_config_{default_body_config_};
    body_config_.set_element_subdivision_count(subd);
    DeformableBodyId body_id = deformable_model_ptr_->RegisterDeformableBody(
        std::move(geometry), body_config_, kRezHint);

    /* Verify that a corresponding FemModel has been built. */
    EXPECT_NO_THROW(deformable_model_ptr_->GetFemModel(body_id));
  }

  plant_->Finalize();
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
      plant_->get_deformable_body_configuration_output_port().Allocate();
  /* Compute the configuration for each geometry in the model. */
  plant_->get_deformable_body_configuration_output_port().Calc(
      *context, output_value.get());
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
      plant_->AddRigidBody("box", SpatialInertia<double>::NaN());
  geometry::Box box(1.0, 1.0, 1.0);
  const RigidTransformd X_BA(Vector3d(-2, 0, 0));
  const RigidTransformd X_BG(Vector3d(-1, 0, 0));
  deformable_model_ptr_->AddFixedConstraint(deformable_id, rigid_body, X_BA,
                                            box, X_BG);
  const DeformableBody<double>& body =
      deformable_model_ptr_->GetBody(deformable_id);
  ASSERT_EQ(body.body_id(), deformable_id);
  EXPECT_TRUE(deformable_model_ptr_->HasConstraint(deformable_id));
  EXPECT_TRUE(body.has_fixed_constraint());
  ASSERT_EQ(body.fixed_constraint_specs().size(), 1);
  const DeformableRigidFixedConstraintSpec& spec =
      body.fixed_constraint_specs()[0];

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
  EXPECT_TRUE(CompareMatrices(X_BA * p_APi, p_BQi,
                              4.0 * std::numeric_limits<double>::epsilon()));

  /* Throw conditions */
  /* Non-existant deformable body. */
  DeformableBodyId fake_deformable_id = DeformableBodyId::get_new_id();
  EXPECT_THROW(deformable_model_ptr_->AddFixedConstraint(
                   fake_deformable_id, rigid_body, X_BA, box, X_BG),
               std::exception);
  /* Non-existant rigid body (registered with a different MbP). */
  MultibodyPlant<double> other_plant(0.0);
  const RigidBody<double>& wrong_rigid_body =
      other_plant.AddRigidBody("wrong body", SpatialInertia<double>::NaN());
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
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->AddFixedConstraint(
          deformable_id, rigid_body, X_BA, box,
          RigidTransformd(Vector3d(100, 100, 100))),
      "AddFixedConstraint.*No constraint has been added.*box.*");
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
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstantForceDensityField);

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

    std::unique_ptr<ForceDensityFieldBase<double>> DoClone() const final {
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

  const std::vector<const ForceDensityFieldBase<double>*>& external_forces =
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

  /* Registering an external force within a continuous plant throws. */
  MultibodyPlant<double> continuous_plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      continuous_plant.mutable_deformable_model().AddExternalForce(
          std::make_unique<ConstantForceDensityField>(force_field)),
      ".*AddExternalForce.*continuous MultibodyPlant.*not allowed.*");
}

TEST_F(DeformableModelTest, CloneBeforeFinalizeThrows) {
  MultibodyPlant<double> double_plant(0.01);
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->CloneToScalar<double>(&double_plant),
      ".*owning plant.*must be finalized.*");
  plant_->Finalize();
  EXPECT_NO_THROW(deformable_model_ptr_->CloneToScalar<double>(&double_plant));
}

/* Test that cloning to other scalar types is allowed when the model is empty.
 */
TEST_F(DeformableModelTest, EmptyClone) {
  /* The plants for the cloned models. */
  MultibodyPlant<double> double_plant(0.01);
  MultibodyPlant<AutoDiffXd> autodiff_plant(0.01);
  MultibodyPlant<symbolic::Expression> symbolic_plant(0.01);
  /* Plant owning the cloned from models need to be finalized. */
  plant_->Finalize();
  EXPECT_TRUE(deformable_model_ptr_->is_empty());

  EXPECT_TRUE(deformable_model_ptr_->is_cloneable_to_double());
  EXPECT_TRUE(deformable_model_ptr_->is_cloneable_to_autodiff());
  EXPECT_TRUE(deformable_model_ptr_->is_cloneable_to_symbolic());

  /* double -> double */
  EXPECT_NO_THROW(deformable_model_ptr_->CloneToScalar<double>(&double_plant));
  /* double -> autodiff */
  EXPECT_NO_THROW(
      deformable_model_ptr_->CloneToScalar<AutoDiffXd>(&autodiff_plant));
  /* double -> symbolic */
  EXPECT_NO_THROW(deformable_model_ptr_->CloneToScalar<symbolic::Expression>(
      &symbolic_plant));
}

/* Test that, for a non-empty model, only cloning to double is allowed. */
TEST_F(DeformableModelTest, NonEmptyClone) {
  /* The plants for the cloned models. */
  MultibodyPlant<double> double_plant(0.01);
  MultibodyPlant<AutoDiffXd> autodiff_plant(0.01);
  MultibodyPlant<symbolic::Expression> symbolic_plant(0.01);
  /* Make the model non-empty. */
  DeformableBodyId body_id = RegisterSphere(0.5);
  const RigidBody<double>& rigid_body =
      plant_->AddRigidBody("box", SpatialInertia<double>::NaN());
  geometry::Box box(1.0, 1.0, 1.0);
  const RigidTransformd X_BA(Vector3d(-2, 0, 0));
  const RigidTransformd X_BG(Vector3d(-1, 0, 0));
  deformable_model_ptr_->AddFixedConstraint(body_id, rigid_body, X_BA, box,
                                            X_BG);

  EXPECT_FALSE(deformable_model_ptr_->is_empty());
  /* Plant owning the cloned from models need to be finalized. */
  plant_->Finalize();

  EXPECT_TRUE(deformable_model_ptr_->is_cloneable_to_double());
  EXPECT_FALSE(deformable_model_ptr_->is_cloneable_to_autodiff());
  EXPECT_FALSE(deformable_model_ptr_->is_cloneable_to_symbolic());

  std::unique_ptr<PhysicalModel<double>> double_clone;
  /* double -> double */
  EXPECT_NO_THROW(double_clone = deformable_model_ptr_->CloneToScalar<double>(
                      &double_plant));

  /* Now verify the double clone is indeed a copy of the original. */
  const DeformableModel<double>* double_clone_ptr =
      dynamic_cast<const DeformableModel<double>*>(double_clone.get());
  ASSERT_NE(double_clone_ptr, nullptr);
  EXPECT_EQ(double_clone_ptr->num_bodies(),
            deformable_model_ptr_->num_bodies());
  /* We don't compare the result of GetDiscreteStateIndex and GetExternalForces
   since they are set during DeclareSystemResources(). */
  EXPECT_EQ(double_clone_ptr->GetFemModel(body_id).num_dofs(),
            deformable_model_ptr_->GetFemModel(body_id).num_dofs());
  EXPECT_EQ(double_clone_ptr->GetReferencePositions(body_id),
            deformable_model_ptr_->GetReferencePositions(body_id));
  /* We don't compare the result of any function that involves
   DeformableBodyIndex because it's set during DeclareSystemResources(). */
  const geometry::GeometryId geometry_id =
      deformable_model_ptr_->GetGeometryId(body_id);
  EXPECT_EQ(double_clone_ptr->GetGeometryId(body_id),
            deformable_model_ptr_->GetGeometryId(body_id));
  EXPECT_EQ(double_clone_ptr->GetBodyId(geometry_id),
            deformable_model_ptr_->GetBodyId(geometry_id));
  EXPECT_EQ(double_clone_ptr->HasConstraint(body_id),
            deformable_model_ptr_->HasConstraint(body_id));
}

// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/* An empty DeformableModel doesn't get in the way of a TAMSI plant. */
TEST_F(DeformableModelTest, EmptyDeformableModelWorksWithTamsi) {
  plant_->set_discrete_contact_approximation(
      multibody::DiscreteContactApproximation::kTamsi);
  EXPECT_TRUE(deformable_model_ptr_->is_empty());
  EXPECT_NO_THROW(plant_->Finalize());
}
#pragma GCC diagnostic push

/* If a DeformableModel is not empty, we require the owning plant to use the SAP
 solver. */
TEST_F(DeformableModelTest, NonEmptyDeformableModelOnlyWorksWithSap) {
  plant_->set_discrete_contact_approximation(
      multibody::DiscreteContactApproximation::kTamsi);
  RegisterSphere(0.5);
  EXPECT_FALSE(deformable_model_ptr_->is_empty());
  DRAKE_EXPECT_THROWS_MESSAGE(plant_->Finalize(),
                              ".*DeformableModel is only supported by.*SAP.*");
}

TEST_F(DeformableModelTest, RegistrationNotAllowedForNonDoubleModel) {
  MultibodyPlant<AutoDiffXd> autodiff_plant(0.01);
  DeformableModel<AutoDiffXd> autodiff_deformable_model(&autodiff_plant);
  DRAKE_EXPECT_THROWS_MESSAGE(
      RegisterSphere(&autodiff_deformable_model, 0.5),
      ".*RegisterDeformableBody.*T != double.*not allowed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      autodiff_deformable_model.AddExternalForce(
          std::make_unique<GravityForceField<AutoDiffXd>>(Vector3d(0, 0, -10),
                                                          1.0)),
      ".*AddExternalForce.*T != double.*not allowed.*");
}

TEST_F(DeformableModelTest, EnableDisable) {
  auto model_id = RegisterSphere(0.5);

  plant_->Finalize();
  const int num_dofs = deformable_model_ptr_->GetFemModel(model_id).num_dofs();
  auto diagram = builder_.Build();

  systems::DiscreteStateIndex state_index =
      deformable_model_ptr_->GetDiscreteStateIndex(model_id);
  auto context = diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(context.get());

  /* Assign arbitrary position, velocity, and acceleration values. */
  VectorX<double> initial_discrete_state =
      plant_context.get_discrete_state(state_index).get_value();
  initial_discrete_state.head(num_dofs) =
      3.14 * initial_discrete_state.head(num_dofs);
  initial_discrete_state.tail(2 * num_dofs) =
      VectorXd::LinSpaced(2 * num_dofs, 0.0, 1.0);
  plant_context.SetDiscreteState(state_index, initial_discrete_state);
  const VectorX<double> q0 = initial_discrete_state.head(num_dofs);

  /* Verify properties of disabled body. */
  {
    deformable_model_ptr_->Disable(model_id, &plant_context);
    VectorX<double> discrete_state =
        plant_context.get_discrete_state(state_index).get_value();

    EXPECT_FALSE(deformable_model_ptr_->is_enabled(model_id, plant_context));
    /* Verify that the position values are unchanged upon disabling. */
    EXPECT_EQ(discrete_state.head(num_dofs), q0);
    /* Verify that the velocity and acceleration values are set to zero upon
    disabling. */
    EXPECT_EQ(discrete_state.tail(2 * num_dofs),
              VectorX<double>::Zero(2 * num_dofs));
    diagram->ExecuteForcedEvents(context.get());
    const VectorXd& disabled_next_state =
        plant_context.get_discrete_state(state_index).value();
    /* The position, velocity, and acceleration persist for the next time step.
     */
    EXPECT_EQ(disabled_next_state.head(num_dofs), q0);
    EXPECT_EQ(disabled_next_state.tail(2 * num_dofs),
              VectorX<double>::Zero(2 * num_dofs));
  }

  /* Verify properties of re-enabled body. */
  {
    deformable_model_ptr_->Enable(model_id, &plant_context);
    VectorX<double> discrete_state =
        plant_context.get_discrete_state(state_index).get_value();

    EXPECT_TRUE(deformable_model_ptr_->is_enabled(model_id, plant_context));
    /* Verify that the position values are unchanged after enabling. */
    EXPECT_EQ(discrete_state.head(num_dofs), q0);
    /* Verify that the velocity and acceleration values remain zero after
    enabling. */
    EXPECT_EQ(discrete_state.tail(2 * num_dofs),
              VectorX<double>::Zero(2 * num_dofs));
    /* The position, velocity, and acceleration for the next step. */
    diagram->ExecuteForcedEvents(context.get());
    const VectorXd& enabled_next_state =
        plant_context.get_discrete_state(state_index).value();
    EXPECT_FALSE(CompareMatrices(enabled_next_state.head(num_dofs), q0, 1e-4));
    EXPECT_FALSE(CompareMatrices(enabled_next_state.tail(2 * num_dofs),
                                 VectorXd::Zero(2 * num_dofs), 1e-2));
  }
}

TEST_F(DeformableModelTest, GetAndSetPositions) {
  auto model_id = RegisterSphere(0.5);

  plant_->Finalize();
  const int num_dofs = deformable_model_ptr_->GetFemModel(model_id).num_dofs();
  auto diagram = builder_.Build();

  auto context = diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(context.get());

  /* Get the intial positions q0. */
  systems::DiscreteStateIndex state_index =
      deformable_model_ptr_->GetDiscreteStateIndex(model_id);
  VectorX<double> initial_discrete_state =
      plant_context.get_discrete_state(state_index).get_value();
  const VectorX<double> q0 = initial_discrete_state.head(num_dofs);
  const Matrix3X<double> q0_matrix =
      Eigen::Map<const Matrix3X<double>>(q0.data(), 3, num_dofs / 3);

  EXPECT_EQ(plant_->deformable_model().GetPositions(plant_context, model_id),
            q0_matrix);
  const Matrix3X<double> q1_matrix = 2.0 * q0_matrix;
  plant_->deformable_model().SetPositions(&plant_context, model_id, q1_matrix);
  EXPECT_EQ(plant_->deformable_model().GetPositions(plant_context, model_id),
            q1_matrix);
}

TEST_F(DeformableModelTest, GetAndSetVelocities) {
  auto model_id = RegisterSphere(0.5);

  plant_->Finalize();
  const int num_dofs = deformable_model_ptr_->GetFemModel(model_id).num_dofs();
  auto diagram = builder_.Build();

  auto context = diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(context.get());

  /* Get the initial velocities v0. */
  systems::DiscreteStateIndex state_index =
      deformable_model_ptr_->GetDiscreteStateIndex(model_id);
  VectorX<double> initial_discrete_state =
      plant_context.get_discrete_state(state_index).get_value();
  const VectorX<double> v0 = initial_discrete_state.segment(num_dofs, num_dofs);
  const Matrix3X<double> v0_matrix =
      Eigen::Map<const Matrix3X<double>>(v0.data(), 3, num_dofs / 3);

  EXPECT_EQ(plant_->deformable_model().GetVelocities(plant_context, model_id),
            v0_matrix);
  const Matrix3X<double> v1_matrix = Matrix3X<double>::Ones(3, num_dofs / 3);
  plant_->deformable_model().SetVelocities(&plant_context, model_id, v1_matrix);
  EXPECT_EQ(plant_->deformable_model().GetVelocities(plant_context, model_id),
            v1_matrix);
}

TEST_F(DeformableModelTest, GetAndSetPositionsAndVelocities) {
  auto model_id = RegisterSphere(0.5);

  plant_->Finalize();
  const int num_dofs = deformable_model_ptr_->GetFemModel(model_id).num_dofs();
  auto diagram = builder_.Build();

  auto context = diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(context.get());

  const int num_nodes = num_dofs / 3;
  Matrix3X<double> q_matrix(3, num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    q_matrix.col(i) = Vector3d(0.1 * i, 0.2 * i, 0.3 * i);
  }
  Matrix3X<double> v_matrix(3, num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    v_matrix.col(i) = Vector3d(0.4 * i, 0.5 * i, 0.6 * i);
  }
  plant_->deformable_model().SetPositionsAndVelocities(&plant_context, model_id,
                                                       q_matrix, v_matrix);
  const Matrix3X<double> qv_matrix =
      plant_->deformable_model().GetPositionsAndVelocities(plant_context,
                                                           model_id);
  EXPECT_EQ(qv_matrix.cols(), 2 * num_nodes);
  EXPECT_TRUE(CompareMatrices(qv_matrix.leftCols(num_nodes), q_matrix));
  EXPECT_TRUE(CompareMatrices(qv_matrix.rightCols(num_nodes), v_matrix));
}

/* Test the many throw conditions of GetPositions and SetPositions. */
TEST_F(DeformableModelTest, GetAndSetPositionsThrowConditions) {
  auto model_id = RegisterSphere(0.5);
  plant_->Finalize();
  auto diagram = builder_.Build();
  auto context = diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      plant_->GetMyMutableContextFromRoot(context.get());
  const int num_dofs = deformable_model_ptr_->GetFemModel(model_id).num_dofs();

  /* Wrong context. */
  EXPECT_THROW(deformable_model_ptr_->GetPositions(*context, model_id),
               std::exception);
  EXPECT_THROW(
      deformable_model_ptr_->SetPositions(
          context.get(), model_id, Matrix3X<double>::Zero(3, num_dofs / 3)),
      std::exception);

  /* Wrong id. */
  DeformableBodyId fake_id = DeformableBodyId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->GetPositions(plant_context, fake_id),
      fmt::format(".*No.*id.*{}.*registered.*", fake_id));
  DRAKE_EXPECT_THROWS_MESSAGE(
      deformable_model_ptr_->SetPositions(
          &plant_context, fake_id, Matrix3X<double>::Zero(3, num_dofs / 3)),
      fmt::format(".*No.*id.*{}.*registered.*", fake_id));

  /* Wrong size. */
  EXPECT_THROW(
      deformable_model_ptr_->SetPositions(
          &plant_context, model_id, Matrix3X<double>::Zero(3, num_dofs / 2)),
      std::exception);

  /* Non-finite values. */
  EXPECT_THROW(
      deformable_model_ptr_->SetPositions(
          &plant_context, model_id,
          Matrix3X<double>::Constant(3, num_dofs / 3,
                                     std::numeric_limits<double>::infinity())),
      std::exception);
}

TEST_F(DeformableModelTest, Parallelism) {
  EXPECT_EQ(deformable_model_ptr_->parallelism().num_threads(), 1);
  DeformableBodyId body_id = RegisterSphere(1.0);
  EXPECT_EQ(
      deformable_model_ptr_->GetFemModel(body_id).parallelism().num_threads(),
      1);

  Parallelism parallelism(2);
  EXPECT_EQ(parallelism.num_threads(), 2);
  deformable_model_ptr_->SetParallelism(parallelism);
  EXPECT_EQ(deformable_model_ptr_->parallelism().num_threads(), 2);
  EXPECT_EQ(
      deformable_model_ptr_->GetFemModel(body_id).parallelism().num_threads(),
      2);
}

/* Tests getting a deformable body by name. */
TEST_F(DeformableModelTest, BodyName) {
  const DeformableBodyId body_id = RegisterSphere(0.5);
  EXPECT_TRUE(deformable_model_ptr_->HasBodyNamed("sphere"));
  EXPECT_EQ(deformable_model_ptr_->GetBodyByName("sphere").body_id(), body_id);
  EXPECT_FALSE(deformable_model_ptr_->HasBodyNamed("nonexistent_body_name"));
}

/* Tests registering deformable bodies into a prescribed model instance as well
 as getting deformable bodies by model instance index. */
TEST_F(DeformableModelTest, ModelInstance) {
  const double kRezHint = 0.5;
  ModelInstanceIndex invalid_model_instance(42);
  DRAKE_EXPECT_THROWS_MESSAGE(
      RegisterSphere(deformable_model_ptr_, kRezHint, RigidTransformd{},
                     invalid_model_instance),
      ".*Invalid model instance.*");
  const ModelInstanceIndex model_instance =
      plant_->AddModelInstance("test_instance");
  const DeformableBodyId body_id = RegisterSphere(
      deformable_model_ptr_, kRezHint, RigidTransformd{}, model_instance);
  EXPECT_EQ(deformable_model_ptr_->num_bodies(), 1);
  const std::vector<DeformableBodyId> bodies_in_model_instance =
      deformable_model_ptr_->GetBodyIds(model_instance);
  ASSERT_EQ(bodies_in_model_instance.size(), 1);
  EXPECT_EQ(bodies_in_model_instance[0], body_id);
}

TEST_F(DeformableModelTest, DuplicatedNames) {
  const double kRezHint = 0.5;
  const ModelInstanceIndex instance0 = plant_->AddModelInstance("instance0");
  const ModelInstanceIndex instance1 = plant_->AddModelInstance("instance1");
  const DeformableBodyId body0_id = RegisterSphere(
      deformable_model_ptr_, kRezHint, RigidTransformd{}, instance0);
  /* Registering a body with duplicated name within the same model instance is a
   throw. */
  DRAKE_EXPECT_THROWS_MESSAGE(RegisterSphere(deformable_model_ptr_, kRezHint,
                                             RigidTransformd{}, instance0),
                              ".*instance0.*already contains.*sphere.*");
  /* Getting body by name without specifying model instance is fine as long as
   the name is unique within DeformableModel. */
  const DeformableBody<double>& body0 =
      deformable_model_ptr_->GetBodyByName("sphere");
  EXPECT_EQ(body0.body_id(), body0_id);

  /* Registering a body with duplicated name in a different model instance is
   fine. */
  const DeformableBodyId body1_id = RegisterSphere(
      deformable_model_ptr_, kRezHint, RigidTransformd{}, instance1);
  EXPECT_EQ(deformable_model_ptr_->num_bodies(), 2);
  /* If a body name is shared across bodies in different model instances,
   specifying just the name is ambiguous, and thus we should throw. */
  DRAKE_EXPECT_THROWS_MESSAGE(deformable_model_ptr_->GetBodyByName("sphere"),
                              ".*The name sphere is not unique.*");
  /* We can get body by name after specifying the model instance. */
  const DeformableBody<double>& body1 =
      deformable_model_ptr_->GetBodyByName("sphere", instance1);
  EXPECT_EQ(body1.body_id(), body1_id);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
