#include <gtest/gtest.h>

#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

using Eigen::Vector3d;

namespace drake {

using geometry::ContactSurface;
using geometry::MeshField;
using geometry::SurfaceFace;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using geometry::SurfaceMesh;

namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static std::vector<geometry::ContactSurface<double>> ComputeContactSurfaces(
      const MultibodyPlant<double>& plant,
      const geometry::QueryObject<double>& query_object) {
    return plant.hydroelastics_engine_.ComputeContactSurfaces(query_object);
  }
};

namespace {
class HydroelasticContactResultsOutputTester : public ::testing::Test {
 protected:
  void SetUp() {
    const double radius = 1.0;  // sphere radius (m).

    // The vertical location of the sphere. If this value is smaller than the
    // sphere radius, the sphere will intersect the half-space described by
    // z <= 0.
    const double z0 = 0.95 * radius;

    // Set some reasonable, but arbitrary, parameters: none of these will be
    // affect the test results.
    const double mass = 2.0;                           // kg.
    const double elastic_modulus = 1e7;                // Pascals.
    const double dissipation = 1.0;                    // s/m.
    const CoulombFriction<double> friction(1.0, 1.0);  // Static/dynamic.
    const Vector3<double> gravity_W(0, 0, -9.8);       // m/s^2.

    // Create the plant.
    systems::DiagramBuilder<double> builder;
    geometry::SceneGraph<double>& scene_graph =
        *builder.AddSystem<geometry::SceneGraph<double>>();
    scene_graph.set_name("scene_graph");
    plant_ = builder.AddSystem(
        examples::multibody::bouncing_ball::MakeBouncingBallPlant(
            radius, mass, elastic_modulus, dissipation, friction, gravity_W,
            &scene_graph));
    plant_->set_contact_model(ContactModel::kHydroelasticsOnly);
    plant_->Finalize();

    // Sanity check on the availability of the optional source id before using
    // it.
    DRAKE_DEMAND(!!plant_->get_source_id());

    builder.Connect(scene_graph.get_query_output_port(),
                    plant_->get_geometry_query_input_port());
    builder.Connect(
        plant_->get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(plant_->get_source_id().value()));

    diagram_ = builder.Build();

    // Create a context for this system:
    diagram_context_ = diagram_->CreateDefaultContext();
    diagram_->SetDefaultContext(diagram_context_.get());
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

    // Set the sphere's initial pose.
    math::RigidTransformd X_WB(Vector3d(0.0, 0.0, z0));
    plant_->SetFreeBodyPose(plant_context_, plant_->GetBodyByName("Ball"),
                            X_WB);
  }

  const HydroelasticContactInfo<double>& contact_results() const {
    // Get the contact results from the plant.
    const ContactResults<double>& contact_results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            *plant_context_);
    DRAKE_DEMAND(contact_results.num_hydroelastic_contacts() == 1);
    return contact_results.hydroelastic_contact_info(0);
  }

  // Checks to see whether two SurfaceMesh objects are equal (all data match
  // to bit-wise precision).
  static bool Equal(const SurfaceMesh<double>& mesh1,
                    const SurfaceMesh<double>& mesh2) {
    if (mesh1.num_faces() != mesh2.num_faces()) return false;
    if (mesh1.num_vertices() != mesh2.num_vertices()) return false;

    // Check face indices.
    for (SurfaceFaceIndex i(0); i < mesh1.num_faces(); ++i) {
      const SurfaceFace& face1 = mesh1.element(i);
      const SurfaceFace& face2 = mesh2.element(i);
      for (int j = 0; j < 3; ++j)
        if (face1.vertex(j) != face2.vertex(j)) return false;
    }

    // Check vertices.
    for (SurfaceVertexIndex i(0); i < mesh1.num_vertices(); ++i) {
      if (mesh1.vertex(i).r_MV() != mesh2.vertex(i).r_MV()) return false;
    }

    // All checks passed.
    return true;
  }

  // Checks to see whether two MeshField objects are equal (all data
  // match to bit-wise precision).
  // Note: currently requires the objects to be of type MeshFieldLinear.
  template <typename T>
  static bool Equal(const MeshField<T, SurfaceMesh<double>>& field1,
                    const MeshField<T, SurfaceMesh<double>>& field2) {
    // If the objects are not of type MeshFieldLinear then the simple checking
    // of equal values at vertices is insufficient.
    const geometry::MeshFieldLinear<T, SurfaceMesh<double>>* linear_field1 =
        dynamic_cast<const geometry::MeshFieldLinear<T, SurfaceMesh<double>>*>(
            &field1);
    const geometry::MeshFieldLinear<T, SurfaceMesh<double>>* linear_field2 =
        dynamic_cast<const geometry::MeshFieldLinear<T, SurfaceMesh<double>>*>(
            &field2);
    DRAKE_DEMAND(linear_field1);
    DRAKE_DEMAND(linear_field2);

    if (field1.mesh().num_faces() != field2.mesh().num_faces()) return false;
    if (field1.mesh().num_vertices() != field2.mesh().num_vertices())
      return false;

    for (SurfaceVertexIndex i(0); i < field1.mesh().num_vertices(); ++i) {
      if (field1.EvaluateAtVertex(i) != field2.EvaluateAtVertex(i))
        return false;
    }

    // All checks passed.
    return true;
  }

  // Checks to see whether two ContactSurface objects are equal (all
  // data match to bit-wise precision).
  static bool Equal(const ContactSurface<double>& surface1,
                    const ContactSurface<double>& surface2) {
    // First check the meshes.
    if (!Equal(surface1.mesh_W(), surface2.mesh_W())) return false;

    // Now examine the pressure field.
    if (!Equal<double>(surface1.e_MN(), surface2.e_MN())) return false;

    // Now examine the grad_h field.
    if (!Equal<Vector3d>(surface1.grad_h_MN_W(), surface2.grad_h_MN_W()))
      return false;

    // All checks passed.
    return true;
  }

  MultibodyPlant<double>* plant_{};
  systems::Context<double>* plant_context_{};

 private:
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> diagram_context_{};
};

TEST_F(HydroelasticContactResultsOutputTester, Equivalent) {
  // Get the query object so that we can compute the contact surfaces.
  const auto& query_object =
      plant_->get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<double>>(*plant_context_);

  // Compute the contact surface using the hydroelastic engine.
  std::vector<geometry::ContactSurface<double>> contact_surfaces =
      MultibodyPlantTester::ComputeContactSurfaces(*plant_, query_object);

  // Check that the two contact surfaces are equivalent.
  ASSERT_EQ(contact_surfaces.size(), 1);
  EXPECT_TRUE(
      Equal(contact_results().contact_surface(), contact_surfaces.front()));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
