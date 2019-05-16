#include "drake/multibody/plant/multibody_plant.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/surface_mesh.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::MeshFieldLinear;
using geometry::SceneGraph;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceFace;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using multibody::Parser;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

namespace multibody {

// A friend class to MultibodyPlant.
class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  static VectorX<double> CalcGeneralizedTractionAtPoint(
    const MultibodyPlant<double>& plant,
    const Context<double>& context,
    GeometryId geometryM_id, GeometryId geometryN_id,
    const ContactSurface<double>& surface,
    SurfaceFaceIndex face_index,
    const SurfaceMesh<double>::Barycentric& r_barycentric,
    double dissipation, double mu_coulomb) {
        return plant.CalcGeneralizedTractionAtPoint(
            context, geometryM_id, geometryN_id, surface, face_index,
            r_barycentric, dissipation, mu_coulomb);
    }

  static GeometryId FindBoxGeometry(const MultibodyPlant<double>& plant) {
    for (BodyIndex i(0); i < plant.num_bodies(); ++i) {
      if (plant.get_body(i).name() == "box") {
        DRAKE_DEMAND(plant.collision_geometries_[i].size() == 1);
        return plant.collision_geometries_[i].front();
      }
    }

    throw std::runtime_error("Box geometry unexpectedly not found.");
  }

  static GeometryId FindGroundGeometry(const MultibodyPlant<double>& plant) {
    for (BodyIndex i(0); i < plant.num_bodies(); ++i) {
      if (plant.get_body(i).name() == "ground") {
        DRAKE_DEMAND(plant.collision_geometries_[i].size() == 1);
        return plant.collision_geometries_[i].front();
      }
    }

    throw std::runtime_error("Ground geometry unexpectedly not found.");
  }
};

class MultibodyPlantHydroelasticTractionTests : public ::testing::Test {
 public:
  const MultibodyPlant<double>& plant() const { return *plant_; }
  const Context<double>& plant_context() const { return *plant_context_; }
  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }

 private:
  void SetUp() override {
    // Read the two bodies into the plant.
    DiagramBuilder<double> builder;
    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    scene_graph.set_name("scene_graph");
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/plant/test/block_on_halfspace.sdf");
    plant_ = builder.AddSystem<MultibodyPlant>();
    MultibodyPlant<double>& plant = *plant_;
    Parser(&plant, &scene_graph).AddModelFromFile(full_name);

    // Weld the ground frame.
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"));

    // Add gravity to the model.
    plant.AddForceElement<UniformGravityFieldElement>();

    plant.Finalize();

    DRAKE_DEMAND(plant.num_velocities() == 6);
    DRAKE_DEMAND(plant.num_positions() == 7);

    // Connect MBP and SceneGraph.
    DRAKE_DEMAND(!!plant.get_source_id());
    builder.Connect(scene_graph.get_query_output_port(),
                    plant.get_geometry_query_input_port());
    builder.Connect(
        plant.get_geometry_poses_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id().value()));

    diagram_ = builder.Build();

    // Create a context for this system.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(plant, context_.get());

    contact_surface_ = CreateContactSurface();

    // Set the pose for the box.
    auto& state_vector = plant_context_->get_mutable_continuous_state_vector();
    state_vector[6] = 1.0;
  }

  std::unique_ptr<ContactSurface<double>> CreateContactSurface() const {
    // Get the geometry IDs for the ground halfspace and the block.
    GeometryId ground_geom = MultibodyPlantTester::FindGroundGeometry(*plant_);
    GeometryId block_geom = MultibodyPlantTester::FindBoxGeometry(*plant_);

    // Create the surface mesh first.
    auto mesh = CreateSurfaceMesh();

    // Create the "e" field values using negated "z" values.
    std::vector<double> e_MN(mesh->num_vertices());
    for (SurfaceVertexIndex i(0); i < mesh->num_vertices(); ++i)
      e_MN[i] = -mesh->vertex(i).r_MV()[2];

    // Create the gradient of the "h" field, pointing toward what will be
    // geometry "M" (the halfspace).
    std::vector<Vector3<double>> h_MN_M(mesh->num_vertices(),
        Vector3<double>(0, 0, -1));

    return std::make_unique<ContactSurface<double>>(
      ground_geom, block_geom, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh.get()),
      std::make_unique<MeshFieldLinear<Vector3<double>, SurfaceMesh<double>>>(
          "h_MN_M", std::move(h_MN_M), mesh.get()));
  }

  // Creates a surface mesh that covers the "wetted surface", i.e., the part of
  // the box that would be wet if the halfspace were a fluid. This yields an
  // open box with five faces comprising ten triangles.
  std::unique_ptr<SurfaceMesh<double>> CreateSurfaceMesh() const {
    std::vector<SurfaceVertex<double>> vertices;
    std::vector<SurfaceFace> faces;

    // Create the vertices.
    vertices.emplace_back(Vector3<double>(0.5, 0.5, 0));
    vertices.emplace_back(Vector3<double>(-0.5, 0.5, 0));
    vertices.emplace_back(Vector3<double>(-0.5, -0.5, 0));
    vertices.emplace_back(Vector3<double>(0.5, -0.5, 0));
    vertices.emplace_back(Vector3<double>(0.5, 0.5, -0.5));
    vertices.emplace_back(Vector3<double>(-0.5, 0.5, -0.5));
    vertices.emplace_back(Vector3<double>(-0.5, -0.5, -0.5));
    vertices.emplace_back(Vector3<double>(0.5, -0.5, -0.5));

    // Create the -z face.
    faces.emplace_back(
        SurfaceVertexIndex(4), SurfaceVertexIndex(5), SurfaceVertexIndex(6));
    faces.emplace_back(
        SurfaceVertexIndex(6), SurfaceVertexIndex(7), SurfaceVertexIndex(4));

    // Create the +x face.
    faces.emplace_back(
        SurfaceVertexIndex(0), SurfaceVertexIndex(3), SurfaceVertexIndex(7));
    faces.emplace_back(
        SurfaceVertexIndex(7), SurfaceVertexIndex(4), SurfaceVertexIndex(0));

    // Create the -x face.
    faces.emplace_back(
        SurfaceVertexIndex(1), SurfaceVertexIndex(2), SurfaceVertexIndex(6));
    faces.emplace_back(
        SurfaceVertexIndex(6), SurfaceVertexIndex(5), SurfaceVertexIndex(1));

    // Create the +y face.
    faces.emplace_back(
        SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(5));
    faces.emplace_back(
        SurfaceVertexIndex(5), SurfaceVertexIndex(4), SurfaceVertexIndex(0));

    // Create the -y face.
    faces.emplace_back(
        SurfaceVertexIndex(2), SurfaceVertexIndex(3), SurfaceVertexIndex(7));
    faces.emplace_back(
        SurfaceVertexIndex(7), SurfaceVertexIndex(6), SurfaceVertexIndex(2));

    return std::make_unique<SurfaceMesh<double>>(
        std::move(faces), std::move(vertices));
  }

  MultibodyPlant<double>* plant_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  systems::Context<double>* plant_context_;
  std::unique_ptr<ContactSurface<double>> contact_surface_;
};

// Tests the traction calculation without any frictional or dissipation forces.
TEST_F(MultibodyPlantHydroelasticTractionTests, VanillaTraction) {
  const double dissipation = 0.0;
  const double mu_coulomb = 0.0;

  // Compute traction vectors at the two corners of the box.
  VectorX<double> t1 = MultibodyPlantTester::CalcGeneralizedTractionAtPoint(
      plant(), plant_context(),
      MultibodyPlantTester::FindGroundGeometry(plant()),
      MultibodyPlantTester::FindBoxGeometry(plant()),
      contact_surface(), SurfaceFaceIndex(0),
      SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
      dissipation, mu_coulomb);
  VectorX<double> t2 = MultibodyPlantTester::CalcGeneralizedTractionAtPoint(
      plant(), plant_context(),
      MultibodyPlantTester::FindGroundGeometry(plant()),
      MultibodyPlantTester::FindBoxGeometry(plant()),
      contact_surface(), SurfaceFaceIndex(1),
      SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0),
      dissipation, mu_coulomb);

  // Sum the tractions together.
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  VectorX<double> traction = t1 + t2;
  std::cout << traction << std::endl;
  EXPECT_NEAR(traction.segment(0, 5).norm(), 0.0, eps);
  EXPECT_NEAR(traction[5], 1.0, eps);
}

}  // namespace multibody
}  // namespace drake
