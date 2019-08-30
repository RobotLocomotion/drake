#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::MeshFieldLinear;
using geometry::PenetrationAsPointPair;
using geometry::SceneGraph;
using geometry::SurfaceFace;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using math::RigidTransform;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::acrobot::AcrobotParameters;
// TODO(edrumwri) Remove the next line and the corresponding #include when
// hydroelastic contact results are factored out into test utilities.
using multibody::internal::HydroelasticTractionCalculator;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

namespace multibody {
namespace {

// Confirm that an empty multibody plant produces an empty lcm message.
GTEST_TEST(ContactResultsToLcmSystem, EmptyMultibodyPlant) {
  MultibodyPlant<double> plant;
  plant.Finalize();
  ContactResultsToLcmSystem<double> lcm_system(plant);
  auto lcm_context = lcm_system.AllocateContext();
  lcm_context->FixInputPort(
      lcm_system.get_contact_result_input_port().get_index(),
      Value<ContactResults<double>>());

  Value<lcmt_contact_results_for_viz> lcm_message_value;
  lcm_system.get_lcm_message_output_port().Calc(*lcm_context,
                                                &lcm_message_value);

  const lcmt_contact_results_for_viz& lcm_message =
      lcm_message_value.get_value();

  // We haven't stepped, so we should assume the time is the context's default
  // value.
  EXPECT_EQ(lcm_message.timestamp, 0);
  EXPECT_EQ(lcm_message.num_point_pair_contacts, 0);
  EXPECT_EQ(lcm_message.num_hydroelastic_contacts, 0);
}

// Common case: confirm that the reported contacts map to the right lcm message.
// In this test, we're using a MBP that doesn't actually have collision
// geometry, but simulates collision results by reporting that two bodies are
// colliding. That is enough to test the ContactResultsToLcmSystem.
GTEST_TEST(ContactResultsToLcmSystem, NonEmptyMultibodyPlantEmptyContact) {
  using std::to_string;
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, true /* finalize */);

  ContactResultsToLcmSystem<double> lcm_system(*plant);

  // Create ContactResults with single reported contact.
  ContactResults<double> contacts;
  // NOTE: The values in penetration_data are irrelevant except for nhat_BA_W.
  // So, only that value is set; all other values are left as default.
  PenetrationAsPointPair<double> penetration_data;
  penetration_data.nhat_BA_W << 1, 2, 3;
  const auto& link1 = plant->GetBodyByName(parameters.link1_name());
  const BodyIndex index1 = link1.index();
  const ModelInstanceIndex model_instance = link1.model_instance();
  const std::string name1 =
      parameters.link1_name() + "(" + to_string(model_instance) + ")";
  const BodyIndex index2 =
      plant->GetBodyByName(parameters.link2_name()).index();
  // Assume that link1 and link2 belong to the same model instance.
  const std::string name2 =
      parameters.link2_name() + "(" + to_string(model_instance) + ")";
  const Vector3<double> f_BC_W = Vector3<double>{1, 2, 3};
  const Vector3<double> p_WC = Vector3<double>{-1, -2, -2};
  const double separation_speed = 0.25;
  const double slip_speed = 0.5;
  PointPairContactInfo<double> pair_info{
      index1,           index2,     f_BC_W,          p_WC,
      separation_speed, slip_speed, penetration_data};
  contacts.AddContactInfo(pair_info);
  Value<ContactResults<double>> contacts_value(contacts);
  auto lcm_context = lcm_system.AllocateContext();
  lcm_context->FixInputPort(
      lcm_system.get_contact_result_input_port().get_index(),
      Value<ContactResults<double>>(contacts));

  Value<lcmt_contact_results_for_viz> lcm_message_value;
  lcm_system.get_lcm_message_output_port().Calc(*lcm_context,
                                                &lcm_message_value);
  const lcmt_contact_results_for_viz& lcm_message =
      lcm_message_value.get_value();

  // We haven't stepped, so we should assume the time is the context's default
  // value.
  EXPECT_EQ(lcm_message.timestamp, 0);
  ASSERT_EQ(lcm_message.num_point_pair_contacts, 1);
  EXPECT_EQ(lcm_message.num_hydroelastic_contacts, 0);
  const lcmt_point_pair_contact_info_for_viz& info_msg =
      lcm_message.point_pair_contact_info[0];
  EXPECT_EQ(info_msg.timestamp, 0);
  EXPECT_EQ(info_msg.body1_name, name1);
  EXPECT_EQ(info_msg.body2_name, name2);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(info_msg.contact_point), p_WC, 0,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Vector3<double>(info_msg.contact_force), f_BC_W,
                              0, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Vector3<double>(info_msg.normal),
                              penetration_data.nhat_BA_W, 0,
                              MatrixCompareType::absolute));
}

// Confirm that the system can be transmogrified to other supported scalars.
GTEST_TEST(ContactResultsToLcmSystem, Transmogrify) {
  MultibodyPlant<double> plant;
  plant.Finalize();
  ContactResultsToLcmSystem<double> lcm_system(plant);

  ContactResultsToLcmSystem<AutoDiffXd> lcm_system_ad(lcm_system);
}

GTEST_TEST(ConnectContactResultsToDrakeVisualizer, BasicTest) {
  systems::DiagramBuilder<double> builder;

  // Make a trivial plant with at least one body.
  auto plant = builder.AddSystem<MultibodyPlant>();
  plant->AddRigidBody("link", SpatialInertia<double>());
  plant->Finalize();

  auto publisher = ConnectContactResultsToDrakeVisualizer(&builder, *plant);

  // Confirm that we get a non-null result.
  EXPECT_NE(publisher, nullptr);

  // Check that the publishing event was set as documented.
  auto periodic_events = publisher->GetPeriodicEvents();
  EXPECT_EQ(periodic_events.size(), 1);
  EXPECT_EQ(periodic_events.begin()->first.period_sec(), 1/60.0);
}

GTEST_TEST(ConnectContactResultsToDrakeVisualizer, NestedDiagramTest) {
  systems::DiagramBuilder<double> builder;

  // Make a trivial plant with at least one body.
  MultibodyPlant<double>* plant;
  SceneGraph<double>* scene_graph;
  std::tie(plant, scene_graph) = AddMultibodyPlantSceneGraph(&builder);
  plant->AddRigidBody("link", SpatialInertia<double>());
  plant->Finalize();

  builder.ExportOutput(plant->get_contact_results_output_port(),
      "contact_results");

  auto diagram = builder.AddSystem(builder.Build());

  auto publisher = ConnectContactResultsToDrakeVisualizer(
      &builder, *plant, diagram->GetOutputPort("contact_results"));

  // Confirm that we get a non-null result.
  EXPECT_NE(publisher, nullptr);

  // Check that the publishing event was set as documented.
  auto periodic_events = publisher->GetPeriodicEvents();
  EXPECT_EQ(periodic_events.size(), 1);
  EXPECT_EQ(periodic_events.begin()->first.period_sec(), 1/60.0);
}

// TODO(edrumwri) Refactor these ContactSurface helper functions to be more
// broadly available to unit tests that depend on ContactSurfaces.

// Creates a surface mesh.
std::unique_ptr<SurfaceMesh<double>> CreateSurfaceMesh() {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;

  // Create the vertices, all of which are offset vectors defined in the
  // halfspace body frame.
  vertices.emplace_back(Vector3<double>(0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<double>(-0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<double>(-0.5, -0.5, -0.5));
  vertices.emplace_back(Vector3<double>(0.5, -0.5, -0.5));

  // Create the face comprising two triangles.
  faces.emplace_back(
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2));
  faces.emplace_back(
      SurfaceVertexIndex(2), SurfaceVertexIndex(3), SurfaceVertexIndex(0));

  return std::make_unique<SurfaceMesh<double>>(
      std::move(faces), std::move(vertices));
}

// Creates a contact surface between the two given geometries.
std::unique_ptr<ContactSurface<double>> CreateContactSurface(
    GeometryId halfspace_id, GeometryId block_id) {
  // Create the surface mesh first.
  auto mesh = CreateSurfaceMesh();

  // Create the "e" field values (i.e., "hydroelastic pressure") using
  // negated "z" values.
  std::vector<double> e_MN(mesh->num_vertices());
  for (SurfaceVertexIndex i(0); i < mesh->num_vertices(); ++i)
    e_MN[i] = -mesh->vertex(i).r_MV()[2];

  // Create the gradient of the "h" field, pointing toward what will be
  // geometry "M" (the halfspace).
  std::vector<Vector3<double>> h_MN_W(mesh->num_vertices(),
      Vector3<double>(0, 0, -1));

  SurfaceMesh<double>* mesh_pointer = mesh.get();
  return std::make_unique<ContactSurface<double>>(
      halfspace_id, block_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh_pointer),
      std::make_unique<MeshFieldLinear<Vector3<double>, SurfaceMesh<double>>>(
          "h_MN_W", std::move(h_MN_W), mesh_pointer));
}

ContactResults<double> GenerateHydroelasticContactResults(
    const MultibodyPlant<double>& plant,
    std::unique_ptr<ContactSurface<double>>* contact_surface) {
  // Get the geometries for the two bodies.
  DRAKE_DEMAND(plant.num_bodies() == 2);
  const Body<double>& world_body = plant.world_body();
  const Body<double>& block_body = plant.GetBodyByName("BodyB");
  const std::vector<geometry::GeometryId>& world_geoms =
      plant.GetCollisionGeometriesForBody(world_body);
  const std::vector<geometry::GeometryId>& block_geoms =
      plant.GetCollisionGeometriesForBody(block_body);
  DRAKE_DEMAND(world_geoms.size() == 1);
  DRAKE_DEMAND(block_geoms.size() == 1);

  // Create the contact surface using an arbitrary geometry from each body.
  *contact_surface =
      CreateContactSurface(world_geoms.front(), block_geoms.front());

  // Create the calculator data populated with arbitrary dummy values since we
  // only care that the values get transformed correctly (and their physical
  // correctness is irrelevant).
  const RigidTransform<double> X_WA = RigidTransform<double>::Identity();
  const RigidTransform<double> X_WB = RigidTransform<double>::Identity();
  const SpatialVelocity<double> V_WA = SpatialVelocity<double>::Zero();
  const SpatialVelocity<double> V_WB = SpatialVelocity<double>::Zero();
  HydroelasticTractionCalculator<double>::Data data(
      X_WA, X_WB, V_WA, V_WB, contact_surface->get());

  // Material properties are also dummies (the test will be unaffected by
  // their settings).
  const double dissipation = 0.0;
  const double mu_coulomb = 0.0;

  // Create the HydroelasticContactInfo.
  HydroelasticTractionCalculator<double> calculator;
  ContactResults<double> output;
  output.AddContactInfo(
      HydroelasticContactInfo<double>(
          calculator.ComputeContactInfo(data, dissipation, mu_coulomb)));
  return output;
}

double square(double x) { return x * x; }

// Computes the distance between two triangles, which we define as the largest
// norm between two corresponding vertices. The first triangle is given
// by the vertices p_WA, p_WB, and p_WC. The second triangle is specified using
// the array `vertices_W` as well as the permutation array that describes how
// `vertices` should be indexed.
double ComputeDistanceBetweenTriangles(
    const double p_WA[3], const double p_WB[3], const double p_WC[3],
    const std::array<Vector3<double>, 3>& vertices_W,
    const std::array<int, 3>& vertex_vector_permutation) {
  double sq_dist_A = 0.0, sq_dist_B = 0.0, sq_dist_C = 0.0;

  for (int j = 0; j < 3; ++j) {  // Loop over the three dimensions.
    sq_dist_A += square(p_WA[j] - vertices_W[vertex_vector_permutation[0]][j]);
    sq_dist_B += square(p_WB[j] - vertices_W[vertex_vector_permutation[1]][j]);
    sq_dist_C += square(p_WC[j] - vertices_W[vertex_vector_permutation[2]][j]);
  }

  return std::sqrt(std::max(sq_dist_A, std::max(sq_dist_B, sq_dist_C)));
}

// Searches for a permutation of the vertices of each triangle T in `mesh_W`
// such that a T is found to be equivalent to the triangle (p_WA, p_WB, p_WC).
void ValidateCloseToMeshTriangle(const double p_WA[3], const double p_WB[3],
                                 const double p_WC[3],
                                 const SurfaceMesh<double>& mesh_W,
                                 double tol) {
  double closest_distance = std::numeric_limits<double>::max();

  for (geometry::SurfaceFaceIndex i(0); i < mesh_W.num_faces(); ++i) {
    // Get the triangle as three vertices measured and expressed in the world
    // frame.
    const std::array<Vector3<double>, 3> vertices_W = {
        mesh_W.vertex(mesh_W.element(i).vertex(0)).r_MV(),
        mesh_W.vertex(mesh_W.element(i).vertex(1)).r_MV(),
        mesh_W.vertex(mesh_W.element(i).vertex(2)).r_MV()};

    // Set a vector of indices into the vector of vertices.
    std::array<int, 3> indices = { 0, 1, 2 };

    // Compute the distance using the various permutations.
    do {
      double distance = ComputeDistanceBetweenTriangles(p_WA, p_WB, p_WC,
                                                        vertices_W, indices);
      closest_distance = std::min(distance, closest_distance);
      if (closest_distance < tol)
        break;
    } while (std::next_permutation(indices.begin(), indices.end()));
  }

  EXPECT_LT(closest_distance, tol);
}

// TODO(edrumwri) Remove the function below when hydroelastic plugins are all in
// the codebase.
// This is not a test, just some code that exists to visualize the contact
// results while DrakeVisualizer plugins are still in development.
#if 0
GTEST_TEST(ContactResultsToLcmTest, HydroelasticContactResultsVisualization) {
  DiagramBuilder<double> builder;

  // Note: the plant will never be connected in the Diagram. It is used only to
  // set up the ContactResultsToLcmSystem.
  MultibodyPlant<double>* plant;
  geometry::SceneGraph<double>* scene_graph;
  std::tie(plant, scene_graph) = AddMultibodyPlantSceneGraph(&builder);

  // We need some geometries for this test. Parameters below are selected
  // arbitrarily and do not affect this test.
  const double gravity = 0.0;
  const double plane_angle = 0.0;
  const CoulombFriction<double> mu_plane;
  const CoulombFriction<double> mu_block;
  const double block_mass = 1.0;
  const Vector3<double> block_dim(1.0, 1.0, 1.0);
  benchmarks::inclined_plane::AddInclinedPlaneWithBlockToPlant(
      gravity, plane_angle, {} /* default plane "dimensions" */,
      mu_plane, mu_block, block_mass, block_dim, false /* no spheres */,
      plant);
  plant->Finalize();

  // Connect the plant to the visualizer so that there is some context to
  // visualize the contact surface.
  lcm::DrakeLcm lcm;
  geometry::ConnectDrakeVisualizer(&builder, *scene_graph, &lcm);

  const auto& contact_results_to_lcm_system =
      *builder.AddSystem<ContactResultsToLcmSystem<double>>(*plant);

  // TODO(edrumwri) Replace this code block when MultibodyPlant outputs
  // hydroelastic contact results: at that point, replace this code block
  // with a call to ConnectContactResultsToDrakeVisualizer().
  // Hook a publisher to the contact results system.
  auto& contact_results_publisher = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm, 1.0 / 60 /* publish period */));
  contact_results_publisher.set_name("contact_results_publisher");
  builder.Connect(contact_results_to_lcm_system.get_output_port(0),
                  contact_results_publisher.get_input_port());
  const systems::InputPortIndex contact_results_input_port_index =
      builder.ExportInput(
          contact_results_to_lcm_system.get_contact_result_input_port());
  const systems::OutputPortIndex
      lcm_hydroelastic_contact_surface_output_port_index = builder.ExportOutput(
          contact_results_to_lcm_system.get_lcm_message_output_port());

  // Finish constructing the diagram; note that we use the default pose for
  // the box, which will make the bottom of the box's surface lie at z=-0.5.
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();

  std::unique_ptr<ContactSurface<double>> contact_surface;
  diagram_context->FixInputPort(
      contact_results_input_port_index,
      Value<ContactResults<double>>(
          GenerateHydroelasticContactResults(*plant, &contact_surface)));

  // Publish the first message so that this test can be interpreted in the
  // visualizer.
  geometry::DispatchLoadMessage(*scene_graph, &lcm);
  diagram->Publish(*diagram_context);

  // Get the LCM message that corresponds to the contact results.
  Value<lcmt_contact_results_for_viz> lcm_message_value;
  diagram->get_output_port(
      lcm_hydroelastic_contact_surface_output_port_index).Calc(
          *diagram_context, &lcm_message_value);
  const lcmt_contact_results_for_viz& lcm_message =
      lcm_message_value.get_value();
}
#endif

// Verifies that the LCM message is consistent with the hydroelastic contact
// surface that we create.
GTEST_TEST(ContactResultsToLcmTest, HydroelasticContactResults) {
  // Note: the plant will never be connected in the Diagram. It is used only to
  // set up the ContactResultsToLcmSystem. SceneGraph is necessary for
  // AddInclinedPlaneWithBlockToPlant() to work properly.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>* plant;
  geometry::SceneGraph<double>* scene_graph;
  std::tie(plant, scene_graph) = AddMultibodyPlantSceneGraph(&builder);

  // We need some geometries for this test. Parameters below are selected
  // arbitrarily and do not affect this test.
  const double gravity = 0.0;
  const double plane_angle = 0.0;
  const CoulombFriction<double> mu_plane;
  const CoulombFriction<double> mu_block;
  const double block_mass = 1.0;
  const Vector3<double> block_dim(1.0, 1.0, 1.0);
  benchmarks::inclined_plane::AddInclinedPlaneWithBlockToPlant(
      gravity, plane_angle, {} /* default plane "dimensions" */,
      mu_plane, mu_block, block_mass, block_dim, false /* no spheres */,
      plant);
  plant->Finalize();

  ContactResultsToLcmSystem<double> contact_results_to_lcm_system(*plant);

  const systems::InputPortIndex contact_results_input_port_index =
      contact_results_to_lcm_system.get_contact_result_input_port().get_index();
  const systems::OutputPortIndex
      lcm_hydroelastic_contact_surface_output_port_index =
          contact_results_to_lcm_system.get_lcm_message_output_port()
              .get_index();

  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<Context<double>> context =
      contact_results_to_lcm_system.CreateDefaultContext();
  context->FixInputPort(
      contact_results_input_port_index,
      Value<ContactResults<double>>(
          GenerateHydroelasticContactResults(*plant, &contact_surface)));

  // Get the LCM message that corresponds to the contact results.
  Value<lcmt_contact_results_for_viz> lcm_message_value;
  contact_results_to_lcm_system.get_output_port(
      lcm_hydroelastic_contact_surface_output_port_index).Calc(
          *context, &lcm_message_value);
  const lcmt_contact_results_for_viz& lcm_message =
      lcm_message_value.get_value();

  EXPECT_EQ(lcm_message.num_point_pair_contacts, 0);
  ASSERT_EQ(lcm_message.num_hydroelastic_contacts, 1);
  const lcmt_hydroelastic_contact_surface_for_viz& surface_msg =
      lcm_message.hydroelastic_contacts[0];
  EXPECT_EQ(surface_msg.body1_name, "WorldBody");
  EXPECT_EQ(surface_msg.body2_name, "BodyB");

  // Verify that the total number of triangles match.
  ASSERT_EQ(surface_msg.num_triangles, contact_surface->mesh_W().num_faces());

  // Verify that the contact surface has the expected vertex locations.
  for (int i = 0; i < surface_msg.num_triangles; ++i) {
    for (int j = 0; j < 3; ++j) {
      ValidateCloseToMeshTriangle(
          surface_msg.triangles[i].p_WA, surface_msg.triangles[i].p_WB,
          surface_msg.triangles[i].p_WC, contact_surface->mesh_W(),
          10 * std::numeric_limits<double>::epsilon());
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
