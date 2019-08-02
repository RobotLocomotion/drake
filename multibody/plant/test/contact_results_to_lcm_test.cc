#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::MeshFieldLinear;
using geometry::PenetrationAsPointPair;
using geometry::SurfaceFace;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using math::RigidTransform;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::acrobot::AcrobotParameters;
// TODO(edrumwri) Remove this code when MultibodyPlant outputs hydroelastic
// contact results.  
using multibody::internal::HydroelasticTractionCalculator;
using systems::Context;

namespace multibody {
namespace {

// Confirm that an empty multibody plant produces an empty lcm message.
GTEST_TEST(ContactResultToLcmSystem, EmptyMultibodyPlant) {
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
  EXPECT_EQ(lcm_message.num_hydroelastic_contact_surfaces, 0);
}

// Common case: confirm that the reported contacts map to the right lcm message.
// In this test, we're using a MBP that doesn't actually have collision
// geometry, but simulating collision results by reporting that two bodies are
// colliding. That is enough to test the ContactResultsToLcmSystem.
GTEST_TEST(ContactResultToLcmSystem, NonEmptyMultibodyPlantEmptyContact) {
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
  contacts.AddPointPairContactInfo(pair_info);
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
GTEST_TEST(ContactResultToLcmSystem, Transmogrify) {
  MultibodyPlant<double> plant;
  plant.Finalize();
  ContactResultsToLcmSystem<double> lcm_system(plant);

  ContactResultsToLcmSystem<AutoDiffXd> lcm_system_ad(lcm_system);
}

GTEST_TEST(ConnectContactResultsToDrakeVisualizer, BasicTest) {
  systems::DiagramBuilder<double> builder;

  // Make a trivial plant with at least one body and a discrete time step.
  auto plant = builder.AddSystem<MultibodyPlant>(0.001);
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
  systems::DiagramBuilder<double> interior_builder;

  // Make a trivial plant with at least one body and a discrete time step.
  auto plant = interior_builder.AddSystem<MultibodyPlant>(0.001);
  plant->AddRigidBody("link", SpatialInertia<double>());
  plant->Finalize();

  interior_builder.ExportOutput(plant->get_contact_results_output_port(),
      "contact_results");

  systems::DiagramBuilder<double> builder;
  auto interior_diagram = builder.AddSystem(interior_builder.Build());

  auto publisher = ConnectContactResultsToDrakeVisualizer(
      &builder, *plant, interior_diagram->GetOutputPort("contact_results"));

  // Confirm that we get a non-null result.
  EXPECT_NE(publisher, nullptr);

  // Check that the publishing event was set as documented.
  auto periodic_events = publisher->GetPeriodicEvents();
  EXPECT_EQ(periodic_events.size(), 1);
  EXPECT_EQ(periodic_events.begin()->first.period_sec(), 1/60.0);
}

// TODO(edrumwri) Remove this code (which is a duplicate of that in
// hydroelastic_traction_test.cc) when MultibodyPlant outputs hydroelastic
// contact results.  
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

// TODO(edrumwri) Remove this code (which is a duplicate of that in
// hydroelastic_traction_test.cc) when MultibodyPlant outputs hydroelastic
// contact results.  
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
  std::vector<Vector3<double>> h_MN_M(mesh->num_vertices(),
      Vector3<double>(0, 0, -1));

  SurfaceMesh<double>* mesh_pointer = mesh.get();
  return std::make_unique<ContactSurface<double>>(
      halfspace_id, block_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh_pointer),
      std::make_unique<MeshFieldLinear<Vector3<double>, SurfaceMesh<double>>>(
          "h_MN_M", std::move(h_MN_M), mesh_pointer),
      RigidTransform<double>::Identity());
}

// A class for outputting a ContactResults structure with non-empty
// hydroelastic contact surface.
class DummyOutputter : public systems::LeafSystem<double> {
 public:
  DummyOutputter() {
    this->DeclareAbstractOutputPort("contact_results", ContactResults<double>(),
        &DummyOutputter::CopyContactResultsOutput);
  }

 private:
  void CopyContactResultsOutput(
      const Context<double>& context,
      ContactResults<double>* output) const {
    // Create the contact surface using a duplicated arbitrary ID: geometry IDs
    // are irrelevant for this test.
    GeometryId arbitrary_id = GeometryId::get_new_id();
    std::unique_ptr<ContactSurface<double>> contact_surface =
        CreateContactSurface(arbitrary_id, arbitrary_id);

    // Create the calculator data, populated with dummy values since we're only
    // testing that the structure can be created.
    const RigidTransform<double> X_WA = RigidTransform<double>::Identity();
    const RigidTransform<double> X_WB = RigidTransform<double>::Identity();
    const RigidTransform<double> X_WM = RigidTransform<double>::Identity();
    const SpatialVelocity<double> V_WA = SpatialVelocity<double>::Zero();
    const SpatialVelocity<double> V_WB = SpatialVelocity<double>::Zero();
    HydroelasticTractionCalculator<double>::Data data(
        X_WA, X_WB, V_WA, V_WB, X_WM, contact_surface.get());

    // Material properties are also dummies (the test will be unaffected by
    // their settings).
    const double dissipation = 0.0;
    const double mu_coulomb = 0.0;

    // Create the HydroelasticContactInfo and validate the contact surface
    // pointer is correct. Correctness of field values is validated elsewhere in
    // this file.
    HydroelasticTractionCalculator<double> calculator;
    output->AddHydroelasticContactInfo(
        std::make_unique<HydroelasticContactInfo<double>>(
            calculator.ComputeContactInfo(data, dissipation, mu_coulomb)));
  } 
};

// TODO(edrumwri) Replace this with a test that uses MultibodyPlant when
// MultibodyPlant outputs hydroelastic contact results.
class HydroelasticContactResults : public ::testing::Test {
 private:
  void SetUp() override {
    
  }
};

}  // namespace
}  // namespace multibody
}  // namespace drake
