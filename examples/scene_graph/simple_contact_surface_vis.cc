/** @file
 A simple binary for exercising and visualizing computation of ContactSurfaces.
 This is decoupled from dynamics so that just the geometric components can be
 evaluated in as light-weight a fashion as possible.

 This can serve as a test bed for evaluating the various cases of the
 ContactSurface-computing algorithms. Simply swap the geometry types (moving
 and anchored) and their properties to see the effect on contact surface.  */

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/value.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace examples {
namespace scene_graph {
namespace contact_surface {

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::Box;
using geometry::ConnectDrakeVisualizer;
using geometry::ContactSurface;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::AddRigidHydroelasticProperties;
using geometry::AddSoftHydroelasticProperties;
using geometry::IllustrationProperties;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using geometry::SurfaceMesh;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceVertex;
using lcm::DrakeLcm;
using math::RigidTransformd;
using std::make_unique;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::DiagramBuilder;
using systems::lcm::LcmPublisherSystem;
using systems::LeafSystem;
using systems::Simulator;

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_bool(real_time, true, "Set to false to run as fast as possible");
DEFINE_double(length, 1.0,
              "Measure of sphere edge length -- smaller numbers produce a "
              "denser, more expensive mesh");

/** Places a ball at the world's origin and defines its velocity as being
 sinusoidal in time in the z direction.

 @system{MovingBall,, @output_port{geometry_pose} }
 */
class MovingBall final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MovingBall)

  explicit MovingBall(SceneGraph<double>* scene_graph) {
    this->DeclareContinuousState(2);

    // Add geometry for a ball that moves based on sinusoidal derivatives.
    source_id_ = scene_graph->RegisterSource("moving_ball");
    frame_id_ =
        scene_graph->RegisterFrame(source_id_, GeometryFrame("moving_frame"));
    geometry_id_ = scene_graph->RegisterGeometry(
        source_id_, frame_id_,
        make_unique<GeometryInstance>(RigidTransformd(),
                                      make_unique<Sphere>(1.0), "ball"));

    ProximityProperties prox_props;
    prox_props.AddProperty(geometry::kMaterialGroup, geometry::kElastic, 1e8);
    AddSoftHydroelasticProperties(FLAGS_length, &prox_props);
    scene_graph->AssignRole(source_id_, geometry_id_, prox_props);

    IllustrationProperties illus_props;
    illus_props.AddProperty("phong", "diffuse", Vector4d(0.8, 0.1, 0.1, 0.25));
    scene_graph->AssignRole(source_id_, geometry_id_, illus_props);

    geometry_pose_port_ =
        this->DeclareAbstractOutputPort("geometry_pose",
                                        &MovingBall::CalcFramePoseOutput)
            .get_index();
  }

  SourceId source_id() const { return source_id_; }

  const systems::OutputPort<double>& get_geometry_pose_output_port() const {
    return systems::System<double>::get_output_port(geometry_pose_port_);
  }

 private:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    BasicVector<double>& derivative_values =
        dynamic_cast<BasicVector<double>&>(derivatives->get_mutable_vector());
    derivative_values.SetAtIndex(0, std::sin(context.get_time()));
  }

  void CalcFramePoseOutput(
      const Context<double>& context, FramePoseVector<double>* poses) const {
    RigidTransformd pose;
    const double pos_z = context.get_continuous_state().get_vector()[0];
    pose.set_translation({0.0, 0.0, pos_z});
    *poses = {{frame_id_, pose}};
  }

  SourceId source_id_;
  FrameId frame_id_;
  GeometryId geometry_id_;

  int geometry_pose_port_{-1};
};

/** A system that evaluates contact surfaces from SceneGraph and outputs a fake
 ContactResults with the actual contact surfaces.

 @system{ContactResultMaker,
   @intput_port{query_object},
   @output_port{contact_result}
 }
 */
class ContactResultMaker final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultMaker)

  ContactResultMaker() {
    geometry_query_input_port_ =
        this->DeclareAbstractInputPort("query_object",
                                       Value<QueryObject<double>>())
            .get_index();
    contact_result_output_port_ =
        this->DeclareAbstractOutputPort("contact_result",
                                        &ContactResultMaker::CalcContactResults)
            .get_index();
  }

  const systems::InputPort<double>& get_geometry_query_port() const {
    return systems::System<double>::get_input_port(geometry_query_input_port_);
  }

 private:
  void CalcContactResults(const Context<double>& context,
                          lcmt_contact_results_for_viz* results) const {
    const auto& query_object =
        get_geometry_query_port().Eval<QueryObject<double>>(context);
    std::vector<ContactSurface<double>> contacts =
        query_object.ComputeContactSurfaces();
    const int num_contacts = static_cast<int>(contacts.size());

    auto& msg = *results;
    msg.timestamp = context.get_time() * 1e6;  // express in microseconds.
    msg.num_point_pair_contacts = 0;
    msg.point_pair_contact_info.resize(msg.num_point_pair_contacts);
    msg.num_hydroelastic_contacts = num_contacts;
    msg.hydroelastic_contacts.resize(num_contacts);

    auto write_double3 = [](const Vector3d& src, double* dest) {
      dest[0] = src(0);
      dest[1] = src(1);
      dest[2] = src(2);
    };

    for (int i = 0; i < num_contacts; ++i) {
      lcmt_hydroelastic_contact_surface_for_viz& surface_msg =
          msg.hydroelastic_contacts[i];

      surface_msg.body1_name = "Id_" + to_string(contacts[i].id_M());
      surface_msg.body2_name = "Id_" + to_string(contacts[i].id_N());

      const SurfaceMesh<double>& mesh_W = contacts[i].mesh_W();
      surface_msg.num_triangles = mesh_W.num_faces();
      surface_msg.triangles.resize(surface_msg.num_triangles);

      // Loop through each contact triangle on the contact surface.
      for (SurfaceFaceIndex j(0); j < surface_msg.num_triangles; ++j) {
        lcmt_hydroelastic_contact_surface_tri_for_viz& tri_msg =
            surface_msg.triangles[j];

        // Get the three vertices.
        const auto& face = mesh_W.element(j);
        const SurfaceVertex<double>& vA = mesh_W.vertex(face.vertex(0));
        const SurfaceVertex<double>& vB = mesh_W.vertex(face.vertex(1));
        const SurfaceVertex<double>& vC = mesh_W.vertex(face.vertex(2));

        write_double3(vA.r_MV(), tri_msg.p_WA);
        write_double3(vB.r_MV(), tri_msg.p_WB);
        write_double3(vC.r_MV(), tri_msg.p_WC);

        tri_msg.pressure_A = contacts[i].EvaluateE_MN(face.vertex(0));
        tri_msg.pressure_B = contacts[i].EvaluateE_MN(face.vertex(1));
        tri_msg.pressure_C = contacts[i].EvaluateE_MN(face.vertex(2));
      }
    }
  }

  int geometry_query_input_port_;
  int contact_result_output_port_;
};

int do_main() {
  DiagramBuilder<double> builder;

  auto& scene_graph = *builder.AddSystem<SceneGraph<double>>();

  // Add the bouncing ball.
  auto& moving_ball = *builder.AddSystem<MovingBall>(&scene_graph);
  builder.Connect(moving_ball.get_geometry_pose_output_port(),
                  scene_graph.get_source_pose_port(moving_ball.source_id()));

  // Add a large box, such that intersection occurs at the edge.
  SourceId source_id = scene_graph.RegisterSource("world");
  const double edge_len = 10;
  const RigidTransformd X_WB(Eigen::AngleAxisd(M_PI / 4, Vector3d::UnitX()),
      Vector3d{0, 0, -sqrt(2.0) * edge_len / 2});
  GeometryId ground_id = scene_graph.RegisterAnchoredGeometry(
      source_id,
      make_unique<GeometryInstance>(
          X_WB, make_unique<Box>(edge_len, edge_len, edge_len), "box"));
  ProximityProperties rigid_props;
  // A rigid hydroelastic geometry must have an infinite elastic modulus.
  rigid_props.AddProperty(geometry::kMaterialGroup, geometry::kElastic,
                          std::numeric_limits<double>::infinity());
  AddRigidHydroelasticProperties(edge_len, &rigid_props);
  scene_graph.AssignRole(source_id, ground_id, rigid_props);
  IllustrationProperties illus_props;
  illus_props.AddProperty("phong", "diffuse", Vector4d{0.5, 0.5, 0.45, 1.0});
  scene_graph.AssignRole(source_id, ground_id, illus_props);

  // Make and visualize contacts.
  auto& contact_results = *builder.AddSystem<ContactResultMaker>();
  builder.Connect(scene_graph.get_query_output_port(),
                  contact_results.get_geometry_query_port());

  // Now visualize.
  DrakeLcm lcm;

  // Visualize geometry.
  ConnectDrakeVisualizer(&builder, scene_graph, &lcm);

  // Visualize contacts.
  auto& contact_to_lcm =
      *builder.AddSystem(LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm, 1.0 / 60));
  builder.Connect(contact_results, contact_to_lcm);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(FLAGS_real_time ? 1.f : 0.f);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
  return 0;
}

}  // namespace contact_surface
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::scene_graph::contact_surface::do_main();
}
