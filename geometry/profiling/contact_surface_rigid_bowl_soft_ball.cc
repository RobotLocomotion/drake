/* @file
 A simple binary for profiling computation of ContactSurface between an
 anchored rigid bowl and a dynamic compliant ball. The rigid bowl is a realistic
 non-convex object represented by 7,910 triangles. The compliant ball is
 represented by a coarse tetrahedral mesh, which is typical in hydroelastic
 contact model. This is decoupled from dynamics so that just the geometric
 components can be evaluated in as light-weight a fashion as possible. The
 ball moves moderately and stays in contact with the bowl all the time.
 Optionally it can use a rigid ball or a rigid box instead of the rigid bowl.
 It can also use a compliant box instead of the compliant ball.
*/

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/value.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace examples {
namespace scene_graph {
namespace contact_surface {

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::AddContactMaterial;
using geometry::AddRigidHydroelasticProperties;
using geometry::AddSoftHydroelasticProperties;
using geometry::Box;
using geometry::Capsule;
using geometry::ContactSurface;
using geometry::Cylinder;
using geometry::DrakeVisualizerd;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::HydroelasticContactRepresentation;
using geometry::IllustrationProperties;
using geometry::Mesh;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using geometry::TriangleSurfaceMesh;
using lcm::DrakeLcm;
using math::RigidTransformd;
using std::make_unique;
using systems::Context;
using systems::DiagramBuilder;
using systems::ExplicitEulerIntegrator;
using systems::LeafSystem;
using systems::Simulator;
using systems::lcm::LcmPublisherSystem;

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds. "
              "By default, it is 10 seconds, which is the time for the "
              "moving compliant geometry to complete one period of sinusoidal "
              "vertical motion while contacting the rigid geometry.");
DEFINE_double(real_time, 1.0, "Real time factor.");
DEFINE_double(resolution_hint, 0.0125,
              "Target resolution for the compliant ball's mesh-- smaller "
              "numbers produce a denser, more expensive mesh. The compliant "
              "ball is 2.5cm in radius. By default, its mesh resolution is "
              "1.25cm, which is half the radius. This parameter affects "
              "none of the rigid bowl, the rigid box, or the compliant box.");
DEFINE_string(rigid, "bowl",
              "Specify the shape of the rigid geometry.\n"
              "[--rigid={ball,bowl,box,capsule,cylinder}]\n"
              "By default, it is the bowl.\n");
DEFINE_string(compliant, "ball",
              "Specify the shape of the compliant geometry.\n"
              "[--compliant={ball,box,capsule,cylinder}]\n"
              "By default, it is the ball.\n");
DEFINE_bool(polygons, true,
            "Set to true to use polygons to represent contact surfaces.\n"
            "Set to false to use triangles to represent contact surfaces.\n"
            "By default, it is true.");

/* Places a compliant geometry (a ball by default) and defines its velocity as
 being sinusoidal in time in World z direction.

 The center of the moving compliant geometry starts from a point O at the
 mid-height of the default rigid bowl, which can change to a rigid ball or a
 rigid box via a command-line option.
   1. It goes up to the rim of the bowl, creating the smallest contact patch
      (in terms of area).
   2. It comes back to O.
   3. It goes down deeper into the bowl, creating the largest contact patch
      (in terms of area).
   4. It comes back to O.
 The center of the ball moves along Z-axis of World frame with coordinates
 (0, 0, z(t)) in World frame at time t. The sinusoidal motion has z(t) as:

       z(t) = Zo + A * sin(2π t/T) => Set in CalcFramePoseOutput()

 where Zo is the z-coordinate of O in World frame, A is the amplitude of
 motion, and T is the time period.

 @system
 name: MovingCompliantGeometry
 output_ports:
 - geometry_pose
 @endsystem

 This system's output is strictly a function of time and has no state.
 */
class MovingCompliantGeometry final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MovingCompliantGeometry)

  // Ball radius 2.5cm.
  static constexpr double kRadius = 0.025;
  // Time period, T = 10 seconds.
  static constexpr double kT = 10.0;
  // 2π/T
  static constexpr double k2Pi_T = 2.0 * M_PI / kT;
  // z(t) ∈ [5cm, 6cm], so Zo = 5.5cm and A = 0.5cm.
  // Initial position, Zo = 5.5cm above ground.
  static constexpr double kZo = 0.055;
  // Amplitude of motion, A = 0.5cm.
  static constexpr double kA = 0.005;

  explicit MovingCompliantGeometry(SceneGraph<double>* scene_graph) {
    // Add a compliant geometry that moves based on sinusoidal derivatives.
    source_id_ = scene_graph->RegisterSource("moving_geometry");
    frame_id_ =
        scene_graph->RegisterFrame(source_id_, GeometryFrame("moving_frame"));
    if (FLAGS_compliant == "box") {
      geometry_id_ = scene_graph->RegisterGeometry(
          source_id_, frame_id_,
          make_unique<GeometryInstance>(
              RigidTransformd(), make_unique<Box>(Box::MakeCube(1.5 * kRadius)),
              "compliant box"));
    } else if (FLAGS_compliant == "capsule") {
      geometry_id_ = scene_graph->RegisterGeometry(
          source_id_, frame_id_,
          make_unique<GeometryInstance>(
              RigidTransformd(),
              make_unique<Capsule>(Capsule(1.2 * kRadius, 1.5 * kRadius)),
              "compliant capsule"));
    } else if (FLAGS_compliant == "cylinder") {
      geometry_id_ = scene_graph->RegisterGeometry(
          source_id_, frame_id_,
          make_unique<GeometryInstance>(
              RigidTransformd(),
              make_unique<Cylinder>(Cylinder(1.2 * kRadius, 1.5 * kRadius)),
              "compliant cylinder"));
    } else {
      geometry_id_ = scene_graph->RegisterGeometry(
          source_id_, frame_id_,
          make_unique<GeometryInstance>(RigidTransformd(),
                                        make_unique<Sphere>(kRadius),
                                        "compliant ball"));
      if (FLAGS_compliant != "ball") {
        std::cout << "Unsupported value for --compliant=" << FLAGS_compliant
                  << ", default to a compliant ball.\n"
                  << "Supported values are ball, box, capsule or cylinder."
                  << std::endl;
      }
    }
    ProximityProperties prox_props;
    // Resolution Hint affects the compliant ball but not the compliant box.
    AddSoftHydroelasticProperties(FLAGS_resolution_hint, 1e8, &prox_props);
    scene_graph->AssignRole(source_id_, geometry_id_, prox_props);

    IllustrationProperties illus_props;
    illus_props.AddProperty("phong", "diffuse", Vector4d(0.8, 0.1, 0.1, 0.25));
    scene_graph->AssignRole(source_id_, geometry_id_, illus_props);

    geometry_pose_port_ =
        this->DeclareAbstractOutputPort(
                "geometry_pose", &MovingCompliantGeometry::CalcFramePoseOutput)
            .get_index();
  }

  SourceId source_id() const { return source_id_; }

  const systems::OutputPort<double>& get_geometry_pose_output_port() const {
    return systems::System<double>::get_output_port(geometry_pose_port_);
  }

 private:
  void CalcFramePoseOutput(const Context<double>& context,
                           FramePoseVector<double>* poses) const {
    const double t = context.get_time();
    // z(t) = Zo + A * sin(2π t/T)
    const double z = kZo + kA * std::sin(k2Pi_T * t);
    *poses = {{frame_id_, RigidTransformd(Vector3d(0, 0, z))}};
  }

  SourceId source_id_;
  FrameId frame_id_;
  GeometryId geometry_id_;
  int geometry_pose_port_{-1};
};

/* A system that evaluates contact surfaces from SceneGraph and outputs a fake
 ContactResults with the actual contact surfaces.

 @system
 name:ContactResultMaker
 input_ports:
 - query_object
 output_ports:
 - contact_result
 @endsystem
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
    const auto contact_representation =
        FLAGS_polygons ? HydroelasticContactRepresentation::kPolygon
                       : HydroelasticContactRepresentation::kTriangle;
    std::vector<ContactSurface<double>> contacts =
        query_object.ComputeContactSurfaces(contact_representation);
    const int num_contacts = static_cast<int>(contacts.size());

    auto& msg = *results;
    msg.timestamp = context.get_time() * 1e6;  // express in microseconds.
    msg.num_point_pair_contacts = 0;
    msg.point_pair_contact_info.resize(msg.num_point_pair_contacts);
    msg.num_hydroelastic_contacts = num_contacts;
    msg.hydroelastic_contacts.resize(num_contacts);

    for (int i = 0; i < num_contacts; ++i) {
      lcmt_hydroelastic_contact_surface_for_viz& surface_message =
          msg.hydroelastic_contacts[i];
      const ContactSurface<double>& surface = contacts[i];

      // TODO(SeanCurtis-TRI): This currently skips the full naming and doesn't
      //  report any dynamics (e.g., force, moment, or quadrature data).

      surface_message.body1_name = "Id_" + to_string(surface.id_M());
      surface_message.body2_name = "Id_" + to_string(surface.id_N());

      const int num_vertices = surface.num_vertices();
      surface_message.num_vertices = num_vertices;
      surface_message.p_WV.resize(num_vertices);
      surface_message.pressure.resize(num_vertices);
      if (surface.is_triangle()) {
        const auto& mesh_W = surface.tri_mesh_W();
        const auto& e_MN_W = surface.tri_e_MN();

        // Write vertices and per vertex pressure values.
        for (int v = 0; v < num_vertices; ++v) {
          const Vector3d& p_WV = mesh_W.vertex(v);
          surface_message.p_WV[v] = {p_WV.x(), p_WV.y(), p_WV.z()};
          surface_message.pressure[v] =
              ExtractDoubleOrThrow(e_MN_W.EvaluateAtVertex(v));
        }

        // Write faces.
        surface_message.poly_data_int_count = mesh_W.num_triangles() * 4;
        surface_message.poly_data.resize(surface_message.poly_data_int_count);
        int index = -1;
        for (int t = 0; t < mesh_W.num_triangles(); ++t) {
          const geometry::SurfaceTriangle& tri = mesh_W.element(t);
          surface_message.poly_data[++index] = 3;
          surface_message.poly_data[++index] = tri.vertex(0);
          surface_message.poly_data[++index] = tri.vertex(1);
          surface_message.poly_data[++index] = tri.vertex(2);
        }
      } else {
        const auto& mesh_W = surface.poly_mesh_W();
        const auto& e_MN_W = surface.poly_e_MN();

        // Write vertices and per vertex pressure values.
        for (int v = 0; v < num_vertices; ++v) {
          const Vector3d& p_WV = mesh_W.vertex(v);
          surface_message.p_WV[v] = {p_WV.x(), p_WV.y(), p_WV.z()};
          surface_message.pressure[v] =
              ExtractDoubleOrThrow(e_MN_W.EvaluateAtVertex(v));
        }

        surface_message.poly_data_int_count = mesh_W.face_data().size();
        surface_message.poly_data = mesh_W.face_data();
      }
    }
  }

  int geometry_query_input_port_{};
  int contact_result_output_port_{};
};

int do_main() {
  DiagramBuilder<double> builder;

  auto& scene_graph = *builder.AddSystem<SceneGraph<double>>();

  auto& moving_geometry =
      *builder.AddSystem<MovingCompliantGeometry>(&scene_graph);
  builder.Connect(
      moving_geometry.get_geometry_pose_output_port(),
      scene_graph.get_source_pose_port(moving_geometry.source_id()));

  SourceId source_id = scene_graph.RegisterSource("world");

  // Rigid anchored bowl with frame B. It is a non-convex mesh.
  std::string bowl_absolute_path =
      FindResourceOrThrow("drake/geometry/profiling/evo_bowl_no_mtl.obj");
  // The bowl's bounding box is about 14.7cm x 14.7cm x 6.1cm with its
  // center at the origin Bo of frame B. Place B at 3.05cm above the ground
  // plane, so the bottom of the bowl is on the ground. Furthermore,
  // place B at 5cm in +Y direction in World frame, so the compliant ball
  // moving along Z-axis in World frame will contact the edge of the bowl.
  const RigidTransformd X_WB(Vector3d(0, 0.05, 0.0305));
  GeometryId rigid_geometry_id;
  if (FLAGS_rigid == "ball") {
    rigid_geometry_id = scene_graph.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(
                       X_WB, make_unique<Sphere>(0.04), "rigid ball"));
  } else if (FLAGS_rigid == "box") {
    rigid_geometry_id = scene_graph.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(
                       X_WB, make_unique<Box>(0.15, 0.15, 0.02), "rigid box"));
  } else if (FLAGS_rigid == "capsule") {
    rigid_geometry_id = scene_graph.RegisterAnchoredGeometry(
        source_id,
        make_unique<GeometryInstance>(X_WB, make_unique<Capsule>(0.05, 0.02),
                                      "rigid capsule"));
  } else if (FLAGS_rigid == "cylinder") {
    rigid_geometry_id = scene_graph.RegisterAnchoredGeometry(
        source_id,
        make_unique<GeometryInstance>(X_WB, make_unique<Cylinder>(0.05, 0.02),
                                      "rigid cylinder"));
  } else {
    rigid_geometry_id = scene_graph.RegisterAnchoredGeometry(
        source_id,
        make_unique<GeometryInstance>(
            X_WB, make_unique<Mesh>(bowl_absolute_path), "rigid bowl"));
    if (FLAGS_rigid != "bowl") {
      std::cout << "Unsupported value for --rigid==" << FLAGS_rigid
                << ", default to bowl.\n"
                << "Supported values are ball, bowl, box, capsule or cylinder."
                << std::endl;
    }
  }
  ProximityProperties rigid_props;
  // Resolution Hint affects the ball but neither the box nor the bowl.
  AddRigidHydroelasticProperties(FLAGS_resolution_hint, &rigid_props);
  scene_graph.AssignRole(source_id, rigid_geometry_id, rigid_props);
  IllustrationProperties illus_props;
  illus_props.AddProperty("phong", "diffuse", Vector4d{0.5, 0.5, 0.45, 0.25});
  scene_graph.AssignRole(source_id, rigid_geometry_id, illus_props);

  // Make and visualize contacts.
  auto& contact_results = *builder.AddSystem<ContactResultMaker>();
  builder.Connect(scene_graph.get_query_output_port(),
                  contact_results.get_geometry_query_port());

  // Now visualize.
  DrakeLcm lcm;

  // Visualize geometry.
  DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);

  // Visualize contacts.
  auto& contact_to_lcm =
      *builder.AddSystem(LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm, 1.0 / 60.0));
  builder.Connect(contact_results, contact_to_lcm);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  constexpr double h = 1e-3;  // 1ms time step.
  simulator.reset_integrator<ExplicitEulerIntegrator<double>>(h);
  simulator.set_target_realtime_rate(FLAGS_real_time);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  std::cout << "Number of time steps taken: " << simulator.get_num_steps_taken()
            << std::endl;
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
