/** @file
 A simple binary for exercising and visualizing computation of ContactSurfaces.
 This is decoupled from dynamics so that just the geometric components can be
 evaluated in as light-weight a fashion as possible.

 This can serve as a test bed for evaluating the various cases of the
 ContactSurface-computing algorithms. Simply swap the geometry types (moving
 and anchored) and their properties to see the effect on contact surface.  */

#include <memory>
#include <unordered_map>

#include <gflags/gflags.h>

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
using geometry::AddContactMaterial;
using geometry::AddRigidHydroelasticProperties;
using geometry::AddSoftHydroelasticProperties;
using geometry::Box;
using geometry::ContactSurface;
using geometry::Cylinder;
using geometry::DrakeVisualizerd;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::Role;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using geometry::SurfaceMesh;
using lcm::DrakeLcm;
using math::RigidTransformd;
using std::make_unique;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::DiagramBuilder;
using systems::LeafSystem;
using systems::Simulator;
using systems::lcm::LcmPublisherSystem;

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_bool(real_time, true, "Set to false to run as fast as possible");
DEFINE_double(length, 1.0,
              "Measure of sphere edge length -- smaller numbers produce a "
              "denser, more expensive mesh");
DEFINE_bool(rigid_cylinders, true,
            "Set to true, the cylinders are given a rigid "
            "hydroelastic representation");
DEFINE_bool(hybrid, false, "Set to true to run hybrid hydroelastic");
DEFINE_bool(force_full_name, false,
            "If true, the message will declare the body names are not unique, "
            "forcing the visualizer to use the full model/body names.");

/* To help simulate MultibodyPlant; we're going to assign frames "frame groups"
 that correlate with MbP's "model instance indices". We're defining the indices
 here and defining the look up table in the function that uses it. */
constexpr int kBallModelInstance = 1;
constexpr int kCylinderModelInstance = 2;

/** Places a ball at the world's origin and defines its velocity as being
 sinusoidal in time in the z direction.

 @system
 name: MovingBall
 output_ports:
 - geometry_pose
 @endsystem
 */
class MovingBall final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MovingBall)

  explicit MovingBall(SceneGraph<double>* scene_graph) {
    this->DeclareContinuousState(2);

    // Add geometry for a ball that moves based on sinusoidal derivatives.
    source_id_ = scene_graph->RegisterSource("moving_ball");
    frame_id_ = scene_graph->RegisterFrame(
        source_id_, GeometryFrame("moving_frame", kBallModelInstance));
    geometry_id_ = scene_graph->RegisterGeometry(
        source_id_, frame_id_,
        make_unique<GeometryInstance>(RigidTransformd(),
                                      make_unique<Sphere>(1.0), "ball"));

    ProximityProperties prox_props;
    AddContactMaterial(1e8, {}, {}, &prox_props);
    AddSoftHydroelasticProperties(FLAGS_length, &prox_props);
    scene_graph->AssignRole(source_id_, geometry_id_, prox_props);

    IllustrationProperties illus_props;
    illus_props.AddProperty("phong", "diffuse", Vector4d(0.1, 0.8, 0.1, 0.25));
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

  void CalcFramePoseOutput(const Context<double>& context,
                           FramePoseVector<double>* poses) const {
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

 @system
 name: ContactResultMaker
 input_ports:
 - query_object
 output_ports:
 - contact_result
 @endsystem
 */
class ContactResultMaker final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultMaker)

  explicit ContactResultMaker(bool use_strict_hydro = true)
      : use_strict_hydro_{use_strict_hydro} {
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
  static std::string ModelInstanceName(int frame_group) {
    const std::unordered_map<int, std::string> kModelInstanceNames(
        {{kBallModelInstance, "MovingBall"},
         {kCylinderModelInstance, "FixedCylinders"}});
    auto iter = kModelInstanceNames.find(frame_group);
    if (iter == kModelInstanceNames.end()) return "DefaultModelInstance";
    return iter->second;
  }

  void CalcContactResults(const Context<double>& context,
                          lcmt_contact_results_for_viz* results) const {
    const auto& query_object =
        get_geometry_query_port().Eval<QueryObject<double>>(context);
    std::vector<ContactSurface<double>> surfaces;
    std::vector<PenetrationAsPointPair<double>> points;
    if (use_strict_hydro_) {
      surfaces = query_object.ComputeContactSurfaces();
    } else {
      query_object.ComputeContactSurfacesWithFallback(&surfaces, &points);
    }
    const int num_surfaces = static_cast<int>(surfaces.size());
    const int num_pairs = static_cast<int>(points.size());

    auto& msg = *results;
    msg.timestamp = context.get_time() * 1e6;  // express in microseconds.
    msg.num_point_pair_contacts = num_pairs;
    msg.point_pair_contact_info.resize(num_pairs);
    msg.num_hydroelastic_contacts = num_surfaces;
    msg.hydroelastic_contacts.resize(num_surfaces);

    auto write_double3 = [](const Vector3d& src, double* dest) {
      dest[0] = src(0);
      dest[1] = src(1);
      dest[2] = src(2);
    };

    const auto& inspector = query_object.inspector();

    // Contact surfaces.
    for (int i = 0; i < num_surfaces; ++i) {
      lcmt_hydroelastic_contact_surface_for_viz& surface_msg =
          msg.hydroelastic_contacts[i];

      // We'll simulate MbP's model instance/body paradigm. We have a look up
      // function to define a model instance. The body name will be the frame
      // name as there is a 1-to-1 correspondence between MbP bodies and
      // geometry frames. The geometry will use the name stored in SceneGraph.
      const GeometryId id1 = surfaces[i].id_M();
      const FrameId f_id1 = inspector.GetFrameId(id1);
      surface_msg.body1_name = inspector.GetName(f_id1);
      surface_msg.model1_name =
          ModelInstanceName(inspector.GetFrameGroup(f_id1));
      surface_msg.geometry1_name = inspector.GetName(id1);
      surface_msg.body1_unique = !FLAGS_force_full_name;
      surface_msg.collision_count1 = inspector.NumGeometriesForFrameWithRole(
          inspector.GetFrameId(id1), Role::kProximity);

      const GeometryId id2 = surfaces[i].id_N();
      const FrameId f_id2 = inspector.GetFrameId(id2);
      surface_msg.body2_name = inspector.GetName(inspector.GetFrameId(id2));
      surface_msg.model2_name =
          ModelInstanceName(inspector.GetFrameGroup(f_id2));
      surface_msg.geometry2_name = inspector.GetName(id2);
      surface_msg.body2_unique = !FLAGS_force_full_name;
      surface_msg.collision_count2 = inspector.NumGeometriesForFrameWithRole(
          inspector.GetFrameId(id2), Role::kProximity);

      const SurfaceMesh<double>& mesh_W = surfaces[i].mesh_W();
      surface_msg.num_triangles = mesh_W.num_faces();
      surface_msg.triangles.resize(surface_msg.num_triangles);
      write_double3(mesh_W.centroid(), surface_msg.centroid_W);
      surface_msg.num_quadrature_points = surface_msg.num_triangles;
      surface_msg.quadrature_point_data.resize(
          surface_msg.num_quadrature_points);

      // Loop through each contact triangle on the contact surface.
      const auto& field = surfaces[i].e_MN();
      for (int j = 0; j < surface_msg.num_triangles; ++j) {
        lcmt_hydroelastic_contact_surface_tri_for_viz& tri_msg =
            surface_msg.triangles[j];
        lcmt_hydroelastic_quadrature_per_point_data_for_viz& quad_msg =
            surface_msg.quadrature_point_data[j];

        // Get the three vertices.
        const auto& face = mesh_W.element(j);
        const Vector3d& vA = mesh_W.vertex(face.vertex(0));
        const Vector3d& vB = mesh_W.vertex(face.vertex(1));
        const Vector3d& vC = mesh_W.vertex(face.vertex(2));

        write_double3(vA, tri_msg.p_WA);
        write_double3(vB, tri_msg.p_WB);
        write_double3(vC, tri_msg.p_WC);
        write_double3((vA + vB + vC) / 3.0, quad_msg.p_WQ);

        tri_msg.pressure_A = field.EvaluateAtVertex(face.vertex(0));
        tri_msg.pressure_B = field.EvaluateAtVertex(face.vertex(1));
        tri_msg.pressure_C = field.EvaluateAtVertex(face.vertex(2));

        // Fake contact *traction* and *slip velocity* data, with some
        // variations across different data instances. The variation allows one
        // to appreciate the differences among the vector scaling models for
        // hydroelastics (fixed-length, scaled, and auto-scale). Without the
        // variation, there's no visible difference between the fixed-length
        // mode and the auto-scale mode.
        write_double3(Vector3<double>(0, 0.2 + (j * 0.005), 0),
                      quad_msg.vt_BqAq_W);
        write_double3(Vector3<double>(0, -0.2 - (j * 0.005), 0),
                      quad_msg.traction_Aq_W);
      }
      // Fake contact *force* and *moment* data, with some variations across
      // different data instances to facilitate visualizer testing (see above).
      write_double3(Vector3<double>(1.2 * (i + 1), 0, 0),
                    surface_msg.force_C_W);
      write_double3(Vector3<double>(0, 0, 0.5 * (i + 1)),
                    surface_msg.moment_C_W);
    }

    // Point pairs.
    for (int i = 0; i < num_pairs; ++i) {
      lcmt_point_pair_contact_info_for_viz& info_msg =
          msg.point_pair_contact_info[i];
      info_msg.timestamp = msg.timestamp;
      const PenetrationAsPointPair<double>& pair = points[i];

      info_msg.body1_name = query_object.inspector().GetName(pair.id_A);
      info_msg.body1_name = query_object.inspector().GetName(pair.id_B);

      // Fake contact *force* data from strictly contact data. Contact point
      // is midway between the two contact points and force = normal.
      const Vector3d contact_point = (pair.p_WCa + pair.p_WCb) / 2.0;
      write_double3(contact_point, info_msg.contact_point);
      write_double3(pair.nhat_BA_W, info_msg.contact_force);
      write_double3(pair.nhat_BA_W, info_msg.normal);
    }
  }

  int geometry_query_input_port_{-1};
  int contact_result_output_port_{-1};
  const bool use_strict_hydro_{true};
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
  AddRigidHydroelasticProperties(edge_len, &rigid_props);
  scene_graph.AssignRole(source_id, ground_id, rigid_props);
  IllustrationProperties illustration_box;
  illustration_box.AddProperty("phong", "diffuse",
                               Vector4d{0.5, 0.5, 0.45, 1.0});
  scene_graph.AssignRole(source_id, ground_id, illustration_box);

  // Add two cylinders to bang into -- if the rigid_cylinders flag is set to
  // false, this should crash in strict hydroelastic mode, but report point
  // contact in non-strict mode.

  // We provide two cylinders affixed to a single frame to more robustly
  // exercise the visualization logic; it allows *two* contacts between the
  // double-can "body" and the moving ball.
  const FrameId can_frame_id = scene_graph.RegisterFrame(
      source_id, GeometryFrame("double_can", kCylinderModelInstance));
  const RigidTransformd X_WC1(Vector3d{-0.5, 0, 3});
  const RigidTransformd X_WC2(Vector3d{0.5, 0, 3});
  const GeometryId can1_id = scene_graph.RegisterGeometry(
      source_id, can_frame_id, make_unique<GeometryInstance>(
                     X_WC1, make_unique<Cylinder>(0.5, 1.0), "can1"));
  const GeometryId can2_id = scene_graph.RegisterGeometry(
      source_id, can_frame_id, make_unique<GeometryInstance>(
                     X_WC2, make_unique<Cylinder>(0.5, 1.0), "can2"));
  ProximityProperties proximity_cylinder;
  if (FLAGS_rigid_cylinders) {
    AddRigidHydroelasticProperties(0.5, &proximity_cylinder);
  }
  scene_graph.AssignRole(source_id, can1_id, proximity_cylinder);
  scene_graph.AssignRole(source_id, can2_id, proximity_cylinder);
  IllustrationProperties illustration_cylinder;
  illustration_cylinder.AddProperty("phong", "diffuse",
                                    Vector4d{0.5, 0.5, 0.45, 0.5});
  scene_graph.AssignRole(source_id, can1_id, illustration_cylinder);
  scene_graph.AssignRole(source_id, can2_id, illustration_cylinder);

  // Make and visualize contacts.
  auto& contact_results = *builder.AddSystem<ContactResultMaker>(!FLAGS_hybrid);
  builder.Connect(scene_graph.get_query_output_port(),
                  contact_results.get_geometry_query_port());

  // Now visualize.
  DrakeLcm lcm;

  // Visualize geometry.
  DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);

  // Visualize contacts.
  auto& contact_to_lcm =
      *builder.AddSystem(LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm, 1.0 / 64));
  builder.Connect(contact_results, contact_to_lcm);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& diagram_context = simulator.get_mutable_context();
  systems::Context<double>& sg_context =
      scene_graph.GetMyMutableContextFromRoot(&diagram_context);
  scene_graph.get_source_pose_port(source_id).FixValue(
      &sg_context,
      geometry::FramePoseVector<double>{{can_frame_id, RigidTransformd{}}});

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
