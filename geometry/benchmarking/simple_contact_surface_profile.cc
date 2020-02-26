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
namespace geometry {
namespace internal {

/** @defgroup contact_surface_profiles Contact Surface Profiles
 @ingroup proximity_queries

 This profiles a simulation that computes the contact surface between a soft
 and rigid mesh.

 This is a simple simulation where a soft sphere moves with sinusoidal motion in
 and out of contact with a rigid box. Eventually we'd like to profile a full
 system through multibody plant, but this is decoupled from dynamics so that
 just the geometric components can be evaluated.

 Arguments include:
 - __sphere_resolution_hint__: Measure of the soft sphere's resolution, where
   smaller numbers produce a denser, more expensive mesh. Defaults to 1. A
   value larger than 1.5 will produce the coarsest mesh possible.

 <h2>Running the profile</h2>

 The profile can be executed as:

 ```
 perf record -g bazel-bin/geometry/benchmarking/simple_contact_surface_profile
 ```

 To run with different arguments, for example, a sphere edge length of 0.5:

 ```
 perf record -g bazel-bin/geometry/benchmarking/simple_contact_surface_profile --sphere_resolution_hint=0.5
 ```

 To view the resulting profile:

 ```
 perf report -g
 ```

 Excerpts from the output can be seen below. The 'Children' column displays
 the percentage of execution spent in the function specified in 'Symbol' as well
 as any child calls. The 'Self' column displays the percentage of execution
 spent only in the function itself. The 'Enter' key can be used to expand lines
 beginning with '+' to view the child calls. The 'a' key can also be used to
 view the corresponding assembly.

 ```
Samples: 2K of event 'cycles:ppp', Event count (approx.): 994926367
  Children      Self  Command          Shared Object                   Symbol
...
+   60.09%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::QueryObject<double>::ComputeContactSurfaces                                    ▒
+   60.09%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ProximityEngine<double>::ComputeContactSurfaces                      ▒
+   60.09%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ProximityEngine<double>::Impl::ComputeContactSurfaces                ▒
+   60.07%     0.00%  simple_contact_  simple_contact_surface_profile  [.] fcl::detail::dynamic_AABB_tree::collisionRecurse<double>                                        ▒
+   60.05%     0.00%  simple_contact_  simple_contact_surface_profile  [.] fcl::DynamicAABBTreeCollisionManager<double>::collide                                           ▒
+   60.00%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::hydroelastic::Callback<double>                                       ▒
+   59.96%     0.05%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::hydroelastic::MaybeCalcContactSurface<double>                        ▒
+   59.64%     0.05%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::SampleVolumeFieldOnSurface<double>                                   ▒
+   59.64%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ComputeContactSurfaceFromSoftVolumeRigidSurface<double>              ▒
+   45.46%     0.99%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::BoundingVolumeHierarchy<drake::geometry::VolumeMesh<double> >::Collid▒
+   44.34%     0.05%  simple_contact_  simple_contact_surface_profile  [.] std::_Function_handler<drake::geometry::internal::BvttCallbackResult (drake::TypeSafeIndex<drake▒
+   44.24%     1.25%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::SampleVolumeFieldOnSurface<double>(drake::geometry::MeshField<double,▒
-   34.51%    13.95%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ClipTriangleByTetrahedron<double>                                    ▒
   - 20.57% drake::geometry::internal::ClipTriangleByTetrahedron<double>                                                                                                   ▒
      + 19.89% drake::geometry::internal::ClipPolygonByHalfSpace<double>                                                                                                   ▒
        0.54% drake::geometry::internal::RemoveDuplicateVertices<double>                                                                                                   ▒
   + 13.95% 0x85d6258d4c544155
+   19.98%    12.72%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ClipPolygonByHalfSpace<double>                                       ▒
+   13.36%    13.23%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::Aabb::HasOverlap                                                     ▒
+   10.28%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::Simulator<double>::IntegrateContinuousState                                     ▒
+   10.01%     0.05%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::IntegratorBase<double>::IntegrateNoFurtherThanTime                              ▒
+    9.32%     9.32%  simple_contact_  simple_contact_surface_profile  [.] Eigen::internal::partial_lu_impl<double, 0, int>::unblocked_lu                                  ▒
+    9.08%     0.06%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::IntegratorBase<double>::StepOnceErrorControlledAtMost                           ▒
+    7.50%     0.09%  simple_contact_  [unknown]                       [.] 0000000000000000                                                                                ▒
+    6.61%     6.61%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::DependencyTracker::NotifySubscribers
 ```

 To view the simulation, first run drake visualiser in a separate terminal:
 ```
 bazel-bin/tools/drake_visualizer
 ```

 <h2>Interpreting the profile</h2>

 Top culprits for taking execution time were in clipping triangles, i.e.
 ClipTrangleByTetrahedron/ClipPolygonByHalfSpace (13.95% + 12.72%),  HasOverlap
 (13.23%). These functions should thus be further investigated to determine if
 these times are justified or if optimizations can be made instead.

 Take ClipTriangleByTetrahedron for example, we can look further at the assembly
 in perf's report, using the 'a' key. The below excerpt shows that 11.54% is
 spent on the sqrtpd operation, which is the square root of double-precision
 floating-point values.

 ```
  0.64 │       movapd %xmm2,-0x60(%rbp)                                                                                                                                    ▒
       │       movsd  %xmm0,-0x50(%rbp)                                                                                                                                    ▒
       │       movsd  %xmm1,-0x48(%rbp)                                                                                                                                    ▒
       │       movq   %xmm3,%xmm0                                                                                                                                          ▒
 11.54 │       sqrtpd %xmm0,%xmm0                                                                                                                                          ▒
  2.56 │       addsd  0x12f0b4(%rip),%xmm0        # 1dcb50 <_fini+0x8c>                                                                                                    ▒
  0.96 │       andpd  0x12f34c(%rip),%xmm0        # 1dcdf0 <_fini+0x32c>                                                                                                   ▒
       │       movsd  0x12f0ac(%rip),%xmm1        # 1dcb58 <_fini+0x94>                                                                                                    ◆
       │       ucomisd %xmm0,%xmm1                                                                                                                                         ▒
  0.64 │     ↓ jb     adc8b <std::vector<Eigen::Matrix<double, 3, 62b
 ```

 In ClipTriangleByHalfspace, the excerpt below shows 15.55% is spent on the idiv
 operation, which is the signed divide, likely due to calculating the modulus.

 ```
  0.71 │       add    $0x1,%r14                                                                                                                                            ▒
  0.71 │ 7e:   mov    %ebx,%eax                                                                                                                                            ▒
       │       cltd                                                                                                                                                        ▒
 15.55 │       idiv   %ecx                                                                                                                                                 ▒
  1.77 │       movslq %edx,%rax                                                                                                                                            ▒
  0.71 │       lea    (%rax,%rax,2),%rax                                                                                                                                   ▒
  0.35 │       movupd (%r12),%xmm2
 ```

 */

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::AddContactMaterial;
using geometry::AddRigidHydroelasticProperties;
using geometry::AddSoftHydroelasticProperties;
using geometry::Box;
using geometry::ConnectDrakeVisualizer;
using geometry::ContactSurface;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
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
DEFINE_bool(real_time, false, "Set to false to run as fast as possible");
DEFINE_double(sphere_resolution_hint, 1.0,
              "Measure of sphere resolution -- smaller numbers produce a "
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
    AddContactMaterial(1e8, {}, {}, &prox_props);
    AddSoftHydroelasticProperties(FLAGS_sphere_resolution_hint, &prox_props);
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
    std::vector<ContactSurface<double>> surfaces;
    surfaces = query_object.ComputeContactSurfaces();
    const int num_surfaces = static_cast<int>(surfaces.size());

    auto& msg = *results;
    msg.timestamp = context.get_time() * 1e6;  // express in microseconds.
    msg.num_hydroelastic_contacts = num_surfaces;
    msg.hydroelastic_contacts.resize(num_surfaces);

    auto write_double3 = [](const Vector3d& src, double* dest) {
      dest[0] = src(0);
      dest[1] = src(1);
      dest[2] = src(2);
    };

    // Contact surfaces.
    for (int i = 0; i < num_surfaces; ++i) {
      lcmt_hydroelastic_contact_surface_for_viz& surface_msg =
          msg.hydroelastic_contacts[i];

      surface_msg.body1_name = "Id_" + to_string(surfaces[i].id_M());
      surface_msg.body2_name = "Id_" + to_string(surfaces[i].id_N());

      const SurfaceMesh<double>& mesh_W = surfaces[i].mesh_W();
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

        tri_msg.pressure_A = surfaces[i].EvaluateE_MN(face.vertex(0));
        tri_msg.pressure_B = surfaces[i].EvaluateE_MN(face.vertex(1));
        tri_msg.pressure_C = surfaces[i].EvaluateE_MN(face.vertex(2));
      }
    }
  }

  int geometry_query_input_port_{-1};
  int contact_result_output_port_{-1};
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

}  // namespace internal
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::internal::do_main();
}
