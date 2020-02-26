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
 - __real_time_rate__: Set to zero to run as fast as possible. Defaults to 0.0.
 - __sphere_resolution_hint__: Measure of the soft sphere's resolution, where
   smaller numbers produce a denser, more expensive mesh. Defaults to 1. A
   value of at least 1.5 will produce the coarsest mesh possible.
 - __box_resolution_hint__: Measure of the rigid box's resolution, where
   smaller numbers produce a denser, more expensive mesh. Defaults to 10. A
   value of at least 10 will produce the coarsest mesh possible.

 <h2>Running the profile</h2>

 The profile can be executed as:

 ```
 perf record -g bazel-bin/geometry/benchmarking/simple_contact_surface_profile
 ```

 To run with different arguments, for example, a sphere resolution hint of 0.5:

 ```
 perf record -g bazel-bin/geometry/benchmarking/simple_contact_surface_profile --sphere_resolution_hint=0.5
 ```

 To view the resulting profile:

 ```
 perf report -g
 ```

 Excerpts from the output with a sphere_resolution_hint of 0.5 can be seen
 below. The 'Children' column displays the percentage of execution spent in the
 function specified in 'Symbol' as well as any child calls. The 'Self' column
 displays the percentage of execution spent only in the function itself. The
 'Enter' key can be used to expand lines beginning with '+' to view the child
 calls. The 'a' key can also be used to view the corresponding assembly.

 ```
Samples: 687  of event 'cycles:ppp', Event count (approx.): 525060146
  Children      Self  Command          Shared Object                   Symbol
...
+   45.12%     0.60%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ContactResultMaker::CalcContactResults                                            ▒
+   44.52%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::QueryObject<double>::ComputeContactSurfaces                                                 ▒
+   44.52%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ProximityEngine<double>::ComputeContactSurfaces                                   ▒
+   44.52%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ProximityEngine<double>::Impl::ComputeContactSurfaces                             ▒
+   44.52%     0.00%  simple_contact_  simple_contact_surface_profile  [.] fcl::DynamicAABBTreeCollisionManager<double>::collide                                                        ▒
+   44.52%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::hydroelastic::Callback<double>                                                    ▒
+   44.37%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::hydroelastic::MaybeCalcContactSurface<double>                                     ▒
+   44.37%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ComputeContactSurfaceFromSoftVolumeRigidSurface<double>                           ▒
+   44.37%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::SampleVolumeFieldOnSurface<double>                                                ▒
+   35.60%     0.46%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::BoundingVolumeHierarchy<drake::geometry::VolumeMesh<double> >::Collide<drake::geom▒
+   34.98%     0.00%  simple_contact_  simple_contact_surface_profile  [.] std::_Function_handler<drake::geometry::internal::BvttCallbackResult (drake::TypeSafeIndex<drake::geometry::V▒
+   34.84%     1.38%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::SampleVolumeFieldOnSurface<double>(drake::geometry::MeshField<double, drake::geome▒
-   26.72%    10.58%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ClipTriangleByTetrahedron<double>                                                 ▒
   - 16.15% drake::geometry::internal::ClipTriangleByTetrahedron<double>                                                                                                                ▒
      + 14.94% drake::geometry::internal::ClipPolygonByHalfSpace<double>                                                                                                                ▒
        1.05% drake::geometry::internal::RemoveDuplicateVertices<double>                                                                                                                ▒
   + 10.58% 0xa4d6258d4c544155                                                                                                                                                          ▒
+   18.57%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::Simulator<double>::IntegrateContinuousState                                                  ▒
+   18.13%     0.00%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::IntegratorBase<double>::IntegrateNoFurtherThanTime                                           ▒
+   16.89%     0.17%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::IntegratorBase<double>::StepOnceErrorControlledAtMost                                        ▒
+   14.94%     9.20%  simple_contact_  simple_contact_surface_profile  [.] drake::geometry::internal::ClipPolygonByHalfSpace<double>                                                    ▒
+   11.74%    11.57%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::DependencyTracker::NotifySubscribers                                                         ▒
+   11.12%     0.15%  simple_contact_  simple_contact_surface_profile  [.] drake::systems::RungeKutta3Integrator<double>::DoStep                                                        ▒
 ```

 To view the simulation, first run drake visualiser in a separate terminal:
 ```
 bazel-bin/tools/drake_visualizer
 ```

 <h2>Interpreting the profile</h2>

 Top culprits for taking execution time were in clipping triangles, i.e.
 ClipTrangleByTetrahedron/ClipPolygonByHalfSpace (10.58% + 9.2%), and
 NotifySubscribers (11.57%). These functions should thus be further investigated
 to determine if these times are justified or if optimizations can be made
 instead.

 Take ClipTriangleByTetrahedron for example, we can look further at the assembly
 in perf's report, using the 'a' key. The below excerpt shows that 12.86% is
 spent on the sqrtpd operation, which is the square root of double-precision
 floating-point values.

 ```
  1.43 │       movapd %xmm2,-0x60(%rbp)                                                                                                                                                 ▒
       │       movsd  %xmm0,-0x50(%rbp)                                                                                                                                                 ◆
       │       movsd  %xmm1,-0x48(%rbp)                                                                                                                                                 ▒
       │       movq   %xmm3,%xmm0                                                                                                                                                       ▒
 12.86 │       sqrtpd %xmm0,%xmm0                                                                                                                                                       ▒
  5.71 │       addsd  0x12fe64(%rip),%xmm0        # 1dcdd0 <_fini+0x8c>                                                                                                                 ▒
  1.43 │       andpd  0x1300fc(%rip),%xmm0        # 1dd070 <_fini+0x32c>                                                                                                                ▒
       │       movsd  0x12fe5c(%rip),%xmm1        # 1dcdd8 <_fini+0x94>                                                                                                                 ▒
  1.43 │       ucomisd %xmm0,%xmm1                                                                                                                                                      ▒                                                                                                                                                   ▒
 ```

 In ClipPolygonByHalfSpace, the excerpt below shows 11.48% is spent on the idiv
 operation, which is the signed divide, likely due to calculating the modulus.

 ```
  1.64 │       add    $0x1,%r14
       │ 7e:   mov    %ebx,%eax
       │       cltd
 11.48 │       idiv   %ecx
       │       movslq %edx,%rax
  1.64 │       lea    (%rax,%rax,2),%rax
       │       movupd (%r12),%xmm2
 ```

 <h2>Timing and stats<h2>

 For a breakdown of timing and cycles used on, `perf` also has a `stat` option.
 The `-r` argument can be added to repeat the measurement and output the
 standard deviation from the mean. For example, to run it 5 times:

 ```
 perf stat -r5 bazel-bin/geometry/benchmarking/simple_contact_surface_profile --sphere_resolution_hint=0.5
 ```

 The output will be similar to:

 ```
 Performance counter stats for 'bazel-bin/geometry/benchmarking/simple_contact_surface_profile --sphere_resolution_hint=0.5' (5 runs):

        171.564682      task-clock (msec)         #    1.053 CPUs utilized            ( +-  0.52% )
               876      context-switches          #    0.005 M/sec                    ( +-  1.07% )
                11      cpu-migrations            #    0.065 K/sec                    ( +- 27.58% )
               539      page-faults               #    0.003 M/sec                    ( +-  0.16% )
       563,893,328      cycles                    #    3.287 GHz                      ( +-  1.72% )
     1,094,996,457      instructions              #    1.94  insn per cycle           ( +-  0.02% )
       141,836,308      branches                  #  826.722 M/sec                    ( +-  0.02% )
         1,108,104      branch-misses             #    0.78% of all branches          ( +-  3.60% )

       0.162931434 seconds time elapsed                                          ( +-  0.58% )
 ```

 This can be used to analyse how the program runs on the user's machine and
 provide a clearer comparison of overall runtime as optimization progress is
 made.

 */

using Eigen::Vector3d;
using Eigen::Vector4d;
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
DEFINE_double(real_time_rate, 0.0, "Set to 0 to run as fast as possible");
DEFINE_double(sphere_resolution_hint, 1.0,
              "Measure of sphere resolution -- smaller numbers produce a "
              "denser, more expensive mesh");
DEFINE_double(box_resolution_hint, 10.0,
              "Measure of box resolution -- smaller numbers produce a "
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
  AddRigidHydroelasticProperties(FLAGS_box_resolution_hint, &rigid_props);
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
  simulator.set_target_realtime_rate(FLAGS_real_time_rate);
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
