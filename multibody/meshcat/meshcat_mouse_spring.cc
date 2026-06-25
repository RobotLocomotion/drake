#include "drake/multibody/meshcat/meshcat_mouse_spring.h"

#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"

namespace drake {
namespace multibody {
namespace meshcat {

using Eigen::Vector3d;
using geometry::FrameId;
using geometry::Meshcat;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using math::RigidTransform;
using systems::Context;
using systems::DiagramBuilder;

MeshcatMouseSpring::MeshcatMouseSpring(std::shared_ptr<Meshcat> meshcat,
                                       const MultibodyPlant<double>* plant,
                                       const SceneGraph<double>& scene_graph,
                                       double stiffness)
    : systems::LeafSystem<double>(),
      meshcat_(std::move(meshcat)),
      plant_(DRAKE_DEREF(plant)),
      stiffness_(stiffness) {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  // N.B. Unlike geometry::meshcat::MeshcatVisualizer, we don't need to track
  // geometry changes because bodies cannot be changed after plant finalization.
  DRAKE_THROW_UNLESS(plant_.is_finalized());
  DRAKE_THROW_UNLESS(plant_.geometry_source_is_registered());
  DRAKE_THROW_UNLESS(stiffness_ >= 0.0);

  BuildPathToBodyMap(scene_graph);

  body_poses_input_port_ =
      this->DeclareAbstractInputPort(
              "body_poses", Value<std::vector<RigidTransform<double>>>())
          .get_index();
  body_spatial_velocities_input_port_ =
      this->DeclareAbstractInputPort(
              "body_spatial_velocities",
              Value<std::vector<SpatialVelocity<double>>>())
          .get_index();
  spatial_forces_output_port_ =
      this->DeclareAbstractOutputPort(
              "spatial_forces",
              std::vector<ExternallyAppliedSpatialForce<double>>{},
              &MeshcatMouseSpring::CalcSpatialForces)
          .get_index();
}

MeshcatMouseSpring::~MeshcatMouseSpring() = default;

void MeshcatMouseSpring::BuildPathToBodyMap(
    const SceneGraph<double>& scene_graph) {
  const SceneGraphInspector<double>& inspector = scene_graph.model_inspector();
  for (const FrameId frame_id : inspector.GetAllFrameIds()) {
    if (frame_id == inspector.world_frame_id()) continue;
    const RigidBody<double>* body = plant_.GetBodyFromFrameId(frame_id);
    if (body == nullptr) continue;  // Not a body frame of this plant.
    // MeshcatVisualizer publishes each frame's geometry under a node named by
    // the frame's name with "::" model-instance separators replaced by "/",
    // e.g. "/drake/<prefix>/my_model/my_body/<geometry>". We read that frame
    // name straight from SceneGraph (rather than reconstructing it from the
    // plant) and key on it ("my_model/my_body"), matching it within the dragged
    // path below.
    std::string segment = inspector.GetName(frame_id);
    for (size_t pos = 0;
         (pos = segment.find("::", pos)) != std::string::npos;) {
      segment.replace(pos, 2, "/");
      pos += 1;
    }
    path_to_body_[std::move(segment)] = body->index();
  }
}

void MeshcatMouseSpring::CalcSpatialForces(
    const Context<double>& context,
    std::vector<ExternallyAppliedSpatialForce<double>>* forces) const {
  forces->clear();
  const std::optional<Meshcat::VirtualSpringKinematics> drag =
      meshcat_->GetVirtualSpringKinematics();
  if (!drag.has_value()) {
    return;
  }

  const BodyIndex body_index =
      internal::FindBodyForPath(path_to_body_, drag->path);
  if (!body_index.is_valid()) {
    // The dragged object doesn't belong to a movable body of this plant.
    return;
  }

  const auto& X_WB_all =
      get_body_poses_input_port().Eval<std::vector<RigidTransform<double>>>(
          context);
  const auto& V_WB_all =
      get_body_spatial_velocities_input_port()
          .Eval<std::vector<SpatialVelocity<double>>>(context);
  const RigidTransform<double>& X_WB = X_WB_all[body_index];
  const SpatialVelocity<double>& V_WB = V_WB_all[body_index];

  // The attachment point Bq (a point fixed to body B) and the cursor target T,
  // in world.
  const Vector3d& p_WBq = drag->body_point_in_world;
  const Vector3d& p_WT = drag->target_point_in_world;

  // The attachment point expressed in the body frame, where the force is
  // applied.
  const Vector3d p_BoBq_B = X_WB.inverse() * p_WBq;

  // The world velocity of the attachment point, for damping.
  const Vector3d p_BoBq_W = p_WBq - X_WB.translation();
  const Vector3d v_WBq = V_WB.Shift(p_BoBq_W).translational();

  // Mass-scaled spring + damper force: scaling by the body's mass makes the
  // translational response frequency (sqrt(stiffness)) and damping ratio
  // independent of mass. Note that we're using default_mass() because we expect
  // the likelihood of a user tweaking the parameterized mass beyond the initial
  // model declaration for an interactive visualization session where they want
  // to drag things around to be vanishingly small. Getting access to the
  // current parameterized mass is more trouble than it's worth.
  // TODO(vincekurtz): consider using composite mass instead of body mass.
  const double mass = plant_.get_body(body_index).default_mass();
  const Vector3d f_W =
      mass * stiffness_ * (p_WT - p_WBq) - mass * std::sqrt(stiffness_) * v_WBq;

  ExternallyAppliedSpatialForce<double> force;
  force.body_index = body_index;
  force.p_BoBq_B = p_BoBq_B;
  force.F_Bq_W = SpatialForce<double>(Vector3d::Zero(), f_W);
  forces->push_back(force);
}

MeshcatMouseSpring& MeshcatMouseSpring::AddToBuilder(
    DiagramBuilder<double>* builder, const MultibodyPlant<double>* plant,
    const SceneGraph<double>& scene_graph, std::shared_ptr<Meshcat> meshcat,
    double stiffness) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(plant != nullptr);
  auto& spring = *builder->AddSystem<MeshcatMouseSpring>(
      std::move(meshcat), plant, scene_graph, stiffness);
  spring.set_name("meshcat_mouse_spring");
  builder->Connect(plant->get_body_poses_output_port(),
                   spring.get_body_poses_input_port());
  builder->Connect(plant->get_body_spatial_velocities_output_port(),
                   spring.get_body_spatial_velocities_input_port());
  builder->Connect(spring.get_spatial_forces_output_port(),
                   plant->get_applied_spatial_force_input_port());
  return spring;
}

namespace internal {

BodyIndex FindBodyForPath(const std::map<std::string, BodyIndex>& path_to_body,
                          const std::string& path) {
  // Identify the body associated with `path`.
  //
  //  1) Search for the stored body path segment in `path`. It must appear
  //     prefixed by `/` (so "foo" doesn't match "some_foo"), and either be
  //     suffixed by '/' or be at the end of the path string.
  //     - We do a left search to find the portion of the path that is closest
  //       to the root of meshcat's scene hierarchy.
  //  2) Because not all path segments consist of "model/body" (i.e., those
  //     bodies in the default model and world model instances), we need to make
  //     sure "body" doesn't match "m1/body", "m2/body", etc. As both "body"
  //     and "m1/body" would match "drake/foo/m1/body", we prefer the *longest*
  //     matching path segment (the pre- and post `/` delimiters prevent us from
  //     catching false matches based on common substrings).
  //
  // Note: we don't have to worry about multiple matches (e.g., matching body
  // path segment "m/b" to the path "drake/foo/m/b/and/m/b") because we are
  // only interested in Meshcat paths that correspond to MultibodyPlant links.
  // Those are generally defined in SceneGraph as a flat list of bodies and,
  // therefore, appear in Meshcat without nesting. Anything nested in that
  // manner came from an alternative, and therefore ignorable, source.
  //
  // This search allows us to identify the body whether the user drags on the
  // illustration or proximity geometry -- as long as the geometry is ultimately
  // a child of an identifiable body.
  BodyIndex body_index;
  size_t best_len = 0;
  for (const auto& [segment, index] : path_to_body) {
    // A shorter-or-equal segment can't beat the current best match.
    if (segment.size() <= best_len) continue;
    const std::string needle = "/" + segment;
    for (size_t pos = path.find(needle); pos != std::string::npos;
         pos = path.find(needle, pos + 1)) {
      const size_t after = pos + needle.size();
      if (after == path.size() || path[after] == '/') {
        best_len = segment.size();
        body_index = index;
        break;
      }
    }
  }
  return body_index;
}

}  // namespace internal

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
