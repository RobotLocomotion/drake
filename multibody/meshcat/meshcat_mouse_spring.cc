#include "drake/multibody/meshcat/meshcat_mouse_spring.h"

#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"

namespace drake {
namespace multibody {
namespace meshcat {

using Eigen::Vector3d;
using geometry::Meshcat;
using math::RigidTransform;
using systems::Context;
using systems::DiagramBuilder;

namespace {

// Replicates MultibodyPlant's body-frame naming (see GetScopedName in
// multibody_plant.cc) and converts the "::" model-instance separators into "/"
// the same way MeshcatVisualizer does, so the result matches the body's path in
// the Meshcat scene tree.
template <typename T>
std::string BodyFramePathSegment(const MultibodyPlant<T>& plant,
                                 const RigidBody<T>& body) {
  std::string name;
  const ModelInstanceIndex model_instance = body.model_instance();
  if (model_instance != world_model_instance() &&
      model_instance != default_model_instance()) {
    name = plant.GetModelInstanceName(model_instance) + "::" + body.name();
  } else {
    name = body.name();
  }
  // MultibodyPlant declares frames with SceneGraph using "::";
  // MeshcatVisualizer replaces those with "/" to expose the full tree.
  for (size_t pos = 0; (pos = name.find("::", pos)) != std::string::npos;) {
    name.replace(pos, 2, "/");
    pos += 1;
  }
  return name;
}

}  // namespace

template <typename T>
MeshcatMouseSpring<T>::MeshcatMouseSpring(std::shared_ptr<Meshcat> meshcat,
                                          const MultibodyPlant<T>* plant,
                                          double stiffness)
    : systems::LeafSystem<T>(),
      meshcat_(std::move(meshcat)),
      plant_(plant),
      stiffness_(stiffness) {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  DRAKE_THROW_UNLESS(plant_ != nullptr);
  DRAKE_THROW_UNLESS(plant_->is_finalized());
  DRAKE_THROW_UNLESS(stiffness_ >= 0.0);

  BuildPathToBodyMap(*plant_);

  body_poses_input_port_ =
      this->DeclareAbstractInputPort("body_poses",
                                     Value<std::vector<RigidTransform<T>>>())
          .get_index();
  body_spatial_velocities_input_port_ =
      this->DeclareAbstractInputPort("body_spatial_velocities",
                                     Value<std::vector<SpatialVelocity<T>>>())
          .get_index();
  spatial_forces_output_port_ =
      this->DeclareAbstractOutputPort(
              "spatial_forces", std::vector<ExternallyAppliedSpatialForce<T>>{},
              &MeshcatMouseSpring<T>::CalcSpatialForces)
          .get_index();
}

template <typename T>
MeshcatMouseSpring<T>::~MeshcatMouseSpring() = default;

template <typename T>
void MeshcatMouseSpring<T>::BuildPathToBodyMap(const MultibodyPlant<T>& plant) {
  for (BodyIndex index(0); index < plant.num_bodies(); ++index) {
    if (index == plant.world_body().index()) continue;
    const RigidBody<T>& body = plant.get_body(index);
    // MeshcatVisualizer publishes each body's geometry under a node named by
    // the body's scoped frame name (with "::" replaced by "/"), e.g.
    // "/drake/<prefix>/my_model/my_body/<geometry>". We key on just the scoped
    // name ("my_model/my_body") and match it within the dragged path below.
    path_to_body_[BodyFramePathSegment(plant, body)] = index;
  }
}

template <typename T>
void MeshcatMouseSpring<T>::CalcSpatialForces(
    const Context<T>& context,
    std::vector<ExternallyAppliedSpatialForce<T>>* forces) const {
  forces->clear();
  if constexpr (std::is_same_v<T, double>) {
    const std::optional<Meshcat::ObjectDrag> drag = meshcat_->GetObjectDrag();
    if (!drag.has_value()) {
      return;
    }

    // Find the body whose scoped frame name appears as a run of path segments
    // in the dragged object's Meshcat path. The path looks like
    // "/drake/<vis_prefix>/.../<model>/<body>/<geometry>...", so we look for
    // the scoped name ("<model>/<body>") bounded by '/' on both sides. Matching
    // this way is independent of which visualization layer (illustration,
    // proximity, inertia, ...) was clicked. Among matches we keep the longest
    // (most specific) scoped name.
    BodyIndex body_index;
    size_t best_len = 0;
    const std::string& drag_path = drag->path;
    for (const auto& [segment, index] : path_to_body_) {
      if (segment.size() <= best_len) continue;
      const std::string needle = "/" + segment;
      for (size_t pos = drag_path.find(needle); pos != std::string::npos;
           pos = drag_path.find(needle, pos + 1)) {
        const size_t after = pos + needle.size();
        if (after == drag_path.size() || drag_path[after] == '/') {
          best_len = segment.size();
          body_index = index;
          break;
        }
      }
    }
    if (!body_index.is_valid()) {
      // The dragged object doesn't belong to a movable body of this plant.
      return;
    }

    const auto& X_WB_all =
        get_body_poses_input_port()
            .template Eval<std::vector<RigidTransform<T>>>(context);
    const auto& V_WB_all =
        get_body_spatial_velocities_input_port()
            .template Eval<std::vector<SpatialVelocity<T>>>(context);
    const RigidTransform<double>& X_WB = X_WB_all[body_index];
    const SpatialVelocity<double>& V_WB = V_WB_all[body_index];

    // The attachment point A (anchor) and the cursor target T, in world.
    const Vector3d& p_WA = drag->anchor_in_world;
    const Vector3d& p_WT = drag->target_in_world;

    // The anchor expressed in the body frame, where the force is applied.
    const Vector3d p_BoBq_B = X_WB.inverse() * p_WA;

    // The world velocity of the attachment point, for damping.
    const Vector3d p_BoA_W = p_WA - X_WB.translation();
    const Vector3d v_WA = V_WB.Shift(p_BoA_W).translational();

    // Mass-scaled spring + damper force: scaling by the body's mass makes the
    // translational response frequency (sqrt(stiffness)) and damping ratio
    // independent of mass.
    // TODO(vincekurtz): consider using composite mass instead of body mass.
    const double mass = plant_->get_body(body_index).default_mass();
    const Vector3d f_W =
        mass * stiffness_ * (p_WT - p_WA) - mass * std::sqrt(stiffness_) * v_WA;

    ExternallyAppliedSpatialForce<double> force;
    force.body_index = body_index;
    force.p_BoBq_B = p_BoBq_B;
    force.F_Bq_W = SpatialForce<double>(Vector3d::Zero(), f_W);
    forces->push_back(force);
  } else {
    // Mouse interaction is only meaningful for T == double (Meshcat reports
    // drags as plain doubles); leave the output empty for other scalars.
    unused(context);
  }
}

template <typename T>
MeshcatMouseSpring<T>& MeshcatMouseSpring<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const MultibodyPlant<T>* plant,
    std::shared_ptr<Meshcat> meshcat, double stiffness) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(plant != nullptr);
  auto& spring = *builder->template AddSystem<MeshcatMouseSpring<T>>(
      std::move(meshcat), plant, stiffness);
  spring.set_name("meshcat_mouse_spring");
  builder->Connect(plant->get_body_poses_output_port(),
                   spring.get_body_poses_input_port());
  builder->Connect(plant->get_body_spatial_velocities_output_port(),
                   spring.get_body_spatial_velocities_input_port());
  builder->Connect(spring.get_spatial_forces_output_port(),
                   plant->get_applied_spatial_force_input_port());
  return spring;
}

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::MeshcatMouseSpring);
