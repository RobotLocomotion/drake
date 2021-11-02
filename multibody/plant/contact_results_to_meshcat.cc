#include "drake/multibody/plant/contact_results_to_meshcat.h"

#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/extract_double.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/contact_results.h"

namespace drake {
namespace multibody {

using geometry::GeometryId;
using geometry::Meshcat;

template <typename T>
ContactResultsToMeshcat<T>::ContactResultsToMeshcat(
    std::shared_ptr<Meshcat> meshcat, ContactResultsToMeshcatParams params)
    : systems::LeafSystem<T>(systems::SystemTypeTag<ContactResultsToMeshcat>{}),
      meshcat_(std::move(meshcat)),
      params_(std::move(params)) {
  DRAKE_DEMAND(meshcat_ != nullptr);
  DRAKE_DEMAND(params_.publish_period >= 0.0);
  DRAKE_DEMAND(params_.force_threshold > 0.0);  // Strictly positive.

  this->DeclarePeriodicPublishEvent(params_.publish_period, 0.0,
                                    &ContactResultsToMeshcat::UpdateMeshcat);
  this->DeclareForcedPublishEvent(&ContactResultsToMeshcat::UpdateMeshcat);

  if (params_.delete_on_initialization_event) {
    this->DeclareInitializationPublishEvent(
        &ContactResultsToMeshcat::OnInitialization);
  }

  contact_results_input_port_ =
      this->DeclareAbstractInputPort("contact_results",
                                     Value<ContactResults<T>>())
          .get_index();
}

template <typename T>
template <typename U>
ContactResultsToMeshcat<T>::ContactResultsToMeshcat(
    const ContactResultsToMeshcat<U>& other)
    : ContactResultsToMeshcat(other.meshcat_, other.params_) {}

template <typename T>
void ContactResultsToMeshcat<T>::Delete() const {
  meshcat_->Delete(params_.prefix);
  contacts_.clear();
}

template <typename T>
const ContactResultsToMeshcat<T>& ContactResultsToMeshcat<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder, const MultibodyPlant<T>& plant,
    std::shared_ptr<Meshcat> meshcat, ContactResultsToMeshcatParams params) {
  return AddToBuilder(builder, plant.get_contact_results_output_port(),
                      std::move(meshcat), std::move(params));
}

template <typename T>
const ContactResultsToMeshcat<T>& ContactResultsToMeshcat<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder,
    const systems::OutputPort<T>& contact_results_port,
    std::shared_ptr<Meshcat> meshcat, ContactResultsToMeshcatParams params) {
  auto& visualizer = *builder->template AddSystem<ContactResultsToMeshcat<T>>(
      std::move(meshcat), std::move(params));
  builder->Connect(contact_results_port,
                   visualizer.contact_results_input_port());
  return visualizer;
}

template <typename T>
systems::EventStatus ContactResultsToMeshcat<T>::UpdateMeshcat(
    const systems::Context<T>& context) const {
  const auto& contact_results =
      contact_results_input_port().template Eval<ContactResults<T>>(context);

  std::map<SortedPair<GeometryId>, bool> contacts_to_process = contacts_;

  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    const PointPairContactInfo<T>& info =
        contact_results.point_pair_contact_info(i);
    const geometry::PenetrationAsPointPair<T>& pair = info.point_pair();

    const SortedPair<GeometryId> ids(pair.id_A, pair.id_B);
    const double force_norm = ExtractDoubleOrThrow(info.contact_force()).norm();
    // TODO(russt): Use geometry instance names once they are cleaned up
    // or the body name convention in ContactResultsToLcmSystem.
    const std::string path =
        fmt::format("{}/{}+{}", params_.prefix, pair.id_A, pair.id_B);
    const bool visible = (force_norm >= params_.force_threshold);

    const double arrowhead_height = params_.radius * 2.0;
    const double arrowhead_width = params_.radius * 2.0;

    auto iter = contacts_.find(ids);
    if (iter == contacts_.end()) {
      // This contact hasn't been visualized yet.
      if (!visible) {
        // Continue without visualizing.
        continue;
      }

      // Add the geometry to meshcat.
      // Note: the height of the cylinder is 2 and gets scaled to twice the
      // contact force length because I am drawing both (equal and opposite)
      // forces.
      meshcat_->SetObject(path + "/cylinder",
                          geometry::Cylinder(params_.radius, 2.0),
                          params_.color);
      const geometry::MeshcatCone arrowhead(arrowhead_height, arrowhead_width,
                                            arrowhead_width);
      meshcat_->SetObject(path + "/head", arrowhead, params_.color);
      meshcat_->SetObject(path + "/tail", arrowhead, params_.color);

      contacts_[ids] = true;
    } else {
      // This contact has been visualized before.  We avoid setting or deleting
      // the existing objects, as it leads to visual artifacts (flickering) in
      // the browser, and is incompatible with animations.
      if (visible != iter->second) {
        meshcat_->SetProperty(path, "visible", visible);
        iter->second = visible;
      }
      contacts_[ids] = visible;
      contacts_to_process.erase(ids);
    }

    if (visible) {
      // Set the transforms.
      const double height = force_norm / params_.newtons_per_meter;
      // Stretch the cylinder in z.
      meshcat_->SetTransform(
          path + "/cylinder",
          Eigen::Matrix4d(Eigen::Vector4d{1, 1, height, 1}.asDiagonal()));
      // Translate the arrowheads.
      meshcat_->SetTransform(path + "/head",
                             math::RigidTransformd(Eigen::Vector3d{
                                 0, 0, -height - arrowhead_height}));
      meshcat_->SetTransform(
          path + "/tail",
          math::RigidTransformd(
              math::RotationMatrixd::MakeXRotation(M_PI),
              Eigen::Vector3d{0, 0, height + arrowhead_height}));

      meshcat_->SetTransform(
          path, math::RigidTransformd(
                    math::RotationMatrixd::MakeFromOneVector(
                        ExtractDoubleOrThrow(info.contact_force()), 2),
                    ExtractDoubleOrThrow(info.contact_point())));
    }
  }

  // Set visible=false for any contacts not in the current list.
  for (const auto& [ids, visible] : contacts_to_process) {
    if (visible) {
      const std::string path =
          fmt::format("{}/{}+{}", params_.prefix, ids.first(), ids.second());
      meshcat_->SetProperty(path, "visible", false);
    }
    contacts_[ids] = false;
  }

  return systems::EventStatus::Succeeded();
}

template <typename T>
systems::EventStatus ContactResultsToMeshcat<T>::OnInitialization(
    const systems::Context<T>&) const {
  Delete();
  return systems::EventStatus::Succeeded();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ContactResultsToMeshcat)
