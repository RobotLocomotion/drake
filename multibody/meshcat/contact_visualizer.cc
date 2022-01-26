#include "drake/multibody/meshcat/contact_visualizer.h"

#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/extract_double.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/meshcat/point_contact_visualizer.h"
#include "drake/multibody/plant/contact_results.h"

namespace drake {
namespace multibody {
namespace meshcat {

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::Meshcat;
using meshcat::internal::PointContactVisualizer;
using meshcat::internal::PointContactVisualizerItem;
using systems::CacheEntry;
using systems::Context;
using systems::DiagramBuilder;
using systems::EventStatus;
using systems::InputPort;
using systems::OutputPort;

template <typename T>
ContactVisualizer<T>::ContactVisualizer(
    std::shared_ptr<Meshcat> meshcat, ContactVisualizerParams params)
    : systems::LeafSystem<T>(systems::SystemTypeTag<ContactVisualizer>{}),
      meshcat_(std::move(meshcat)),
      params_(std::move(params)) {
  DRAKE_DEMAND(meshcat_ != nullptr);
  DRAKE_DEMAND(params_.publish_period >= 0.0);
  DRAKE_DEMAND(params_.force_threshold > 0.0);  // Strictly positive.

  ContactVisualizerParams point_params = params_;
  point_params.prefix += "/point";
  point_visualizer_ = std::make_unique<PointContactVisualizer>(
      meshcat_, std::move(point_params));

  this->DeclarePeriodicPublishEvent(params_.publish_period, 0.0,
                                    &ContactVisualizer::UpdateMeshcat);
  this->DeclareForcedPublishEvent(&ContactVisualizer::UpdateMeshcat);

  if (params_.delete_on_initialization_event) {
    this->DeclareInitializationPublishEvent(
        &ContactVisualizer::OnInitialization);
  }

  const InputPort<T>& contact_results = this->DeclareAbstractInputPort(
    "contact_results", Value<ContactResults<T>>());
  contact_results_input_port_ = contact_results.get_index();

  const CacheEntry& point_contacts = this->DeclareCacheEntry(
      "point_contacts", &ContactVisualizer::CalcPointContacts,
      {contact_results.ticket()});
  point_contacts_cache_ = point_contacts.cache_index();
}

template <typename T>
template <typename U>
ContactVisualizer<T>::ContactVisualizer(
    const ContactVisualizer<U>& other)
    : ContactVisualizer(other.meshcat_, other.params_) {}

template <typename T>
ContactVisualizer<T>::~ContactVisualizer() = default;

template <typename T>
void ContactVisualizer<T>::Delete() const {
  point_visualizer_->Delete();
  meshcat_->Delete(params_.prefix);
}

template <typename T>
const ContactVisualizer<T>& ContactVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const MultibodyPlant<T>& plant,
    std::shared_ptr<Meshcat> meshcat, ContactVisualizerParams params) {
  return AddToBuilder(builder, plant.get_contact_results_output_port(),
                      std::move(meshcat), std::move(params));
}

template <typename T>
const ContactVisualizer<T>& ContactVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const OutputPort<T>& contact_results_port,
    std::shared_ptr<Meshcat> meshcat, ContactVisualizerParams params) {
  auto& visualizer = *builder->template AddSystem<ContactVisualizer<T>>(
      std::move(meshcat), std::move(params));
  builder->Connect(contact_results_port,
                   visualizer.contact_results_input_port());
  return visualizer;
}

template <typename T>
EventStatus ContactVisualizer<T>::UpdateMeshcat(
    const Context<T>& context) const {
  const auto& point_contacts =
      this->get_cache_entry(point_contacts_cache_).
      template Eval<std::vector<PointContactVisualizerItem>>(context);
  point_visualizer_->Update(point_contacts);
  return EventStatus::Succeeded();
}

template <typename T>
void ContactVisualizer<T>::CalcPointContacts(
    const Context<T>& context,
    std::vector<PointContactVisualizerItem>* result) const {
  result->clear();

  const auto& contact_results =
      contact_results_input_port().template Eval<ContactResults<T>>(context);
  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    const PointPairContactInfo<T>& info =
        contact_results.point_pair_contact_info(i);
    const geometry::PenetrationAsPointPair<T>& pair = info.point_pair();
    // TODO(russt): Use geometry instance names once they are cleaned up
    // or the body name convention in ContactResultsToLcmSystem.
    const SortedPair<GeometryId> sorted_ids(pair.id_A, pair.id_B);
    const std::string body_A = fmt::format("{}", sorted_ids.first());
    const std::string body_B = fmt::format("{}", sorted_ids.second());
    const Vector3d force = ExtractDoubleOrThrow(info.contact_force());
    const Vector3d point = ExtractDoubleOrThrow(info.contact_point());
    result->push_back({body_A, body_B, force, point});
  }
}

// N.B. This is only called if params_.delete_on_initialization_event was true.
template <typename T>
EventStatus ContactVisualizer<T>::OnInitialization(
    const Context<T>&) const {
  Delete();
  return EventStatus::Succeeded();
}

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::ContactVisualizer)
