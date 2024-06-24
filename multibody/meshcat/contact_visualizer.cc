#include "drake/multibody/meshcat/contact_visualizer.h"

#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/extract_double.h"
#include "drake/geometry/meshcat_graphviz.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/meshcat/hydroelastic_contact_visualizer.h"
#include "drake/multibody/meshcat/point_contact_visualizer.h"
#include "drake/multibody/plant/contact_results.h"

namespace drake {
namespace multibody {
namespace meshcat {

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::Meshcat;
using geometry::QueryObject;
using meshcat::internal::HydroelasticContactVisualizer;
using meshcat::internal::HydroelasticContactVisualizerItem;
using meshcat::internal::PointContactVisualizer;
using meshcat::internal::PointContactVisualizerItem;
using multibody::internal::GeometryNames;
using systems::CacheEntry;
using systems::Context;
using systems::DiagramBuilder;
using systems::EventStatus;
using systems::InputPort;
using systems::OutputPort;
using systems::OutputPortIndex;
using systems::System;
using systems::ValueProducer;

template <typename T>
ContactVisualizer<T>::ContactVisualizer(std::shared_ptr<Meshcat> meshcat,
                                        ContactVisualizerParams params)
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

  ContactVisualizerParams hydro_params = params_;
  hydro_params.prefix += "/hydroelastic";
  hydroelastic_visualizer_ = std::make_unique<HydroelasticContactVisualizer>(
      meshcat_, std::move(hydro_params));

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

  const InputPort<T>& query_object =
      this->DeclareAbstractInputPort("query_object", Value<QueryObject<T>>());
  query_object_input_port_ = query_object.get_index();

  const CacheEntry& geometry_names = this->DeclareCacheEntry(
      "geometry_names",
      ValueProducer(GeometryNames(), &ValueProducer::NoopCalc),
      {this->nothing_ticket()});
  geometry_names_scratch_ = geometry_names.cache_index();

  const CacheEntry& point_contacts = this->DeclareCacheEntry(
      "point_contacts", &ContactVisualizer::CalcPointContacts,
      {contact_results.ticket()});
  point_contacts_cache_ = point_contacts.cache_index();

  const CacheEntry& hydroelastic_contacts = this->DeclareCacheEntry(
      "hydroelastic_contacts", &ContactVisualizer::CalcHydroelasticContacts,
      {contact_results.ticket()});
  hydroelastic_contacts_cache_ = hydroelastic_contacts.cache_index();
}

template <typename T>
template <typename U>
ContactVisualizer<T>::ContactVisualizer(const ContactVisualizer<U>& other)
    : ContactVisualizer(other.meshcat_, other.params_) {}

template <typename T>
ContactVisualizer<T>::~ContactVisualizer() = default;

template <typename T>
void ContactVisualizer<T>::Delete() const {
  point_visualizer_->Delete();
  hydroelastic_visualizer_->Delete();
  meshcat_->Delete(params_.prefix);
}

template <typename T>
const ContactVisualizer<T>& ContactVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const MultibodyPlant<T>& plant,
    std::shared_ptr<Meshcat> meshcat, ContactVisualizerParams params) {
  DRAKE_THROW_UNLESS(builder != nullptr);

  // Delegate to another overload.
  const auto& result =
      AddToBuilder(builder, plant.get_contact_results_output_port(),
                   std::move(meshcat), std::move(params));

  // Add the query connection (if the plant has a SceneGraph).
  builder->ConnectToSame(plant.get_geometry_query_input_port(),
                         result.query_object_input_port());

  return result;
}

template <typename T>
const ContactVisualizer<T>& ContactVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const OutputPort<T>& contact_results_port,
    const OutputPort<T>& query_object_port, std::shared_ptr<Meshcat> meshcat,
    ContactVisualizerParams params) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  auto& result = *builder->template AddSystem<ContactVisualizer<T>>(
      std::move(meshcat), std::move(params));
  const std::string aspirational_name = "meshcat_contact_visualizer";
  if (!builder->HasSubsystemNamed(aspirational_name)) {
    result.set_name(aspirational_name);
  }
  builder->Connect(contact_results_port, result.contact_results_input_port());
  builder->Connect(query_object_port, result.query_object_input_port());
  return result;
}

template <typename T>
const ContactVisualizer<T>& ContactVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const OutputPort<T>& contact_results_port,
    std::shared_ptr<Meshcat> meshcat, ContactVisualizerParams params) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  auto& result = *builder->template AddSystem<ContactVisualizer<T>>(
      std::move(meshcat), std::move(params));
  const std::string aspirational_name = "meshcat_contact_visualizer";
  if (!builder->HasSubsystemNamed(aspirational_name)) {
    result.set_name(aspirational_name);
  }
  builder->Connect(contact_results_port, result.contact_results_input_port());
  return result;
}

template <typename T>
EventStatus ContactVisualizer<T>::UpdateMeshcat(
    const Context<T>& context) const {
  double time = ExtractDoubleOrThrow(context.get_time());
  const auto& point_contacts =
      this->get_cache_entry(point_contacts_cache_)
          .template Eval<std::vector<PointContactVisualizerItem>>(context);
  point_visualizer_->Update(time, point_contacts);

  const auto& hydroelastic_contacts =
      this->get_cache_entry(hydroelastic_contacts_cache_)
          .template Eval<std::vector<HydroelasticContactVisualizerItem>>(
              context);
  hydroelastic_visualizer_->Update(time, hydroelastic_contacts);
  return EventStatus::Succeeded();
}

template <typename T>
const GeometryNames& ContactVisualizer<T>::GetGeometryNames(
    const Context<T>& context, const MultibodyPlant<T>* plant) const {
  // TODO(joemasterjohn): Upgrade `geometry_names` to a real cache entry if/when
  // MultibodyPlant adds the capability to add/remove geometries after finalize.
  GeometryNames& geometry_names =
      this->get_cache_entry(geometry_names_scratch_)
          .get_mutable_cache_entry_value(context)
          .template GetMutableValueOrThrow<GeometryNames>();
  if (geometry_names.entries().empty()) {
    if (query_object_input_port().HasValue(context)) {
      const QueryObject<T>& query_object =
          query_object_input_port().template Eval<QueryObject<T>>(context);
      geometry_names.ResetFull(*plant, query_object.inspector());
    } else {
      geometry_names.ResetBasic(*plant);
    }
  }

  return geometry_names;
}

template <typename T>
void ContactVisualizer<T>::CalcHydroelasticContacts(
    const Context<T>& context,
    std::vector<HydroelasticContactVisualizerItem>* result) const {
  result->clear();

  // Obtain the list of contacts.
  const ContactResults<T>& contact_results =
      contact_results_input_port().template Eval<ContactResults<T>>(context);

  // Freshen the dictionary of contact names for the proximity geometries.
  const MultibodyPlant<T>* const plant = contact_results.plant();
  DRAKE_THROW_UNLESS(plant != nullptr);
  const GeometryNames& geometry_names = this->GetGeometryNames(context, plant);

  result->reserve(contact_results.num_hydroelastic_contacts());

  // Update our output vector of items.
  for (int ci = 0; ci < contact_results.num_hydroelastic_contacts(); ++ci) {
    const HydroelasticContactInfo<T>& info =
        contact_results.hydroelastic_contact_info(ci);

    const geometry::ContactSurface<T>& contact_surface = info.contact_surface();

    const SortedPair<GeometryId> sorted_ids(contact_surface.id_M(),
                                            contact_surface.id_N());
    std::string body_A = geometry_names.GetFullName(sorted_ids.first(), ".");
    std::string body_B = geometry_names.GetFullName(sorted_ids.second(), ".");

    Vector3d centroid_W = ExtractDoubleOrThrow(contact_surface.centroid());
    Vector3d force_C_W = ExtractDoubleOrThrow(info.F_Ac_W().translational());
    Vector3d moment_C_W = ExtractDoubleOrThrow(info.F_Ac_W().rotational());

    if (contact_surface.is_triangle()) {
      const auto& mesh = contact_surface.tri_mesh_W();

      Eigen::Matrix3Xd vertices(3, contact_surface.num_vertices());
      for (int i = 0; i < contact_surface.num_vertices(); ++i) {
        vertices.col(i) = ExtractDoubleOrThrow(mesh.vertex(i));
      }
      Eigen::Matrix3Xi faces(3, mesh.num_triangles());
      for (int i = 0; i < mesh.num_triangles(); ++i) {
        const auto& e = mesh.element(i);
        for (int j = 0; j < 3; ++j) {
          faces(j, i) = e.vertex(j);
        }
      }

      const std::vector<T>& field_pressures =
          contact_surface.tri_e_MN().values();
      const VectorX<T> pressure_T = EigenMapView(field_pressures);
      const Eigen::VectorXd pressure = ExtractDoubleOrThrow(pressure_T);
      result->emplace_back(std::move(body_A), std::move(body_B), centroid_W,
                           force_C_W, moment_C_W, vertices, faces, pressure);
    } else {
      const auto& mesh = contact_surface.poly_mesh_W();
      Eigen::Matrix3Xd vertices(3, contact_surface.num_vertices());
      for (int i = 0; i < contact_surface.num_vertices(); ++i) {
        vertices.col(i) = ExtractDoubleOrThrow(mesh.vertex(i));
      }

      int num_triangles = 0;
      for (int i = 0; i < mesh.num_elements(); ++i) {
        num_triangles += mesh.element(i).num_vertices() - 2;
      }

      Eigen::Matrix3Xi faces(3, num_triangles);
      int f_index = 0;
      for (int i = 0; i < mesh.num_elements(); ++i) {
        const auto& e = mesh.element(i);
        for (int j = 1; j < e.num_vertices() - 1; ++j) {
          faces(0, f_index) = e.vertex(0);
          faces(1, f_index) = e.vertex(j);
          faces(2, f_index) = e.vertex(j + 1);
          ++f_index;
        }
      }

      const std::vector<T>& field_pressures =
          contact_surface.poly_e_MN().values();
      const VectorX<T> pressure_T = EigenMapView(field_pressures);
      const Eigen::VectorXd pressure = ExtractDoubleOrThrow(pressure_T);

      result->emplace_back(std::move(body_A), std::move(body_B), centroid_W,
                           force_C_W, moment_C_W, std::move(vertices),
                           std::move(faces), std::move(pressure));
    }
  }
}

template <typename T>
void ContactVisualizer<T>::CalcPointContacts(
    const Context<T>& context,
    std::vector<PointContactVisualizerItem>* result) const {
  result->clear();

  // Obtain the list of contacts.
  const ContactResults<T>& contact_results =
      contact_results_input_port().template Eval<ContactResults<T>>(context);

  // Freshen the dictionary of contact names for the proximity geometries.
  const MultibodyPlant<T>* const plant = contact_results.plant();
  DRAKE_THROW_UNLESS(plant != nullptr);
  const GeometryNames& geometry_names = this->GetGeometryNames(context, plant);

  result->reserve(contact_results.num_point_pair_contacts());

  // Update our output vector of items.
  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    const PointPairContactInfo<T>& info =
        contact_results.point_pair_contact_info(i);
    const geometry::PenetrationAsPointPair<T>& pair = info.point_pair();
    const SortedPair<GeometryId> sorted_ids(pair.id_A, pair.id_B);
    std::string body_A = geometry_names.GetFullName(sorted_ids.first(), ".");
    std::string body_B = geometry_names.GetFullName(sorted_ids.second(), ".");
    Vector3d force = ExtractDoubleOrThrow(info.contact_force());
    Vector3d point = ExtractDoubleOrThrow(info.contact_point());
    result->emplace_back(std::move(body_A), std::move(body_B), force, point);
  }
}

// N.B. This is only called if params_.delete_on_initialization_event was true.
template <typename T>
EventStatus ContactVisualizer<T>::OnInitialization(const Context<T>&) const {
  Delete();
  return EventStatus::Succeeded();
}

template <typename T>
typename systems::LeafSystem<T>::GraphvizFragment
ContactVisualizer<T>::DoGetGraphvizFragment(
    const typename systems::LeafSystem<T>::GraphvizFragmentParams& params)
    const {
  geometry::internal::MeshcatGraphviz meshcat_graphviz(params_.prefix,
                                                       /* subscribe = */ false);
  return meshcat_graphviz.DecorateResult(
      systems::LeafSystem<T>::DoGetGraphvizFragment(
          meshcat_graphviz.DecorateParams(params)));
}

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::ContactVisualizer);
