#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <memory>

#include "drake/lcmt_contact_results_for_viz.hpp"

namespace drake {
namespace multibody {

using systems::Context;

template <typename T>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(
    const MultibodyPlant<T>& plant)
    : systems::LeafSystem<T>() {
  DRAKE_DEMAND(plant.is_finalized());
  const int body_count = plant.num_bodies();

  body_names_.reserve(body_count);
  using std::to_string;
  for (BodyIndex i{0}; i < body_count; ++i) {
    const Body<T>& body = plant.get_body(i);
    body_names_.push_back(body.name() + "(" + to_string(body.model_instance()) +
                          ")");
    const std::vector<geometry::GeometryId>& geometries =
        plant.GetCollisionGeometriesForBody(body);
    for (auto geometry_id : geometries)
      geometry_id_to_body_index_map_[geometry_id] = i;
  }

  this->set_name("ContactResultsToLcmSystem");
  // Must be the first declared input port to be compatible with the constexpr
  // declaration of contact_result_input_port_index_.
  this->DeclareAbstractInputPort(Value<ContactResults<T>>());
  this->DeclareAbstractOutputPort(
      &ContactResultsToLcmSystem::CalcLcmContactOutput);
}

template <typename T>
const systems::InputPort<T>&
ContactResultsToLcmSystem<T>::get_contact_result_input_port() const {
  return this->get_input_port(contact_result_input_port_index_);
}

template <typename T>
const systems::OutputPort<T>&
ContactResultsToLcmSystem<T>::get_lcm_message_output_port() const {
  return this->get_output_port(message_output_port_index_);
}

template <typename T>
void ContactResultsToLcmSystem<T>::CalcLcmContactOutput(
    const Context<T>& context, lcmt_contact_results_for_viz* output) const {
  // Get input / output.
  const auto& contact_results = get_contact_result_input_port().
      template Eval<ContactResults<T>>(context);
  auto& msg = *output;

  // Time in microseconds.
  msg.timestamp = static_cast<int64_t>(
      ExtractDoubleOrThrow(context.get_time()) * 1e6);
  msg.num_point_pair_contacts = contact_results.num_point_pair_contacts();
  msg.point_pair_contact_info.resize(msg.num_point_pair_contacts);
  msg.num_hydroelastic_contact_surfaces =
      contact_results.num_hydroelastic_contact_elements();

  auto write_double3 = [](const Vector3<T>& src, double* dest) {
    dest[0] = ExtractDoubleOrThrow(src(0));
    dest[1] = ExtractDoubleOrThrow(src(1));
    dest[2] = ExtractDoubleOrThrow(src(2));
  };

  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    lcmt_point_pair_contact_info_for_viz& info_msg =
        msg.point_pair_contact_info[i];
    info_msg.timestamp = msg.timestamp;

    const PointPairContactInfo<T>& contact_info =
        contact_results.point_pair_contact_info(i);

    info_msg.body1_name = body_names_.at(contact_info.bodyA_index());
    info_msg.body2_name = body_names_.at(contact_info.bodyB_index());

    write_double3(contact_info.contact_point(), info_msg.contact_point);
    write_double3(contact_info.contact_force(), info_msg.contact_force);
    write_double3(contact_info.point_pair().nhat_BA_W, info_msg.normal);
  }

  for (int i = 0; i < contact_results.num_hydroelastic_contact_elements();
      ++i) {
    lcmt_hydroelastic_contact_surface_for_viz& surface_msg =
        msg.hydroelastic_contact_surfaces[i];
    surface_msg.timestamp = msg.timestamp;

    const HydroelasticContactInfo<T>& hydroelastic_contact_info =
        contact_results.hydroelastic_contact_info(i);

    // Get the two body names.
    surface_msg.body1_name = body_names_.at(
        geometry_id_to_body_index_map_.at(
            hydroelastic_contact_info.contact_surface().id_M()));
    surface_msg.body2_name = body_names_.at(
        geometry_id_to_body_index_map_.at(
            hydroelastic_contact_info.contact_surface().id_N()));

    const auto& mesh = hydroelastic_contact_info.contact_surface().mesh();
    const int num_triangles = mesh.num_faces();
    surface_msg.num_triangles = num_triangles;
    surface_msg.triangles.resize(surface_msg.num_triangles);

    // Loop through each contact triangle on the contact surface.
    for (geometry::SurfaceFaceIndex j(0); j < surface_msg.num_triangles; ++j) {
      lcmt_hydroelastic_contact_surface_tri_for_viz& tri_msg =
          surface_msg.triangles[j];
      tri_msg.timestamp = msg.timestamp;
       tri_msg.timestamp = msg.timestamp;

      // Get the three vertices.
      const auto& face = mesh.element(j);
      const geometry::SurfaceVertex<T>& vA = mesh.vertex(face.vertex(0));
      const geometry::SurfaceVertex<T>& vB = mesh.vertex(face.vertex(1));
      const geometry::SurfaceVertex<T>& vC = mesh.vertex(face.vertex(2));

      write_double3(vA.r_MV(), tri_msg.a);
      write_double3(vB.r_MV(), tri_msg.b);
      write_double3(vC.r_MV(), tri_msg.c);
    }
  }
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    lcm::DrakeLcmInterface* lcm) {
  return ConnectContactResultsToDrakeVisualizer(
      builder, multibody_plant,
      multibody_plant.get_contact_results_output_port(), lcm);
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm) {
  DRAKE_DEMAND(builder != nullptr);

  auto contact_to_lcm =
      builder->template AddSystem<ContactResultsToLcmSystem<double>>(
          multibody_plant);
  contact_to_lcm->set_name("contact_to_lcm");

  auto contact_results_publisher = builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm, 1.0 / 60 /* publish period */));
  contact_results_publisher->set_name("contact_results_publisher");

  builder->Connect(contact_results_port, contact_to_lcm->get_input_port(0));
  builder->Connect(contact_to_lcm->get_output_port(0),
                   contact_results_publisher->get_input_port());

  return contact_results_publisher;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
