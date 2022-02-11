#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <vector>

namespace drake {
namespace multibody {

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::QueryObject;
using geometry::SceneGraphInspector;
using internal::GeometryNames;
using systems::Context;
using systems::LeafSystem;
using systems::ValueProducer;

template <typename T>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem()
    : LeafSystem<T>(systems::SystemTypeTag<ContactResultsToLcmSystem>{}) {
  this->set_name("ContactResultsToLcmSystem");
  contact_result_input_port_index_ =
      this->DeclareAbstractInputPort(
              "u0", Value<ContactResults<T>>())
          .get_index();
  query_object_input_port_index_ =
      this->DeclareAbstractInputPort(
              "query_object", Value<QueryObject<T>>())
          .get_index();
  message_output_port_index_ =
      this->DeclareAbstractOutputPort(
              "y0", &ContactResultsToLcmSystem::CalcLcmContactOutput)
          .get_index();
  geometry_names_scratch_index_ =
      this->DeclareCacheEntry(
              "geometry_names", ValueProducer(
                  GeometryNames(), &ValueProducer::NoopCalc),
              {this->nothing_ticket()})
          .cache_index();
}

template <typename T>
template <typename U>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(
    const ContactResultsToLcmSystem<U>&)
    : ContactResultsToLcmSystem<T>() {}

template <typename T>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(
    const MultibodyPlant<T>&)
    : ContactResultsToLcmSystem<T>() {}

template <typename T>
const systems::InputPort<T>&
ContactResultsToLcmSystem<T>::get_contact_result_input_port() const {
  return this->get_input_port(contact_result_input_port_index_);
}

template <typename T>
const systems::InputPort<T>&
ContactResultsToLcmSystem<T>::get_query_object_input_port() const {
  return this->get_input_port(query_object_input_port_index_);
}

template <typename T>
const systems::OutputPort<T>&
ContactResultsToLcmSystem<T>::get_lcm_message_output_port() const {
  return this->get_output_port(message_output_port_index_);
}

template <typename T>
void ContactResultsToLcmSystem<T>::CalcLcmContactOutput(
    const Context<T>& context, lcmt_contact_results_for_viz* output) const {
  // Obtain the list of contacts.
  const ContactResults<T>& contact_results =
      get_contact_result_input_port().template Eval<ContactResults<T>>(context);

  // Freshen the dictionary of contact names for the proximity geometries.
  const MultibodyPlant<T>* const plant = contact_results.plant();
  DRAKE_THROW_UNLESS(plant != nullptr);
  GeometryNames& geometry_names =
      this->get_cache_entry(geometry_names_scratch_index_).
      get_mutable_cache_entry_value(context).
      template GetMutableValueOrThrow<GeometryNames>();
  if (geometry_names.entries().empty()) {
    if (get_query_object_input_port().HasValue(context)) {
      const QueryObject<T>& query_object =
          get_query_object_input_port().template Eval<QueryObject<T>>(context);
      geometry_names.ResetFull(*plant, query_object.inspector());
    } else {
      geometry_names.ResetBasic(*plant);
    }
  }

  // Update the output message's values.
  const double timestamp = ExtractDoubleOrThrow(context.get_time());
  output->timestamp = static_cast<int64_t>(timestamp * 1e6);
  ContactResultsToLcm(geometry_names, contact_results, output);
}

namespace {

// Writes a Vector3<T> to an array of doubles (with a conversion to double as
// necessary).
// @pre dest points to a block of memory sufficient to hold three doubles.
template <typename T>
static void write_double3(const Vector3<T>& src, double* dest) {
  dest[0] = ExtractDoubleOrThrow(src(0));
  dest[1] = ExtractDoubleOrThrow(src(1));
  dest[2] = ExtractDoubleOrThrow(src(2));
}

}  // namespace

namespace internal {

template <typename T>
void ContactResultsToLcm(
    const GeometryNames& geometry_names,
    const ContactResults<T>& contact_results,
    lcmt_contact_results_for_viz* output) {
  auto& message = *output;

  message.timestamp = 0;
  message.num_point_pair_contacts = contact_results.num_point_pair_contacts();
  message.point_pair_contact_info.resize(message.num_point_pair_contacts);

  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    lcmt_point_pair_contact_info_for_viz& info_msg =
        message.point_pair_contact_info[i];
    info_msg.timestamp = message.timestamp;

    const PointPairContactInfo<T>& contact_info =
        contact_results.point_pair_contact_info(i);

    const auto& name1 = geometry_names.Find(contact_info.point_pair().id_A);
    const auto& name2 = geometry_names.Find(contact_info.point_pair().id_B);
    info_msg.body1_name = fmt::format("{}({})",
        name1.body_name, name1.model_instance_name);
    info_msg.body2_name = fmt::format("{}({})",
        name2.body_name, name2.model_instance_name);

    write_double3(contact_info.contact_point(), info_msg.contact_point);
    write_double3(contact_info.contact_force(), info_msg.contact_force);
    write_double3(contact_info.point_pair().nhat_BA_W, info_msg.normal);
  }

  message.num_hydroelastic_contacts =
      contact_results.num_hydroelastic_contacts();
  message.hydroelastic_contacts.resize(message.num_hydroelastic_contacts);

  for (int i = 0; i < contact_results.num_hydroelastic_contacts(); ++i) {
    const HydroelasticContactInfo<T>& hydroelastic_contact_info =
        contact_results.hydroelastic_contact_info(i);
    const geometry::ContactSurface<T>& contact_surface =
        hydroelastic_contact_info.contact_surface();

    lcmt_hydroelastic_contact_surface_for_viz& surface_message =
        message.hydroelastic_contacts[i];

    // Get the two body names.
    const auto& name1 = geometry_names.Find(contact_surface.id_M());
    surface_message.body1_name = name1.body_name;
    surface_message.model1_name = name1.model_instance_name;
    surface_message.geometry1_name = name1.geometry_name.value_or(
        fmt::format("Id({})", contact_surface.id_M()));
    surface_message.body1_unique = name1.body_name_is_unique_within_plant;
    surface_message.collision_count1 =
        name1.is_sole_geometry_within_body ? 1 : 2;

    const auto& name2 = geometry_names.Find(contact_surface.id_N());
    surface_message.body2_name = name2.body_name;
    surface_message.model2_name = name2.model_instance_name;
    surface_message.geometry2_name = name2.geometry_name.value_or(
        fmt::format("Id({})", contact_surface.id_N()));
    surface_message.body2_unique = name2.body_name_is_unique_within_plant;
    surface_message.collision_count2 =
        name2.is_sole_geometry_within_body ? 1 : 2;

    // Resultant force quantities.
    write_double3(contact_surface.centroid(), surface_message.centroid_W);
    write_double3(hydroelastic_contact_info.F_Ac_W().translational(),
                  surface_message.force_C_W);
    write_double3(hydroelastic_contact_info.F_Ac_W().rotational(),
                  surface_message.moment_C_W);

    // Write all quadrature points on the contact surface.
    const std::vector<HydroelasticQuadraturePointData<T>>&
        quadrature_point_data =
            hydroelastic_contact_info.quadrature_point_data();
    surface_message.num_quadrature_points = quadrature_point_data.size();
    surface_message.quadrature_point_data.resize(
        surface_message.num_quadrature_points);

    for (int j = 0; j < surface_message.num_quadrature_points; ++j) {
      lcmt_hydroelastic_quadrature_per_point_data_for_viz& quad_data_message =
          surface_message.quadrature_point_data[j];
      write_double3(quadrature_point_data[j].p_WQ, quad_data_message.p_WQ);
      write_double3(quadrature_point_data[j].vt_BqAq_W,
                    quad_data_message.vt_BqAq_W);
      write_double3(quadrature_point_data[j].traction_Aq_W,
                    quad_data_message.traction_Aq_W);
    }

    // Now build the mesh.
    const int num_vertices = contact_surface.num_vertices();
    surface_message.num_vertices = num_vertices;
    surface_message.p_WV.resize(num_vertices);
    surface_message.pressure.resize(num_vertices);

    if (contact_surface.is_triangle()) {
      const auto& mesh_W = contact_surface.tri_mesh_W();
      const auto& e_MN_W = contact_surface.tri_e_MN();

      // Write vertices and per vertex pressure values.
      for (int v = 0; v < num_vertices; ++v) {
        const Vector3d p_WV = ExtractDoubleOrThrow(mesh_W.vertex(v));
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
      // TODO(DamrongGuoy) Make sure the unit tests cover this specific code
      //  path. It is currently uncovered.
      const auto& mesh_W = contact_surface.poly_mesh_W();
      const auto& e_MN_W = contact_surface.poly_e_MN();

      // Write vertices and per vertex pressure values.
      for (int v = 0; v < num_vertices; ++v) {
        const Vector3d p_WV = ExtractDoubleOrThrow(mesh_W.vertex(v));
        surface_message.p_WV[v] = {p_WV.x(), p_WV.y(), p_WV.z()};
        surface_message.pressure[v] =
            ExtractDoubleOrThrow(e_MN_W.EvaluateAtVertex(v));
      }

      surface_message.poly_data_int_count = mesh_W.face_data().size();
      surface_message.poly_data = mesh_W.face_data();
    }
  }
}

}  // namespace internal

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm,
    std::optional<double> publish_period) {
  return ConnectContactResultsToDrakeVisualizer(
      builder, plant, scene_graph, plant.get_contact_results_output_port(),
      lcm, publish_period);
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm,
    std::optional<double> publish_period) {
  DRAKE_DEMAND(builder != nullptr);
  unused(scene_graph);

  auto converter = builder->AddSystem<ContactResultsToLcmSystem<double>>();
  converter->set_name("contact_to_lcm");

  // Add the contact results connection.
  builder->Connect(
      contact_results_port,
      converter->get_contact_result_input_port());

  // Add the query connection (if the plant has a SceneGraph).
  builder->ConnectToSame(
      plant.get_geometry_query_input_port(),
      converter->get_query_object_input_port());

  // To help avoid small timesteps, use a default period that has an exact
  // representation in binary floating point (see drake#15021).
  const double default_publish_period = 1.0 / 64;
  auto publisher = builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm, publish_period.value_or(
              default_publish_period)));
  publisher->set_name("contact_results_publisher");
  builder->Connect(
      converter->get_lcm_message_output_port(),
      publisher->get_input_port());

  return publisher;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
