#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_hydroelastic_contact_surface_for_viz.hpp"

namespace drake {
namespace multibody {

using geometry::GeometryId;
using internal::FullBodyName;
using Eigen::Vector3d;
using systems::Context;

namespace internal {

bool operator==(const FullBodyName& n1, const FullBodyName& n2) {
  return n1.model == n2.model && n1.body == n2.body &&
         n1.geometry == n2.geometry &&
         n1.body_name_is_unique == n2.body_name_is_unique &&
         n1.geometry_count == n2.geometry_count;
}

}  // namespace internal

namespace {

/* Wrapper for plant.GetCollisionGeometriesForBody() that can provide a
 warning that visualization would be improved by providing a geometry name
 generator. */
template <typename T>
const std::vector<GeometryId>& GetCollisionGeometriesForBody(
    const MultibodyPlant<T>& plant, const Body<T>& body,
    bool warn_for_multi_geometry_body) {
  const std::vector<GeometryId>& geometries =
      plant.GetCollisionGeometriesForBody(body);
  if (warn_for_multi_geometry_body && geometries.size() > 1) {
    static const logging::Warn log_once(
        "MultibodyPlant has at least one body '{}/{}' with multiple contact "
        "geometries. Contacts with this body may be unclear in the visualizer "
        "if contact is made with multiple geometries simultaneously. To "
        "clarify the visualization, use ConnectContactResultsToDrakeVisualizer "
        "instead of the ContactResultsToLcm constructor, and pass a SceneGraph "
        "to that function. See the documentation for ContactResultsToLcmSystem "
        "for details.",
        plant.GetModelInstanceName(body.model_instance()), body.name());
    unused(log_once);
  }
  return geometries;
}

/* The default functor that converts GeometryId to a simple stringified version.
 */
std::string id_as_label(GeometryId id) {
  return fmt::format("Id({})", id);
}

std::function<std::string(GeometryId)> make_geometry_name_lookup(
    const geometry::SceneGraph<double>& scene_graph) {
  return [&inspector = scene_graph.model_inspector()](GeometryId id) {
    return inspector.GetName(id);
  };
}

}  // namespace

template <typename T>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(
    const MultibodyPlant<T>& plant)
    : ContactResultsToLcmSystem<T>(plant, nullptr) {}

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
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(
    const MultibodyPlant<T>& plant,
    const std::function<std::string(GeometryId)>& geometry_name_lookup)
    : ContactResultsToLcmSystem<T>(true) {
  DRAKE_DEMAND(plant.is_finalized());
  const int body_count = plant.num_bodies();

  body_names_.reserve(body_count);
  const bool use_default_namer = geometry_name_lookup == nullptr;
  const std::function<std::string(GeometryId)>& namer =
      use_default_namer ? &id_as_label : geometry_name_lookup;
  for (BodyIndex i{0}; i < body_count; ++i) {
    const Body<T>& body = plant.get_body(i);
    using std::to_string;
    body_names_.push_back(body.name() + "(" + to_string(body.model_instance()) +
                          ")");
    for (auto geometry_id :
         GetCollisionGeometriesForBody(plant, body, use_default_namer)) {
      const std::string& model_name =
          plant.GetModelInstanceName(body.model_instance());
      const bool body_name_is_unique =
          plant.NumBodiesWithName(body.name()) == 1;
      // TODO(SeanCurtis-TRI): collision geometries can be added to SceneGraph
      //  after the plant has been finalized. Those geometries will not be found
      //  in this map. What *should* happen is that this should *also* be
      //  connected to SceneGraph's query object output port and it should ask
      //  scene graph about things like this when evaluating the output port.
      //  However, this is not an immediate problem for *this* system, because
      //  MultibodyPlant is authored such that if someone were to add such a
      //  geometry and it participated in collision, MultibodyPlant would have
      //  already thrown an exception in computing the contact. Until MbP gets
      //  out of the way, there's no reason to update here.
      const int collision_count =
          static_cast<int>(plant.GetCollisionGeometriesForBody(body).size());
      geometry_id_to_body_name_map_[geometry_id] = {
          model_name, body.name(), namer(geometry_id),
          body_name_is_unique, collision_count};
    }
  }
}

template <typename T>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(bool dummy)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<ContactResultsToLcmSystem>{}) {
  // Simply omitting the unused parameter `dummy` causes a linter error; cpplint
  // thinks it is a C-style cast.
  unused(dummy);
  this->set_name("ContactResultsToLcmSystem");
  contact_result_input_port_index_ =
      this->DeclareAbstractInputPort(systems::kUseDefaultName,
                                     Value<ContactResults<T>>())
          .get_index();
  message_output_port_index_ =
      this->DeclareAbstractOutputPort(
              systems::kUseDefaultName,
              &ContactResultsToLcmSystem::CalcLcmContactOutput)
          .get_index();
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

template <typename T>
void ContactResultsToLcmSystem<T>::CalcLcmContactOutput(
    const Context<T>& context, lcmt_contact_results_for_viz* output) const {
  // Get input / output.
  const auto& contact_results =
      get_contact_result_input_port().template Eval<ContactResults<T>>(context);
  auto& message = *output;

  // Time in microseconds.
  message.timestamp =
      static_cast<int64_t>(ExtractDoubleOrThrow(context.get_time()) * 1e6);
  message.num_point_pair_contacts = contact_results.num_point_pair_contacts();
  message.point_pair_contact_info.resize(message.num_point_pair_contacts);

  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    lcmt_point_pair_contact_info_for_viz& info_msg =
        message.point_pair_contact_info[i];
    info_msg.timestamp = message.timestamp;

    const PointPairContactInfo<T>& contact_info =
        contact_results.point_pair_contact_info(i);

    info_msg.body1_name = body_names_.at(contact_info.bodyA_index());
    info_msg.body2_name = body_names_.at(contact_info.bodyB_index());

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
    const FullBodyName& name1 =
        geometry_id_to_body_name_map_.at(contact_surface.id_M());
    surface_message.body1_name = name1.body;
    surface_message.model1_name = name1.model;
    surface_message.geometry1_name = name1.geometry;
    surface_message.body1_unique = name1.body_name_is_unique;
    surface_message.collision_count1 = name1.geometry_count;

    const FullBodyName& name2 =
        geometry_id_to_body_name_map_.at(contact_surface.id_N());
    surface_message.body2_name = name2.body;
    surface_message.model2_name = name2.model;
    surface_message.geometry2_name = name2.geometry;
    surface_message.body2_unique = name2.body_name_is_unique;
    surface_message.collision_count2 = name2.geometry_count;

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

systems::lcm::LcmPublisherSystem* ConnectWithNameLookup(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_results_port,
    const std::function<std::string(GeometryId)>& name_lookup,
    lcm::DrakeLcmInterface* lcm,
    std::optional<double> publish_period) {
  DRAKE_DEMAND(builder != nullptr);

  // Note: Can't use AddSystem<System> or make_unique<System> because neither
  // of those have access to the private constructor.
  ContactResultsToLcmSystem<double>* contact_to_lcm =
      builder->AddSystem(std::unique_ptr<ContactResultsToLcmSystem<double>>(
          new ContactResultsToLcmSystem<double>(multibody_plant, name_lookup)));
  contact_to_lcm->set_name("contact_to_lcm");

  // To help avoid small timesteps, use a default period that has an exact
  // representation in binary floating point (see drake#15021).
  const double default_publish_period = 1.0 / 64;
  auto contact_results_publisher = builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm, publish_period.value_or(
              default_publish_period)));
  contact_results_publisher->set_name("contact_results_publisher");

  builder->Connect(contact_results_port,
                   contact_to_lcm->get_contact_result_input_port());
  builder->Connect(*contact_to_lcm, *contact_results_publisher);

  return contact_results_publisher;
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    lcm::DrakeLcmInterface* lcm,
    std::optional<double> publish_period) {
  return ConnectWithNameLookup(
      builder, multibody_plant,
      multibody_plant.get_contact_results_output_port(),
      nullptr, lcm, publish_period);
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm,
    std::optional<double> publish_period) {
  return ConnectWithNameLookup(
      builder, multibody_plant,
      multibody_plant.get_contact_results_output_port(),
      make_geometry_name_lookup(scene_graph), lcm, publish_period);
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm,
    std::optional<double> publish_period) {
  return ConnectWithNameLookup(
      builder, multibody_plant, contact_results_port,
      nullptr, lcm, publish_period);
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const geometry::SceneGraph<double>& scene_graph,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm,
    const std::optional<double> publish_period) {
  return ConnectWithNameLookup(
      builder, multibody_plant,
      contact_results_port,
      make_geometry_name_lookup(scene_graph), lcm, publish_period);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
