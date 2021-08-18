#include "drake/multibody/plant/contact_results_to_lcm.h"

#include <memory>

#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/lcmt_contact_results_for_viz.hpp"

namespace drake {
namespace multibody {

using geometry::GeometryId;
using internal::FullBodyName;
using Eigen::Vector3d;
using systems::Context;

namespace internal {

bool operator==(const FullBodyName& n1, const FullBodyName& n2) {
  return n1.model == n2.model && n1.body == n2.body &&
         n1.geometry == n2.geometry;
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
        "clarify the visualization, pass in a geometry naming functor to the "
        "constructor. See the documentation for ContactResultsToLcmSystem for "
        "details.",
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
      geometry_id_to_body_name_map_[geometry_id] = {model_name, body.name(),
                                                    namer(geometry_id)};
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

template <typename T>
void ContactResultsToLcmSystem<T>::CalcLcmContactOutput(
    const Context<T>& context, lcmt_contact_results_for_viz* output) const {
  // Get input / output.
  const auto& contact_results =
      get_contact_result_input_port().template Eval<ContactResults<T>>(context);
  // TODO(SeanCurtis-TRI): Here, and below, the abbreviation "msg" is not
  //  style guide-compliant.
  auto& msg = *output;

  // Time in microseconds.
  msg.timestamp =
      static_cast<int64_t>(ExtractDoubleOrThrow(context.get_time()) * 1e6);
  msg.num_point_pair_contacts = contact_results.num_point_pair_contacts();
  msg.point_pair_contact_info.resize(msg.num_point_pair_contacts);
  msg.num_hydroelastic_contacts = contact_results.num_hydroelastic_contacts();
  msg.hydroelastic_contacts.resize(msg.num_hydroelastic_contacts);

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

  // TODO(DamrongGuoy): Refactor the following long `for` loop into a function.

  // Here we enable the standard triangulated contact patches, so the unit
  // test //multibody/plant:contact_results_to_lcm_test can pass.
  // However, you might want to switch `#if 1` to `#if 0` when visualizing
  // the polygonal contact patches instead of the triangulated contact patches.
#if 1
  for (int i = 0; i < contact_results.num_hydroelastic_contacts(); ++i) {
    lcmt_hydroelastic_contact_surface_for_viz& surface_msg =
        msg.hydroelastic_contacts[i];

    const HydroelasticContactInfo<T>& hydroelastic_contact_info =
        contact_results.hydroelastic_contact_info(i);
    const std::vector<HydroelasticQuadraturePointData<T>>&
        quadrature_point_data =
            hydroelastic_contact_info.quadrature_point_data();

    // Get the two body names.
    const FullBodyName& name1 = geometry_id_to_body_name_map_.at(
        hydroelastic_contact_info.contact_surface().id_M());
    surface_msg.body1_name = name1.body;
    surface_msg.model1_name = name1.model;
    surface_msg.geometry1_name = name1.geometry;
    const FullBodyName& name2 = geometry_id_to_body_name_map_.at(
        hydroelastic_contact_info.contact_surface().id_N());
    surface_msg.body2_name = name2.body;
    surface_msg.model2_name = name2.model;
    surface_msg.geometry2_name = name2.geometry;

    const geometry::ContactSurface<T>& contact_surface =
        hydroelastic_contact_info.contact_surface();
    const geometry::SurfaceMesh<T>& mesh_W = contact_surface.mesh_W();
    surface_msg.num_triangles = mesh_W.num_faces();
    surface_msg.triangles.resize(surface_msg.num_triangles);
    surface_msg.num_quadrature_points = quadrature_point_data.size();
    surface_msg.quadrature_point_data.resize(surface_msg.num_quadrature_points);

    write_double3(contact_surface.mesh_W().centroid(), surface_msg.centroid_W);
    write_double3(hydroelastic_contact_info.F_Ac_W().translational(),
                  surface_msg.force_C_W);
    write_double3(hydroelastic_contact_info.F_Ac_W().rotational(),
                  surface_msg.moment_C_W);

    // Write all quadrature points on the contact surface.
    const int num_quadrature_points_per_tri =
        surface_msg.num_quadrature_points / surface_msg.num_triangles;
    for (int j = 0; j < surface_msg.num_quadrature_points; ++j) {
      // Verify the ordering is consistent with that advertised.
      DRAKE_DEMAND(quadrature_point_data[j].face_index ==
                   j / num_quadrature_points_per_tri);

      lcmt_hydroelastic_quadrature_per_point_data_for_viz& quad_data_msg =
          surface_msg.quadrature_point_data[j];
      write_double3(quadrature_point_data[j].p_WQ, quad_data_msg.p_WQ);
      write_double3(quadrature_point_data[j].vt_BqAq_W,
                    quad_data_msg.vt_BqAq_W);
      write_double3(quadrature_point_data[j].traction_Aq_W,
                    quad_data_msg.traction_Aq_W);
    }

    // Loop through each contact triangle on the contact surface.
    const auto& field = contact_surface.e_MN();
    for (geometry::SurfaceFaceIndex j(0); j < surface_msg.num_triangles; ++j) {
      lcmt_hydroelastic_contact_surface_tri_for_viz& tri_msg =
          surface_msg.triangles[j];

      // Get the three vertices.
      const auto& face = mesh_W.element(j);
      const geometry::SurfaceVertex<T>& vA = mesh_W.vertex(face.vertex(0));
      const geometry::SurfaceVertex<T>& vB = mesh_W.vertex(face.vertex(1));
      const geometry::SurfaceVertex<T>& vC = mesh_W.vertex(face.vertex(2));

      write_double3(vA.r_MV(), tri_msg.p_WA);
      write_double3(vB.r_MV(), tri_msg.p_WB);
      write_double3(vC.r_MV(), tri_msg.p_WC);

      // Record the pressures.
      tri_msg.pressure_A =
          ExtractDoubleOrThrow(field.EvaluateAtVertex(face.vertex(0)));
      tri_msg.pressure_B =
          ExtractDoubleOrThrow(field.EvaluateAtVertex(face.vertex(1)));
      tri_msg.pressure_C =
          ExtractDoubleOrThrow(field.EvaluateAtVertex(face.vertex(2)));
    }
  }
#endif

  // At the moment, some code (plane or halfspace contact) only compute
  // triangulated contact surfaces without polygonal contact surfaces.
  // The other code (mesh_intersection) compute both. The number of
  // hydroleastic contact patches is therefore the upper bound of the
  // polygonal contact patches.
  const int max_num_hydroelastic_poly_contacts =
      contact_results.num_hydroelastic_contacts();
  // We will count how many polygonal contact patches we have in the `for` loop.
  msg.num_hydroelastic_poly_contacts = 0;
  msg.hydroelastic_poly_contacts.resize(max_num_hydroelastic_poly_contacts);

  // TODO(DamrongGuoy): Refactor the following long `for` loop into a function.

  for (int s = 0; s < max_num_hydroelastic_poly_contacts; ++s) {
    lcmt_hydroelastic_poly_contact_surface_for_viz& poly_message =
        msg.hydroelastic_poly_contacts[s];

    const HydroelasticContactInfo<T>& hydroelastic_contact_info =
        contact_results.hydroelastic_contact_info(s);

    // Get the two body names.
    const FullBodyName& name1 = geometry_id_to_body_name_map_.at(
        hydroelastic_contact_info.contact_surface().id_M());
    poly_message.body1_name =
        "Poly_" + name1.body + "_" + name1.model + "_" + name1.geometry;
    const FullBodyName& name2 = geometry_id_to_body_name_map_.at(
        hydroelastic_contact_info.contact_surface().id_N());
    poly_message.body2_name =
        "Poly_" + name2.body + "_" + name2.model + "_" + name2.geometry;

    const geometry::ContactSurface<T>& surface =
        hydroelastic_contact_info.contact_surface();

    write_double3(Vector3<double>(0.0, 0, 0), poly_message.force_C_W);
    write_double3(Vector3<double>(0, 0, 0), poly_message.moment_C_W);

    if (!surface.HasPolygonalMesh()) {
      continue;
    }
    msg.num_hydroelastic_poly_contacts++;

    const geometry::PolygonalSurfaceMesh<T>& polygonal_mesh_W =
        surface.polygonal_mesh_W();
    const int num_vertices = polygonal_mesh_W.num_vertices();

    poly_message.num_vertices = num_vertices;
    poly_message.p_WV.resize(num_vertices);
    poly_message.pressure.resize(num_vertices);
    std::vector<Vector3d> p_WVs(num_vertices);
    for (int i = 0; i < num_vertices; ++i) {
      const geometry::SurfaceVertexIndex v(i);
      double xyz[3];
      write_double3(polygonal_mesh_W.vertex(v).r_MV(), xyz);
      double pressure = ExtractDoubleOrThrow(polygonal_mesh_W.vertex_value(v));

      p_WVs[i] = Vector3d(xyz);
      poly_message.pressure[i] = pressure;
    }
    // This is the actual centroid of the fake mesh due to the symmetry of
    // the two quads.
    Vector3<double> centroid_W = (p_WVs[0] + 2 * p_WVs[1] + 3 * p_WVs[2] +
        3 * p_WVs[3] + 2 * p_WVs[4] + p_WVs[5]) /
        12.0;
    write_double3(centroid_W, poly_message.centroid_W);
    for (int i = 0; i < poly_message.num_vertices; ++i) {
      const auto& p_WV = p_WVs[i];
      poly_message.p_WV[i] = {p_WV.x(), p_WV.y(), p_WV.z()};
    }

    const int num_polygons = polygonal_mesh_W.num_faces();
    // Start as num_polygons + summation of num_vertices_of_polygon later.
    int poly_data_int_count = num_polygons;
    for (const geometry::PolygonalSurfaceFace& face :
        polygonal_mesh_W.faces()) {
      poly_data_int_count += face.num_vertices();
    }
    std::vector<int> poly_data;
    poly_data.reserve(poly_data_int_count);
    for (const geometry::PolygonalSurfaceFace& face :
        polygonal_mesh_W.faces()) {
      poly_data.emplace_back(face.num_vertices());
      for (const geometry::SurfaceVertexIndex v : face.vertices()) {
        poly_data.emplace_back(v);
      }
    }
    poly_message.poly_data_int_count = poly_data_int_count;
    poly_message.poly_data = poly_data;
  }
  msg.hydroelastic_poly_contacts.resize(msg.num_hydroelastic_poly_contacts);
}

systems::lcm::LcmPublisherSystem* ConnectWithNameLookup(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_results_port,
    const std::function<std::string(GeometryId)>& name_lookup,
    lcm::DrakeLcmInterface* lcm) {
  DRAKE_DEMAND(builder != nullptr);

  // Note: Can't use AddSystem<System> or make_unique<System> because neither
  // of those have access to the private constructor.
  ContactResultsToLcmSystem<double>* contact_to_lcm =
      builder->AddSystem(std::unique_ptr<ContactResultsToLcmSystem<double>>(
          new ContactResultsToLcmSystem<double>(multibody_plant, name_lookup)));
  contact_to_lcm->set_name("contact_to_lcm");

  auto contact_results_publisher = builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", lcm, 1.0 / 60 /* publish period */));
  contact_results_publisher->set_name("contact_results_publisher");

  builder->Connect(contact_results_port,
                   contact_to_lcm->get_contact_result_input_port());
  builder->Connect(*contact_to_lcm, *contact_results_publisher);

  return contact_results_publisher;
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    lcm::DrakeLcmInterface* lcm) {
  return ConnectWithNameLookup(
      builder, multibody_plant,
      multibody_plant.get_contact_results_output_port(), nullptr, lcm);
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const geometry::SceneGraph<double>& scene_graph,
    lcm::DrakeLcmInterface* lcm) {
  return ConnectWithNameLookup(
      builder, multibody_plant,
      multibody_plant.get_contact_results_output_port(),
      make_geometry_name_lookup(scene_graph), lcm);
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm) {
  return ConnectWithNameLookup(
      builder, multibody_plant, contact_results_port, nullptr, lcm);
}

systems::lcm::LcmPublisherSystem* ConnectContactResultsToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const geometry::SceneGraph<double>& scene_graph,
    const systems::OutputPort<double>& contact_results_port,
    lcm::DrakeLcmInterface* lcm) {
  return ConnectWithNameLookup(
      builder, multibody_plant,
      contact_results_port,
      make_geometry_name_lookup(scene_graph), lcm);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ContactResultsToLcmSystem)
