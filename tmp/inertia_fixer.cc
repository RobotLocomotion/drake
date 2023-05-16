#include <regex>

#include <drake_vendor/tinyxml2.h>
#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/geometry_spatial_inertia.h"

namespace drake {
namespace multibody {
namespace {

using geometry::Role;
using geometry::SceneGraph;
using tinyxml2::XMLNode;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;


std::optional<SpatialInertia<double>> MaybeFixBodyInertia(
    MultibodyPlant<double>* plant,
    const SceneGraph<double>& scene_graph,
    BodyIndex body_index) {
  // * Leverage the maybe-damaged model that got parsed by the parser:
  //   * for geometries
  //   * for mass/density parameters
  // * Build proper inertias from geometries:
  //   * cf. mujoco parser's techniques
  //   * figure out how to combine inertias for multiple geometries?
  const auto maybe_frame_id = plant->GetBodyFrameIdIfExists(body_index);
  if (!maybe_frame_id.has_value()) {
    return {};
  }
  const auto& body_name = plant->get_body(body_index).name();
  const auto& rigid_body = plant->GetRigidBodyByName(body_name);
  const auto& old_inertia = rigid_body.default_spatial_inertia();
  const double mass = old_inertia.get_mass();
  const auto& inspector = scene_graph.model_inspector();
  const auto geoms = inspector.GetGeometries(*maybe_frame_id, Role::kProximity);
  SpatialInertia<double> M_BBo_B(0, {0, 0, 0}, {0, 0, 0});
  for (const auto& geom : geoms) {
    const auto M_GG_G_one = CalcSpatialInertia(inspector.GetShape(geom), 1.0);
    SpatialInertia<double> M_GG_G(mass, M_GG_G_one.get_com(),
                                  M_GG_G_one.get_unit_inertia());
    const auto& X_BG = inspector.GetPoseInFrame(geom);
    const auto M_GBo_B = M_GG_G.ReExpress(X_BG.rotation())
                         .Shift(-X_BG.translation());
    // XXX presumably this accumulates CoM. How do we accumulate/understand the
    // inertia frame rotation?
    M_BBo_B += M_GBo_B;
  }
  return M_BBo_B;
}

XMLElement* EnsureChildElement(XMLElement* el, const char* child) {
  XMLElement* kid = el->FirstChildElement(child);
  if (!kid) {
    kid = el->InsertNewChildElement(child);
  }
  return kid;
}

void UpdateInertiaUrdf(
    const MultibodyPlant<double>& plant,
    BodyIndex body_index,
    const SpatialInertia<double>& inertia,
    XMLDocument* doc) {
  unused(plant, body_index, inertia, doc);
  const auto& body_name = plant.get_body(body_index).name();
  XMLElement* el{};
  for (el = doc->FirstChildElement("link");
       el;
       el = el->NextSiblingElement("link")) {
    const char* name = el->Attribute("name");
    if (!name) { continue; }
    if (std::string(name) != body_name) { continue; }
    break;
  }
  DRAKE_DEMAND(el != nullptr);  // Named element was found.
  XMLElement* inertial = EnsureChildElement(el, "inertial");
  XMLElement* origin = EnsureChildElement(inertial, "origin");
  origin->SetAttribute("xyz", "0 0 0");
  origin->SetAttribute("rpy", "0 0 0");
  XMLElement* mass = EnsureChildElement(inertial, "mass");
  mass->SetAttribute("value", inertia.get_mass());
  XMLElement* ertia = EnsureChildElement(inertial, "inertia");
  const auto rot = inertia.CalcRotationalInertia();
  const auto mom = rot.get_moments();
  const auto prod = rot.get_products();
  ertia->SetAttribute("ixx", mom(0));
  ertia->SetAttribute("iyy", mom(1));
  ertia->SetAttribute("izz", mom(2));
  ertia->SetAttribute("ixy", prod(0));
  ertia->SetAttribute("ixz", prod(1));
  ertia->SetAttribute("iyz", prod(2));
}

void UpdateInertiaSdf(
    const MultibodyPlant<double>& plant,
    BodyIndex body_index,
    const SpatialInertia<double>& inertia,
    XMLDocument* doc) {
  unused(plant, body_index, inertia, doc);
}

void UpdateInertiaXml(
    const MultibodyPlant<double>& plant,
    BodyIndex body_index,
    const SpatialInertia<double>& inertia,
    XMLDocument* doc) {
  // * Edit fixed-up inertias back into doc.
  std::string root_name(doc->RootElement()->Name());
  if (root_name == "sdf") {
    UpdateInertiaSdf(plant, body_index, inertia, doc);
  } else if (root_name == "robot") {
    UpdateInertiaUrdf(plant, body_index, inertia, doc);
  } else {
    drake::log()->error("Unknown file type: root element was {}", root_name);
    ::exit(EXIT_FAILURE);
  }
  // Find the link/inertial node for the body we have.
  // Rewrite the i{xyz}{xyz} values.
}

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage("[INPUT-FILE-OR-URL]\n"
                          "Rewrite URDF/SDFormat with fixed-up inertias");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    drake::log()->error("missing input filename");
    return 1;
  }
  const char* outfile = (argc > 2) ? argv[2] : "/dev/stdout";

  XMLDocument xml_doc;
  xml_doc.LoadFile(argv[1]);
  if (xml_doc.ErrorID()) {
    drake::log()->error("Failed to parse XML file: {}",
                        xml_doc.ErrorName());
    ::exit(EXIT_FAILURE);
  }

  // TODO(rpoyner-tri): Somehow, in here, inertia-fixing magic happens.

  // * Figure out which inertias need fixing, and locate their nodes in the doc.
  //   * It might be feasible to use parser internals for:
  //     * checking the model file type
  //     * hijacking the diagnostic output
  //   * URDF warning messages point to the <inertial> tag.
  //   * SDFormat warning messages point to the <link> tag.
  // * XXX For now, maybe we just fix them all.

  MultibodyPlant<double> plant{0.0};
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  Parser parser{&plant};
  parser.package_map().PopulateFromRosPackagePath();

  std::vector<ModelInstanceIndex> models;
  models = parser.AddModels(argv[1]);

  for (BodyIndex k(1); k < plant.num_bodies(); ++k) {
    const auto& maybe_inertia = MaybeFixBodyInertia(&plant, scene_graph, k);
    if (maybe_inertia.has_value()) {
      UpdateInertiaXml(plant, k, *maybe_inertia, &xml_doc);
    }
  }

  // tinyxml2 preserves the input text almost exactly. Probably good enough for
  // this purpose.
  xml_doc.SaveFile(outfile);


  return models.empty();
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::do_main(argc, argv);
}
