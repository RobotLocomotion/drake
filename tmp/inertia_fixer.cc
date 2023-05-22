#include <fstream>
#include <functional>

#include <drake_vendor/tinyxml2.h>
#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/geometry_spatial_inertia.h"

DEFINE_bool(invalid_only, false,
            "if true, only fix physically invalid inertias");

namespace drake {
namespace multibody {
namespace {

using geometry::Role;
using geometry::SceneGraph;
using tinyxml2::XMLNode;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLPrinter;

// XXX Does tinyxml2 quote-swapping ('' to "") break any strings?

// Nope: it does entity substitution. So things may look worse in source, but
// information will be preserved.
/*
For Input:
  <material name='silver-"ish"'>

We get Output:
  <material name="silver-&quot;ish&quot;">
*/

// We jump through hoops here, just to get 2-space indented output.
class XmlPrinter final : public XMLPrinter {
 public:
  XmlPrinter() : XMLPrinter() {}
  void PrintSpace(int depth) final {
    for (int i = 0; i < depth; ++i) {
      Write("  ");
    }
  }
};

// Create a child element if it does not already exist.
XMLElement* EnsureChildElement(XMLElement* el, const char* child) {
  XMLElement* kid = el->FirstChildElement(child);
  if (!kid) {
    kid = el->InsertNewChildElement(child);
  }
  return kid;
}

// Format a double in shortest round-trip representation.
std::string roundtrip(double x) {
  return fmt::format("{}", x);
}

// Recursively find all of the descendant elements of 'el` named "link", and
// accumulate them in an externally supplied vector.
void FindLinks(XMLElement* el, std::vector<XMLElement*>* links) {
  if (std::string(el->Name()) == "link") {
    links->push_back(el);
    return;
  }
  for (XMLElement* kid = el->FirstChildElement();
       kid;
       kid = kid->NextSiblingElement()) {
    FindLinks(kid, links);
  }
}


class XmlInertiaMapper {
 public:
  using Mapping = std::unordered_map<BodyIndex, XMLElement*>;
  virtual ~XmlInertiaMapper() {}
  virtual const Mapping& mapping() const = 0;
  virtual void UpdateInertia(XMLElement*, const SpatialInertia<double>&) = 0;
};

class UrdfInertiaMapper final : public XmlInertiaMapper {
 public:
  UrdfInertiaMapper(
      const MultibodyPlant<double>& plant,
      const std::vector<ModelInstanceIndex>& models,
      XMLDocument* doc)
      : plant_(plant) {
    DRAKE_ASSERT(models.size() == 1);
    std::vector<XMLElement*> links;
    FindLinks(doc->RootElement(), &links);
    DRAKE_ASSERT(drake::ssize(links) == plant_.num_bodies() - 1);
    for (int k = 1; k < plant_.num_bodies(); ++k) {
      const BodyIndex body_index{k};
      mapping_[body_index] = links[k - 1];
      if (kDrakeAssertIsArmed) {
        const auto& body_name = plant_.get_body(body_index).name();
        const XMLElement* link = links[k - 1];
        const char* name = link->Attribute("name");
        DRAKE_ASSERT(name != nullptr);
        DRAKE_ASSERT(std::string(name) == body_name);
      }
    }
  }

  const Mapping& mapping() const final { return mapping_; }

  void UpdateInertia(XMLElement* el,
                     const SpatialInertia<double>& inertia) final {
    XMLElement* inertial = EnsureChildElement(el, "inertial");
    XMLElement* origin = EnsureChildElement(inertial, "origin");
    origin->SetAttribute("xyz", "0 0 0");
    origin->SetAttribute("rpy", "0 0 0");
    XMLElement* mass = EnsureChildElement(inertial, "mass");
    mass->SetAttribute("value", roundtrip(inertia.get_mass()).c_str());
    XMLElement* coeffs = EnsureChildElement(inertial, "inertia");
    const auto rot = inertia.CalcRotationalInertia();
    const auto mom = rot.get_moments();
    const auto prod = rot.get_products();
    coeffs->SetAttribute("ixx", roundtrip(mom(0)).c_str());
    coeffs->SetAttribute("iyy", roundtrip(mom(1)).c_str());
    coeffs->SetAttribute("izz", roundtrip(mom(2)).c_str());
    coeffs->SetAttribute("ixy", roundtrip(prod(0)).c_str());
    coeffs->SetAttribute("ixz", roundtrip(prod(1)).c_str());
    coeffs->SetAttribute("iyz", roundtrip(prod(2)).c_str());
  }

 private:
  const MultibodyPlant<double>& plant_;
  Mapping mapping_;
};


class SdfInertiaMapper final : public XmlInertiaMapper {
 public:
  SdfInertiaMapper(
      const MultibodyPlant<double>& plant,
      const std::vector<ModelInstanceIndex>& models,
      XMLDocument* doc)
      : plant_(plant) {
    DRAKE_ASSERT(models.size() >= 1);
    std::vector<XMLElement*> links;
    FindLinks(doc->RootElement(), &links);
    // XXX This assumption is likely violated by files that use include tags.
    DRAKE_ASSERT(drake::ssize(links) == plant_.num_bodies() - 1);
    for (int k = 1; k < plant_.num_bodies(); ++k) {
      const BodyIndex body_index{k};
      mapping_[body_index] = links[k - 1];
      if (kDrakeAssertIsArmed) {
        const auto& body_name = plant_.get_body(body_index).name();
        const XMLElement* link = links[k - 1];
        const char* name = link->Attribute("name");
        DRAKE_ASSERT(name != nullptr);
        DRAKE_ASSERT(std::string(name) == body_name);
      }
    }
  }

  const Mapping& mapping() const final { return mapping_; }

  void UpdateInertia(XMLElement* el,
                     const SpatialInertia<double>& inertia) final {
    // Fill out the inertial properties.
    XMLElement* inertial = EnsureChildElement(el, "inertial");
    XMLElement* pose = EnsureChildElement(inertial, "pose");
    pose->SetText("0 0 0 0 0 0");
    auto write_child = [&](XMLElement* parent, const char* name, double value) {
      XMLElement* kid = EnsureChildElement(parent, name);
      kid->SetText(roundtrip(value).c_str());
    };
    write_child(inertial, "mass", inertia.get_mass());
    XMLElement* coeffs = EnsureChildElement(inertial, "inertia");
    const auto rot = inertia.CalcRotationalInertia();
    const auto mom = rot.get_moments();
    const auto prod = rot.get_products();
    write_child(coeffs, "ixx", mom(0));
    write_child(coeffs, "iyy", mom(1));
    write_child(coeffs, "izz", mom(2));
    write_child(coeffs, "ixy", prod(0));
    write_child(coeffs, "ixz", prod(1));
    write_child(coeffs, "iyz", prod(2));
  }

 private:
  const MultibodyPlant<double>& plant_;
  Mapping mapping_;
};


class InertiaProcessor {
 public:
  InertiaProcessor(
      const MultibodyPlant<double>& plant,
      const SceneGraph<double>& scene_graph,
      const std::vector<ModelInstanceIndex>& models,
      XMLDocument* doc)
      : plant_(plant), scene_graph_(scene_graph) {
    XMLElement* root = doc->RootElement();
    std::string root_name(root->Name());
    if (root_name == "sdf") {
      mapper_ = std::make_unique<SdfInertiaMapper>(plant, models, doc);
    } else if (root_name == "robot") {
      mapper_ = std::make_unique<UrdfInertiaMapper>(plant, models, doc);
    } else {
      drake::log()->error("Unknown file type: root element was {}", root_name);
      ::exit(EXIT_FAILURE);
    }
  }

  void Process() {
    for (const auto& pair : mapper_->mapping()) {
      const auto& [body_index, el] = pair;
      const auto maybe_inertia = MaybeFixBodyInertia(body_index);
      if (maybe_inertia.has_value()) {
        mapper_->UpdateInertia(el, *maybe_inertia);
      }
    }
  }

 private:
  std::optional<SpatialInertia<double>> MaybeFixBodyInertia(
      BodyIndex body_index) {
    const auto& body = plant_.get_body(body_index);
    const auto* rigid_body = dynamic_cast<const RigidBody<double>*>(&body);
    if (!rigid_body) {
      // Only rigid bodies have constant inertia, for which model file fixups
      // make sense.
      return {};
    }
    const auto maybe_frame_id = plant_.GetBodyFrameIdIfExists(body_index);
    if (!maybe_frame_id.has_value()) {
      // No geometry to fix inertia from.
      return {};
    }
    const auto& old_inertia = rigid_body->default_spatial_inertia();
    if (FLAGS_invalid_only && old_inertia.IsPhysicallyValid()) {
      // Skip valid inertias by user preference.
      return {};
    }
    const auto& inspector = scene_graph_.model_inspector();
    const auto geoms = inspector.GetGeometries(*maybe_frame_id,
                                               Role::kProximity);
    if (geoms.empty()) {
      // No geometry to fix inertia from.
      return {};
    }
    SpatialInertia<double> M_BBo_B(0, {0, 0, 0}, {0, 0, 0});
    // XXX mass calculations are all wrong for the case of multiple geometries.
    const double mass = old_inertia.get_mass();
    for (const auto& geom : geoms) {
      const auto M_GG_G_one = CalcSpatialInertia(inspector.GetShape(geom), 1.0);
      SpatialInertia<double> M_GG_G(mass, M_GG_G_one.get_com(),
                                    M_GG_G_one.get_unit_inertia());
      const auto& X_BG = inspector.GetPoseInFrame(geom);
      const auto M_GBo_B = M_GG_G.ReExpress(X_BG.rotation())
                           .Shift(-X_BG.translation());
      M_BBo_B += M_GBo_B;
    }
    return M_BBo_B;
  }

  const MultibodyPlant<double>& plant_;
  const SceneGraph<double>& scene_graph_;
  std::unique_ptr<XmlInertiaMapper> mapper_;
};

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage("[INPUT-FILE] [OUTPUT-FILE]\n"
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

  MultibodyPlant<double> plant{0.0};
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  Parser parser{&plant};
  parser.package_map().PopulateFromRosPackagePath();

  // Chances are good that parsing may end in an error (implemented as throw).
  // For purposes of this tool, we should make its message usable. To do that,
  // catch it and display it via logging. Yes, this violates the coding
  // standard; in this case it makes the tool significantly more useful.
  std::vector<ModelInstanceIndex> models;
  try {
    models = parser.AddModels(argv[1]);
  } catch (const std::exception& e) {
    drake::log()->error(e.what());
    ::exit(EXIT_FAILURE);
  }

  InertiaProcessor processor(plant, scene_graph, models, &xml_doc);
  processor.Process();

  // Use our custom XmlPrinter to "print to memory", then dump that to file.
  XmlPrinter printer;
  xml_doc.Print(&printer);
  std::ofstream out(outfile);
  out << printer.CStr();

  return EXIT_SUCCESS;
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::do_main(argc, argv);
}
