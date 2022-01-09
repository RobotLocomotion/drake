#include "drake/multibody/parsing/detail_mujoco_parser.h"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "drake/multibody/parsing/detail_tinyxml.h"
#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::GeometryInstance;
using math::RigidTransformd;
using math::RotationMatrixd;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

// For tags that exist in the MuJoCo XML schema, but are not (yet) supported
// here, we print a warning to the console.  This method checks the node for
// the child element, and prints the warning if any occurrences exist.
void WarnUnsupportedElement(const XMLElement& node, const std::string& tag) {
  if (node.FirstChildElement(tag.c_str())) {
    drake::log()->warn(
        "The tag '{}' found in MuJoCo XML is currently unsupported and will be "
        "ignored.",
        tag);
  }
}

class MujocoParser {
 public:
  explicit MujocoParser(MultibodyPlant<double>* plant) : plant_{plant} {}

  ModelInstanceIndex Parse(const std::string& model_name_in,
                           const std::optional<std::string>& parent_model_name,
                           XMLDocument* xml_doc) {
    XMLElement* node = xml_doc->FirstChildElement("mujoco");
    if (!node) {
      throw std::runtime_error("ERROR: XML does not contain a mujoco tag.");
    }

    std::string model_name = model_name_in;
    if (model_name.empty() &&
        !ParseStringAttribute(node, "model", &model_name)) {
      throw std::runtime_error(
          "ERROR: Your robot must have a name attribute or a model name "
          "must be specified.");
    }
    model_name =
        parsing::PrefixName(parent_model_name.value_or(""), model_name);

    model_instance_ = plant_->AddModelInstance(model_name);

    // These will all be (at least partially) supported via #16369.
    WarnUnsupportedElement(*node, "compiler");
    WarnUnsupportedElement(*node, "option");
    WarnUnsupportedElement(*node, "worldbody");
    WarnUnsupportedElement(*node, "body");
    WarnUnsupportedElement(*node, "actuator");

    // These will not be supported initially.
    WarnUnsupportedElement(*node, "size");
    WarnUnsupportedElement(*node, "visual");
    WarnUnsupportedElement(*node, "statistic");
    WarnUnsupportedElement(*node, "default");
    WarnUnsupportedElement(*node, "custom");
    WarnUnsupportedElement(*node, "asset");
    WarnUnsupportedElement(*node, "contact");
    WarnUnsupportedElement(*node, "equality");
    WarnUnsupportedElement(*node, "tendon");
    WarnUnsupportedElement(*node, "sensor");
    WarnUnsupportedElement(*node, "keyframe");

    return model_instance_;
  }

 private:
  MultibodyPlant<double>* plant_;
  ModelInstanceIndex model_instance_{};
};

}  // namespace

ModelInstanceIndex AddModelFromMujocoXml(
    const DataSource& data_source, const std::string& model_name_in,
    const std::optional<std::string>& parent_model_name,
    MultibodyPlant<double>* plant, geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());
  data_source.DemandExactlyOne();

  XMLDocument xml_doc;
  if (data_source.file_name) {
    xml_doc.LoadFile(data_source.file_name->c_str());
    if (xml_doc.ErrorID()) {
      throw std::runtime_error(fmt::format("Failed to parse XML file {}:\n{}",
                                           *data_source.file_name,
                                           xml_doc.ErrorName()));
    }
  } else {
    DRAKE_DEMAND(data_source.file_contents != nullptr);
    xml_doc.Parse(data_source.file_contents->c_str());
    if (xml_doc.ErrorID()) {
      throw std::runtime_error(
          fmt::format("Failed to parse XML string: {}", xml_doc.ErrorName()));
    }
  }

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  MujocoParser parser(plant);
  return parser.Parse(model_name_in, parent_model_name, &xml_doc);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
