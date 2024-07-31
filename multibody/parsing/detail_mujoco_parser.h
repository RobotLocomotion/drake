#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Parses a `<mujoco>` element from the MuJoCo MJCF XML context specified by @p
// data_source and adds it to @p plant.  A new model instance will be added to
// @p plant.
//
// Note: This is still just a stub implementation of the parser.  See Drake
// issue #16369 for the plan to make it useful.
//
// The parser reports unsupported elements and attributes from the XML
// specification using drake::log() warnings.
//
// Emits an error if the file is not in accordance with the MuJoCo XML
// specification: https://mujoco.readthedocs.io/en/latest/XMLreference.html, or
// if an unsupported element/tag would cause obviously erroneous behavior.
//
// If an error is emitted (and errors are not configured to throw), the
// function may (but does not always) return std::nullopt. Any returned model
// from an erroneous parse will likely be incomplete.
//
// @param data_source The XML data to be parsed.
// @param model_name The name given to the newly created instance of this
//   model.  If empty, the "name" attribute from the model tag will be used.
// @param parent_model_name Optional name of parent model. If set, the model
//   name of the parsed model (either `model_name` or from the "name"
//   attribute) will be prefixed with the parent_model_name, using the SDFormat
//   scope delimiter "::". The prefixed name will used as the name given to the
//   newly created instance of this model.
// @param plant A pointer to a mutable MultibodyPlant object to which the model
//   will be added.
// @returns The model instance index for the newly added model.
std::optional<ModelInstanceIndex> AddModelFromMujocoXml(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace);

class MujocoParserWrapper final : public ParserInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MujocoParserWrapper);
  MujocoParserWrapper();
  ~MujocoParserWrapper() final;
  std::optional<ModelInstanceIndex> AddModel(
      const DataSource& data_source, const std::string& model_name,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) final;

  std::string MergeModel(const DataSource& data_source,
                         const std::string& model_name,
                         ModelInstanceIndex merge_into_model_instance,
                         const ParsingWorkspace& workspace) final;

  std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource& data_source,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) final;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
