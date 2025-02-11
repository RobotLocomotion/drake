#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Parses a `<robot>` element from the URDF file specified by @p data_source
// and adds it to @p plant.  A new model instance will be added to @p plant.
//
// @throws std::exception if the file is not in accordance with the URDF
// specification.  The exception contains a message with a list of errors
// encountered while parsing the file.
//
// @param data_source
//   The URDF data to be parsed.
// @param model_name
//   The name given to the newly created instance of this model.  If
//   empty, the "name" attribute from the model tag will be used.
// @param parent_model_name Optional name of parent model. If set, the model
//   name of the parsed model (either `model_name` or from the "name"
//   attribute) will be prefixed with the parent_model_name, using the SDFormat
//   scope delimiter "::". The prefixed name will used as the name given to the
//   newly created instance of this model.
// @param workspace
//   The ParsingWorkspace.
// @returns The model instance index for the newly added model, or std::nullopt
//          if no model instance was allocated. An instance will be allocated
//          as long as a valid model name can be constructed, by consulting the
//          supplied name parameters, and the <robot> tag (if any) in the @p
//          data_source.
// @throws std::exception on parse errors.
//
// TODO(rpoyner-tri): Eventually, all errors and warnings will flow through the
// diagnostic policy object supplied as part of @p workspace. At that point,
// the parse may or may not throw on error, at the discretion of the supplied
// policy. Also, many incomplete or erroneous parses may return a valid (though
// unfinished) model instance. It will be up to the supplier of the policy to
// indicate the overall parse result via public interfaces.
std::optional<ModelInstanceIndex> AddModelFromUrdf(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace);

class UrdfParserWrapper final : public ParserInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UrdfParserWrapper);
  UrdfParserWrapper();
  ~UrdfParserWrapper() final;
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
