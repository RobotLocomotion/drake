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

// Parses a `<model>` element from the SDF file specified by `data_source` and
// adds it to `plant`. The SDF file can only contain a single `<model>`
// element. `<world>` elements (used for instance to specify gravity) are
// ignored by this method.  A new model instance will be added to @p plant.
//
// @throws std::exception if the file is not in accordance with the SDF
// specification containing a message with a list of errors encountered while
// parsing the file.
// @throws std::exception if there is more than one `<model>` element or
// zero of them.
// @throws std::exception if plant is nullptr or if MultibodyPlant::Finalize()
// was already called on `plant`.
//
// @param data_source
//   The SDF data to be parsed.
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
// @returns The model instance index for the newly added model; this might be
//   null if there were parsing errors reported through the workspace.diagnostic
//   policy.
std::optional<ModelInstanceIndex> AddModelFromSdf(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace);

// Parses all `<model>` elements from the SDF file specified by `data_source`
// and adds them to `plant`. The SDF file can contain multiple `<model>`
// elements. New model instances will be added to @p plant for each `<model>`
// tag in the SDF file.
//
// @throws std::exception if the file is not in accordance with the SDF
// specification containing a message with a list of errors encountered while
// parsing the file.
// @throws std::exception if the file contains no models.
// @throws std::exception if plant is nullptr or if MultibodyPlant::Finalize()
// was already called on `plant`.
//
// @param data_source
//   The SDF data to be parsed.
// @param parent_model_name Optional name of parent model. If set, the model
//   names of all parsed models will be prefixed with the parent_model_name,
//   using the SDFormat scope delimiter "::". The prefixed name will used as
//   the name given to the newly created instances of these models.
// @param workspace
//   The ParsingWorkspace.
// @returns The set of model instance indices for the newly added models. This
//   might be fewer models than were declared in the file if there were parsing
//   errors reported through the workspace.diagnostic policy.
std::vector<ModelInstanceIndex> AddModelsFromSdf(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace);

class SdfParserWrapper final : public ParserInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SdfParserWrapper);
  SdfParserWrapper();
  ~SdfParserWrapper() final;
  std::optional<ModelInstanceIndex> AddModel(
      const DataSource& data_source, const std::string& model_name,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) final;

  std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource& data_source,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) final;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
