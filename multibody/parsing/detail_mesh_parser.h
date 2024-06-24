#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/multibody/parsing/detail_parsing_workspace.h"

namespace drake {
namespace multibody {
namespace internal {

// Adds a single model instance with a single body to `plant` based on the mesh
// data specified by `data_source`. The mesh data must be in the Wavefront OBJ
// format and contain only a _single_ object specification.
//
// @throws std::exception if the file is not in accordance with the OBJ
// specification containing a message with a list of errors encountered while
// parsing the file.
// @throws std::exception if there is not a single object in the mesh data.
// @throws std::exception if plant is nullptr or if MultibodyPlant::Finalize()
// was already called on `plant`.
// @throws if `data_source` is not a filename.
//
// @param data_source
//   The OBJ data to be parsed.
// @param model_name
//   The name given to the newly created instance of this model (if not empty).
//   Otherwise, applies the protocol defined in the documentation for Parser.
// @param parent_model_name
//   Optional name of parent model. If set, the model name of the parsed model
//   will be prefixed with the parent_model_name, using the scope delimiter
//   "::". The body name does not get prefixed.
// @param workspace
//   The ParsingWorkspace.
// @returns The model instance index for the newly added model; this might be
//   null if there were parsing errors reported through the workspace.diagnostic
//   policy.
std::optional<ModelInstanceIndex> AddModelFromMesh(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace);

class MeshParserWrapper final : public ParserInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshParserWrapper);
  MeshParserWrapper();
  ~MeshParserWrapper() final;
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
