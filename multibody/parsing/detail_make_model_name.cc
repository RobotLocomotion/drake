#include "drake/multibody/parsing/detail_make_model_name.h"

#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace multibody {
namespace internal {

std::string MakeModelName(std::string_view candidate_name,
                          const std::optional<std::string>& parent_model_name,
                          const ParsingWorkspace& workspace) {
  std::string model_name = parsing::PrefixName(parent_model_name.value_or(""),
                                               std::string(candidate_name));
  if (workspace.plant->HasModelInstanceNamed(model_name)) {
    if (workspace.options.enable_auto_renaming) {
      model_name = fmt::format(
          "{}_{}", model_name, workspace.plant->num_model_instances());
    }
  }
  return model_name;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

