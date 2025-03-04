#include "drake/multibody/parsing/detail_make_model_name.h"

#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {
namespace internal {

std::string MakeModelName(std::string_view candidate_name,
                          const std::optional<std::string>& parent_model_name,
                          const ParsingWorkspace& workspace) {
  std::string model_name =
      ScopedName::Join(parent_model_name.value_or(""), candidate_name)
          .to_string();

  if (workspace.options.enable_auto_renaming &&
      workspace.plant->HasModelInstanceNamed(model_name)) {
    std::string subscripted;
    // A note on the loop upper bound: searches don't expect to hit it. There
    // just needs to be some bound to make the loop guaranteed to terminate,
    // but large enough to allow finding an unused subscript. In the simplest
    // case {world_model_instance, default_model_instance, "candidate"}
    // num_model_instances will already be 3, and the search will find an
    // answer at 1. Any other models that happen to be around will only
    // increase the bound. If future model instance features interfere with
    // these assumptions, the algorithm here will need revisiting.
    for (int k = 1; k < workspace.plant->num_model_instances(); ++k) {
      subscripted = fmt::format("{}_{}", model_name, k);
      if (!workspace.plant->HasModelInstanceNamed(subscripted)) {
        break;
      }
    }
    model_name = subscripted;
  }
  return model_name;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
