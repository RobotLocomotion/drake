#pragma once

#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/sdf_model.h"

namespace drake {
namespace multibody {
namespace parsing {

/// This class provides a representation of an SDF specification.
class SDFSpec {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SDFSpec)

  /// Creates a new SDF specification with the given `version`.
  explicit SDFSpec(const std::string& version) : version_(version) {}

  /// Returns the version of `this` specification.
  const std::string& version() const { return version_; }

  int get_num_models() {
    return static_cast<int>(model_ids_.size());
  }

  SDFModel& AddModel(const std::string& model_name) {
    const int model_id = get_num_models();
    model_ids_.insert({model_name, model_id});
    models_.push_back(std::make_unique<SDFModel>(model_name));
    DRAKE_DEMAND(model_ids_.size() == models_.size());
    return *models_.back();
  }

  bool HasModel(const std::string& model_name) const {
    const auto it = model_ids_.find(model_name);
    return it != model_ids_.end();
  }

  int GetModelIdByName(const std::string& model_name) const {
    auto it = GetValidModelIteratorOrThrow(model_name);
    return it->second;
  }

  const SDFModel& GetModelByName(const std::string& model_name) const {
    return *models_[GetModelIdByName(model_name)];
  }

 private:
  // Helper to returns an iterator in model_ids_ for the model with name given
  // by `model_name`. This method throws an excpetion if `model_name` is not in
  // model_ids_.
  std::unordered_map<std::string, int>::const_iterator
  GetValidModelIteratorOrThrow(const std::string& model_name) const {
    const auto it = model_ids_.find(model_name);
    if (it == model_ids_.end()) {
      throw std::runtime_error("Model \"" + model_name + "\" not found");
    }
    return it;
  }

  // The SDF specification version.
  std::string version_;

  // Map from model name to model id.
  std::unordered_map<std::string, int> model_ids_;

  // SDF models ordered by id.
  std::vector<std::unique_ptr<SDFModel>> models_;
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
