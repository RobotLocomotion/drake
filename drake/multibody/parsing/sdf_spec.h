#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/sdf_model.h"

namespace drake {
namespace multibody {
namespace parsing {

/// This class provides a representation for an SDF specification.
/// For details on the SDF specification, including conventions and default
/// values, please refer to the documentation for the
/// <a href="http://sdformat.org">SDFormat library</a>.
class SdfSpec {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SdfSpec)

  /// Creates a new SDF specification with the given `version`.
  explicit SdfSpec(const std::string& version) : version_(version) {}

  /// Returns the version of `this` specification.
  const std::string& version() const { return version_; }

  /// Returns the number of models, corresponding to `<model>` elements,
  /// in `this` specification.
  int get_num_models() {
    return static_cast<int>(model_ids_.size());
  }

  /// Returns `true` if `this` specification contains a model named
  /// `model_name`.
  bool HasModel(const std::string& model_name) const {
    return model_ids_.count(model_name) > 0;
  }

  /// Returns a const reference to a model uniquely identified by its name,
  /// `model_name`.
  /// This method throws an std::runtime_error If `model_name` is no present in
  /// this specification.
  const SdfModel& GetModelByName(const std::string& model_name) const {
    return *models_[GetModelIdByName(model_name)];
  }

  /// Adds a new model, named `model_name`, to `this` specification.
  SdfModel& AddModel(const std::string& model_name) {
    const int model_id = get_num_models();
    model_ids_.insert({model_name, model_id});
    models_.push_back(std::make_unique<SdfModel>(model_name));
    DRAKE_DEMAND(model_ids_.size() == models_.size());
    return *models_.back();
  }

 private:
  // Helper method. Given the name of a model, it returns the index of this
  // model into the internal array (models_) of models.
  // It throws an exception if `model_name` is not present.
  int GetModelIdByName(const std::string& model_name) const {
    const auto it = model_ids_.find(model_name);
    if (it == model_ids_.end()) {
      throw std::runtime_error("Model \"" + model_name + "\" not found");
    }
    return it->second;
  }

  // The SDF specification version.
  std::string version_;

  // Map from model name to model id.
  std::unordered_map<std::string, int> model_ids_;

  // SDF models ordered by id.
  std::vector<std::unique_ptr<SdfModel>> models_;
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
