#pragma once

#include <istream>
#include <memory>
#include <string>
#include <unordered_map>

#include "sdf/sdf.hh"

#include "drake/multibody/multibody_tree/parsing/sdf/frame_cache.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_model.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_spec.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_link.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

class SDFParser {
 public:
  SDFParser() {}

  /// Parses a single `<model>` from file `sdf_file`.
  std::unique_ptr<SDFSpec> ParseSDFModelFromFile(
      const std::string& sdf_path);

 private:
  void ParseModel(sdf::ElementPtr sdf_model_element, SDFSpec* spec);

  // Parses from `sdf_link_element` a new link that gets added to `sdf_model`.
  // It caches the the link's poses.
  void ParseLink(const sdf::ElementPtr sdf_link_element, SDFModel* sdf_model);

  // Parses from `sdf_inertial_element` the properties specified in an
  // <inertial> element and adds them to `link`.
  void ParseInertial(sdf::ElementPtr sdf_inertial_element, SDFLink* link);

  // Helper to return the FrameCache for a particular model being parsed by
  // name.
  FrameCache<double>& GetModelFrameCache(const std::string& model_name) {
    auto it = model_name_to_frame_cache_map_.find(model_name);
    DRAKE_DEMAND(it != model_name_to_frame_cache_map_.end());
    return *it->second;
  }

  // So far we have a cache object per model. Probably better having a single
  // cache per root <world> and all models get added to it.
  std::unordered_map<std::string, std::unique_ptr<FrameCache<double>>>
      model_name_to_frame_cache_map_;
};

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
