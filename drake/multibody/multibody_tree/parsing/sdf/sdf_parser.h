#pragma once

#include <istream>
#include <memory>
#include <string>

#include "sdf/sdf.hh"

#include "drake/multibody/multibody_tree/parsing/sdf/frame_cache.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_model.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_spec.h"

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

  //std::unique_ptr<FrameCache<double>> frames_;
};

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
