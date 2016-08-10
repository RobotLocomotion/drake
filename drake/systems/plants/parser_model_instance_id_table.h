#pragma once

#include <map>
#include <string>

namespace drake {
namespace parsers {
  /**
   * Defines a data type that maps from model names to their instance IDs.
   * The model names are defined in a single URDF or SDF file and are thus
   * guaranteed to be unique. Note that model names are not the same as
   * "model instance names", which can be assigned externally to the URDF and
   * SDF parsers (it is often useful to assign each model its own instance name
   * since multiple instances of the same model may co-exist within the same
   * `RigidBodyTree`). This data type is used to communicate the model instance
   * IDs that are assigned when the model instances are instantiated by the SDF
   * and URDF parsers.
   */
  typedef std::map<std::string, int> ModelInstanceIdTable;

}  // namespace parsers
}  // namespace drake
