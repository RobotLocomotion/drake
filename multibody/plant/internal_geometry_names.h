#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

/* Maintains a mapping from GeomtryId to its "full" name, for use by
visualization tools. */
class GeometryNames {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryNames);

  /* Stores the "full" name of a body in contact: its model instance name,
  body name, and geometry name.

  A model instance name is always unique within a plant.
  A body name is always unique within its model instance.
  A geometry name is always unique within its body.

  This struct also includes information that allows visualizers to streamline
  their display (the "is_unique" bools), so they can omit redundant qualifiers.
  */
  struct Entry {
    std::string_view model_instance_name;
    std::string_view body_name;
    std::optional<std::string_view> geometry_name;

    bool body_name_is_unique_within_plant{};
    bool geometry_is_unique_within_body{};

    // Note that it would be easy to add the ModelInstanceIndex, BodyIndex, and
    // GeometryId to this struct if we ever needed them.
  };

  /* Creates and empty mapping. */
  GeometryNames();

  ~GeometryNames();

  /* Clears this object and re-populates it using the given plant and
  inspector. This is the preferred form of reset (vs ResetBasic).
  @pre plant.is_finalized().
  @tparam_nonsymbolic_scalar */
  template <typename T>
  void ResetFull(const MultibodyPlant<T>& plant,
                 const geometry::SceneGraphInspector<T>& inspector);

  /* Clears this object and re-populates using the given plant.
  This overload setse the geometry_name fields to nullopt.
  This is NOT the preferred form; use ResetFull if you can do so.
  @pre plant.is_finalized().
  @tparam_default_scalar */
  template <typename T>
  void ResetBasic(const MultibodyPlant<T>& plant);

  /* Returns the entry associated with the given ID.
  The return value is no longer valid after any call to a reset function. */
  const Entry& Find(geometry::GeometryId) const;

  /* Returns the full mapping.
  The return value is no longer valid after any call to a reset function. */
  const std::unordered_map<geometry::GeometryId, Entry>& entries() const {
    return entries_;
  }

 private:
  std::unordered_map<geometry::GeometryId, Entry> entries_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
