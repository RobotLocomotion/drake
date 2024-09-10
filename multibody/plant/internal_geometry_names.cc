#include "drake/multibody/plant/internal_geometry_names.h"

#include <stdexcept>

#include <fmt/ranges.h>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace internal {

using geometry::GeometryId;
using geometry::SceneGraphInspector;

GeometryNames::GeometryNames() = default;

GeometryNames::~GeometryNames() = default;

namespace {

// There's an anonymous helper function in multibody_plant.cc ("GetScopedName")
// that rewrites the user-provided geometry names by prefixing them with the
// model instance name, such that the names reported by GeometryInspector are
// a pack of lies. This function undoes that rewrite, so that we can report
// the actual names that the user specified. Hopefully the MbP is repaired in
// the future to stop screwing up the geometry names.
template <typename T>
std::string_view UndoGetScopedName(const MultibodyPlant<T>& plant,
                                   ModelInstanceIndex model,
                                   std::string_view geometry_name) {
  if (model != world_model_instance() && model != default_model_instance()) {
    const std::string_view prefix = plant.GetModelInstanceName(model);
    if (geometry_name.substr(0, prefix.size()) == prefix) {
      const std::string_view remaining = geometry_name.substr(prefix.size());
      if (remaining.substr(0, 2) == "::") {
        return remaining.substr(2);
      }
    }
  }
  return geometry_name;
}

/* Resets the `entries` output parameter to reflect the collision geometries
from the given plant; use `lookup` to obtain the geometry_name strings. */
template <typename T>
void ResetHelper(
    const MultibodyPlant<T>& plant,
    const std::function<std::optional<std::string_view>(GeometryId)>& lookup,
    std::unordered_map<GeometryId, GeometryNames::Entry>* entries) {
  DRAKE_DEMAND(entries != nullptr);
  DRAKE_THROW_UNLESS(plant.is_finalized());
  entries->clear();
  const int num_bodies = plant.num_bodies();
  for (BodyIndex i{0}; i < num_bodies; ++i) {
    const RigidBody<T>& body = plant.get_body(i);
    const auto& geometry_ids = plant.GetCollisionGeometriesForBody(body);
    if (geometry_ids.empty()) {
      continue;
    }

    // Create an Entry with the body's details (no geometry details yet).
    GeometryNames::Entry prototype;
    prototype.model_instance_name =
        plant.GetModelInstanceName(body.model_instance());
    prototype.body_name = body.name();
    prototype.body_name_is_unique_within_plant =
        (plant.NumBodiesWithName(body.name()) == 1);
    prototype.is_sole_geometry_within_body = (geometry_ids.size() == 1);

    // Add an Entry for each collision geometry.
    for (const auto& geometry_id : geometry_ids) {
      GeometryNames::Entry entry = prototype;
      std::optional<std::string_view> geometry_name = lookup(geometry_id);
      if (geometry_name.has_value()) {
        geometry_name =
            UndoGetScopedName(plant, body.model_instance(), *geometry_name);
      }
      entry.geometry_name = geometry_name;
      entries->insert({geometry_id, entry});
    }
  }
}

}  // namespace

template <typename T>
void GeometryNames::ResetFull(const MultibodyPlant<T>& plant,
                              const SceneGraphInspector<T>& inspector) {
  DRAKE_THROW_UNLESS(plant.is_finalized());
  std::function lookup =
      [&inspector](GeometryId id) -> std::optional<std::string_view> {
    return inspector.GetName(id);
  };
  ResetHelper(plant, lookup, &entries_);
}

template <typename T>
void GeometryNames::ResetBasic(const MultibodyPlant<T>& plant) {
  DRAKE_THROW_UNLESS(plant.is_finalized());
  std::function lookup = [](GeometryId) -> std::optional<std::string_view> {
    return std::nullopt;
  };
  ResetHelper(plant, lookup, &entries_);
}

const GeometryNames::Entry& GeometryNames::Find(GeometryId id) const {
  auto iter = entries_.find(id);
  if (iter != entries_.end()) {
    return iter->second;
  }
  throw std::logic_error("GeometryNames::Find could not find the given id");
}

std::string GeometryNames::GetFullName(GeometryId id,
                                       std::string_view sep) const {
  const Entry& entry = Find(id);

  // We'll join these at the end.
  std::vector<std::string_view> tokens;

  // Only use the model name if it matters.
  if (!entry.body_name_is_unique_within_plant) {
    tokens.push_back(entry.model_instance_name);
  }

  // Always use the body name.
  tokens.push_back(entry.body_name);

  // Only use the geometry name if it matters.
  std::string id_as_string;
  if (!entry.is_sole_geometry_within_body) {
    if (entry.geometry_name.has_value()) {
      tokens.push_back(*entry.geometry_name);
    } else {
      id_as_string = fmt::format("Id({})", id);
      tokens.push_back(id_as_string);
    }
  }

  return fmt::format("{}", fmt::join(tokens, sep));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&GeometryNames::ResetFull<T>));

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&GeometryNames::ResetBasic<T>));

}  // namespace internal
}  // namespace multibody
}  // namespace drake
