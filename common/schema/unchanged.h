#pragma once

namespace drake {
namespace schema {

/** A tag type intended for use by YAML Serialization to affirmatively indicate
that some value should remain unchanged.
Refer to @ref yaml_serialization "YAML Serialization" for background. */
struct Unchanged {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive*) {}
};

}  // namespace schema
}  // namespace drake
