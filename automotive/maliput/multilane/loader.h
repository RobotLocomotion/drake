#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/multilane/builder.h"

namespace drake {
namespace maliput {

namespace api {
class RoadGeometry;
}

namespace multilane {

/// @file
/// Loader for serialized multilane road networks.
///
/// The serialization is a fairly straightforward mapping of the
/// \ref drake::maliput::multilane::Builder "Builder" interface onto YAML.
/// See yaml_spec_doxygen.h for more detail of the format.
///
/// The basic idea is, however:
///  - general parameters (i.e., lane_width, elevation bounds, linear and
///    angular tolerance)
///  - a collection of named 'points', which are specifications of explicitly
///    named Endpoints
///  - a collection of named 'connections', whose start Endpoints are specified
///    by reference to either a named Endpoint or the start or end of
///    a named Connection
///  - a collection of named 'groups', specified by sequences of named
///    Connections
///
/// Parsing will fail if there is no way to concretely resolve all of the
/// Endpoint references, e.g., if a document specifies that Connection-A
/// is an arc starting at the end of Connection-B and that Connection-B
/// is an arc starting at the end of Connection-A.  All referential chains
/// must bottom out in explicitly-named Endpoints.

/// Loads the `input` string as a maliput_multilane_builder document using the
/// provided `builder_factory`. See loader.h for further details.
///
/// Application code must use a BuilderFactory reference. It is provided so that
/// the \ref drake::maliput::multilane::Builder "Builder" to be created can be
/// mocked and code can be tested.
std::unique_ptr<const api::RoadGeometry> Load(
    const BuilderFactoryBase& builder_factory, const std::string& input);

/// Loads the named file as a maliput_multilane_builder document using the
/// provided `builder_factory`. See loader.h for further details.
///
/// Application code must use a BuilderFactory reference. It is provided so that
/// the \ref drake::maliput::multilane::Builder "Builder" to be created can be
/// mocked and code can be tested.
std::unique_ptr<const api::RoadGeometry> LoadFile(
    const BuilderFactoryBase& builder_factory, const std::string& filename);

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
