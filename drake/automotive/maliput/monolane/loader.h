#pragma once

#include <memory>
#include <string>

namespace drake {
namespace maliput {

namespace api {
class RoadGeometry;
}

namespace monolane {

/// @file
/// Loader for serialized monolane road networks.
///
/// The serialization is a fairly straightforward mapping of the Builder
/// interface onto YAML. See (TBD) for more detail of the format.
///
/// The basic idea is, however:
///  - general parameters (i.e., as supplied to Builder constructor)
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
// TODO(maddog@tri.global)  Describe complete format somewhere.

/// Loads the input string as a maliput_monolane_builder document.
std::unique_ptr<const api::RoadGeometry> Load(const std::string& input);

/// Loads the named file as a maliput_monolane_builder document.
std::unique_ptr<const api::RoadGeometry> LoadFile(const std::string& filename);

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
