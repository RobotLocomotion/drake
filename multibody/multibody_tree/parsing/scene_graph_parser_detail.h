#pragma once

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsers/package_map.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

/// Given an sdf::Geometry object representing a <geometry> element from an SDF
/// file, this method makes a new drake::geometry::Shape object from this
/// specification.
/// For `sdf_geometry.Type() == sdf::GeometryType::EMPTY`, corresponding to the
/// <empty/> SDF tag, it returns `nullptr`.
std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry);

/// Given an sdf::Visual object representing a <visual> element from an SDF
/// file, this method makes a new drake::geometry::GeometryInstance object from
/// this specification.
/// This method returns nullptr when the given SDF specification corresponds
/// to a geometry of type `sdf::GeometryType::EMPTY` (<empty/> SDF tag.)
std::unique_ptr<geometry::GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const sdf::Visual& sdf_visual);

// TODO(sam.creasey) Making this operate specifically on sdf::Visual
// is overly specific since we're going to be parsing collision meshes
// at some point.

/// Given an sdf::Visual object representing a <visual> element from
/// an SDF file, this method makes a new Visual object which resolves
/// the uri for the mesh element, if present.  If the mesh element is
/// not present, the new object will be identical to the original.
/// See parsers::ResolveFilename() for more detail on this operation.
///
/// @throws std::runtime_error if the <mesh> tag is present but
/// missing <uri> or if the file referenced in <uri> can not be found.
sdf::Visual ResolveVisualUri(const sdf::Visual& original,
                             const parsers::PackageMap& package_map,
                             const std::string& root_dir);

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
