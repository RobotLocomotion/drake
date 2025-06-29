#pragma once

/** @file
 This files declares the functions which bind various portions of the geometry
 namespace. These functions form a complete partition of all geometry
 bindings. The implementation of these methods are parceled out into various .cc
 files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/** Define the bounding box functions. See geometry_py_bounding_box.cc. */
void DefineGeometryBoundingBox(py::module m);

/** Defines the common elements in the drake::geometry namespace. See
 geometry_py_common.cc. */
void DefineGeometryCommon(py::module m);

/** Defines all of the hydroelastic-specific entities. See geometry_py_hydro.cc
 */
void DefineGeometryHydro(py::module m);

/** Defines the basic mesh types (and some parsing operations) on those types.
 See geometry_py_meshes.cc. */
void DefineGeometryMeshes(py::module m);

/** Defines all elements in the drake::geometry::optimization namespace.
 See geometry_py_optimization.cc. */
void DefineGeometryOptimization(py::module m);

/** Binds the public API of the drake/geometry/render and
 drake/geometry/render_* directories. See geometry_py_render.cc. */
void DefineGeometryRender(py::module m);

/** Binds SceneGraph and its query-related classes. See
 geometry_py_scene_graph.cc. */
void DefineGeometrySceneGraph(py::module m);

/** Binds the visualizers in drake::geometry. See geometry_py_visualizers.cc. */
void DefineGeometryVisualizers(py::module m);

/** Defines the mesh refinement functions. See geometry_py_refine.cc. */
void DefineGeometryRefine(py::module m);

}  // namespace pydrake
}  // namespace drake
