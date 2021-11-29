/** @file  Definition of a system that paints meshes and its supporting classes.
 This defines three primary classes:

   - TexturedCanvasMesh: the definition of a mesh that serves as a canvas.
   - PainterMesh: the definition of a mesh that serves as a painter.
   - MeshPainterSystem: the definition of the system that paints on a texture
     applied to the canvas mesh based on contact with the painter mesh.
 */
#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace examples {
namespace mesh_painter {

template <typename T>
using Bvh = geometry::internal::Bvh<geometry::internal::Obb, T>;

/** The definition of a canvas mesh: a 3D surface mesh with per-vertex texture
 coordinates. Generally, the domain of the texture coordinates would lie in
 [0, 1] X [0, 1]. However, values outside of that domain are valid -- it leads
 to tiled textures in the rendered mesh.
 */
struct TexturedCanvasMesh {
  TexturedCanvasMesh(
      std::unique_ptr<geometry::TriangleSurfaceMesh<double>> mesh_in,
      std::vector<Eigen::Vector2d> uvs_in,
      std::unique_ptr<Bvh<geometry::TriangleSurfaceMesh<double>>> bvh_in)
      : mesh(std::move(mesh_in)),
        uvs(std::move(uvs_in)),
        bvh(std::move(bvh_in)) {}

  std::unique_ptr<geometry::TriangleSurfaceMesh<double>> mesh;
  std::vector<Eigen::Vector2d> uvs;
  std::unique_ptr<Bvh<geometry::TriangleSurfaceMesh<double>>> bvh;
};

/** The definition of a pinter mesh; a 3D volume mesh. A canvas mesh is painted
 in its UV space based on the intersection with this volume mesh. */
struct PainterMesh {
  PainterMesh(
      std::unique_ptr<geometry::VolumeMesh<double>> mesh_in,
      std::unique_ptr<Bvh<geometry::VolumeMesh<double>>> bvh_in)
      : mesh(std::move(mesh_in)), bvh(std::move(bvh_in)) {}

  std::unique_ptr<geometry::VolumeMesh<double>> mesh;
  std::unique_ptr<Bvh<geometry::VolumeMesh<double>>> bvh;
};

// TODO(SeanCurtis-TRI): Consider adding a zero-order-hold painter system that
//  contains an image as state and evaluates the painter system on a periodic
//  basis.

/** System for using contact between two geometries to paint onto an image.

@system
name : MeshPainterSystem
input_ports:
- geometry_query
output_ports:
- texture
@endsystem

 One geometry is the "canvas" geometry. It has texture coordinates which map
 points on the surface of the geometry with a 2D image. The other geometry is
 the "painter" geometry. When the painter geometry makes contact with the
 surface of the canvas geometry, the contact patch is drawn onto the 2D image
 based on the canvas's texture coordinates. This is an accumulative act.

 Not all geometries are supported.

   - Painter: the painter geometry can be one of: Box, Cylinder, Ellipsoid, or
     Sphere.
   - Canvas: Only a Mesh shape can serve as a canvas.
       - It must be specified as an OBJ file
       - The OBJ _must_ contain texture coordinates for all faces.
       - The texture coordinate values must all be non-negative.
       - The texture coordinates of any single triangle _cannot_ straddle the
         periodic boundaries of the texture space. A triangle straddles the
         boundary if, for an edge of the triangle the texture coordinates of the
         two end points (u₁, v₁) and (u₂, v₂), either u₁, u₂ ∈ [k, k+1] or
         v₁, v₂ ∈ [k, k+1] is _not_ satisfied for some integer value k.

The geometries are identified by the GeometryId assigned to them by a SceneGraph
instance. The %MeshPainterSystem must be connected to that same SceneGraph
instance's query object output port to determine the positions of the two
geometries at any time.

The image will initialized to an all-white image. Each contact will be drawn
with black based on the update period, and the result will be available on
the output port.

Painting will be subject to temporal aliasing due to the update period.
This update period depends on the relative speed between contacting canvas and
painter. The higher the relative velocity, the more frequent the evaluations
should be.
*/
class MeshPainterSystem final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshPainterSystem)

  /** Constructs a %MeshPainterSystem with the _given_ image as the initial
   image.

   @param canvas_id
      The id of the shape that serves as canvas.
   @param painter_id
      The id of the geometry that paints on the mesh.
   @param image
      The initial texture image.
   @param scene_graph
      The SceneGraph instances that contains the given ids.
   @param period_sec
      Number of seconds for the update. See class-level
      documentation for more information.
   @param parallel_raster
      Use OpenMP to speed up rasterizing.
   @throws if either geometry is an unsupported type or there is an error in
           instantiating the shape.
  */
  MeshPainterSystem(geometry::GeometryId canvas_id,
                    geometry::GeometryId painter_id,
                    const systems::sensors::ImageRgba8U& image,
                    const geometry::SceneGraph<double>& scene_graph,
                    double period_sec, bool parallelize_raster = false);

  /** Constructs a %MeshPainterSystem using a default initial image.

  The initial image is a pure white, opaque image with the given dimensions
  (`width`, `height`). All other parameters are the same as the constructor
  with a specified image.
  */
  MeshPainterSystem(geometry::GeometryId canvas_id,
                    geometry::GeometryId painter_id, int width,
                    int height,
                    const geometry::SceneGraph<double>& scene_graph,
                    double period_sec, bool parallelize_raster = false);

  /** Returns the input port that must be connected to the query object output
   port of the SceneGraph instance that owns the ids of the painter and canvas
   geometries.  */
  const systems::InputPort<double>& geometry_query_input_port() const {
    return *geometry_query_input_port_;
  }

  /** Returns the ImageRgba8U-valued output port. */
  const systems::OutputPort<double>& texture_output_port() const {
    return *texture_output_port_;
  }

  const systems::sensors::ImageRgba8U& get_texture_state(
      const systems::Context<double>& context) const;

  systems::sensors::ImageRgba8U& get_mutable_texture_state(
      systems::Context<double>* context) const;

 private:
  /* Updates the texture image.  */
  void UpdateTextureImage(const systems::Context<double>& context,
                          systems::State<double>* state) const;

  /* Copies the updated texture to an output value.  */
  void OutputTextureImage(const systems::Context<double>& context,
                          systems::sensors::ImageRgba8U* texture) const;

  const geometry::GeometryId canvas_id_;
  const geometry::GeometryId painter_id_;
  const std::unique_ptr<PainterMesh> painter_mesh_;
  const std::unique_ptr<TexturedCanvasMesh> canvas_mesh_;
  const systems::InputPort<double>* geometry_query_input_port_{};
  systems::AbstractStateIndex texture_state_index_;
  const systems::OutputPort<double>* texture_output_port_{};
  const bool parallelize_raster_;
};

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
