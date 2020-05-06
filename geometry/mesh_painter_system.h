#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bounding_volume_hierarchy.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {

/** The definition of a canvas mesh: a 3D surface mesh with per-vertex texture
 coordinates. Generally, the domain of the texture coordinates would lie in
 [0, 1] X [0, 1]. However, values outside of that domain are valid -- it leads
 to tiled textures in the rendered mesh.
 */
struct TexturedCanvasMesh {
  TexturedCanvasMesh(
      std::unique_ptr<SurfaceMesh<double>> mesh_in,
      std::vector<Vector2<double>> uvs_in,
      std::unique_ptr<internal::BoundingVolumeHierarchy<SurfaceMesh<double>>>
          bvh_in)
      : mesh(std::move(mesh_in)),
        uvs(std::move(uvs_in)),
        bvh(std::move(bvh_in)) {}

  std::unique_ptr<SurfaceMesh<double>> mesh;
  std::vector<Vector2<double>> uvs;
  std::unique_ptr<internal::BoundingVolumeHierarchy<SurfaceMesh<double>>> bvh;
};

/** The definition of a pinter mesh; a 3D volume mesh. A canvas mesh is painted
 in its UV space based on the intersection with this volume mesh. */
struct PainterMesh {
  PainterMesh(
      std::unique_ptr<VolumeMesh<double>> mesh_in,
      std::unique_ptr<internal::BoundingVolumeHierarchy<VolumeMesh<double>>>
          bvh_in)
      : mesh(std::move(mesh_in)), bvh(std::move(bvh_in)) {}

  std::unique_ptr<VolumeMesh<double>> mesh;
  std::unique_ptr<internal::BoundingVolumeHierarchy<VolumeMesh<double>>> bvh;
};

// TODO(SeanCurtis-TRI): Consider adding a zero-order-hold painter system that
//  contains an image as state and evaluates the painter system on a periodic
//  basis.

/** System for using contact between two geometries to paint onto an image.

 @system{MeshPainterSystem,
   @input_port{geometry_query},
   @output_port{texture}
 }

 One geometry is the "canvas" geometry. It has texture coordinates which
 association points on the surface of the geometry with a 2D image. The other
 geometry is the "painter" geometry. When the painter geometry makes contact
 with the surface of the canvas geometry, the contact patch is drawn onto the
 2D image based on the canvas's texture coordinates. This is an accumulative
 act.

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
with black. The resulting image is available on an output port. Painting only
occurs when the output image port is evaluated. Therefore, it will be subject
to temporal aliasing. The frequency at which it shoudl be evaluated depends on
the relative speed between contacting canvas and painter. The higher the
relative velocity, the more frequent the evaluations should be.  */
class MeshPainterSystem final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshPainterSystem)

  /** Constructs a %MeshPainterSystem.

   @param canvas_id     The id of the shape that serveas as canvas.
   @param painter_id    The id of the geometry that paints on the mesh.
   @param width         The width of the texture.
   @param height        The height of the texture.
   @param scene_graph   The SceneGraph instances that contains the given ids.
   @throws if either geometry is an unsupported type or there is an error in
           instantiating the shape.
   */
  MeshPainterSystem(GeometryId canvas_id, GeometryId painter_id, int width,
                    int height, const SceneGraph<double>& scene_graph);

  /** Returns the input ort that must be connected to the query object output
   port of the SceneGraph instance that owns the ids of the painter and canvas
   geometries.  */
  const systems::InputPort<double>& geometry_query_input_port() const;

  /** Returns the ImageRgba8U-valued output port. */
  const systems::OutputPort<double>& texture_output_port() const;

 private:
  /* Update function for the texture map. This will paint onto the given texture
   based on overlap between the painter and canvas. It only accumulates
   and makes no assumptions about the previous state of the texture.  */
  void CalcTextureImage(const systems::Context<double>& context,
                        systems::sensors::ImageRgba8U* texture) const;

  const GeometryId canvas_id_;
  const GeometryId painter_id_;
  const std::unique_ptr<PainterMesh> painter_mesh_;
  const std::unique_ptr<TexturedCanvasMesh> canvas_mesh_;
  const systems::InputPort<double>* geometry_query_input_port_{};
  const systems::OutputPort<double>* texture_output_port_{};
};

}  // namespace geometry
}  // namespace drake
