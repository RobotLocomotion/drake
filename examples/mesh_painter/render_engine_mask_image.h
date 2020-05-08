#pragma once

#include <memory>

#include "drake/geometry/render/render_engine_vtk.h"

namespace drake {
namespace examples {
namespace mesh_painter {

/** A thin wrapper around RenderEngineVtk. It gives the ability to modify
 actors to support rendering masked rgb and label values. */
class RenderEngineMaskImage : public geometry::render::RenderEngineVtk {
 public:
  RenderEngineMaskImage(
      const geometry::render::RenderEngineVtkParams& parameters =
          geometry::render::RenderEngineVtkParams())
      : geometry::render::RenderEngineVtk(parameters) {}

  // TODO(SeanCurtis-TRI): Document the mask semantics (or link to the shader).
  /** @name  Initialize an actor to use a mask for appearance in the color
   rendering.

   The variants differ in how the two source colors are defined.  */
  //@{

  /** Possibly initializes the geometry with the given `id` for masked rendering
   based on the given `properties`. See PainterShader for details.
   */
  void InitializeRgbaMask(
      geometry::GeometryId id,
      const geometry::PerceptionProperties& properties) const;

  //@}

  /** Updates the color mask for the geometry that has previously been
    initialized to use a mask.
   @throws if the given id had not previously been initialized.  */
  void UpdateRgbaMask(geometry::GeometryId id,
                      const systems::sensors::ImageRgba8U& mask_texture) const;

  /** Updates the label mask for the geometry that has previously been
   initialized to use a mask.  */
  void UpdateLabelMask(geometry::GeometryId id,
                       const systems::sensors::ImageRgba8U& mask_texture) const;

 private:
  // @see RenderEngine:DoClone()
  std::unique_ptr<geometry::render::RenderEngine> DoClone() const override;

  /* Reports true if the mask texture for the given geometry was updated for the
   image type.  */
  bool UpdateActorMask(geometry::GeometryId id, int image_type_index,
                       const systems::sensors::ImageRgba8U& mask_texture) const;

  // Note: This class adds no new state; it simply provides unique functionality
  // for manipulating the VTK representation of geometries.
};

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
