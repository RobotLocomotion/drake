#pragma once

#include <memory>

#include "drake/geometry/render/render_engine_vtk.h"

namespace drake {
namespace examples {
namespace mesh_painter {

/** A thin wrapper around RenderEngineVtk. It gives the ability to modify
 actors to support rendering masked rgb and label values. */
class RenderEngineVtkWithMaskImages
    : public geometry::render::RenderEngineVtk {
 public:
  RenderEngineVtkWithMaskImages(
      const geometry::render::RenderEngineVtkParams& parameters =
          geometry::render::RenderEngineVtkParams())
      : geometry::render::RenderEngineVtk(parameters) {}

  // TODO(SeanCurtis-TRI): Document the mask semantics (or link to the shader).
  /** @name  Initialize an actor to use a mask for appearance in the color
   rendering.

   The variants differ in how the two source colors are defined.  */
  //@{

  /** Possibly initializes the geometry with the given `id` for masked rendering
   based on the given `properties`. Will attempt to initialize masked RGBA
   and label definitions (as supported by the `properties`). See PainterShader
   for details.
   */
  void InitializeMasksForGeometry(
      geometry::GeometryId id,
      const geometry::PerceptionProperties& properties) const;

  //@}

  /** Updates the color mask for the geometry that has previously been
    initialized to use a mask.
   @throws if the given id had not previously been initialized.  */
  void UpdateRgbaMaskForGeometry(
      geometry::GeometryId id,
      const systems::sensors::ImageRgba8U& mask_texture) const;

  /** Updates the label mask for the geometry that has previously been
   initialized to use a mask.  */
  void UpdateLabelMaskForGeometry(
      geometry::GeometryId id,
      const systems::sensors::ImageRgba8U& mask_texture) const;

 private:
  /* @see RenderEngine:DoClone().  */
  std::unique_ptr<geometry::render::RenderEngine> DoClone()
      const override;

  // TODO(SeanCurtis-TRI): This is copy-pasta'd from render_engine_vtk.cc. Make
  //  that enumeration a protected member of RenderEngineVtk to eliminate the
  //  need for this kind of thing.
  enum ImageType {
    kColor = 0,
    kLabel = 1,
    kDepth = 2,
  };

  /* Reports true if the mask texture for the given geometry was updated for the
   image type.  */
  bool UpdateMaskForActor(
      geometry::GeometryId id, ImageType image_type,
      const systems::sensors::ImageRgba8U& mask_texture) const;

  // Note: This class adds no new state; it simply provides unique functionality
  // for manipulating the VTK representation of geometries.
};

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
