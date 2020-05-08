#pragma once

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include <vtkActor.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_label.h"
#include "drake/systems/sensors/color_palette.h"

namespace drake {
namespace examples {
namespace mesh_painter {

/** Maps a painter shader, as defined by perception properties, to the VTK
 implementation. The painter shader uses a mask image to combine two sources:
 canvas and paint. Where the mask image is white, the fragment value comes from
 the canvas source, where black, the paint source. Specifically, the resultant
 value will be:

    `out = mask.r * canvas + (1 - mask.R) * paint`

 where `mask.r` is the red channel of the mask image in the range [0, 1] and
 `canvas` and `paint` are the source values. The mask is a texture image and the
 sources are _optionally_ texture images. And the values of the various colors
 are evaluated at the fragment's texture coordinates.

 The mask can apply to the appearance of the geometry in a color image as well
 as the label image. For label images, the out value is computed as:

    `out = mask.r > 0.5 ? canvas : paint`

 Instantiating an instance of %PainterShader requires a _full_ specification of
 the _color_ properties (using it in the label is optional). This means that
 both a canvas and a paint _color_ source must be specified. If both a canvas
 label _and_ paint label are provided, the mask will affect the label image
 as well.

 The painter shader is configured via properties stored in PerceptionProperties.
 All required properties are stored in the `paint_shader` group. The propeprties
 are as follows:

   |    Property    |        Type        | Note
   | :------------: | :----------------: | :-----------------------------------------------------------------------------
   | canvas_diffuse | Vector4d or string | Defines the canvas color. Either the RGBA color or the name of a texture image.
   | paint_diffuse  | Vector4d or string | Defines the paint color. Either the RGBA color or the name of a texture image.
   |  canvas_label  | RenderLabel        | (optional) Defines the label value for canvas-colored pixels. Cannot be RenderLabel::kDoNotRender.
   |  paint_label   | RenderLabel        | (optional) Defines the label value for paint-colored pixels. Cannot be RenderLabel::kDoNotRender.

 Note: Specifying only one of `canvas_label` or `paint_label` is considered a
 specification error and will throw. Omitting or including both is valid.
*/
class PainterShader {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PainterShader);

  /** Maybe construct a PainterShader from the specification found in the
   given `properties`.

   @return A %PainterShader if `properties` contain a fully specified painter
           shader or `nullopt` if no painter shader specified.
   @throws std::runtime_error if the properties specify a malformed painter
           shader.  */
  static std::optional<PainterShader> Make(
      const geometry::PerceptionProperties& properties);

  /** Assigns this shader to the given `actor`.  */
  void AssignRgbaToActor(vtkActor* actor) const;

  bool has_labels() const { return canvas_label_.has_value(); }

  /** Assigns this shader to the given _label_ `actor` encoding the defined
   labels as rgb color using the given `label_encoder`.  */
  void AssignLabelToActor(
      vtkActor* actor,
      const std::function<systems::sensors::ColorD(
          geometry::render::RenderLabel)>& label_encoder) const;

 private:
  PainterShader(std::variant<Eigen::Vector4d, std::string> canvas,
                std::variant<Eigen::Vector4d, std::string> paint,
                std::optional<geometry::render::RenderLabel> canvas_label = {},
                std::optional<geometry::render::RenderLabel> paint_label = {})
      : canvas_color_(std::move(canvas)),
        paint_color_(std::move(paint)),
        canvas_label_(canvas_label),
        paint_label_(paint_label) {}

  void AssignMaskTextureToActor(vtkActor* actor) const;

  std::string canvas_shader_string() const;

  std::string paint_shader_string() const;

  static std::string color_shader_string(
      const std::string& texture_name,
      const std::variant<Eigen::Vector4d, std::string>& color);

 private:
  std::variant<Eigen::Vector4d, std::string> canvas_color_;
  std::variant<Eigen::Vector4d, std::string> paint_color_;
  std::optional<geometry::render::RenderLabel> canvas_label_;
  std::optional<geometry::render::RenderLabel> paint_label_;
};

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
