#include "drake/examples/mesh_painter/render_engine_vtk_with_mask_images.h"

#include <optional>

#include <fmt/format.h>
#include <vtkProperty.h>
#include <vtkTexture.h>

#include "drake/examples/mesh_painter/painter_shader.h"

namespace drake {
namespace examples {
namespace mesh_painter {

using geometry::GeometryId;
using geometry::PerceptionProperties;
using geometry::render::RenderEngine;
using systems::sensors::ImageRgba8U;
using std::optional;

void RenderEngineVtkWithMaskImages::InitializeMasksForGeometry(
    GeometryId id, const PerceptionProperties& properties) const {
  optional<PainterShader> painter_shader = PainterShader::Make(properties);
  if (painter_shader) {
    const auto& actor_triple = actors().at(id);
    if (painter_shader->has_labels()) {
      // Shader has configured render labels.
      painter_shader->AssignLabelToActor(actor_triple[ImageType::kLabel].Get(),
                                         &RenderEngine::GetColorDFromLabel);
    }
    painter_shader->AssignRgbaToActor(actor_triple[ImageType::kColor].Get());
  }
}

void RenderEngineVtkWithMaskImages::UpdateRgbaMaskForGeometry(
    GeometryId id, const ImageRgba8U& mask_texture) const {
  if (!UpdateMaskForActor(id, ImageType::kColor, mask_texture)) {
    throw std::runtime_error(
        fmt::format("Attempting to update the mask for a geometry that was "
                    "not initialized to have a mask: {}",
                    id));
  }
}

void RenderEngineVtkWithMaskImages::UpdateLabelMaskForGeometry(
    GeometryId id, const ImageRgba8U& mask_texture) const {
  // Not being able to update the mask for a label is not an error, the label
  // doesn't *have* to be initialized.
  UpdateMaskForActor(id, ImageType::kLabel, mask_texture);
}

std::unique_ptr<RenderEngine> RenderEngineVtkWithMaskImages::DoClone() const {
  return std::make_unique<RenderEngineVtkWithMaskImages>(*this);
}

bool RenderEngineVtkWithMaskImages::UpdateMaskForActor(
    GeometryId id, ImageType image_type,
    const ImageRgba8U& mask_texture) const {
  const auto& actor_triple = actors().at(id);
  const auto& actor = actor_triple[image_type];
  return PainterShader::UpdateMaskForActor(actor, mask_texture);
}

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
