#include "drake/examples/mesh_painter/render_engine_mask_image.h"

#include <optional>

#include <fmt/format.h>
#include <vtkProperty.h>
#include <vtkTexture.h>

#include "drake/examples/mesh_painter/painter_shader.h"

namespace drake {
namespace examples {
namespace mesh_painter {

namespace {

// TODO(SeanCurtis-TRI): This is copy-pasta'd from render_engine_vtk.cc. Make
//  that enumeration a protected member of RenderEngineVtk to eliminate the need
//  for this kind of thing.
enum ImageType {
  kColor = 0,
  kLabel = 1,
  kDepth = 2,
};

}  // namespace

using geometry::GeometryId;
using geometry::PerceptionProperties;
using geometry::render::RenderEngine;
using std::optional;

void RenderEngineMaskImage::InitializeRgbaMask(
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

void RenderEngineMaskImage::UpdateRgbaMask(
    geometry::GeometryId id,
    const systems::sensors::ImageRgba8U& mask_texture) const {
  if (!UpdateActorMask(id, ImageType::kColor, mask_texture)) {
    throw std::runtime_error(
        fmt::format("Attempting to update the mask for a geometry that was "
                    "not initialized to have a mask: {}",
                    id));
  }
}

void RenderEngineMaskImage::UpdateLabelMask(
    geometry::GeometryId id,
    const systems::sensors::ImageRgba8U& mask_texture) const {
  // Not being able to update the mask for a label is not an error, the label
  // doens't *have* to be initialized.
  UpdateActorMask(id, ImageType::kLabel, mask_texture);
}

std::unique_ptr<geometry::render::RenderEngine> RenderEngineMaskImage::DoClone()
    const {
  return std::unique_ptr<RenderEngineMaskImage>(
      new RenderEngineMaskImage(*this));
}

bool RenderEngineMaskImage::UpdateActorMask(
    geometry::GeometryId id, int image_type_index,
    const systems::sensors::ImageRgba8U& mask_texture) const {
  const auto& actor_triple = actors().at(id);

  const auto& actor = actor_triple[image_type_index];
  auto* vtk_texture = actor->GetProperty()->GetTexture("masktexture");

  // Can't update the texture if it hasn't been initialized.
  if (vtk_texture == nullptr) return false;

  vtkImageData* image_data = vtk_texture->GetInput();
  DRAKE_DEMAND(image_data != nullptr);
  const int* dim = image_data->GetDimensions();
  if (dim[0] != mask_texture.width() || dim[1] != mask_texture.height()) {
    image_data->SetDimensions(mask_texture.width(), mask_texture.height(), 1);
    image_data->AllocateScalars(VTK_UNSIGNED_CHAR, 4);
  }
  unsigned char* pixel =
      static_cast<unsigned char*>(image_data->GetScalarPointer());
  const int byte_count = mask_texture.width() * mask_texture.height() * 4;
  memcpy(pixel, mask_texture.at(0, 0), byte_count);
  vtk_texture->Modified();

  return true;
}

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
