#include "drake/examples/mesh_painter/painter_shader.h"

#include <fstream>

#include <fmt/format.h>
#include <vtkActor.h>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLTexture.h>
#include <vtkPNGReader.h>
#include <vtkProperty.h>
#include <vtkShader.h>

namespace drake {
namespace examples {
namespace mesh_painter {

using geometry::PerceptionProperties;
using geometry::render::RenderLabel;
using geometry::Rgba;
using systems::sensors::ColorD;
using systems::sensors::ImageRgba8U;
using std::optional;
using std::string;
using std::variant;

namespace {

variant<Rgba, string> FindColor(const char* group, const char* property,
                                const PerceptionProperties& properties) {
  if (!properties.HasProperty(group, property)) {
    throw std::runtime_error(fmt::format(
        "Error in declaration of painter shader; no definition for '{}'. "
        "Should be either a texture name or an RGBA color", property));
  }
  const AbstractValue& value = properties.GetPropertyAbstract(group, property);
  const auto* file_name = value.maybe_get_value<string>();
  if (file_name) {
    if (!std::ifstream(*file_name)) {
      throw std::runtime_error(
          fmt::format("Error in declaration of painter shader; the file for "
                      "'{}' cannot be found: '{}'", property, *file_name));
    }
    return *file_name;
  } else {
    const auto* rgba = value.maybe_get_value<Rgba>();
    if (rgba) return *rgba;
  }
  throw std::runtime_error(fmt::format(
      "Error in declaration of painter shader; the definition for '{}' should "
      "be either a texture file name (std::string) or an RGBA color "
      "(Rgba) -- {} found", property, value.GetNiceTypeName()));
}

void AddTextureFromFile(const string& texture_name, const string& file_name,
                        vtkProperty* property) {
  // TODO(SeanCurtis-TRI): Support more than PNG.
  vtkNew<vtkPNGReader> texture_reader;
  texture_reader->SetFileName(file_name.c_str());
  texture_reader->Update();
  vtkNew<vtkOpenGLTexture> texture;
  texture->SetInputConnection(texture_reader->GetOutputPort());
  texture->SetRepeat(false);
  texture->InterpolateOn();
  property->SetTexture(texture_name.c_str(), texture.Get());
}

constexpr char kRgbaFragmentShader[] = R"__(
    vec4 mask_color = texture({}, tcoordVCVSOutput);
    vec4 canvas_color = {};
    vec4 paint_color = {};
    vec4 final_color =
        mask_color.x * canvas_color + (1 - mask_color.x) * paint_color;
    gl_FragData[0] = gl_FragData[0] * final_color;
)__";

constexpr char kLabelFragmentShader[] = R"__(
    vec4 mask_color = texture({}, tcoordVCVSOutput);
    vec4 canvas_color = {};
    vec4 paint_color = {};
    vec4 final_color = mask_color.x > 0.5 ? canvas_color : paint_color;
    gl_FragData[0] = gl_FragData[0] * final_color;
)__";

}  // namespace

optional<PainterShader> PainterShader::Make(
    const PerceptionProperties& properties) {
  const char* kGroup = "paint_shader";

  if (!properties.HasGroup(kGroup)) return std::nullopt;

  variant<Rgba, string> canvas =
      FindColor(kGroup, "canvas_diffuse", properties);
  variant<Rgba, string> paint =
      FindColor(kGroup, "paint_diffuse", properties);

  const bool has_canvas_label = properties.HasProperty(kGroup, "canvas_label");
  const bool has_paint_label = properties.HasProperty(kGroup, "paint_label");
  if (has_canvas_label != has_paint_label) {
    throw std::runtime_error(
        fmt::format("Error in declaration of 'paint_shader' property group; "
                    "when specifying shader labels, both 'canvas_label' and "
                    "'paint_label' must be specified: only '{}_label' has been "
                    "defined", has_canvas_label ? "canvas" : "paint"));
  }
  optional<RenderLabel> canvas_label{};
  optional<RenderLabel> paint_label{};
  if (has_canvas_label) {
    canvas_label = properties.GetProperty<RenderLabel>(kGroup, "canvas_label");
    paint_label = properties.GetProperty<RenderLabel>(kGroup, "paint_label");
    if (*canvas_label == RenderLabel::kDoNotRender ||
        *paint_label == RenderLabel::kDoNotRender) {
      throw std::runtime_error(
          fmt::format("Error in declaration of 'paint_shader' property group; "
                      "Neither 'canvas_label' ({}) nor 'paint_label' ({}) are "
                      "allowed to be '{}'",
                      *canvas_label, *paint_label, RenderLabel::kDoNotRender));
    }
  }

  return PainterShader{canvas, paint, canvas_label, paint_label};
}

void PainterShader::AssignRgbaToActor(vtkActor* actor) const {
  auto* mapper = vtkOpenGLPolyDataMapper::SafeDownCast(actor->GetMapper());

  const std::string canvas_color = canvas_shader_string();
  const std::string paint_color = paint_shader_string();
  mapper->AddShaderReplacement(
      vtkShader::Fragment, "//VTK::TCoord::Impl", true,
      fmt::format(kRgbaFragmentShader, kMaskTextureName, canvas_color,
                  paint_color)
          .c_str(),
      true);

  // Add textures where appropriate.
  AssignMaskTextureToActor(actor);
  if (canvas_color[0] == 't') {
    AddTextureFromFile(kCanvasTextureName, std::get<string>(canvas_color_),
                       actor->GetProperty());
  }

  if (paint_color[0] == 't') {
    AddTextureFromFile(kPaintTextureName, std::get<string>(paint_color_),
                       actor->GetProperty());
  }
  actor->GetProperty()->SetColor(1, 1, 1);
  mapper->Modified();
}

void PainterShader::AssignLabelToActor(
    vtkActor* actor,
    const std::function<ColorD(RenderLabel)>& label_encoder) const {
  DRAKE_DEMAND(canvas_label_.has_value() && paint_label_.has_value());

  const ColorD canvas_color = label_encoder(*canvas_label_);
  const std::string canvas_color_str = fmt::format(
      "vec4({}, {}, {}, 1.0)", canvas_color.r, canvas_color.g, canvas_color.b);
  const ColorD paint_color = label_encoder(*paint_label_);
  const std::string paint_color_str = fmt::format(
      "vec4({}, {}, {}, 1.0)", paint_color.r, paint_color.g, paint_color.b);

  auto* mapper = vtkOpenGLPolyDataMapper::SafeDownCast(actor->GetMapper());
  mapper->AddShaderReplacement(
      vtkShader::Fragment, "//VTK::TCoord::Impl", true,
      fmt::format(kLabelFragmentShader, kMaskTextureName, canvas_color_str,
                  paint_color_str)
          .c_str(),
      true);

  // Add textures where appropriate.
  AssignMaskTextureToActor(actor);
  actor->GetProperty()->SetColor(1, 1, 1);
  mapper->Modified();
}

bool PainterShader::UpdateMaskForActor(
    vtkActor* actor, const systems::sensors::ImageRgba8U& mask_texture) {
  auto* vtk_texture = actor->GetProperty()->GetTexture(kMaskTextureName);

  // Can't update the texture if it hasn't been initialized.
  if (vtk_texture == nullptr) return false;

  vtkImageData* image_data = vtk_texture->GetInput();
  DRAKE_DEMAND(image_data != nullptr);
  const int* dim = image_data->GetDimensions();
  if (dim[0] != mask_texture.width() || dim[1] != mask_texture.height()) {
    image_data->SetDimensions(mask_texture.width(), mask_texture.height(), 1);
    image_data->AllocateScalars(VTK_UNSIGNED_CHAR, ImageRgba8U::kPixelSize);
  }
  unsigned char* pixel =
      static_cast<unsigned char*>(image_data->GetScalarPointer());
  const int byte_count =
      mask_texture.width() * mask_texture.height() * ImageRgba8U::kPixelSize;
  memcpy(pixel, mask_texture.at(0, 0), byte_count);
  vtk_texture->Modified();

  return true;
}

void PainterShader::AssignMaskTextureToActor(vtkActor* actor) const {
  // For initialization, we insert a texture with a place holder chunk of
  // image data.
  vtkNew<vtkImageData> image_data;
  vtkNew<vtkOpenGLTexture> texture;
  texture->SetRepeat(false);
  texture->InterpolateOn();
  texture->SetInputDataObject(image_data.Get());
  actor->GetProperty()->SetTexture(kMaskTextureName, texture.Get());
}

string PainterShader::canvas_shader_string() const {
  return color_shader_string(kCanvasTextureName, canvas_color_);
}

string PainterShader::paint_shader_string() const {
  return color_shader_string(kPaintTextureName, paint_color_);
}

string PainterShader::color_shader_string(
    const string& texture_name, const variant<Rgba, string>& color) {
  const bool is_texture = std::holds_alternative<string>(color);
  if (is_texture) {
    return fmt::format("texture({}, tcoordVCVSOutput)", texture_name);
  } else {
    const Rgba& c = std::get<Rgba>(color);
    return fmt::format("vec4({}, {}, {}, {})", c.r(), c.g(), c.b(), c.a());
  }
}

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
