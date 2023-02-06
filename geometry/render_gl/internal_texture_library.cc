#include "drake/geometry/render_gl/internal_texture_library.h"

#include <algorithm>
#include <cctype>
#include <filesystem>

#include <fmt/format.h>
#include <vtkImageCast.h>
#include <vtkImageData.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkPNGReader.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

TextureLibrary::TextureLibrary(const OpenGlContext* context)
    : context_(context) {
  DRAKE_DEMAND(context_ != nullptr);
}

std::optional<GLuint> TextureLibrary::GetTextureId(
    const std::string& file_name) {
  const auto iter = textures_.find(file_name);
  if (iter != textures_.end()) return iter->second;
  // If it's not a string from which we can even *consider* finding an
  // extension, simply bail out.
  if (file_name.size() < 4) return std::nullopt;

  context_->MakeCurrent();
  // Otherwise load the texture, register the texture.
  std::string ext = file_name.substr(file_name.size() - 4, 4);
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  // TODO(SeanCurtis-TRI) Support other image types.
  if (ext != ".png") return std::nullopt;

  // Exit quickly if the file doesn't exist or is a directory.
  // Otherwise, vtkPNGReader spams stdout.
  if (!std::filesystem::exists(file_name) ||
      std::filesystem::is_directory(file_name)) {
    return std::nullopt;
  }

  vtkNew<vtkPNGReader> png_reader;
  png_reader->SetFileName(file_name.c_str());
  png_reader->Update();
  if (png_reader->GetOutput()->GetScalarType() != VTK_UNSIGNED_CHAR) {
    log()->warn(
        "Texture map '{}' has an unsupported bit depth, casting it to uchar "
        "channels.",
        file_name);
  }

  vtkNew<vtkImageCast> caster;
  caster->SetOutputScalarType(VTK_UNSIGNED_CHAR);
  caster->SetInputConnection(png_reader->GetOutputPort());
  caster->Update();

  vtkNew<vtkImageExport> exporter;
  exporter->SetInputConnection(caster->GetOutputPort());
  exporter->ImageLowerLeftOff();
  exporter->Update();
  vtkImageData* image = exporter->GetInput();
  if (image == nullptr) return std::nullopt;

  const int* dim = image->GetDimensions();
  const int width = dim[0];
  const int height = dim[1];
  // We should have either rgb or rgba data.
  const int num_channels = image->GetNumberOfScalarComponents();
  const GLint internal_format =
      num_channels == 4 ? GL_RGBA : (num_channels == 3 ? GL_RGB : 0);
  if (internal_format == 0) {
      return std::nullopt;
  }

  GLuint texture_id;
  glGenTextures(1, &texture_id);
  glBindTexture(GL_TEXTURE_2D, texture_id);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  unsigned char* pixel =
      static_cast<unsigned char*>(image->GetScalarPointer());
  glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0,
               internal_format, GL_UNSIGNED_BYTE, pixel);
  glGenerateMipmap(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);

  textures_[file_name] = texture_id;

  return texture_id;
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
