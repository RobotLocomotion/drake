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
namespace render_gl {
namespace internal {

namespace fs = std::filesystem;

std::optional<GLuint> TextureLibrary::GetTextureId(
    const std::string& file_name_in) {
  const std::string file_name = GetTextureKey(file_name_in);

  // We'll simply serialize all calls to GetTextureId(). As this act is purely
  // an initialization operation (i.e., we get the texture id when *adding*
  // an object with a texture), simple serialization isn't harmful.
  std::lock_guard<std::mutex> lock(mutex_);
  const auto iter = textures_.find(file_name);
  if (iter != textures_.end()) {
    // This assertion attempts to validate the precondition. It will fail if an
    // OpenGL context isn't bound, or if the "wrong" context is bound -- one
    // that knows nothing of the texture recorded in the library. The test isn't
    // perfect. Two different contexts can both use the same identifier but
    // refer to different textures. However, this much test will be good to
    // help CI detect if we've got systemic issues.
    DRAKE_ASSERT(glIsTexture(iter->second));
    return iter->second;
  }

  // Technically, if this fails, file_name won't be a valid key, so this test
  // could be outside the lock. But as it touches the filesystem, we'll defer
  // the test until we know we have to do it.
  if (!IsSupportedImage(file_name)) {
    return std::nullopt;
  }

  vtkNew<vtkPNGReader> png_reader;
  png_reader->SetFileName(file_name.c_str());
  png_reader->Update();
  if (png_reader->GetOutput()->GetScalarType() != VTK_UNSIGNED_CHAR) {
    log()->warn(
        "Texture map '{}' has an unsupported bit depth, casting it to uchar "
        "channels.",
        file_name_in);
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
  // This will catch the problem that an OpenGl context is not bound; we have no
  // way to tell if the *wrong* context is bound and we create the texture in
  // the wrong place.
  DRAKE_ASSERT(glGetError() == GL_NO_ERROR);
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

std::string TextureLibrary::GetTextureKey(const std::string& file_name) {
  const fs::path path_in(file_name);
  const fs::path file_path =
      fs::is_symlink(path_in) ? fs::read_symlink(path_in) : path_in;
  return file_path.string();
}

bool TextureLibrary::IsSupportedImage(const std::string& file_name) {
  // TODO(SeanCurtis-TRI): We should dispatch warnings if a `file_name` is not
  // "supported" (and why) instead of silently ignoring it.

  // If it's not a string from which we can even *consider* finding an
  // extension, simply bail out.
  if (file_name.size() < 4) {
    return false;
  }

  // Test for supported extension.
  std::string ext = file_name.substr(file_name.size() - 4, 4);
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  // TODO(SeanCurtis-TRI) Support other image types.
  if (ext != ".png") {
    return false;
  }

  // Exit quickly if the file doesn't exist or is a directory.
  const fs::path file_path(file_name);
  if (!fs::exists(file_path) || fs::is_directory(file_path)) {
    return false;
  }

  return true;
}

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
