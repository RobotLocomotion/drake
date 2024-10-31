/* This file implements a program that can consume a glTF file and produce a
single rendered image.  The sample code is kept straightforward deliberately
without too much optimization.  As this program and RenderEngineVtk both use VTK
to render images, some functions are ported directly from RenderEngineVtk.
See also:
https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1render_1_1_render_engine_vtk.html

In the glTF Render Client-Server pipeline, the server receives HTTP requests
from a client and invokes this program as a rendering backend to render images.
See README.md in this folder for more details.

The program serves as a working example and is used in conjunction with other
programs for the glTF Render Client-Server integration test. */

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>

#include <gflags/gflags.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkActorCollection.h>       // vtkRenderingCore
#include <vtkAutoInit.h>              // vtkCommonCore
#include <vtkCamera.h>                // vtkRenderingCore
#include <vtkGLTFImporter.h>          // vtkIOImport
#include <vtkImageExport.h>           // vtkIOImage
#include <vtkLight.h>                 // vtkRenderingCore
#include <vtkMatrix4x4.h>             // vtkCommonMath
#include <vtkOpenGLPolyDataMapper.h>  // vtkRenderingOpenGL2
#include <vtkOpenGLShaderProperty.h>  // vtkRenderingOpenGL2
#include <vtkPNGWriter.h>             // vtkIOImage
#include <vtkProperty.h>              // vtkRenderingCore
#include <vtkRenderWindow.h>          // vtkRenderingCore
#include <vtkRenderer.h>              // vtkRenderingCore
#include <vtkTIFFWriter.h>            // vtkIOImage
#include <vtkWindowToImageFilter.h>   // vtkRenderingCore

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/render/shaders/depth_shaders.h"
#include "drake/geometry/render_vtk/internal_make_render_window.h"
#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"
#include "drake/systems/sensors/image.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);

// Note: All arguments are required to be supplied on the command line. Other
// than `input_path` and `output_path`, this program assumes the rest of the
// inputs are validated beforehand.
DEFINE_string(input_path, "", "The input glTF scene to render.");
DEFINE_string(output_path, "",
              "The path to save the rendered image. Only `.png` and `.tiff` "
              "are supported.");
DEFINE_string(image_type, "color,depth,label",
              "The type of image being rendered.");
DEFINE_int32(width, 0, "The width of the image to render.");
DEFINE_int32(height, 0, "The height of the image to render.");
DEFINE_double(near, 0.0, "The near clipping plane in meters.");
DEFINE_double(far, 0.0, "The far clipping plane in meters.");
DEFINE_double(focal_x, 0.0, "The focal length x in pixels.");
DEFINE_double(focal_y, 0.0, "The focal length y in pixels.");
DEFINE_double(fov_x, 0.0, "The field of view x in radians.");
DEFINE_double(fov_y, 0.0, "The field of view y in radians.");
DEFINE_double(center_x, 0.0, "The principal point's x coordinate in pixels.");
DEFINE_double(center_y, 0.0, "The principal point's y coordinate in pixels.");
DEFINE_double(
    min_depth, -1.0,
    "The minimum depth range.  Only required when rendering a depth image.");
DEFINE_double(
    max_depth, -1.0,
    "The maximum depth range.  Only required when rendering a depth image.");

/* Requires that the input path is specified, exists and with a `.gltf`
 extension. */
static bool ValidateInputPath(const char*, const std::string& path) {
  if (path.empty()) {
    drake::log()->debug("Input path is not specified");
    return false;
  }
  const std::filesystem::path input_path{path};
  if (!std::filesystem::exists(path) || input_path.extension() != ".gltf") {
    drake::log()->debug("Invalid input path: {}", path);
    return false;
  }
  return true;
}
DEFINE_validator(input_path, &ValidateInputPath);

/* Requires that an output path is specified, does not exist, and has a
 supported extension, i.e., .png or .tiff. */
static bool ValidateOutputPath(const char*, const std::string& path) {
  if (path.empty()) {
    drake::log()->debug("Output path is not specified");
    return false;
  }
  const std::filesystem::path output_path{path};
  const std::string ext{output_path.extension()};
  if (std::filesystem::exists(output_path) ||
      (ext != ".png" && ext != ".tiff")) {
    drake::log()->debug("Invalid output path: {}", path);
    return false;
  }
  return true;
}
DEFINE_validator(output_path, &ValidateOutputPath);

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

using render_vtk::internal::MakeRenderWindow;
using render_vtk::internal::RenderEngineVtkBackend;
using render_vtk::internal::ShaderCallback;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;

// Imported from internal_render_engine_vtk.cc, for converting the depth image.
float CheckRangeAndConvertToMeters(float z_buffer_value, double z_near,
                                   double z_far) {
  // This assumes the values have already been encoded to the range [0, 1]
  // inside the image itself. This includes sentinel values:
  //   - The value 1 indicates the depth is too far away.
  //   - The value 0 indicates the depth is too near.
  if (z_buffer_value == 0) return ImageTraits<PixelType::kDepth32F>::kTooClose;
  if (z_buffer_value == 1) return ImageTraits<PixelType::kDepth32F>::kTooFar;
  return static_cast<float>(z_buffer_value * (z_far - z_near) + z_near);
}

// Validates the output file extension based on `image_type`.
bool ValidateOutputExtension() {
  if (FLAGS_image_type == "depth") {
    if (std::filesystem::path(FLAGS_output_path).extension() == ".png") {
      log()->debug("Depth images must have a .tiff extension.");
      return false;
    }
  } else {
    const std::filesystem::path output_path{FLAGS_output_path};
    if (output_path.extension() != ".png") {
      log()->debug("Color and label images must have a .png extension.");
      return false;
    }
  }
  return true;
}

/* Constructs and sets the explicit projection matrix rather than using the one
 from glTF which only supports limited intrinsic specification.  The
 implementation is imported from RenderCameraCore::CalcProjectionMatrix().
 @sa
 https://github.com/RobotLocomotion/drake/blob/master/geometry/render/render_camera.cc
 */
void CalcProjectionMatrix(vtkRenderer* renderer) {
  vtkCamera* camera = renderer->GetActiveCamera();
  camera->UseExplicitProjectionTransformMatrixOn();

  vtkNew<vtkMatrix4x4> projection;
  projection->Zero();
  const double fx = FLAGS_focal_x;
  const double fy = FLAGS_focal_y;
  const double n = FLAGS_near;
  const double f = FLAGS_far;
  const double w = FLAGS_width;
  const double h = FLAGS_height;
  const double cx = FLAGS_center_x;
  const double cy = FLAGS_center_y;
  const double d = f - n;
  projection->SetElement(0, 0, 2 * fx / w);
  projection->SetElement(0, 2, (w - 2 * cx) / w);
  projection->SetElement(1, 1, 2 * fy / h);
  projection->SetElement(1, 2, -(h - 2 * cy) / h);
  projection->SetElement(2, 2, -(f + n) / d);
  projection->SetElement(2, 3, -2 * f * n / d);
  projection->SetElement(3, 2, -1);
  camera->SetExplicitProjectionTransformMatrix(projection);
}

/* Sets the lighting to match the default setting in RenderEngineVtk that are
 not encapsulated by the glTF export. */
void SetDefaultLighting(vtkRenderer* renderer) {
  renderer->AutomaticLightCreationOff();
  vtkNew<vtkLight> light;
  light->SetPosition(0.0, 0.0, 1.0);
  light->SetLightTypeToCameraLight();
  light->SetConeAngle(45.0);
  light->SetAttenuationValues(1.0, 0.0, 0.0);
  light->SetIntensity(1.0);
  vtkNew<vtkMatrix4x4> identity;
  identity->Identity();
  light->SetTransformMatrix(identity);
  renderer->AddLight(light);
}

int DoMain() {
  // All the input args should be validated past this point.
  if (!ValidateOutputExtension()) return 1;

  vtkNew<vtkRenderer> renderer;
  renderer->UseHiddenLineRemovalOn();

  vtkSmartPointer<vtkRenderWindow> render_window = MakeRenderWindow(
#ifdef __APPLE__
      RenderEngineVtkBackend::kCocoa
#else
      RenderEngineVtkBackend::kEgl
#endif
  );  // NOLINT(whitespace/parens)
  render_window->SetSize(FLAGS_width, FLAGS_height);
  render_window->AddRenderer(renderer);
  render_window->SetOffScreenRendering(true);
  // Applied to all pipelines.
  render_window->SetMultiSamples(0);

  vtkNew<vtkWindowToImageFilter> window_to_image_filter;
  window_to_image_filter->SetInput(render_window);
  window_to_image_filter->SetScale(1);
  window_to_image_filter->SetInputBufferTypeToRGBA();
  window_to_image_filter->ReadFrontBufferOff();
  // See notes in internal_render_engine_vtk.cc.
  window_to_image_filter->SetShouldRerender(false);

  /* Import the glTF scene and pose the camera.  `camera 0` is assumed to be
   defined in the glTF file.  The vtkGLTFExporter in drake always provides one
   camera per scene. */
  vtkNew<vtkGLTFImporter> importer;
  importer->SetFileName(FLAGS_input_path.c_str());
  importer->SetRenderWindow(render_window);
  importer->SetCamera(0);
  importer->Update();

  CalcProjectionMatrix(renderer);
  SetDefaultLighting(renderer);

  /* Apply render-specific settings, such as the background color and texture
   property based on the type of the rendered image. */
  if (FLAGS_image_type == "color") {
    renderer->SetUseDepthPeeling(1);
    renderer->UseFXAAOn();
    // Same default background color as defined in RenderEngineVtk.
    renderer->SetBackground(204.0 / 255.0, 229.0 / 255.0, 255.0 / 255.0);
    renderer->SetBackgroundAlpha(1.0);

    /* The glTF import / export uses the newer "PBR Materials" in VTK 9+,
     including for materials without textures (it is a different lighting system
     altogether).  To match the original RenderEngineVtk, for any actor that
     does not have an explicit texture applied, revert to the old lighting.

     NOTE: The glTF standard requires PBR textures. */
    vtkActorCollection* actor_collection = renderer->GetActors();
    actor_collection->InitTraversal();
    for (int i = 0; i < actor_collection->GetNumberOfItems(); ++i) {
      vtkActor* actor = actor_collection->GetNextActor();
      const std::map<std::string, vtkTexture*>& texture_maps =
          actor->GetProperty()->GetAllTextures();
      if (texture_maps.size() == 0) {
        actor->GetProperty()->SetInterpolationToGouraud();
      }
    }
  } else if (FLAGS_image_type == "depth") {
    renderer->SetBackground(1.0, 1.0, 1.0);

    vtkNew<ShaderCallback> uniform_setting_callback;
    uniform_setting_callback->set_z_near(FLAGS_min_depth);
    uniform_setting_callback->set_z_far(FLAGS_max_depth);

    // Set the custom drake vertex / fragment shader for each imported actor.
    vtkActorCollection* actor_collection = renderer->GetActors();
    actor_collection->InitTraversal();
    for (int i = 0; i < actor_collection->GetNumberOfItems(); ++i) {
      vtkActor* actor = actor_collection->GetNextActor();
      vtkOpenGLPolyDataMapper* mapper =
          vtkOpenGLPolyDataMapper::SafeDownCast(actor->GetMapper());
      if (mapper && mapper->GetInput()) {
        vtkOpenGLShaderProperty* shader_property =
            vtkOpenGLShaderProperty::SafeDownCast(actor->GetShaderProperty());
        DRAKE_DEMAND(shader_property != nullptr);
        shader_property->SetVertexShaderCode(render::shaders::kDepthVS);
        shader_property->SetFragmentShaderCode(render::shaders::kDepthFS);
        mapper->AddObserver(vtkCommand::UpdateShaderEvent,
                            uniform_setting_callback.Get());
      }
    }
  } else {  // FLAGS_image_type == "label"
    // Following the client-server API (see render_gltf_client_doxygen.h), the
    // background of a label image should be set to white.
    renderer->SetBackground(1.0, 1.0, 1.0);

    // Same as RenderEngineVtk, label actors have lighting disabled.  Labels
    // have already been encoded as geometry materials in the glTF.  By
    // disabling lighting we also don't have to worry about the same PBR issues
    // as for color images.
    vtkActorCollection* actor_collection = renderer->GetActors();
    actor_collection->InitTraversal();
    for (int i = 0; i < actor_collection->GetNumberOfItems(); ++i) {
      vtkActor* actor = actor_collection->GetNextActor();
      actor->GetProperty()->LightingOff();
    }
  }

  // Render the scene and export to a vtkWindowToImageFilter.
  render_window->Render();

  // Export to the image file via vtk{PNG, TIFF}Writer.
  if (FLAGS_image_type == "depth") {
    /* For the depth image, we need to unpack the RGB channels from the shader
     in the same way that RenderEngineVtk does. */
    // First: export the image to a local buffer.
    vtkNew<vtkImageExport> image_export;
    image_export->SetInputConnection(window_to_image_filter->GetOutputPort());
    image_export->Update();
    ImageRgba8U rgb_image(FLAGS_width, FLAGS_height);
    image_export->Export(rgb_image.at(0, 0));

    // Next: convert this to a 32-bit floating point format.
    ImageDepth32F depth_image_out(FLAGS_width, FLAGS_height);
    const double min_depth = FLAGS_min_depth;
    const double max_depth = FLAGS_max_depth;
    for (int v = 0; v < FLAGS_height; ++v) {
      for (int u = 0; u < FLAGS_width; ++u) {
        if (rgb_image.at(u, v)[0] == 255u && rgb_image.at(u, v)[1] == 255u &&
            rgb_image.at(u, v)[2] == 255u) {
          depth_image_out.at(u, v)[0] =
              ImageTraits<PixelType::kDepth32F>::kTooFar;
        } else {
          /* Decoding three channel color values to a float value. For the
           detail, see depth_shaders.h. */
          float shader_value = rgb_image.at(u, v)[0] +
                               rgb_image.at(u, v)[1] / 255. +
                               rgb_image.at(u, v)[2] / (255. * 255.);

          // Dividing by 255 so that the range gets to be [0, 1].
          shader_value /= 255.f;
          depth_image_out.at(u, v)[0] =
              CheckRangeAndConvertToMeters(shader_value, min_depth, max_depth);
        }
      }
    }

    /* Allocate a single channel float image.  Note that vtkImageData supports
     3D data, the last channel in SetDimensions being `z`. */
    vtkNew<vtkTIFFWriter> writer;
    vtkNew<vtkImageData> image_data;
    writer->SetInputDataObject(image_data);
    image_data->SetDimensions(FLAGS_width, FLAGS_height, 1);
    image_data->AllocateScalars(VTK_FLOAT, 1);
    float* ptr = static_cast<float*>(image_data->GetScalarPointer(0, 0, 0));
    for (int y = 0; y < FLAGS_height; ++y) {
      for (int x = 0; x < FLAGS_width; ++x) {
        *ptr++ = depth_image_out.at(x, y)[0];
      }
    }
    writer->SetFileName(FLAGS_output_path.c_str());
    writer->Write();
  } else {  // FLAGS_image_type == "label" || FLAGS_image_type == "color"
    /* Exporting the color image is straightforward, just use the normal VTK
     pipeline to save the final results from the render_window.

     For the label image, the server returns a colored label image instead. The
     client, RenderEngineGltfClient, is responsible for reading the colored
     label image and converting it to the actual label image.  This decision
     was made because Drake has its own internal interpretation from a color
     value to a label value.  The glTF file for the label images already
     contains RGB values encoding labels. */
    vtkNew<vtkPNGWriter> writer;
    writer->SetFileName(FLAGS_output_path.c_str());
    writer->SetInputConnection(window_to_image_filter->GetOutputPort());
    writer->Write();
  }

  return 0;
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::render_gltf_client::internal::DoMain();
}
