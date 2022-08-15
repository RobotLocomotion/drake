#include <gflags/gflags.h>
#include <vtkActorCollection.h>
#include <vtkAutoInit.h>
#include <vtkCamera.h>
#include <vtkGLTFImporter.h>
#include <vtkImageExport.h>
#include <vtkLight.h>
#include <vtkMatrix4x4.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLShaderProperty.h>
#include <vtkPNGWriter.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTIFFWriter.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_assert.h"
#include "drake/common/filesystem.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render/shaders/depth_shaders.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/image.h"
// Depth shaders need to use the same callback.
#include "drake/geometry/render_vtk/factory.h"
#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);

// All arguments are required to be supplied on the command line.
DEFINE_string(input, "<input>.gltf", "Input glTF scene to render.");
const char output_default[] = "<output>.png";  // used in validator
DEFINE_string(output, output_default,
              "Where to save the rendered image. .png and .tiff allowed.");
DEFINE_string(
    image_type, "none",
    "Type of image being rendered, one of 'color', 'depth', or 'label'.");
DEFINE_int32(width, 0, "The width of the image to render.");
DEFINE_int32(height, 0, "The height of the image to render.");
DEFINE_double(near, 0.0, "The near clipping plane.");
DEFINE_double(far, 0.0, "The far clipping plane.");
DEFINE_double(focal_x, 0.0, "The focal length x in pixels.");
DEFINE_double(focal_y, 0.0, "The focal length y in pixels.");
DEFINE_double(fov_x, 0.0, "The field of view x in radians.");
DEFINE_double(fov_y, 0.0, "The field of view x in radians.");
DEFINE_double(center_x, 0.0, "The principal point's x coordinate in pixels.");
DEFINE_double(center_y, 0.0, "The principal point's y coordinate in pixels.");
DEFINE_double(min_depth, -1.0,
              "The minimum depth range.  Required when image_type='depth'.");
DEFINE_double(max_depth, -1.0,
              "The maximum depth range.  Required when image_type='depth'.");

// Require that input gltf file exists and has the `.gltf` extension.
static bool valid_input(const char* flagname, const std::string& f_path) {
  const drake::filesystem::path input_f{f_path};
  std::string failure{""};
  const std::string ext = input_f.extension();
  if (!drake::filesystem::exists(input_f)) {
    failure = "file does not exist!";
  } else if (ext != ".gltf") {
    failure = "unrecognized file extension " + ext + "'.";
  }
  if (failure.length() > 0) {
    printf("Invalid value for --%s: %s, %s\n", flagname, f_path.c_str(),
           failure.c_str());
    return false;
  }
  return true;
}
DEFINE_validator(input, &valid_input);

// Require that output png specified, does not exist, and has `.png` extension.
static bool valid_output(const char* flagname, const std::string& f_path) {
  const drake::filesystem::path output_f{f_path};
  std::string failure{""};
  const std::string ext = output_f.extension();
  if (f_path == output_default) {
    failure = "output path explicitly required.";
  } else if (drake::filesystem::exists(output_f)) {
    failure = "file already exists!";
  } else if (ext != ".png" && ext != ".tiff") {
    failure = "unrecognized file extension '" + ext +
              "', only '.png' and "
              "'.tiff' are supported.";
  }
  if (failure.length() > 0) {
    printf("Invalid value for --%s: %s, %s\n", flagname, f_path.c_str(),
           failure.c_str());
    return false;
  }
  return true;
}
DEFINE_validator(output, &valid_output);

// Require that image_type is provided explicitly and in {color, depth, label}.
static bool valid_image_type(const char* flagname,
                             const std::string& image_type) {
  if (image_type == "color")
    return true;
  else if (image_type == "depth")
    return true;
  else if (image_type == "label")
    return true;

  printf("Invalid value for --%s: %s, %s\n", flagname, image_type.c_str(),
         "must be one of 'color', 'depth', or 'label'.");
  return false;
}
DEFINE_validator(image_type, &valid_image_type);

// Require that width and height are at least greater than 0.
static bool valid_dimension(const char* flagname, int val) {
  if (val > 0) return true;
  printf("Invalid value for --%s: %d, must be larger than 0.\n", flagname, val);
  return false;
}
DEFINE_validator(width, &valid_dimension);
DEFINE_validator(height, &valid_dimension);

// Require that camera intrinsics are provided.
static bool valid_intrinsic(const char* flagname, double val) {
  if (val > 0.0) return true;
  printf("Invalid value for --%s: %f, must be larger than 0.\n", flagname, val);
  return false;
}
DEFINE_validator(near, &valid_intrinsic);
DEFINE_validator(far, &valid_intrinsic);
DEFINE_validator(focal_x, &valid_intrinsic);
DEFINE_validator(focal_y, &valid_intrinsic);
DEFINE_validator(fov_x, &valid_intrinsic);
DEFINE_validator(fov_y, &valid_intrinsic);
DEFINE_validator(center_x, &valid_intrinsic);
DEFINE_validator(center_y, &valid_intrinsic);

namespace drake {
namespace geometry {
namespace render {
namespace render_server {
namespace {

using drake::geometry::render::RenderLabel;
using drake::systems::sensors::ColorD;
using drake::systems::sensors::ColorI;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageRgba8U;
using drake::systems::sensors::ImageTraits;
using drake::systems::sensors::PixelType;

// Imported from render_engine_vtk.cc, for converting the depth image.
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

// Imported from RenderEngine (protected), for converting the label image.
ColorI GetColorIFromLabel(const RenderLabel& label) {
  using ValueType = RenderLabel::ValueType;
  ValueType label_value = static_cast<ValueType>(label);
  return systems::sensors::ColorI{label_value & 0xFF, (label_value >> 8) & 0xFF,
                                  0};
}

// Imported from RenderEngine (protected), for converting the label image.
ColorD GetColorDFromLabel(const RenderLabel& label) {
  systems::sensors::ColorI i_color = GetColorIFromLabel(label);
  return systems::sensors::ColorD{i_color.r / 255., i_color.g / 255.,
                                  i_color.b / 255.};
}

int do_main() {
  if (FLAGS_image_type == "depth") {
    if (FLAGS_min_depth < 0.0 || FLAGS_max_depth < 0.0) {
      printf(
          "min_depth=%f and max_depth=%f must be provided for depth images, "
          "and must be positive.\n",
          FLAGS_min_depth, FLAGS_max_depth);
      return 1;
    }
    if (FLAGS_max_depth <= FLAGS_min_depth) {
      printf("max_depth=%f must be greater than min_depth=%f.\n",
             FLAGS_min_depth, FLAGS_max_depth);
      return 1;
    }
    // TODO(svenevs): add support for 16U PNG images.
    if (drake::filesystem::path(FLAGS_output).extension() == ".png") {
      printf("Depth render to .png currently not supported...\n");
      return 0;
    }
  } else {  // color and label are exported as PNG files.
    drake::filesystem::path output_path{FLAGS_output};
    const std::string ext = output_path.extension();
    if (ext != ".png") {
      printf(
          "image_type=%s may only save to a .png file, but requested output "
          "was '%s'.\n",
          FLAGS_image_type.c_str(), FLAGS_output.c_str());
      return 1;
    }
  }

  if (FLAGS_near >= FLAGS_far) {
    printf(
        "Camera near plane of %f cannot be greater than or equal to the "
        "far plane of %f.\n",
        FLAGS_near, FLAGS_far);
    return 1;
  }

  vtkNew<vtkRenderer> renderer;
  renderer->UseHiddenLineRemovalOn();

  vtkNew<vtkRenderWindow> render_window;
  render_window->SetSize(FLAGS_width, FLAGS_height);
  render_window->AddRenderer(renderer);
  render_window->SetOffScreenRendering(true);

  vtkNew<vtkWindowToImageFilter> window_to_image_filter;
  window_to_image_filter->SetInput(render_window);
  window_to_image_filter->SetScale(1);
  window_to_image_filter->SetInputBufferTypeToRGBA();
  window_to_image_filter->ReadFrontBufferOff();

  // Import the glTF scene.
  vtkNew<vtkGLTFImporter> importer;
  importer->SetFileName(FLAGS_input.c_str());
  importer->SetRenderWindow(render_window);
  // Assumes `camera 0` has been defined in the glTF file.  The vtkGLTFExporter
  // in drake always provides one camera per scene.  If not present, a warning
  // will be issued.  This will put the camera at the correct location.
  importer->SetCamera(0);
  importer->Update();

  // Setup the explicit projection matrix rather than use the one from glTF.
  {
    vtkCamera* camera = renderer->GetActiveCamera();
    camera->UseExplicitProjectionTransformMatrixOn();
    // See notes in render_engine_vtk.cc.
    window_to_image_filter->SetShouldRerender(false);

    // See implementation of RenderCameraCore::CalcProjectionMatrix(), imported
    // here since everything comes in on the command line.
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

  // Render specific tuning, items copied from render_engine_vtk.cc to match the
  // desired rendering parameters that are not encapsulated by glTF export.
  {
    // Applied to all pipelines.
    render_window->SetMultiSamples(0);

    // NOTE: the lighting from RenderEngineVtk is not exported, this is the same
    // as the default lighting created in RenderEngineVtk.
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

  if (FLAGS_image_type == "color") {
    renderer->SetUseDepthPeeling(1);
    renderer->UseFXAAOn();
    // Same default as defined in render_vtk/factory.h.
    renderer->SetBackground(204.0 / 255.0, 229.0 / 255.0, 255.0 / 255.0);
    renderer->SetBackgroundAlpha(1.0);

    /* The glTF import / export uses the newer "PBR Materials" in VTK 9+,
     including for materials without textures (it is a different lighting system
     altogether).  To match the original RenderEngineVtk, for any actor that
     does not have an explicit texture applied, revert to the old lighting.

     NOTE: the glTF standard requires PBR textures.
     */
    vtkActorCollection* actor_collection = renderer->GetActors();
    actor_collection->InitTraversal();
    for (vtkIdType i = 0; i < actor_collection->GetNumberOfItems(); ++i) {
      vtkActor* actor = actor_collection->GetNextActor();
      const auto& tex_map = actor->GetProperty()->GetAllTextures();
      // Textured items will have an "albedoTex" in their tex_map.
      if (tex_map.size() == 0) {
        actor->GetProperty()->SetInterpolationToGouraud();
      }
    }
  } else if (FLAGS_image_type == "depth") {
    renderer->SetBackground(1.0, 1.0, 1.0);

    vtkNew<internal::ShaderCallback> uniform_setting_callback;
    uniform_setting_callback->set_z_near(FLAGS_min_depth);
    uniform_setting_callback->set_z_far(FLAGS_max_depth);

    // Set the custom drake vertex / fragment shader for each actor imported.
    vtkActorCollection* actor_collection = renderer->GetActors();
    actor_collection->InitTraversal();
    for (vtkIdType i = 0; i < actor_collection->GetNumberOfItems(); ++i) {
      vtkActor* actor = actor_collection->GetNextActor();
      vtkOpenGLPolyDataMapper* mapper =
          vtkOpenGLPolyDataMapper::SafeDownCast(actor->GetMapper());
      if (mapper && mapper->GetInput()) {
        vtkOpenGLShaderProperty* sp =
            vtkOpenGLShaderProperty::SafeDownCast(actor->GetShaderProperty());
        DRAKE_DEMAND(sp != nullptr);
        sp->SetVertexShaderCode(shaders::kDepthVS);
        sp->SetFragmentShaderCode(shaders::kDepthFS);
        mapper->AddObserver(vtkCommand::UpdateShaderEvent,
                            uniform_setting_callback.Get());
      }
    }
  } else if (FLAGS_image_type == "label") {
    const ColorD empty_color = GetColorDFromLabel(RenderLabel::kEmpty);
    renderer->SetBackground(empty_color.r, empty_color.g, empty_color.b);

    // Same as RenderEngineVtk, label actors have lighting disabled.
    vtkActorCollection* actor_collection = renderer->GetActors();
    actor_collection->InitTraversal();
    for (vtkIdType i = 0; i < actor_collection->GetNumberOfItems(); ++i) {
      vtkActor* actor = actor_collection->GetNextActor();
      actor->GetProperty()->LightingOff();
    }
  }

  // Render the scene and export to a vtkWindowToImageFilter for use with
  // exporting to the desired output file.
  render_window->Render();

  if (FLAGS_image_type == "color") {
    // Exporting the color image is straightforward, just use the normal VTK
    // pipeline to save the final results from the render_window.
    vtkNew<vtkPNGWriter> writer;
    writer->SetFileName(FLAGS_output.c_str());
    writer->SetInputConnection(window_to_image_filter->GetOutputPort());
    writer->Write();
  } else if (FLAGS_image_type == "depth") {
    // For the depth image, we need to unpack the RGB channels from the shader
    // in the same way that RenderEngineVtk does.  The conversion code is nearly
    // identical to RenderEngineVtk.
    // First: export the image to a local buffer.
    vtkNew<vtkImageExport> image_export;
    image_export->SetInputConnection(window_to_image_filter->GetOutputPort());
    image_export->Update();
    ImageRgba8U rgb_image(FLAGS_width, FLAGS_height);
    image_export->Export(rgb_image.at(0, 0));

    // Next: convert this to a 32bit floating point format.
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
          // Decoding three channel color values to a float value. For the
          // detail, see depth_shaders.h.
          float shader_value = rgb_image.at(u, v)[0] +
                               rgb_image.at(u, v)[1] / 255. +
                               rgb_image.at(u, v)[2] / (255. * 255.);

          // Dividing by 255 so that the range gets to be [0, 1].
          shader_value /= 255.f;
          // TODO(kunimatsu-tri) Calculate this in a vertex shader.
          depth_image_out.at(u, v)[0] =
              CheckRangeAndConvertToMeters(shader_value, min_depth, max_depth);
        }
      }
    }

    // TODO(svenevs): support for 16U PNG export here (millimeters).
    /* Allocate a single channel float image.  Note that vtkImageData supports
     3D data, the last channel in SetDimensions being `z`.  We are 2D, so z=1.
     */
    vtkNew<vtkTIFFWriter> writer;
    vtkNew<vtkImageData> image_data;
    writer->SetInputDataObject(image_data);
    image_data->SetDimensions(FLAGS_width, FLAGS_height, 1);  // Last param is z
    image_data->AllocateScalars(VTK_FLOAT, 1);
    float* ptr = static_cast<float*>(image_data->GetScalarPointer(0, 0, 0));
    for (int y = 0; y < FLAGS_height; ++y) {
      for (int x = 0; x < FLAGS_width; ++x) {
        *ptr++ = depth_image_out.at(x, y)[0];
      }
    }
    writer->SetFileName(FLAGS_output.c_str());
    writer->Write();
  } else {  // FLAGS_image_type == "label"
    /* For the label image, we will export to a single channel PNG image with
     16 bits per pixel.  Extract the value using the same code from drake that
     RenderEngineVtk does, then save a PNG.
     */
    // First: export the image to a local buffer.
    vtkNew<vtkImageExport> image_export;
    image_export->SetInputConnection(window_to_image_filter->GetOutputPort());
    image_export->Update();
    ImageRgba8U rgb_image(FLAGS_width, FLAGS_height);
    image_export->Export(rgb_image.at(0, 0));

    /* Allocate a single channel ushort image.  Note that vtkImageData supports
     3D data, the last channel in SetDimensions being `z`.  We are 2D, so z=1.
     */
    vtkNew<vtkPNGWriter> writer;
    vtkNew<vtkImageData> image_data;
    writer->SetInputDataObject(image_data);
    image_data->SetDimensions(FLAGS_width, FLAGS_height, 1);  // Last param is z
    image_data->AllocateScalars(VTK_UNSIGNED_SHORT, 1);

    ColorI color;
    uint16_t* ptr =
        static_cast<uint16_t*>(image_data->GetScalarPointer(0, 0, 0));
    for (int y = 0; y < FLAGS_height; ++y) {
      for (int x = 0; x < FLAGS_width; ++x) {
        color.r = rgb_image.at(x, y)[0];
        color.g = rgb_image.at(x, y)[1];
        color.b = rgb_image.at(x, y)[2];
        // This is from RenderLabel::LabelFromColor(ColorI) protected member.
        RenderLabel::ValueType label =
            static_cast<RenderLabel::ValueType>(color.r | (color.g << 8));
        DRAKE_DEMAND(label >= 0);
        *ptr++ = static_cast<uint16_t>(label);
      }
    }
    writer->SetFileName(FLAGS_output.c_str());
    writer->Write();
  }

  return 0;
}

}  // namespace
}  // namespace render_server
}  // namespace render
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::render::render_server::do_main();
}
