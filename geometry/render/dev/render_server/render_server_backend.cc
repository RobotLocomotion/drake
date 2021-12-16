#include <filesystem>

#include <gflags/gflags.h>
#include <vtkAutoInit.h>
#include <vtkGLTFImporter.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkWindowToImageFilter.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);

using std::filesystem::path;

// All arguments are required to be supplied on the command line.
DEFINE_string(input, "<input>.gltf", "Input glTF scene to render.");
const char output_default[] = "<output>.png";  // used in validator
DEFINE_string(output, output_default, "Where to save the rendered image.");
DEFINE_int32(width, 0, "The width of the image to render.");
DEFINE_int32(height, 0, "The height of the image to render.");

// Require that input gltf file exists and has the `.gltf` extension.
static bool valid_input(const char* flagname, const std::string& f_path) {
  const path input_f{f_path};
  std::string failure{""};
  const std::string ext = input_f.extension();
  if (!std::filesystem::exists(input_f)) {
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
  const path output_f{f_path};
  std::string failure{""};
  const std::string ext = output_f.extension();
  if (f_path == output_default) {
    failure = "output path explicitly required.";
  } else if (std::filesystem::exists(output_f)) {
    failure = "file already exists!";
  } else if (ext != ".png") {
    failure = "unrecognized file extension '" + ext + "'.";
  }
  if (failure.length() > 0) {
    printf("Invalid value for --%s: %s, %s\n", flagname, f_path.c_str(),
           failure.c_str());
    return false;
  }
  return true;
}
DEFINE_validator(output, &valid_output);

// Require that width and height are at least greater than 0.
static bool valid_dimension(const char* flagname, int val) {
  if (val > 0) return true;
  printf("Invalid value for --%s: %d, must be larger than 0.\n", flagname, val);
  return false;
}
DEFINE_validator(width, &valid_dimension);
DEFINE_validator(height, &valid_dimension);

namespace drake {
namespace geometry {
namespace render {
namespace server_backend {
namespace {

int do_main() {
  vtkNew<vtkGLTFImporter> importer;
  importer->SetFileName(FLAGS_input.c_str());

  vtkNew<vtkRenderer> renderer;
  // TODO(svenevs): sane defaults?
  // renderer->SetBackground(backgroundColor.GetData());
  renderer->UseHiddenLineRemovalOn();

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->SetSize(FLAGS_width, FLAGS_height);
  renderWindow->AddRenderer(renderer);
  renderWindow->SetOffScreenRendering(true);

  importer->SetRenderWindow(renderWindow);
  importer->Update();

  renderWindow->Render();

  auto writer = vtkSmartPointer<vtkPNGWriter>::New();
  vtkNew<vtkWindowToImageFilter> window_to_image_filter;
  window_to_image_filter->SetInput(renderWindow);
  window_to_image_filter->SetScale(1);
  // TODO(svenevs): map the drake types?
  window_to_image_filter->SetInputBufferTypeToRGBA();
  window_to_image_filter->ReadFrontBufferOff();
  window_to_image_filter->Update();

  writer->SetFileName(FLAGS_output.c_str());
  writer->SetInputConnection(window_to_image_filter->GetOutputPort());
  writer->Write();

  return 0;
}

}  // namespace
}  // namespace server_backend
}  // namespace render
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::render::server_backend::do_main();
}
