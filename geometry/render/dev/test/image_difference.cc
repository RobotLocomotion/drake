#include <fstream>
#include <iostream>

#include <fmt/format.h>
#include <gflags/gflags.h>
#include <vtkImageDifference.h>
#include <vtkImageExport.h>
#include <vtkImageExtractComponents.h>
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
#include <vtkSmartPointer.h>
#include <vtkTIFFReader.h>
#include <vtkTIFFWriter.h>

#include "drake/common/filesystem.h"

DEFINE_string(source_one, "<first_image>", "The first image.");
DEFINE_string(source_two, "<second_image>", "The second image.");
const char diff_path_default[11] = "<diff.png>";
DEFINE_string(diff_path, diff_path_default,
              "Where to save the image difference.");
const char threshold_path_default[11] = "<diff.txt>";
DEFINE_string(
    threshold_path, threshold_path_default,
    "Path to a text file location to save the computed threshold error "
    "between the images.  The file will contain a single line with a float "
    "that is the result of vtkImageDifference::GetThresholdError().");

// TODO(svenevs): namespaces may need to change...
namespace drake {
namespace geometry {
namespace render {
namespace test {
namespace server {
namespace {

namespace fs = drake::filesystem;

/* Load the specified path as a PNG with RGB components extracted.  Throws
 std::runtime_error if the file cannot be loaded correctly. */
vtkSmartPointer<vtkImageExtractComponents> LoadColorPng(const fs::path& path) {
  vtkNew<vtkPNGReader> reader;
  if (!reader->CanReadFile(path.c_str())) {
    throw std::runtime_error(
        fmt::format("ERROR: cannot read '{}' as a PNG.", path.string()));
  }
  reader->SetFileName(path.c_str());
  reader->Update();
  const int channels = reader->GetNumberOfScalarComponents();
  if (channels != 3 && channels != 4) {
    throw std::runtime_error(fmt::format(
        "ERROR: '{}' has {} channel(s), need either 3 (RGB) or 4 (RGBA).",
        path.string(), channels));
  }
  // vtkImageDifference only supports RGB, mask out alpha if present.
  vtkNew<vtkImageExtractComponents> rgb;
  rgb->SetComponents(0, 1, 2);
  rgb->SetInputConnection(reader->GetOutputPort());
  return rgb;
}

int do_main() {
  const fs::path source_one_path{FLAGS_source_one};
  const fs::path source_two_path{FLAGS_source_two};
  const fs::path diff_path{FLAGS_diff_path};
  const fs::path threshold_path{FLAGS_threshold_path};

  // Make sure we were provided files we can use.
  if (!fs::is_regular_file(source_one_path)) {
    std::cerr << fmt::format("ERROR: '{}' does not exist!\n",
                             source_one_path.string());
    return 1;
  }
  if (!fs::is_regular_file(source_two_path)) {
    std::cerr << fmt::format("ERROR: '{}' does not exist!\n",
                             source_two_path.string());
    return 1;
  }
  // Make sure diff_path and threshold_path were explicitly provided.
  if (diff_path.string() == std::string(diff_path_default)) {
    std::cerr << "ERROR: --diff_path is required to be provided.\n";
    return 1;
  }
  if (threshold_path.string() == std::string(threshold_path_default)) {
    std::cerr << "ERROR: --threshold_path is required to be provided.\n";
    return 1;
  }
  // Make sure we can save to the specified outputs.
  if (fs::exists(diff_path)) {
    std::cerr << fmt::format(
        "ERROR: '{}' already exists, refusing to overwrite.\n",
        diff_path.string());
    return 1;
  }
  if (fs::exists(threshold_path)) {
    std::cerr << fmt::format(
        "ERROR: '{}' already exists, refusing to overwrite.\n",
        threshold_path.string());
    return 1;
  }

  // Load the images.
  vtkSmartPointer<vtkImageExtractComponents> rgb_one{nullptr};
  vtkSmartPointer<vtkImageExtractComponents> rgb_two{nullptr};
  try {
    rgb_one = LoadColorPng(source_one_path);
    rgb_two = LoadColorPng(source_two_path);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  // Compute the image difference and save to disk.
  vtkNew<vtkImageDifference> image_difference;
  image_difference->AllowShiftOff();
  image_difference->SetInputConnection(rgb_one->GetOutputPort());
  image_difference->SetImageConnection(rgb_two->GetOutputPort());
  image_difference->Update();
  vtkNew<vtkPNGWriter> png_out;
  png_out->SetFileName(diff_path.c_str());
  png_out->SetInputConnection(image_difference->GetOutputPort());
  image_difference->Update();
  png_out->Write();

  // Save the computed threshold to disk for the tests to load.
  std::ofstream threshold{threshold_path.string()};
  if (!threshold.good()) {
    std::cerr << fmt::format("ERROR: could not open '{}' for writing.\n",
                             threshold_path.string());
    return 1;
  }
  threshold << image_difference->GetThresholdedError() << '\n';
  threshold.close();

  return 0;
}

}  // namespace
}  // namespace server
}  // namespace test
}  // namespace render
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::render::test::server::do_main();
}
