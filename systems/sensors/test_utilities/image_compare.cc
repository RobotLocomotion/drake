#include "drake/systems/sensors/test_utilities/image_compare.h"

#include <string>

#include <vtkImageExport.h>
#include <vtkPNGReader.h>
#include <vtkSmartPointer.h>
#include <vtkTIFFReader.h>

namespace drake {
namespace systems {
namespace sensors {

namespace fs = std::filesystem;

using ::testing::AssertionFailure;
using ::testing::AssertionResult;
using ::testing::AssertionSuccess;

template <PixelType kPixelType>
void PrintTo(const Image<kPixelType>& image, std::ostream* os) {
  const int width = image.width();
  const int height = image.height();
  fmt::print(*os, "Image(width={}, height={})", width, height);
  if (width * height > 2048) {
    return;
  }
  using T = typename Image<kPixelType>::T;
  using Promoted = std::conditional_t<std::is_integral_v<T>, int, T>;
  constexpr int num_channels = Image<kPixelType>::kNumChannels;
  for (int z = 0; z < num_channels; ++z) {
    *os << "\n";
    const T* const base = image.at(0, 0) + z;
    using Stride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
    Eigen::Map<const MatrixX<T>, 0, Stride> eigen(
        base, height, width, Stride(num_channels, width * num_channels));
    if (num_channels > 1) {
      fmt::print(*os, "Channel {}:\n", z);
    }
    fmt::print(*os, "{}", fmt_eigen(eigen.template cast<Promoted>()));
  }
}

template <PixelType kPixelType>
AssertionResult LoadImage(const fs::path& filename, Image<kPixelType>* image) {
  if (!fs::exists(filename)) {
    return AssertionFailure() << "File not found: " << filename;
  }
  vtkSmartPointer<vtkImageReader2> reader;
  switch (kPixelType) {
    case PixelType::kRgba8U:
    case PixelType::kGrey8U:
    case PixelType::kDepth16U:
    case PixelType::kLabel16I:
      reader = vtkSmartPointer<vtkPNGReader>::New();
      break;
    case PixelType::kDepth32F:
      reader = vtkSmartPointer<vtkTIFFReader>::New();
      break;
    default:
      return AssertionFailure() << "Called with unsupported PixelType";
  }
  reader->SetFileName(filename.string().c_str());
  vtkNew<vtkImageExport> exporter;
  exporter->ImageLowerLeftOff();
  exporter->SetInputConnection(reader->GetOutputPort());
  exporter->Update();
  const int* const dims = exporter->GetDataDimensions();
  const int width = dims[0];
  const int height = dims[1];
  const int depth = dims[2];
  const int channels = exporter->GetDataNumberOfScalarComponents();
  if (depth != 1) {
    return AssertionFailure() << "Found wrong depth=" << depth;
  }
  if (channels != ImageTraits<kPixelType>::kNumChannels) {
    return AssertionFailure() << "Found wrong channels=" << channels;
  }
  image->resize(width, height);
  exporter->Export(image->at(0, 0));
  return AssertionSuccess();
}

// Explicit template instantiations.
// clang-format off
static constexpr auto kInstantiations __attribute__((used)) = std::make_tuple(
  &PrintTo<PixelType::kRgb8U>,
  &PrintTo<PixelType::kBgr8U>,
  &PrintTo<PixelType::kRgba8U>,
  &PrintTo<PixelType::kBgra8U>,
  &PrintTo<PixelType::kGrey8U>,
  &PrintTo<PixelType::kDepth16U>,
  &PrintTo<PixelType::kDepth32F>,
  &PrintTo<PixelType::kLabel16I>,
  &PrintTo<PixelType::kExpr>,
  &LoadImage<PixelType::kRgb8U>,
  &LoadImage<PixelType::kBgr8U>,
  &LoadImage<PixelType::kRgba8U>,
  &LoadImage<PixelType::kBgra8U>,
  &LoadImage<PixelType::kGrey8U>,
  &LoadImage<PixelType::kDepth16U>,
  &LoadImage<PixelType::kDepth32F>,
  &LoadImage<PixelType::kLabel16I>,
  &LoadImage<PixelType::kExpr>
);
// clang-format on

}  // namespace sensors
}  // namespace systems
}  // namespace drake
