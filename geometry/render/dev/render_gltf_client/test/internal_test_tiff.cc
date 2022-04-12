#include "drake/geometry/render/dev/render_gltf_client/test/internal_test_tiff.h"

#include <limits>
#include <vector>

#include <tiffio.h>

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

template <int channels, int bit_depth>
TestTiff<channels, bit_depth>::TestTiff(const std::string& path, int width,
                                        int height)
    : path_{path}, width_{width}, height_{height} {
  /* See the docs:
    http://www.libtiff.org/libtiff.html

    If something goes wrong, throw an exception to fail the test.  Each is
    marked to exclude coverage as they do not happen in normal tests. */
  TIFF* tiff = TIFFOpen(path_.c_str(), "w");
  if (tiff == nullptr) {
    // LCOV_EXCL_START
    throw std::runtime_error("Could not create TIFF* for tiff.");
    // LCOV_EXCL_STOP
  }

  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, width_);
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, height_);
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, channels);
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, sizeof(ChannelType) * 8);
  TIFFSetField(tiff, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
  /* Extra fields by trial and error using `tiffinfo` and `gimp` to read the
    file after writing.  Useful table with links to the field meanings:
    https://www.loc.gov/preservation/digital/formats/content/tiff_tags.shtml */
  if constexpr (channels == 1) {
    TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  } else if constexpr (channels == 3) {
    TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
  }
  if constexpr (std::is_same_v<ChannelType, uint8_t> ||
                std::is_same_v<ChannelType, uint16_t>) {
    TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
  } else if constexpr (std::is_same_v<ChannelType, float>) {
    TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  }

  // First, populate an image buffer.  Then use libtiff to write to disk.
  std::vector<ChannelType> image;
  for (int idx = 0; idx < width_ * height_ * channels; ++idx) {
    image.emplace_back(TestTiffValue(idx));
  }
  for (int j = 0; j < height_; ++j) {
    const int scan_id = j * width_ * channels;
    TIFFWriteScanline(tiff, static_cast<void*>(&image[scan_id]), j, 0);
  }
  TIFFClose(tiff);
}

template <int channels, int bit_depth>
typename TestTiff<channels, bit_depth>::ChannelType
TestTiff<channels, bit_depth>::TestTiffValue(int idx) {
  if constexpr (std::is_same_v<ChannelType, uint8_t> ||
                std::is_same_v<ChannelType, uint16_t>) {
    return static_cast<ChannelType>(idx %
                                    std::numeric_limits<ChannelType>::max());
  } else {
    return static_cast<ChannelType>(idx);
  }
}

// Instantiate the templates that test cases need.
template class TestTiff<1, 8>;   // TestTiffGray8
template class TestTiff<1, 16>;  // TestTiffGray16
template class TestTiff<1, 32>;  // TestTiffGray32
template class TestTiff<3, 32>;  // TestTiffRgb32

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
