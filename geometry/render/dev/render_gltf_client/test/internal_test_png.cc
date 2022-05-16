#include "drake/geometry/render/dev/render_gltf_client/test/internal_test_png.h"

#include <limits>

#include <png.h>

#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace {

using systems::sensors::ImageRgba8U;

// Returns the `color_type` for `png_set_IHDR`.
template <int channels>
int ColorType() {
  static_assert(channels == 1 || channels == 3 || channels == 4,
                "Only Gray, RGB, and RGBA supported.");
  if constexpr (channels == 1) {
    return PNG_COLOR_TYPE_GRAY;
  } else if constexpr (channels == 3) {
    return PNG_COLOR_TYPE_RGB;
  } else {  // channels := 4
    return PNG_COLOR_TYPE_RGB_ALPHA;
  }
}

}  // namespace

template <int channels, int bit_depth>
TestPng<channels, bit_depth>::TestPng(const std::string& path, int width,
                                      int height)
    : path_{path}, width_{width}, height_{height} {
  /* See the docs:
   http://www.libpng.org/pub/png/libpng-1.0.3-manual.html

   If something goes wrong, throw an exception to fail the test.  Each is
   marked to exclude coverage as they do not happen in normal tests. */
  FILE* fp = fopen(path.c_str(), "wb");
  if (fp == nullptr) {
    // LCOV_EXCL_START
    throw std::runtime_error("Could not create FILE* for png.");
    // LCOV_EXCL_STOP
  }
  png_structp png_ptr =
      png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (png_ptr == nullptr) {
    // LCOV_EXCL_START
    fclose(fp);
    throw std::runtime_error("Could not create png_ptr for png.");
    // LCOV_EXCL_STOP
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    // LCOV_EXCL_START
    png_destroy_write_struct(&png_ptr, static_cast<png_infopp>(nullptr));
    fclose(fp);
    throw std::runtime_error("Error creating info_ptr for png.");
    // LCOV_EXCL_STOP
  }

  // Error callback for libpng.
  if (setjmp(png_jmpbuf(png_ptr))) {
    // LCOV_EXCL_START
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
    throw std::runtime_error("Critical error trying to write png.");
    // LCOV_EXCL_STOP
  }

  // Setup and write out the header information.
  png_init_io(png_ptr, fp);
  png_set_IHDR(png_ptr, info_ptr, width_, height_, bit_depth,
               ColorType<channels>(), PNG_INTERLACE_NONE,
               PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png_ptr, info_ptr);

  /* Populate the pixel data.  See constructor comments.
   NOTE: do not use png_set_filler! */
  if (bit_depth > 8) png_set_swap(png_ptr);
  ChannelType* row_data[height_];
  ChannelType gray{kGrayStart};
  ChannelType red{kRedStart};
  ChannelType green{kGreenStart};
  ChannelType blue{kBlueStart};
  ChannelType alpha{kAlphaStart};
  /* Due to the template, and this method supporting gray, RGB, and RGBA, we
   must mark them as "unused" since depending on the template a given variable
   may not be used in the loop. */
  unused(gray);
  unused(red);
  unused(green);
  unused(blue);
  unused(alpha);
  for (int j = 0; j < height_; ++j) {
    row_data[j] = static_cast<ChannelType*>(
        png_malloc(png_ptr, png_get_rowbytes(png_ptr, info_ptr)));
    ChannelType* row_ptr = row_data[j];
    for (int i = 0; i < width_; ++i) {
      for (int k = 0; k < channels; ++k) {
        if constexpr (channels == 1) {
          *row_ptr++ = TestPngValue(&gray);
        } else if constexpr (channels == 3 || channels == 4) {
          if (k == 0) {
            *row_ptr++ = TestPngValue(&red);
          } else if (k == 1) {
            *row_ptr++ = TestPngValue(&green);
          } else if (k == 2) {
            *row_ptr++ = TestPngValue(&blue);
          } else if (k == 3) {
            *row_ptr++ = TestPngValue(&alpha);
          }
        }
      }
    }
  }
  // NOTE: reinterpret_cast is needed for uint16_t data.
  png_write_rows(png_ptr, reinterpret_cast<png_bytepp>(row_data), height);

  // Finalize the png image.
  png_write_end(png_ptr, info_ptr);
  for (int j = 0; j < height_; ++j) {
    png_free(png_ptr, row_data[j]);
  }
  png_destroy_write_struct(&png_ptr, &info_ptr);
  fclose(fp);
}

template <int channels, int bit_depth>
typename TestPng<channels, bit_depth>::ChannelType
TestPng<channels, bit_depth>::TestPngValue(ChannelType* current) {
  const ChannelType ret = *current;
  *current = static_cast<ChannelType>(
      (static_cast<uint32_t>(*current) + static_cast<uint32_t>(1)) %
      static_cast<uint32_t>(std::numeric_limits<ChannelType>::max()));
  return ret;
}

template <int channels, int bit_depth>
void TestPng<channels, bit_depth>::CornerCheckColor(const ImageRgba8U& image) {
  constexpr bool source_had_alpha = channels == 4;
  // Safeguard against incorrect usage of this function with non color types.
  EXPECT_TRUE(channels == 3 || channels == 4);

  EXPECT_EQ(image.at(0, 0)[0], kRedStart);
  EXPECT_EQ(image.at(0, 0)[1], kGreenStart);
  EXPECT_EQ(image.at(0, 0)[2], kBlueStart);
  // For RGBA, use the expected value.  RGB, alpha is padded with 255.
  if constexpr (source_had_alpha) {
    EXPECT_EQ(image.at(0, 0)[3], kAlphaStart);
  } else {
    EXPECT_EQ(image.at(0, 0)[3], kAlphaPad);
  }
}

template <int channels, int bit_depth>
void TestPng<channels, bit_depth>::FullImageCheckColor(
    const ImageRgba8U& image) {
  constexpr bool source_had_alpha = channels == 4;
  // Safeguard against incorrect usage of this function with non color types.
  EXPECT_TRUE(channels == 3 || channels == 4);

  int n_good{0};
  const int n_expected = image.width() * image.height() * 4;
  ChannelType red{kRedStart};
  ChannelType green{kGreenStart};
  ChannelType blue{kBlueStart};
  ChannelType alpha{kAlphaStart};
  unused(alpha);  // Due to template, not used for channels == 3 (RGB).
  for (int j = 0; j < image.height(); ++j) {
    for (int i = 0; i < image.width(); ++i) {
      n_good += static_cast<int>(image.at(i, j)[0] == TestPngValue(&red));
      n_good += static_cast<int>(image.at(i, j)[1] == TestPngValue(&green));
      n_good += static_cast<int>(image.at(i, j)[2] == TestPngValue(&blue));
      // For RGBA, use the expected value.  RGB, alpha is padded with 255.
      if (source_had_alpha) {
        n_good += static_cast<int>(image.at(i, j)[3] == TestPngValue(&alpha));
      } else {
        n_good += static_cast<int>(image.at(i, j)[3] == kAlphaPad);
      }
    }
  }
  EXPECT_EQ(n_good, n_expected);
}

// Instantiate the templates that test cases need.
template class TestPng<3, 8>;
template class TestPng<4, 8>;
template class TestPng<1, 16>;
template class TestPng<3, 16>;
template class TestPng<1, 8>;

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
