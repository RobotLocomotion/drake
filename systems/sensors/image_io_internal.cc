#include "drake/systems/sensors/image_io_internal.h"

#include <array>
#include <fstream>
#include <string>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {

namespace fs = std::filesystem;

std::optional<ImageFileFormat> FileFormatFromExtension(
    const fs::path& filename) {
  std::string ext = filename.extension();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (ext == ".jpeg") {
    return ImageFileFormat::kJpeg;
  } else if (ext == ".jpg") {
    return ImageFileFormat::kJpeg;
  } else if (ext == ".png") {
    return ImageFileFormat::kPng;
  } else if (ext == ".tiff") {
    return ImageFileFormat::kTiff;
  } else if (ext == ".tif") {
    return ImageFileFormat::kTiff;
  }
  return std::nullopt;
}

namespace {
/* Returns the first N bytes of the given input, or else all-zero bytes if
something went wrong (e.g., can't open the file, too short, etc.) */
template <size_t N>
std::array<uint8_t, N> ReadHeader(
    std::variant<const fs::path*, ImageIo::ByteSpan> input_any) {
  std::array<uint8_t, N> result = {0};
  if (input_any.index() == 0) {
    const fs::path& path = *std::get<0>(input_any);
    std::ifstream file(path, std::ios::binary);
    std::array<char, N> buffer;
    file.read(buffer.data(), N);
    if (file.good()) {
      std::memcpy(result.data(), buffer.data(), N);
    }
  } else {
    const ImageIo::ByteSpan& buffer = std::get<1>(input_any);
    if (buffer.size >= N) {
      std::memcpy(result.data(), buffer.data, N);
    }
  }
  return result;
}
}  // namespace

// Implementation note: VTK does have functions to guess the image type, but
// they are pretty complicated. For our narrow set of supported file formats,
// we can do it ourselves on the cheap.
std::optional<ImageFileFormat> GuessFileFormat(
    std::variant<const fs::path*, ImageIo::ByteSpan> input_any) {
  const std::array<uint8_t, 2> header = ReadHeader<2>(input_any);
  // https://en.wikipedia.org/wiki/JPEG_File_Interchange_Format
  if (header[0] == 0xff && header[1] == 0xd8) {
    return ImageFileFormat::kJpeg;
  }
  // https://en.wikipedia.org/wiki/PNG
  if (header[0] == 0x89 && header[1] == 0x50) {
    return ImageFileFormat::kPng;
  }
  // https://en.wikipedia.org/wiki/TIFF
  if ((header[0] == 0x49 && header[1] == 0x49) ||
      (header[0] == 0x4d && header[1] == 0x4d)) {
    return ImageFileFormat::kTiff;
  }
  return std::nullopt;
}

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
