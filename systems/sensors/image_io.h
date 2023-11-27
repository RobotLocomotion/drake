#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <variant>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_file_format.h"

namespace drake {
namespace systems {
namespace sensors {

/** Utility functions for reading and writing images, from/to either files or
memory buffers.

The only file formats supported are JPEG, PNG, and TIFF. The file format is
specified by the ImageFileFormat enum.

If desired, a specific file format can be configured either in the constructor
or via the SetFileFormat() function. When configured, that file format will
always be used, regardless of any filename extension. (In other words, the class
allows loading or saving a JPEG image from/to a filename ending in `.png`.)

If the ImageFileFormat has NOT been configured, the following behaviors apply:
- When loading an image from file or memory, the format will be inferred from
  the data content (ignoring the filename).
- When saving an image to disk, the filename extension will determine the
  format. In this case, the filename extension must be a conventional choice
  (`.jpg`, `.jpeg`, `.png`, `.tif`, or `.tiff`).
- When saving an image to memory, there is no default format; this will throw
  an exception.

The only file format that supports images with floating-point scalars is TIFF.
Trying to load or save a floating-point image from/to a PNG or JPEG file will
throw an exception. */
class ImageIo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageIo);

  /** Some characteristics of an image file.
  Note that Drake's Image<> class can only express `depth == 1`. */
  struct Metadata {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(format));
      a->Visit(DRAKE_NVP(width));
      a->Visit(DRAKE_NVP(height));
      a->Visit(DRAKE_NVP(depth));
      a->Visit(DRAKE_NVP(channels));
      a->Visit(DRAKE_NVP(scalar));
    }

    ImageFileFormat format{ImageFileFormat::kJpeg};
    int width{};
    int height{};
    int depth{};
    int channels{};
    PixelScalar scalar{PixelScalar::k8U};
  };

  /** When loading from memory, this struct denotes a span of raw bytes as
  input. */
  struct ByteSpan {
    /** Pointer to the first byte of the span. */
    const void* data{};
    /** Total number of bytes in the span. */
    size_t size{};
  };

  /** Default constructor (with no file format configured). */
  ImageIo() = default;

  /** Constructs an %ImageIo with SetFileFormat(format) already in place. */
  explicit ImageIo(ImageFileFormat format) : format_(format) {}

  /** Returns the metadata of the given image file, or nullopt if the metadata
  cannot be determined or is unsupported.
  @warning This function obeys the configured file format from the constructor
  or a call to SetFileFormat(). If you want it to *infer* the file format from
  the file contents, be sure not to configure any specific ImageFileFormat. */
  std::optional<Metadata> LoadMetadata(
      const std::filesystem::path& path) const {
    return LoadMetadataImpl(&path);
  }

  /** Returns the metadata of the given image buffer, or nullopt if the metadata
  cannot be determined or is unsupported.
  @warning This function obeys the configured file format from the constructor
  or a call to SetFileFormat(). If you want it to *infer* the file format from
  the file contents, be sure not to configure any specific ImageFileFormat. */
  std::optional<Metadata> LoadMetadata(ByteSpan buffer) const {
    return LoadMetadataImpl(buffer);
  }

  /** Configures this %ImageIo object to load and save using the given format.
  Refer to the class overview documentation for a full explanation. */
  void SetFileFormat(std::optional<ImageFileFormat> format) {
    format_ = format;
  }

  /** Returns the most recent value passed to SetFileFormat, or nullopt if no
  format has ever been set. */
  std::optional<ImageFileFormat> GetFileFormat() const { return format_; }

  /** Loads and returns an image from disk.
  @throws std::exception for any kind of error loading the image file. */
  ImageAny Load(const std::filesystem::path& path) const {
    return LoadImpl(&path);
  }

  /** Loads and returns an image from a memory buffer.
  @throws std::exception for any kind of error loading the image data. */
  ImageAny Load(ByteSpan buffer) const { return LoadImpl(buffer); }

  /** Loads and outputs an image from disk.
  @param[out] image The output image (which will be overwritten).
  @throws std::exception for any kind of error loading the image file.
  @throws std::exception if the loaded image does not match the kPixelType. */
  template <PixelType kPixelType>
  void Load(const std::filesystem::path& path, Image<kPixelType>* image) const {
    LoadImpl(&path, image);
  }

  /** Loads and outputs an image from a memory buffer.
  @param[out] image The output image (which will be overwritten).
  @throws std::exception for any kind of error loading the image data.
  @throws std::exception if the loaded image does not match the kPixelType. */
  template <PixelType kPixelType>
  void Load(ByteSpan buffer, Image<kPixelType>* image) const {
    LoadImpl(buffer, image);
  }

  /** Saves an image to disk.
  @throws std::exception for any kind of error saving the image file. */
  template <PixelType kPixelType>
  void Save(const Image<kPixelType>& image,
            const std::filesystem::path& path) const {
    SaveImpl(&image, &path);
  }

  /** Saves an image to a new memory buffer, returning the buffer.
  @pre SetFormat() has already been called with a non-null format, to determine
  what format to save as.
  @throws std::exception for any kind of error saving the image data. */
  template <PixelType kPixelType>
  std::vector<uint8_t> Save(const Image<kPixelType>& image) const {
    std::vector<uint8_t> result;
    SaveImpl(&image, &result);
    return result;
  }

  /** Saves an image to an existing memory buffer.
  @param[out] buffer The output buffer (which will be overwritten).
  @pre SetFormat() has already been called with a non-null format, to determine
  what format to save as.
  @throws std::exception for any kind of error saving the image data. */
  template <PixelType kPixelType>
  void Save(const Image<kPixelType>& image,
            std::vector<uint8_t>* buffer) const {
    SaveImpl(&image, buffer);
  }

 private:
  // ImageAnyConstPtr is like ImageAny but with `const Image<kPixelType>*`
  // passed by const pointer instead of a `Image<kPixelType>` (by value).
  template <typename...>
  struct variant_add_const_pointer;
  template <typename... Types>
  struct variant_add_const_pointer<std::variant<Types...>> {
    using type = std::variant<std::add_pointer_t<std::add_const_t<Types>>...>;
  };
  using ImageAnyConstPtr = variant_add_const_pointer<ImageAny>::type;

  // ImageAnyMutablePtr is like ImageAny but with `Image<kPixelType>*`
  // passed by mutable pointer instead of `Image<kPixelType>` (by value).
  template <typename...>
  struct variant_add_pointer;
  template <typename... Types>
  struct variant_add_pointer<std::variant<Types...>> {
    using type = std::variant<std::add_pointer_t<Types>...>;
  };
  using ImageAnyMutablePtr = variant_add_pointer<ImageAny>::type;

  // File input is either a filename or a read-only memory buffer.
  using InputAny = std::variant<const std::filesystem::path*, ByteSpan>;

  // File output is either a filename or a growable memory buffer.
  using OutputAny =
      std::variant<const std::filesystem::path*, std::vector<uint8_t>*>;

  // Implementation functions for the public API.
  std::optional<Metadata> LoadMetadataImpl(InputAny input_any) const;
  ImageAny LoadImpl(InputAny input_any) const;
  void LoadImpl(InputAny input_any, ImageAnyMutablePtr image_any) const;
  void SaveImpl(ImageAnyConstPtr image_any, OutputAny output_any) const;

  // Helpers.
  struct LoaderTools;
  LoaderTools MakeLoaderTools(InputAny input_any) const;
  void FlushDiagnostics(const LoaderTools& tools) const;

  std::optional<ImageFileFormat> format_;
  drake::internal::DiagnosticPolicy diagnostic_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
