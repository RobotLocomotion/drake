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

The only file formats supported are JPEG, PNG, and TIFF.

The only format that supports floating-point scalars (e.g., ImageDepth32F) is
TIFF. Trying to load or save a floating-point image from/to a PNG or JPEG file
will throw an exception. */
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

  /** Default constructor. */
  ImageIo() = default;

  /** Returns the metadata of the given image file, or nullopt if the metadata
  cannot be determined or is unsupported. The filename extension has no bearing
  on the result; only the actual file contents determine the file format. */
  std::optional<Metadata> LoadMetadata(
      const std::filesystem::path& path) const {
    return LoadMetadataImpl(&path);
  }

  /** Returns the metadata of the given image buffer, or nullopt if the metadata
  cannot be determined or is unsupported. */
  std::optional<Metadata> LoadMetadata(ByteSpan buffer) const {
    return LoadMetadataImpl(buffer);
  }

  /** Loads and returns an image from disk.
  @param format (Optionally) establishes the required image file format. When
  set, images that are a different file format will throw an exception. When not
  set, the filename extension has no bearing on the result; only the actual file
  contents determine the file format.
  @throws std::exception for any kind of error loading the image file. */
  ImageAny Load(const std::filesystem::path& path,
                std::optional<ImageFileFormat> format = std::nullopt) const {
    return LoadImpl(&path, format);
  }

  /** Loads and returns an image from a memory buffer.
  @param format (Optionally) establishes the required image file format. When
  set, images that are a different file format will throw an exception.
  @throws std::exception for any kind of error loading the image data. */
  ImageAny Load(ByteSpan buffer,
                std::optional<ImageFileFormat> format = std::nullopt) const {
    return LoadImpl(buffer, format);
  }

  /** Loads and outputs an image from disk. The filename extension has no
  bearing on the result; only the actual file contents determine the file
  format.
  @param[out] image The output image (which will be overwritten).
  @throws std::exception for any kind of error loading the image file.
  @throws std::exception if the loaded image does not match the kPixelType.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  template <PixelType kPixelType>
  void Load(const std::filesystem::path& path, Image<kPixelType>* image) const {
    LoadImpl(&path, std::nullopt, image);
  }

  /** Loads and outputs an image from disk.
  @param format Establishes the required image file format; images that are a
  different file format will throw an exception.
  @param[out] image The output image (which will be overwritten).
  @throws std::exception for any kind of error loading the image file.
  @throws std::exception if the loaded image does not match the kPixelType.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  template <PixelType kPixelType>
  void Load(const std::filesystem::path& path, ImageFileFormat format,
            Image<kPixelType>* image) const {
    LoadImpl(&path, format, image);
  }

  /** Loads and outputs an image from a memory buffer.
  @param[out] image The output image (which will be overwritten).
  @throws std::exception for any kind of error loading the image data.
  @throws std::exception if the loaded image does not match the kPixelType.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  template <PixelType kPixelType>
  void Load(ByteSpan buffer, Image<kPixelType>* image) const {
    LoadImpl(buffer, std::nullopt, image);
  }

  /** Loads and outputs an image from a memory buffer.
  @param format Establishes the required image file format; images that are a
  different file format will throw an exception.
  @param[out] image The output image (which will be overwritten).
  @throws std::exception for any kind of error loading the image data.
  @throws std::exception if the loaded image does not match the kPixelType.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  template <PixelType kPixelType>
  void Load(ByteSpan buffer, ImageFileFormat format,
            Image<kPixelType>* image) const {
    LoadImpl(buffer, format, image);
  }

  /** Saves an image to disk.
  @param format (Optionally) chooses the image file format. When not set, the
  filename extension will determine the format and the extension must be a
  supported choice (i.e., `.jpg`, `.jpeg`, `.png`, `.tif`, or `.tiff`).
  @throws std::exception for any kind of error saving the image file. */
  template <PixelType kPixelType>
  void Save(const Image<kPixelType>& image, const std::filesystem::path& path,
            std::optional<ImageFileFormat> format = std::nullopt) const {
    SaveImpl(&image, format, &path);
  }

  /** Saves an image to a new memory buffer, returning the buffer.
  @throws std::exception for any kind of error saving the image data. */
  template <PixelType kPixelType>
  std::vector<uint8_t> Save(const Image<kPixelType>& image,
                            ImageFileFormat format) const {
    std::vector<uint8_t> result;
    SaveImpl(&image, format, &result);
    return result;
  }

  /** Saves an image to an existing memory buffer.
  @param[out] buffer The output buffer (which will be overwritten).
  @throws std::exception for any kind of error saving the image data.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  template <PixelType kPixelType>
  void Save(const Image<kPixelType>& image, ImageFileFormat format,
            std::vector<uint8_t>* buffer) const {
    SaveImpl(&image, format, buffer);
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
  ImageAny LoadImpl(InputAny input_any,
                    std::optional<ImageFileFormat> format) const;
  void LoadImpl(InputAny input_any, std::optional<ImageFileFormat> format,
                ImageAnyMutablePtr image_any) const;
  void SaveImpl(ImageAnyConstPtr image_any,
                std::optional<ImageFileFormat> format,
                OutputAny output_any) const;

  // Helpers.
  struct LoaderTools;
  LoaderTools MakeLoaderTools(InputAny input_any,
                              std::optional<ImageFileFormat> format) const;
  void FlushDiagnostics(const LoaderTools& tools) const;

  // TODO(jwnimmer-tri) Expose this so that Drake-internal callers can customize
  // their error handling.
  drake::internal::DiagnosticPolicy diagnostic_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
