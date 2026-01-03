#include "drake/geometry/meshcat_zip_factory_internal.h"

#include <stdexcept>
#include <utility>

#include <fmt/format.h>
#include <zip.h>

#include "drake/common/scope_exit.h"

namespace drake {
namespace geometry {
namespace internal {

MeshcatZipFactory::MeshcatZipFactory() = default;

MeshcatZipFactory::~MeshcatZipFactory() = default;

void MeshcatZipFactory::AddFile(std::string_view name, std::string contents) {
  Entry entry{.name = std::string{name}, .contents = std::move(contents)};
  entries_.push_back(std::move(entry));
}

std::string MeshcatZipFactory::Build() const {
  // Set aside (unused) storage for error details.
  zip_error_t error;
  zip_error_init(&error);
  ScopeExit error_guard([&error]() {
    zip_error_fini(&error);
  });

  // Create an empty, in-memory archive.
  zip_source_t* buffer = zip_source_buffer_create(
      /* data = */ nullptr, /* len = */ 0, /* freep = */ 1, &error);
  DRAKE_THROW_UNLESS(buffer != nullptr);
  ScopeExit buffer_guard([&buffer]() {
    zip_source_free(buffer);
  });
  zip_t* archive = zip_open_from_source(buffer, /* flags = */ 0, &error);
  DRAKE_THROW_UNLESS(archive != nullptr);
  zip_source_keep(buffer);

  // Add all files to the archive.
  for (const Entry& entry : entries_) {
    auto throw_archive_error = [&entry, &archive](std::string_view function) {
      throw std::runtime_error(fmt::format(
          "Error while creating meshcat.zip while adding '{}': {} during {}",
          entry.name, zip_strerror(archive), function));
    };
    zip_source_t* contents_source = zip_source_buffer_create(
        /* data = */ entry.contents.data(), /* len = */ entry.contents.size(),
        /* freep = */ 0, &error);
    DRAKE_THROW_UNLESS(contents_source != nullptr);
    auto index = zip_file_add(archive, /* name = */ entry.name.c_str(),
                              /* source = */ contents_source,
                              /* flags = */ ZIP_FL_ENC_UTF_8);
    if (index < 0) {
      zip_source_free(contents_source);
      throw_archive_error("zip_file_add");
    }
    auto error_code = zip_set_file_compression(archive, /* index = */ index,
                                               /* comp = */ ZIP_CM_STORE,
                                               /* comp_flags = */ 0);
    if (error_code < 0) {
      throw_archive_error("zip_set_file_compression");
    }
  }

  // Finalize the archive.
  if (auto error_code = zip_close(archive); error_code < 0) {
    throw std::runtime_error(
        fmt::format("Error while creating meshcat.zip: {} during zip_close",
                    zip_strerror(archive)));
  }

  // Copy the buffer into a string.
  auto throw_buffer_error = [&buffer](std::string_view function) {
    throw std::runtime_error(
        fmt::format("Error while creating meshcat.zip: {} during {}",
                    zip_error_strerror(zip_source_error(buffer)), function));
  };
  zip_stat_t stat;
  if (auto error_code = zip_source_stat(buffer, &stat); error_code < 0) {
    throw_buffer_error("zip_source_stat");
  }
  const size_t buffer_size = stat.size;
  if (auto error_code = zip_source_open(buffer); error_code < 0) {
    throw_buffer_error("zip_source_open");
  }
  DRAKE_DEMAND(buffer_size > 0);
  std::string result(buffer_size, '\0');
  const auto bytes_read = zip_source_read(buffer, /* data = */ &result.at(0),
                                          /* len = */ buffer_size);
  if (bytes_read < 0) {
    throw_buffer_error("zip_source_read");
  }
  if (static_cast<size_t>(bytes_read) != buffer_size) {
    throw std::runtime_error("Error while creating meshcat.zip: size mismatch");
  }

  return result;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
