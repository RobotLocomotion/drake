#include "drake/geometry/meshcat_zip_factory_internal.h"

#include <utility>

#include <zip.h>

#include "drake/common/drake_assert.h"

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
  DRAKE_THROW_UNLESS(!entries_.empty());

  // Throughout this function, we call into libzip C functions that allocate but
  // don't use RAII, so if we throw an exception we might leak memory. However,
  // in practice that doesn't matter because none of the exceptions can ever
  // trip because malloc can never fail.

  // Define storage for error details.
  zip_error_t error;
  zip_error_init(&error);
  int error_code{};

  // Create an empty, in-memory archive.
  zip_source_t* buffer = zip_source_buffer_create(
      /* data = */ nullptr, /* len = */ 0, /* freep = */ 1, &error);
  DRAKE_THROW_UNLESS(buffer != nullptr);
  zip_t* archive = zip_open_from_source(buffer, /* flags = */ 0, &error);
  DRAKE_THROW_UNLESS(archive != nullptr);

  // Add all files to the archive.
  for (const Entry& entry : entries_) {
    zip_source_t* contents_source = zip_source_buffer_create(
        /* data = */ entry.contents.data(), /* len = */ entry.contents.size(),
        /* freep = */ 0, &error);
    DRAKE_THROW_UNLESS(contents_source != nullptr);
    const zip_int64_t index =
        zip_file_add(archive, /* name = */ entry.name.c_str(),
                     /* source = */ contents_source,
                     /* flags = */ ZIP_FL_ENC_UTF_8);
    DRAKE_THROW_UNLESS(index >= 0);
    error_code = zip_set_file_compression(archive, /* index = */ index,
                                          /* comp = */ ZIP_CM_STORE,
                                          /* comp_flags = */ 0);
    DRAKE_THROW_UNLESS(error_code == 0);
  }

  // Finalize the archive (but don't de-allocate the buffer).
  zip_source_keep(buffer);
  error_code = zip_close(archive);
  DRAKE_THROW_UNLESS(error_code == 0);

  // Copy the buffer into a string.
  zip_stat_t stat;
  error_code = zip_source_stat(buffer, &stat);
  DRAKE_THROW_UNLESS(error_code == 0);
  const size_t buffer_size = stat.size;
  DRAKE_THROW_UNLESS(buffer_size > 0);
  error_code = zip_source_open(buffer);
  DRAKE_THROW_UNLESS(error_code == 0);
  std::string result(buffer_size, '\0');
  const zip_int64_t bytes_read =
      zip_source_read(buffer, /* data = */ &result.at(0),
                      /* len = */ buffer_size);
  DRAKE_THROW_UNLESS(bytes_read >= 0 &&
                     static_cast<size_t>(bytes_read) == buffer_size);
  zip_source_free(buffer);

  return result;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
