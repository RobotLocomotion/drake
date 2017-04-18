#pragma once

#include <memory>
#include <string>

#include "google/protobuf/io/zero_copy_stream_impl.h"

namespace drake {

/// Returns a self-closing FileInputStream for the file at the given @p path,
/// or else throws std::runtime_error.
std::unique_ptr<google::protobuf::io::FileInputStream>
MakeFileInputStreamOrThrow(const std::string& path);

}  // namespace drake
