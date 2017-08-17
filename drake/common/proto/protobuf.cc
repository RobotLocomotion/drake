#include "drake/common/proto/protobuf.h"

#include <fcntl.h>

#include "google/protobuf/io/coded_stream.h"

namespace drake {

std::unique_ptr<google::protobuf::io::FileInputStream>
MakeFileInputStreamOrThrow(const std::string& path) {
  int fid = open(path.data(), O_RDONLY);
  if (fid < 0) {
    throw std::runtime_error("Cannot open file " + path);
  }
  auto istream = std::make_unique<google::protobuf::io::FileInputStream>(fid);
  istream->SetCloseOnDelete(true);
  return istream;
}

}  // namespace drake
