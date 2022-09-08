#include "drake/multibody/parsing/detail_select_parser.h"

#include <optional>
#include <vector>

#include "drake/common/never_destroyed.h"
#include "drake/multibody/parsing/detail_mujoco_parser.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;

namespace {

enum class FileType { kUnknown, kSdf, kUrdf, kMjcf };
FileType DetermineFileType(const DiagnosticPolicy& policy,
                           const std::string& file_name) {
  if (EndsWithCaseInsensitive(file_name, ".urdf")) {
    return FileType::kUrdf;
  }
  if (EndsWithCaseInsensitive(file_name, ".sdf")) {
    return FileType::kSdf;
  }
  if (EndsWithCaseInsensitive(file_name, ".xml")) {
    return FileType::kMjcf;
  }
  policy.Error(fmt::format(
      "The file '{}' is not a recognized type."
      " Known types are: .urdf, .sdf, .xml (Mujoco)",
      file_name));
  return FileType::kUnknown;
}

// This stub allows continued partial operation when Error() does not throw.
class UnknownParserWrapper final : public ParserInterface {
 public:
  UnknownParserWrapper() {}
  ~UnknownParserWrapper() final {}
  std::optional<ModelInstanceIndex> AddModel(
      const DataSource&, const std::string&,
      const std::optional<std::string>&,
      const ParsingWorkspace&) final { return {}; }

  std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource&,
      const std::optional<std::string>&,
      const ParsingWorkspace&) final { return {}; }
};

}  // namespace

ParserInterface* SelectParser(const DiagnosticPolicy& policy,
                              const std::string& file_name) {
  static never_destroyed<internal::UrdfParserWrapper> urdf;
  static never_destroyed<internal::SdfParserWrapper> sdf;
  static never_destroyed<internal::MujocoParserWrapper> mujoco;
  static never_destroyed<internal::UnknownParserWrapper> unknown;
  const FileType type = DetermineFileType(policy, file_name);
  switch (type) {
    case FileType::kUrdf:
      return &urdf.access();
    case FileType::kSdf:
      return &sdf.access();
    case FileType::kMjcf:
      return &mujoco.access();
    case FileType::kUnknown:
      return &unknown.access();
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
