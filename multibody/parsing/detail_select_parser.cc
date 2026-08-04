#include "drake/multibody/parsing/detail_select_parser.h"

#include <optional>
#include <vector>

#include "drake/common/never_destroyed.h"
#include "drake/multibody/parsing/detail_dmd_parser.h"
#include "drake/multibody/parsing/detail_mesh_parser.h"
#include "drake/multibody/parsing/detail_mujoco_parser.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;

namespace {

enum class FileType { kUnknown, kSdf, kUrdf, kMjcf, kDmd, kMesh };
FileType DetermineFileType(const DiagnosticPolicy& policy,
                           const std::string& filename) {
  if (EndsWithCaseInsensitive(filename, ".urdf")) {
    return FileType::kUrdf;
  }
  if (EndsWithCaseInsensitive(filename, ".sdf")) {
    return FileType::kSdf;
  }
  if (EndsWithCaseInsensitive(filename, ".xml")) {
    return FileType::kMjcf;
  }
  if (EndsWithCaseInsensitive(filename, ".dmd.yaml")) {
    return FileType::kDmd;
  }
  if (EndsWithCaseInsensitive(filename, ".obj")) {
    return FileType::kMesh;
  }
  policy.Error(fmt::format(
      "The file '{}' is not a recognized type."
      " Known types are: .urdf, .sdf, .xml (Mujoco), .dmd.yaml, .obj",
      filename));
  return FileType::kUnknown;
}

// This stub allows continued partial operation when Error() does not throw.
class UnknownParserWrapper final : public ParserInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnknownParserWrapper);
  UnknownParserWrapper() {}
  ~UnknownParserWrapper() final {}
  std::optional<ModelInstanceIndex> AddModel(const DataSource&,
                                             const std::string&,
                                             const std::optional<std::string>&,
                                             const ParsingWorkspace&) final {
    return {};
  }

  std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource&, const std::optional<std::string>&,
      const ParsingWorkspace&) final {
    return {};
  }
};

}  // namespace

ParserInterface& SelectParser(const DiagnosticPolicy& policy,
                              const std::string& filename) {
  static never_destroyed<internal::UrdfParserWrapper> urdf;
  static never_destroyed<internal::SdfParserWrapper> sdf;
  static never_destroyed<internal::MujocoParserWrapper> mujoco;
  static never_destroyed<internal::UnknownParserWrapper> unknown;
  static never_destroyed<internal::DmdParserWrapper> dmd;
  static never_destroyed<internal::MeshParserWrapper> mesh;
  const FileType type = DetermineFileType(policy, filename);
  switch (type) {
    case FileType::kUrdf:
      return urdf.access();
    case FileType::kSdf:
      return sdf.access();
    case FileType::kMjcf:
      return mujoco.access();
    case FileType::kDmd:
      return dmd.access();
    case FileType::kMesh:
      return mesh.access();
    case FileType::kUnknown:
      return unknown.access();
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
