#include "drake/multibody/parsing/detail_select_parser.h"

#include <optional>
#include <vector>

#include "drake/multibody/parsing/detail_dmd_parser.h"
#include "drake/multibody/parsing/detail_mujoco_parser.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;

namespace {

bool EndsWith(const std::string_view str, const std::string_view ext) {
  return (ext.size() < str.size()) &&
      std::equal(str.end() - ext.size(), str.end(), ext.begin());
}

enum class FileType { kUnknown, kSdf, kUrdf, kMjcf, kDmdf };
FileType DetermineFileType(const DiagnosticPolicy& policy,
                           const std::string& file_name) {
  if (EndsWith(file_name, ".urdf") || EndsWith(file_name, ".URDF")) {
    return FileType::kUrdf;
  }
  if (EndsWith(file_name, ".sdf") || EndsWith(file_name, ".SDF")) {
    return FileType::kSdf;
  }
  if (EndsWith(file_name, ".xml") || EndsWith(file_name, ".XML")) {
    return FileType::kMjcf;
  }
  if (EndsWith(file_name, ".dmd.yaml") || EndsWith(file_name, ".DMD.YAML")) {
    return FileType::kDmdf;
  }
  policy.Error(fmt::format(
      "The file '{}' is not a recognized type."
      " Known types are: .urdf, .sdf, .xml (Mujoco), .dmd.yaml",
      file_name));
  return FileType::kUnknown;
}

// This stub allows continued partial operation when Error() does not throw.
class UnknownParser : public ParserInterface {
 public:
  UnknownParser() {}
  ~UnknownParser() override {}
  std::optional<ModelInstanceIndex> AddModel(
      const DataSource&, const std::string&,
      const std::optional<std::string>&,
      const ParsingWorkspace&) override { return {}; }

  std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource&,
      const std::optional<std::string>&,
      const ParsingWorkspace&) override { return {}; }
};

}  // namespace

ParserInterface* SelectParser(const DiagnosticPolicy& policy,
                              const std::string& file_name) {
  static internal::UrdfParser urdf;
  static internal::SdfParser sdf;
  static internal::MujocoParser mujoco;
  static internal::DmdParser dmd;
  static internal::UnknownParser unknown;
  const FileType type = DetermineFileType(policy, file_name);
  switch (type) {
    case FileType::kUrdf:
      return &urdf;
    case FileType::kSdf:
      return &sdf;
    case FileType::kMjcf:
      return &mujoco;
    case FileType::kDmdf:
      return &dmd;
    case FileType::kUnknown:
      return &unknown;
  }
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
