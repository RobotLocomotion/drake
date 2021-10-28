#include "drake/common/yaml/yaml_io.h"

#include "drake/common/unused.h"

namespace drake {
namespace yaml {
namespace {

struct DummySerializable {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(foo));
  }
  double foo{};
};

// Ensure that the SaveYaml... functions can be compiled on their own.
void SaveYamlCheckCompile() {
  const std::string filename;
  const DummySerializable data;
  const std::optional<std::string> child_name{std::string()};
  const std::optional<DummySerializable> defaults{DummySerializable()};
  SaveYamlFile(filename, data, child_name, defaults);
  unused(&SaveYamlCheckCompile);
}

// XXX Check LoadYaml...

}  // namespace
}  // namespace yaml
}  // namespace drake
