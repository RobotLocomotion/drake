#include "drake/geometry/meshcat_params.h"

#include <tuple>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(MeshcatParamsTest, RoundTrip) {
  // Populate a params struct with interesting values.
  const MeshcatParams original{
      .host = "some_host",
      .port = 7001,
      .web_url_pattern = "http://{host}:{port}/proxy",
  };

  // Make sure we can save & re-load it.
  const std::string yaml = yaml::SaveYamlString(original);
  log()->info("Saved round-trip YAML:\n{}", yaml);
  MeshcatParams readback;
  EXPECT_NO_THROW(readback = yaml::LoadYamlString<MeshcatParams>(yaml));

  // Check that everything made it back intact. We can rely on the YAML
  // library's unit tests to check that save and re-load generally works,
  // but we'll do a spot-check to be safe.
  EXPECT_EQ(readback.port, original.port);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
