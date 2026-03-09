#include "drake/geometry/meshcat_params.h"

#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {

// TODO(jwnimmer-tri) Possibly we could promote this into the header file?
// Really, we should code-gen it from the Serialize function automatically.
bool operator==(const MeshcatParams::PropertyTuple& a,
                const MeshcatParams::PropertyTuple& b) {
  return std::tie(a.path, a.property, a.value) ==
         std::tie(b.path, b.property, b.value);
}

namespace {

GTEST_TEST(MeshcatParamsTest, RoundTrip) {
  // Populate a params struct with interesting values.
  const std::vector<double> some_vector{1.0, 2.0};
  const std::string some_string{"hello"};
  const bool some_bool{true};
  const double some_double{22.2};
  const MeshcatParams original{
      .host = "some_host",
      .port = 7001,
      .web_url_pattern = "http://{host}:{port}/proxy",
      .initial_properties =
          {
              {.path = "a", .property = "p1", .value = some_vector},
              {.path = "b", .property = "p2", .value = some_string},
              {.path = "c", .property = "p3", .value = some_bool},
              {.path = "d", .property = "p4", .value = some_double},
          },
  };

  // Make sure we can save & re-load it.
  const std::string yaml = yaml::SaveYamlString(original);
  log()->info("Saved round-trip YAML:\n{}", yaml);
  MeshcatParams readback;
  EXPECT_NO_THROW(readback = yaml::LoadYamlString<MeshcatParams>(yaml));

  // Check that everything made it back intact. We can rely on the YAML
  // library's unit tests to check that save and re-load generally works,
  // but we'll spot-check a few of the more interesting values to be safe.
  EXPECT_EQ(readback.port, original.port);
  EXPECT_EQ(readback.initial_properties, original.initial_properties);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
