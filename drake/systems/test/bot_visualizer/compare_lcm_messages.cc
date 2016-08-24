#include "drake/systems/test/bot_visualizer/compare_lcm_messages.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace test {

bool CompareLoadMessage(
    const drake::lcmt_viewer_load_robot& received,
    const drake::lcmt_viewer_load_robot& expected) {
  // Aborts if the message wasn't received yet.
  if (received.num_links == -1) return false;

  // Verifies that the number of links match.
  EXPECT_EQ(received.num_links, expected.num_links);
  for (int i = 0; i < received.num_links; ++i) {
    EXPECT_EQ(received.link[i].name, expected.link[i].name);
    EXPECT_EQ(received.link[i].robot_num, expected.link[i].robot_num);
    EXPECT_EQ(received.link[i].num_geom, expected.link[i].num_geom);

    for (int j = 0; j < received.link[i].num_geom; ++j) {
      EXPECT_EQ(received.link[i].geom[j].type, expected.link[i].geom[j].type);

      for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(received.link[i].geom[j].position[k],
                    expected.link[i].geom[j].position[k], 1e-6);
      }

      for (int k = 0; k < 4; ++k) {
        EXPECT_NEAR(received.link[i].geom[j].quaternion[k],
                    expected.link[i].geom[j].quaternion[k], 1e-6);
      }

      for (int k = 0; k < 4; ++k) {
        EXPECT_NEAR(received.link[i].geom[j].color[k],
                    expected.link[i].geom[j].color[k], 1e-6);
      }

      EXPECT_EQ(received.link[i].geom[j].num_float_data,
                expected.link[i].geom[j].num_float_data);

      for (int k = 0; k < received.link[i].geom[j].num_float_data; ++k) {
        EXPECT_NEAR(received.link[i].geom[j].float_data[k],
                    expected.link[i].geom[j].float_data[k], 1e-6);
      }
    }
  }

  return true;
}

bool CompareDrawMessage(
    const drake::lcmt_viewer_draw& received,
    const drake::lcmt_viewer_draw& expected) {
  // Aborts if the draw message wasn't received yet.
  if (received.num_links == -1) return false;

  // Verifies that the draw messages have the same number of links and that
  // their names, model instance IDs, positions, and orientations match.
  EXPECT_EQ(received.num_links, expected.num_links);
  EXPECT_EQ(received.link_name.size(), expected.link_name.size());
  EXPECT_EQ(received.robot_num.size(), expected.robot_num.size());
  EXPECT_EQ(received.position.size(), expected.position.size());
  EXPECT_EQ(received.quaternion.size(), expected.quaternion.size());

  for (int i = 0; i < received.num_links; ++i) {
    EXPECT_EQ(received.link_name[i], expected.link_name[i]);
    EXPECT_EQ(received.robot_num[i], expected.robot_num[i]);

    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(received.position[i][j], expected.position[i][j], 1e-6);
    }

    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(received.quaternion[i][j], expected.quaternion[i][j], 1e-6);
    }
  }

  return true;
}

}  // namespace test
}  // namespace systems
}  // namespace drake
