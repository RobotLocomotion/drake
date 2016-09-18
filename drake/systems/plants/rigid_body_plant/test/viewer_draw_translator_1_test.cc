#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/rigid_body_plant/viewer_draw_translator_1.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;

// Tests the basic functionality of the translator.
GTEST_TEST(ViewerDrawTranslator1Tests, BasicTest) {
  // Define the number of rigid bodies to add to the `RigidbodyTree`.
  const int kNumBodies = 2;

  // Creates a `RigidBodyTree` with `kNumBodies` rigid bodies.
  auto tree = make_unique<RigidBodyTree>();
  for (int i = 0; i < kNumBodies; ++i) {
    auto body = make_unique<RigidBody>();
    body->set_name("body" + std::to_string(i));
    tree->bodies.push_back(std::move(body));
  }
  tree->compile();

  // Creates an `LcmtViewerDrawTranslator1` object using the tree that was just
  // created. The name "dut" stands for "Device Under Test".
  ViewerDrawTranslator1 dut(*tree.get());

  // Instantiates a `BasicVector<double>`. Since there are `kNumBodies` bodies,
  // there are `kNumBodies * ViewerDrawTranslator1::kNumStatesPerBody` values.
  int num_states = kNumBodies *
      ViewerDrawTranslator1::kNumStatesPerBody;

  BasicVector<double> original_vector(num_states);
  for (int i = 0; i < num_states; ++i) {
    original_vector.SetAtIndex(i, i);
  }

  // Transforms the basic vector into the byte array representation of a
  // `drake::lcmt_viewer_draw` message.
  double time = 0;
  std::vector<uint8_t> lcm_message_bytes;
  dut.TranslateVectorBaseToLcm(time, original_vector, &lcm_message_bytes);
  EXPECT_GT(lcm_message_bytes.size(), 0);

  // TODO(liang.fok): Verify that the serialized message is correct. I believe
  // this entails (1) manually creating a the correct `drake::lcmt_viewer_draw`
  // message, (2) serializing it into an array of bytes, and (3) verifying that
  // the byte array matches `lcm_message_bytes`.
}

}  // namespace
}  // namespace systems
}  // namespace drake
