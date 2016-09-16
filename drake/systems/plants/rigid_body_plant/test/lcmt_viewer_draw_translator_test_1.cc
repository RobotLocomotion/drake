#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/rigid_body_plant/lcmt_viewer_draw_translator_1.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;

// Tests the basic functionality of the translator.
GTEST_TEST(LcmtViewerDrawTranslator1Tests, BasicTest) {
  // Define the number of rigid bodies in the RigidbodyTree.
  const int kNumRigidBodies = 2;

  // Creates a RigidBodyTree with two rigid bodies.
  auto tree = make_unique<RigidBodyTree>();
  for (int i = 0; i < kNumRigidBodies; ++i) {
    auto body = make_unique<RigidBody>();
    tree->bodies.push_back(std::move(body));
  }
  tree->compile();

  // TODO(liang.fok) Add two rigid bodies.

  // Creates an LcmtViewerDrawTranslator1 object using the tree that was just
  // created. The name "dut" stands for "Device Under Test".
  LcmtViewerDrawTranslator1 dut(*tree.get());

  // Instantiates a BasicVector<double> of the correct length. Since there are
  // two bodies, there are 7 * 2 = 14 values.
  int num_states = kNumRigidBodies *
      LcmtViewerDrawTranslator1::kNumStatesPerBody;

  BasicVector<double> original_vector(num_states);
  for (int i = 0; i < num_states; ++i) {
    original_vector.SetAtIndex(i, i);
  }

  // Encodes the basic vector into the byte representation of an
  // lcmt_viewer_draw message.
  double time = 0;
  std::vector<uint8_t> lcm_message_bytes;
  dut.TranslateVectorBaseToLcm(time, original_vector, &lcm_message_bytes);
  EXPECT_GT(lcm_message_bytes.size(), 0);

  // Decodes the byte representation into the basic vector and ensures it equals
  // the original basic vector that was defined above.
  // values match the original values.
  BasicVector<double> decoded_vector(num_states);
  dut.TranslateLcmToVectorBase(lcm_message_bytes.data(),
      lcm_message_bytes.size(), &decoded_vector);

  for (int i = 0; i < num_states; ++i) {
    EXPECT_EQ(original_vector.GetAtIndex(i), decoded_vector.GetAtIndex(i));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
