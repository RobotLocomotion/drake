#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace systems {
namespace lcm {
namespace test {

// A lcmt_drake_signal translator that preserves coordinate names.
class CustomDrakeSignalTranslator : public LcmAndVectorBaseTranslator {
 public:
  enum : int { kDim = 3 };

  // An output vector type that tests that subscribers permit non-default
  // types.
  using CustomVector = MyVector<kDim, double>;

  CustomDrakeSignalTranslator() : LcmAndVectorBaseTranslator(kDim) {}

  std::unique_ptr<BasicVector<double>> AllocateOutputVector() const override {
    return std::make_unique<CustomVector>();
  }

  static std::string NameFor(int index) {
    return std::to_string(index) + "_name";
  }

  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   VectorBase<double>* vector_base) const override {
    CustomVector* const custom_vector =
        dynamic_cast<CustomVector*>(vector_base);
    ASSERT_NE(nullptr, custom_vector);

    // Decode the LCM message.
    drake::lcmt_drake_signal message;
    int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
    ASSERT_GE(status, 0);
    ASSERT_EQ(kDim, message.dim);

    // Copy message into our custom_vector.
    for (int i = 0; i < kDim; ++i) {
      EXPECT_EQ(message.coord[i], NameFor(i));
      custom_vector->SetAtIndex(i, message.val[i]);
    }
  }

  void Serialize(double time, const VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override {
    const CustomVector* const custom_vector =
        dynamic_cast<const CustomVector*>(&vector_base);
    ASSERT_NE(nullptr, custom_vector);
    ASSERT_NE(nullptr, lcm_message_bytes);

    // Copy custom_vector into the message.
    drake::lcmt_drake_signal message;
    message.dim = kDim;
    message.val.resize(kDim);
    message.coord.resize(kDim);
    message.timestamp = static_cast<int64_t>(time * 1000);

    for (int i = 0; i < kDim; ++i) {
      message.val.at(i) = custom_vector->GetAtIndex(i);
      message.coord.at(i) = NameFor(i);
    }

    // Encode the LCM message.
    const int lcm_message_length = message.getEncodedSize();
    lcm_message_bytes->resize(lcm_message_length);
    message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
  }
};

}  // namespace test
}  // namespace lcm
}  // namespace systems
}  // namespace drake
