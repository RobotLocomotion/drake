#include "drake/systems/lcm/lcm_translator_dictionary.h"

#include "gtest/gtest.h"

#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

// Defines the dimension of the VectorBase used by the translator defined
// below.
const int kDim = 10;

// Tests the functionality of LcmTranslatorDictionary.
GTEST_TEST(LcmTranslatorDictionaryTest, BasicTests) {
  const std::string channel_name = "_=?+pYa8J9c!Hg;V";

  LcmTranslatorDictionary dictionary;
  dictionary.AddEntry(
      channel_name,
      std::make_unique<const LcmtDrakeSignalTranslator>(kDim));

  // Verifies that an exception is thrown when the user attempts to add a
  // duplicate translator to the dictionary.
  EXPECT_THROW(
      dictionary.AddEntry(
          channel_name,
          std::make_unique<const LcmtDrakeSignalTranslator>(kDim)),
      std::runtime_error);

  EXPECT_FALSE(dictionary.HasTranslator("Boo"));
  EXPECT_TRUE(dictionary.HasTranslator(channel_name));

  // Verifies that an exception is thrown if the user attempts to get a
  // translator that does not exist in the dictionary.
  EXPECT_THROW(dictionary.GetTranslator("Boo"), std::runtime_error);

  // Verifies that no exception is thrown if the user attempts to get a
  // translator that exists in the dictionary.
  EXPECT_NO_THROW(dictionary.GetTranslator(channel_name));
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
