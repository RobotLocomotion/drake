#pragma once

#include <map>
#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_export.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * A dictionary that maps between LCM channel names and translators that
 * convert between LCM message objects and VectorBase objects. All
 * translators contained within the dictionary are guaranteed to remain in
 * existence throughout the lifespan of this object.
 */
class DRAKE_EXPORT LcmTranslatorDictionary {
 public:
  /**
   * The constructor. The dictionary is initially empty.
   */
  LcmTranslatorDictionary() {}

  /**
   * Adds a translator this dictionary.
   *
   * @param[in] channel_name The name of the LCM channel. Messages sent on and
   * received from this channel are assumed to be compatible with @p translator.
   *
   * @param[in] translator A pointer to the translator to use with the LCM
   * channel.
   *
   * @throws std::runtime_error If a translator for the @p channel_name is
   * already in the dictionary.
   */
  void AddEntry(
      const std::string& channel_name,
      std::unique_ptr<const LcmAndVectorBaseTranslator> translator);

  /**
   * Returns true if and only if a translator for @p channel_name exists in the
   * dictionary.
   */
  bool HasTranslator(const std::string& channel_name) const;

  /**
   * Returns a reference to the translator to use with the specified channel.
   * This reference is guaranteed to remain valid for the lifetime of this
   * dictionary.
   *
   * @throws std::runtime_error if a translator for @p channel_name does not
   * exist in this dictionary.
   */
  const LcmAndVectorBaseTranslator& GetTranslator(
      const std::string& channel_name) const;

  // Disable copy and assign.
  LcmTranslatorDictionary(const LcmTranslatorDictionary&) = delete;
  LcmTranslatorDictionary& operator=(const LcmTranslatorDictionary&) = delete;

 private:
  // This is the internal data structure for holding the translators and their
  // associated channel names.
  std::map<std::string, std::unique_ptr<const LcmAndVectorBaseTranslator>>
      dictionary_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
