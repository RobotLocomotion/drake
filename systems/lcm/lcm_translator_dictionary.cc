#include "drake/systems/lcm/lcm_translator_dictionary.h"

#include <utility>

namespace drake {
namespace systems {
namespace lcm {

void LcmTranslatorDictionary::AddEntry(
    const std::string& channel_name,
    std::unique_ptr<const LcmAndVectorBaseTranslator> translator) {
  if (!HasTranslator(channel_name)) {
    dictionary_[channel_name] = std::move(translator);
  } else {
    throw std::runtime_error(
        "Attempted to add more than one translator for channel \"" +
        channel_name + "\".");
  }
}

bool LcmTranslatorDictionary::HasTranslator(
    const std::string& channel_name) const {
  return dictionary_.find(channel_name) != dictionary_.end();
}

/**
 * Returns a reference to the translator to use with the specified channel.
 */
const LcmAndVectorBaseTranslator& LcmTranslatorDictionary::GetTranslator(
    const std::string& channel_name) const {
  if (HasTranslator(channel_name)) {
    return *(dictionary_.find(channel_name)->second.get());
  } else {
    throw std::runtime_error("Translator for channel \"" + channel_name +
                             "\" does not exist.");
  }
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
