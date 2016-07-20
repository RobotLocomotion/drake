#pragma once

#include <map>

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_interface.h"
#include "drake/systems/lcm/lcm_and_vector_interface_translator.h"
#include "drake/systems/lcm/lcm_receive_thread.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * A dictionary that maps between LCM channel names and translators that
 * convert between LCM message objects and VectorInterface objects. All
 * translators contained within the dictionary are guaranteed to remain in
 * existence throughout the lifespan of this object.
 */
class DRAKELCMSYSTEM2_EXPORT LcmTranslatorDictionary {
 public:
  /**
   * Adds a translator this dictionary.
   *
   * @param[in] channel_name The name of the LCM channel. Messages sent on and
   * received from this channel are assumed to be compatibl with @p translator.
   *
   * @param[in] translator A pointer to the translator to use with the LCM
   * channel.
   *
   * @throws std::runtime_error If a translator for the @p channel_name is
   * already in the dictionary.
   */
  void AddEntry(const std::string& channel_name,
    std::unique_ptr<const LcmAndVectorInterfaceTranslator> translator);

  /**
   * Returns true if and only if a translator for @p channel_name exists in the
   * dictionary.
   */
  bool HasTranslator(const std::string& channel_name) const;

  /**
   * Returns a reference to the translator to use with the specified channel.
   */
  const LcmAndVectorInterfaceTranslator& GetTranslator(
      const std::string& channel_name) const;

  // Disable copy and assign.
  LcmTranslatorDictionary(const LcmTranslatorDictionary&)
      = delete;
  LcmTranslatorDictionary& operator=(const LcmTranslatorDictionary&) = delete;

 private:
  // This is the internal data structure for holding the translators and their
  // associated channel names.
  std::map<std::string, std::unique_ptr<const LcmAndVectorInterfaceTranslator>>
      dictionary_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
