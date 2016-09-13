#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Publishes an LCM message containing information from its input port.
 */
class DRAKELCMSYSTEM2_EXPORT LcmPublisherSystem : public LeafSystem<double> {
 public:
  /**
   * A constructor.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator The translator that converts between LCM message
   * objects and `drake::systems::VectorBase` objects. This reference
   * is aliased by this constructor and thus must remain valid for the lifetime
   * of this object.
   *
   * @param[in] lcm A pointer to the LCM subsystem.
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmAndVectorBaseTranslator& translator,
                     ::lcm::LCM* lcm);

  /**
   * A constructor.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator_dictionary A dictionary for obtaining the appropriate
   * translator for a particular LCM channel.
   *
   * @param[in] lcm A pointer to the LCM subsystem.
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmTranslatorDictionary& translator_dictionary,
                     ::lcm::LCM* lcm);


  ~LcmPublisherSystem() override;

  // Disable copy and assign.
  LcmPublisherSystem(const LcmPublisherSystem&) = delete;
  LcmPublisherSystem& operator=(const LcmPublisherSystem&) = delete;

  std::string get_name() const override;

  /**
   * Takes the VectorBase from the input port of the context and publishes
   * it onto an LCM channel.
   */
  void DoPublish(const Context<double>& context) const override;

  /**
   * This System has no output ports so EvalOutput() does nothing.
   */
  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {}

 private:
  // The channel on which to publish LCM messages.
  const std::string channel_;

  // The translator that converts between LCM messages and
  // drake::systems::VectorBase objects.
  const LcmAndVectorBaseTranslator& translator_;

  // A pointer to the LCM subsystem.
  ::lcm::LCM* lcm_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
