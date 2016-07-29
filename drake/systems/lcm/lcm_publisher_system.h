#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/lcm/lcm_and_vector_interface_translator.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Publishes an LCM message containing information from its input port.
 */
class DRAKELCMSYSTEM2_EXPORT LcmPublisherSystem : public System<double> {
 public:
  /**
   * A constructor.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator The translator that converts between LCM message
   * objects and `drake::systems::VectorInterface` objects. This reference
   * is aliased by this constructor and thus must remain valid for the lifetime
   * of this object.
   *
   * @param[in] lcm A pointer to the LCM subsystem.
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmAndVectorInterfaceTranslator& translator,
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
   * The default context for this system is one that has one input port and
   * no state.
   */
  std::unique_ptr<ContextBase<double>> CreateDefaultContext() const override;

  /**
   * The output contains zero ports.
   */
  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const ContextBase<double>& context) const override;

  /**
   * Takes the VectorInterface from the input port of the context and publishes
   * it onto an LCM channel. Note that the output is ignored since this system
   * does not output anything.
   */
  void EvalOutput(const ContextBase<double>& context,
                  SystemOutput<double>* output) const override;

 private:
  // The channel on which to publish LCM messages.
  const std::string channel_;

  // The translator that converts between LCM messages and
  // drake::systems::VectorInterface objects.
  const LcmAndVectorInterfaceTranslator& translator_;

  // A pointer to the LCM subsystem.
  ::lcm::LCM* lcm_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
