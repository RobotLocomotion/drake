#pragma once

#include <mutex>

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/lcm/lcm_and_vector_interface_translator.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Receives LCM messages from a given channel and outputs them to a
 * System<double>'s port. The output port value is the most recently
 * decoded message, modulo any network or threading delays.
 */
class DRAKELCMSYSTEM2_EXPORT LcmSubscriberSystem : public System<double> {
 public:
  /**
   * A constructor.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] translator A reference to the translator that converts between
   * LCM message objects and `drake::systems::VectorInterface` objects. This
   * reference must remain valid for the lifetime of this `LcmSubscriberSystem`
   * object.
   *
   * @param[in] lcm A pointer to the LCM subsystem. This pointer must not be
   * null and must be valid during the construction of this
   * `LcmSubscriberSystem`.
   */
  LcmSubscriberSystem(const std::string& channel,
                      const LcmAndVectorInterfaceTranslator& translator,
                      ::lcm::LCM* lcm);

  /**
   * A constructor.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] translator_dictionary A dictionary for obtaining the appropriate
   * translator for a particular LCM channel.
   *
   * @param[in] lcm A pointer to the LCM subsystem. This pointer must not be
   * null.
   */
  LcmSubscriberSystem(const std::string& channel,
                      const LcmTranslatorDictionary& translator_dictionary,
                      ::lcm::LCM* lcm);

  ~LcmSubscriberSystem() override;

  // Disable copy and assign.
  LcmSubscriberSystem(const LcmSubscriberSystem&) = delete;
  LcmSubscriberSystem& operator=(const LcmSubscriberSystem&) = delete;

  std::string get_name() const override;

  /**
   * The default context for this system is one that has zero input ports and
   * no state.
   */
  std::unique_ptr<ContextBase<double>> CreateDefaultContext() const override;

  /**
   * The output consists of a single port containing a
   * `drake::systems::BasicVector<double>` object.
   */
  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const ContextBase<double>& context) const override;

  /**
   * Computes the output for the given context, possibly updating values
   * in the cache. Note that the context is ignored since it contains no
   * information.
   */
  void EvalOutput(const ContextBase<double>& context,
                  SystemOutput<double>* output) const override;

 private:
  // Translates the message contained within the receive buffer by storing its
  // information in basic_vector_.
  void HandleMessage(const ::lcm::ReceiveBuffer* rbuf,
                     const std::string& channel);

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // The translator that converts between LCM messages and
  // drake::systems::VectorInterface objects.
  const LcmAndVectorInterfaceTranslator& translator_;

  // A mutex for protecting data that's shared by the LCM receive thread and
  // the thread that calls LcmSubscriberSystem::Output().
  mutable std::mutex data_mutex_;

  // Holds the information contained with the latest LCM message. This
  // information is copied into this system's output port when Output(...) is
  // called.
  BasicVector<double> basic_vector_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
