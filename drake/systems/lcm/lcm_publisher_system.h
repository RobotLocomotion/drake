#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/system_interface.h"
#include "drake/systems/lcm/lcm_and_vector_interface_translator.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Receives `BasicVector<double>` values from a `SystemInterface<double>`'s
 * output port.
 */
class DRAKELCMSYSTEM2_EXPORT LcmPublisherSystem :
    public SystemInterface<double> {
 public:
  /**
   * The constructor.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator The translator that converts between LCM message
   * objects and `drake::systems::VectorInterface` objects.
   *
   * @param[in] lcm A pointer to the LCM subsystem.
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmAndVectorInterfaceTranslator& translator,
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
  std::unique_ptr<Context<double>> CreateDefaultContext() const override;

  /**
   * The output consists of a single port containing a `BasicVector<double>`.
   */
  std::unique_ptr<SystemOutput<double>> AllocateOutput() const override;

  /**
   * Takes the BasicVector from the input port of the context and publishes it
   * onto an LCM channel. Note that the output is ignored since this system
   * does not output anything.
   */
  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

 private:
  const int kNumInputPorts = 1;
  const int kPortIndex = 0;

  // Translates the message contained within the receive buffer by storing its
  // information in basic_vector_.
  void handleMessage(const ::lcm::ReceiveBuffer* rbuf,
                     const std::string& channel);

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // The translator that converts between LCM messages and
  // drake::systems::BasicVector.
  const LcmAndVectorInterfaceTranslator& translator_;

  // A pointer to the LCM subsystem.
  ::lcm::LCM* lcm_;

  // Holds the information contained with the latest LCM message. This
  // information is copied into this system's output port when Output(...) is
  // called.
  BasicVector<double> basic_vector_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
