#ifndef __DrakeSystem_H__
#define __DrakeSystem_H__

#include <string>
#include <memory>
#include "CoordinateFrame.h"


/// A dynamical system authored in Drake registers it's dynamics as well as information
/// about it's coordinate frames.

class DrakeSystem {
public:

  DrakeSystem(const std::string& name,
              std::shared_ptr<CoordinateFrame> continuous_state_frame,
              std::shared_ptr<CoordinateFrame> discrete_state_frame,
              std::shared_ptr<CoordinateFrame> input_frame,
              std::shared_ptr<CoordinateFrame> output_state_frame);

  DrakeSystem(const std::string& name,
              unsigned int num_continuous_states,
              unsigned int num_discrete_states,
              unsigned int num_inputs,
              unsigned int num_outputs);

  DrakeSystem(const std::string& name) : DrakeSystem(name,0,0,0,0) {}; // build the empty system

  const std::shared_ptr<CoordinateFrame> getInputFrame() { return input_frame; }
  const std::shared_ptr<CoordinateFrame> getStateFrame() { return state_frame; }
  const std::shared_ptr<CoordinateFrame> getOutputFrame() { return output_frame; }

private:
  std::string name;

  std::shared_ptr<CoordinateFrame> input_frame;
  std::shared_ptr<CoordinateFrame> continuous_state_frame, discrete_state_frame, state_frame;
  std::shared_ptr<CoordinateFrame> output_frame;

  /*
  bool is_direct_feedthrough;  // does the output method depend on the input u?  set false if you can!
  bool is_time_invariant; // are all of the dynamics and output methods independent of time? set to true if you can!
   */
};

#endif // #define __DrakeSystem_H_