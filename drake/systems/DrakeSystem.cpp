
#include "DrakeSystem.h"

using namespace std;

DrakeSystem::DrakeSystem(const std::string &name, std::shared_ptr<CoordinateFrame> _continuous_state_frame,
                         std::shared_ptr<CoordinateFrame> _discrete_state_frame,
                         std::shared_ptr<CoordinateFrame> _input_frame,
                         std::shared_ptr<CoordinateFrame> _output_frame)
  : continuous_state_frame(_continuous_state_frame), discrete_state_frame(_discrete_state_frame),
    input_frame(_input_frame), output_frame(_output_frame) {
  state_frame = shared_ptr<CoordinateFrame>(new MultiCoordinateFrame(name+"state",{continuous_state_frame,discrete_state_frame}));
}

DrakeSystem::DrakeSystem(const std::string &name, unsigned int num_continuous_states, unsigned int num_discrete_states,
                         unsigned int num_inputs, unsigned int num_outputs)
        : input_frame(nullptr), continuous_state_frame(nullptr), discrete_state_frame(nullptr), output_frame(nullptr) {
  if (num_inputs>0) input_frame = shared_ptr<CoordinateFrame>(new CoordinateFrame(name+"Input",num_inputs,"u"));
  if (num_continuous_states>0) continuous_state_frame = shared_ptr<CoordinateFrame>(new CoordinateFrame(name+"ContinuousState",num_continuous_states,"xc"));
  if (num_discrete_states>0) discrete_state_frame = shared_ptr<CoordinateFrame>(new CoordinateFrame(name+"DiscreteState",num_discrete_states,"xd"));
  if (num_outputs>0) output_frame = shared_ptr<CoordinateFrame>(new CoordinateFrame(name+"Output",num_outputs,"y"));
  state_frame = shared_ptr<CoordinateFrame>(new MultiCoordinateFrame(name+"state",{continuous_state_frame,discrete_state_frame}));
}