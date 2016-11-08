#include "drake/lcm/lcm_call_matlab.h"

#include <cstring>
#include <limits>

#include "lcm/lcm-cpp.hpp"

#include "drake/lcmt_call_matlab.hpp"
#include "drake/lcmt_matlab_array.hpp"

static int g_uid=0;

LcmMatlabRemoteVariable::LcmMatlabRemoteVariable()
  : uid(g_uid++)
// TODO(russt): replace this with a random int64_t, e.g. http://stackoverflow.com/questions/7114043/random-number-generation-in-c11-how-to-generate-how-do-they-work
// TODO(russt): david-german-tri recommended a more robust (but more complex) solution was to use e.g.
// [IP address + process id + time].  We decided this was sufficient for now.
{}

void to_lcm_matlab_array(const LcmMatlabRemoteVariable& var, drake::lcmt_matlab_array& matlab_array)
{
  matlab_array.type = drake::lcmt_matlab_array::REMOTE_VARIABLE_REFERENCE;
  matlab_array.rows = 1;
  matlab_array.cols = 1;
  matlab_array.num_bytes = sizeof(int64_t);
  matlab_array.data.resize(matlab_array.num_bytes);
  memcpy(matlab_array.data.data(),&var.uid,matlab_array.num_bytes);
}

void to_lcm_matlab_array(const Eigen::Ref<Eigen::MatrixXd>& mat, drake::lcmt_matlab_array& matlab_array)
{
  matlab_array.type = drake::lcmt_matlab_array::DOUBLE;
  matlab_array.rows = mat.rows();
  matlab_array.cols = mat.cols();
  matlab_array.num_bytes = sizeof(double)*mat.rows()*mat.cols();
  matlab_array.data.resize(matlab_array.num_bytes);
  memcpy(matlab_array.data.data(),mat.data(),matlab_array.num_bytes);
}

void to_lcm_matlab_array(const std::string& str, drake::lcmt_matlab_array& matlab_array)
{
  matlab_array.type = drake::lcmt_matlab_array::CHAR;
  matlab_array.rows = 1;
  matlab_array.cols = str.length();
  matlab_array.num_bytes = sizeof(char)*str.length();
  matlab_array.data.resize(matlab_array.num_bytes);
  memcpy(matlab_array.data.data(),str.data(),matlab_array.num_bytes);
}

void internal::publish_lcm_call_matlab(const std::string& channel, const drake::lcmt_call_matlab& msg)
{
  static lcm::LCM lcm;  // just keep a local copy here for publishing
  if (!lcm.good()) return;

  lcm.publish(channel,&msg);
}
