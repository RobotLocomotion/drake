#include "LCMCoordinateFrame.h"

using namespace std;
using namespace Eigen;

static bool encode(const CoordinateFrame* frame, const double t, const VectorXd& x, drake::lcmt_drake_signal& msg) {
  msg.timestamp = static_cast<int64_t>(t*1000);
  msg.dim = frame->getDim();
  msg.coord = frame->getCoordinateNames(); // note: inefficient to do a deep copy every time
  for (int i=0; i<msg.dim; i++) msg.val.push_back(x(i));
  return true;
}

static bool decode(const CoordinateFrame& frame, const drake::lcmt_drake_signal& msg, double& t, VectorXd& x) {
  throw runtime_error("decode lcmt_drake_signal not implemented yet (will be trivial).");
  return false;
}

template <class MessageType>
void LCMCoordinateFrame<MessageType>::publish(const double t, const Eigen::VectorXd& x) {
  MessageType msg;
  if (!encode(this,t,x,msg))
    throw std::runtime_error(std::string("failed to encode")+msg.getTypeName());
  lcm->publish(channel,&msg);
}


template <class MessageType>
DrakeSystemPtr LCMCoordinateFrame<MessageType>::setupLCMOutputs(DrakeSystemPtr sys) {
  return cascade(sys,make_shared<LCMOutput<MessageType> >(this->shared_from_this()));
}

template class LCMCoordinateFrame<drake::lcmt_drake_signal>;