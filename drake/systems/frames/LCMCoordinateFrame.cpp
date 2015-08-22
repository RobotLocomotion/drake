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

static bool decode(const CoordinateFrame* frame, const drake::lcmt_drake_signal& msg, double& t, VectorXd& x) {
  t = double(msg.timestamp)/1000.0;
  for (int i=0; i<msg.dim; i++) { x(i) = msg.val[i]; }  // todo: make this more robust by doing string matching on the coordinate names (with a hashmap?)
  return true;
}

template <class MessageType>
void LCMCoordinateFrame<MessageType>::publish(const double t, const Eigen::VectorXd& x) {
  MessageType msg;
  if (!encode(this,t,x,msg))
    throw std::runtime_error(std::string("failed to encode")+msg.getTypeName());
  lcm->publish(channel,&msg);
}

template <class MessageType>
DrakeSystemPtr LCMCoordinateFrame<MessageType>::setupLCMInputs(const DrakeSystemPtr& sys) {
  return cascade(make_shared<LCMInput<MessageType> >(this->shared_from_this()),sys);
}

template <class MessageType>
DrakeSystemPtr LCMCoordinateFrame<MessageType>::setupLCMOutputs(const DrakeSystemPtr& sys) {
  return cascade(sys,make_shared<LCMOutput<MessageType> >(this->shared_from_this()));
}

template <class MessageType>
void LCMInput<MessageType>::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const MessageType* msg) {
  data_mutex.lock();
  decode(lcm_coordinate_frame.get(),*msg,timestamp,data);
  data_mutex.unlock();
}


template class LCMCoordinateFrame<drake::lcmt_drake_signal>;