#ifndef DRAKE_LCMCOORDINATEFRAME_H
#define DRAKE_LCMCOORDINATEFRAME_H

#include <mutex>
#include <lcm/lcm-cpp.hpp>
#include "CoordinateFrame.h"
#include "DrakeSystem.h"
#include "lcmtypes/drake/lcmt_drake_signal.hpp"

//class LCMInput;
template <class MessageType> class LCMOutput;

/// To use your own lcmtypes, simply define static methods of the form:
///
/// static bool encode(const CoordinateFrame* frame, const double t, const Eigen::VectorXd& x, MessageType& msg);
///    and
/// static bool decode(const CoordinateFrame* frame, const MessageType& msg, double& t, Eigen::VectorXd& x);
///
/// just like the examples in LCMCoordinateFrame.cpp

template <class MessageType = drake::lcmt_drake_signal>
class DLLEXPORT LCMCoordinateFrame : public CoordinateFrame, public std::enable_shared_from_this<LCMCoordinateFrame<MessageType> > {
public:
  LCMCoordinateFrame(const std::string& _name, const std::vector<std::string>& _coordinates, const std::shared_ptr<lcm::LCM>& _lcm)
          : CoordinateFrame(_name,_coordinates), lcm(_lcm), channel(_name) {};
  virtual ~LCMCoordinateFrame(void) {};

  void setChannel(const std::string& channel_name) {
    channel = channel_name;
  }

  void publish(const double t, const Eigen::VectorXd& x);

  virtual DrakeSystemPtr setupLCMInputs(const DrakeSystemPtr& sys);
  virtual DrakeSystemPtr setupLCMOutputs(const DrakeSystemPtr& sys);

  std::shared_ptr<lcm::LCM> lcm;
  std::string channel;
};


template <class MessageType>
class LCMInput : public DrakeSystem {
public:
  LCMInput(const std::shared_ptr<LCMCoordinateFrame<MessageType> >& _lcm_coordinate_frame)
          : DrakeSystem(_lcm_coordinate_frame->name,nullptr,nullptr,nullptr,_lcm_coordinate_frame),
            lcm_coordinate_frame(_lcm_coordinate_frame) {
    // subscribe to the lcm traffic:
    lcm_coordinate_frame->lcm->subscribe(lcm_coordinate_frame->channel,&LCMInput<MessageType>::handleMessage,this);
  }
  virtual ~LCMInput(void) {};

  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const MessageType* msg);

  // note: only vectorxd here (not the templated types).  our lcm traffic will all use doubles
  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
    data_mutex.lock();
    Eigen::VectorXd y = data;  // make a copy of the data
    data_mutex.unlock();
    return y;
  }

protected:
  std::mutex data_mutex;
  double timestamp;
  Eigen::VectorXd data;
  std::shared_ptr<LCMCoordinateFrame<MessageType> > lcm_coordinate_frame;
};


template <class MessageType>
class LCMOutput : public DrakeSystem {
public:
  LCMOutput(const std::shared_ptr<LCMCoordinateFrame<MessageType> >& _lcm_coordinate_frame)
          : DrakeSystem(_lcm_coordinate_frame->name,nullptr,nullptr,_lcm_coordinate_frame,nullptr),
            lcm_coordinate_frame(_lcm_coordinate_frame) {}
  virtual ~LCMOutput(void) {};

  // note: only vectorxd here (not the templated types).  our lcm traffic will all use doubles
  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
    lcm_coordinate_frame->publish(t,u);
    return Eigen::VectorXd::Zero(0);
  }

protected:
  std::shared_ptr<LCMCoordinateFrame<MessageType> > lcm_coordinate_frame;
};


#endif //DRAKE_LCMCOORDINATEFRAME_H
