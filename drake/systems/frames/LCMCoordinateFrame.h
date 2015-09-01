#ifndef DRAKE_LCMCOORDINATEFRAME_H
#define DRAKE_LCMCOORDINATEFRAME_H

#include "CoordinateFrame.h"
#include "DrakeSystem.h"

#include <chrono>
#include <mutex>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drake/lcmt_drake_signal.hpp"

template <class MessageType> class LCMInput;
template <class MessageType> class LCMOutput;

/// To use your own lcmtypes, simply define methods of the form:
///
/// bool encode(const CoordinateFrame& frame, const double t, const Eigen::VectorXd& x, MessageType& msg);
///    and
/// bool decode(const CoordinateFrame& frame, const MessageType& msg, double& t, Eigen::VectorXd& x);
///
/// just like the examples in LCMCoordinateFrame.cpp (externed below)

extern bool encode(const CoordinateFrame& frame, const double t, const Eigen::VectorXd& x, drake::lcmt_drake_signal& msg);
extern bool decode(const CoordinateFrame& frame, const drake::lcmt_drake_signal& msg, double& t, Eigen::VectorXd& x);

template <class MessageType = drake::lcmt_drake_signal>
class DLLEXPORT LCMCoordinateFrame : public CoordinateFrame, public std::enable_shared_from_this<LCMCoordinateFrame<MessageType> > {
public:
  LCMCoordinateFrame(const std::string& _name, const std::vector<std::string>& _coordinates, const std::shared_ptr<lcm::LCM>& _lcm)
          : CoordinateFrame(_name,_coordinates), lcm(_lcm), channel(_name) {};
  virtual ~LCMCoordinateFrame(void) {};


  virtual DrakeSystemPtr setupLCMInputs(const DrakeSystemPtr& sys) const override {
    return cascade(std::make_shared<LCMInput<MessageType> >(this->shared_from_this()), sys);
  }
  virtual DrakeSystemPtr setupLCMOutputs(const DrakeSystemPtr& sys) const override {
    return cascade(sys,std::make_shared<LCMOutput<MessageType> >(this->shared_from_this()));
  }

  std::shared_ptr<lcm::LCM> lcm;
  std::string channel;
};


template <class MessageType>
class LCMInput : public DrakeSystem {
public:
  LCMInput(const std::shared_ptr<const LCMCoordinateFrame<MessageType> >& _lcm_coordinate_frame)
          : DrakeSystem(_lcm_coordinate_frame->name,nullptr,nullptr,nullptr,_lcm_coordinate_frame),
            lcm_coordinate_frame(_lcm_coordinate_frame),
            timestamp(0.0), data(Eigen::VectorXd::Zero(_lcm_coordinate_frame->getDim())) {
    // subscribe to the lcm traffic:
    lcm::Subscription* sub = lcm_coordinate_frame->lcm->subscribe(lcm_coordinate_frame->channel,&LCMInput<MessageType>::handleMessage,this);
    sub->setQueueCapacity(1);
  }
  virtual ~LCMInput(void) {};

  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const MessageType* msg) {
    data_mutex.lock();
    decode(*lcm_coordinate_frame,*msg,timestamp,data);
    data_mutex.unlock();
  }

// note: only vectorxd here (not the templated types).  our lcm traffic will all use doubles
  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    data_mutex.lock();
    Eigen::VectorXd y = data;  // make a copy of the data
    data_mutex.unlock();
    return y;
  }

protected:
  mutable std::mutex data_mutex;
  double timestamp;
  Eigen::VectorXd data;
  std::shared_ptr<const LCMCoordinateFrame<MessageType> > lcm_coordinate_frame;
};


template <class MessageType>
class LCMOutput : public DrakeSystem {
public:
  LCMOutput(const std::shared_ptr<const LCMCoordinateFrame<MessageType> >& _lcm_coordinate_frame)
          : DrakeSystem(_lcm_coordinate_frame->name,nullptr,nullptr,_lcm_coordinate_frame,nullptr),
            lcm_coordinate_frame(_lcm_coordinate_frame) {}
  virtual ~LCMOutput(void) {};

  // note: only vectorxd here (not the templated types).  our lcm traffic will all use doubles
  virtual Eigen::VectorXd output(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override {
    MessageType msg;
    if (!encode(*lcm_coordinate_frame,t,u,msg))
    throw std::runtime_error(std::string("failed to encode")+msg.getTypeName());
    lcm_coordinate_frame->lcm->publish(lcm_coordinate_frame->channel,&msg);
    return Eigen::VectorXd::Zero(0);
  }

protected:
  std::shared_ptr<const LCMCoordinateFrame<MessageType> > lcm_coordinate_frame;
};

// todo:  can I send in only a const DrakeSystem& instead?
extern void runLCM(const DrakeSystemPtr& sys, lcm::LCM& lcm, double t0, double tf, const Eigen::VectorXd& x0, const DrakeSystem::SimulationOptions* options=nullptr);


#endif //DRAKE_LCMCOORDINATEFRAME_H
