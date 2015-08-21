#ifndef DRAKE_LCMCOORDINATEFRAME_H
#define DRAKE_LCMCOORDINATEFRAME_H

#include <lcm/lcm-cpp.hpp>
#include "CoordinateFrame.h"
#include "DrakeSystem.h"

//class LCMInput;
class LCMOutput;

/// To use your own lcmtypes, simply define static methods of the form:
///
/// static bool encode(const CoordinateFrame& frame, const double t, const Eigen::VectorXd& x, MessageType& msg);
///    and
/// static bool decode(const CoordinateFrame& frame, const MessageType& msg, double& t, Eigen::VectorXd& x);
///
/// just like the examples in LCMCoordinateFrame.cpp

template <typename MessageType>
class DLLEXPORT LCMCoordinateFrame : public CoordinateFrame {
public:
  LCMCoordinateFrame(const std::string& _name, const std::vector<std::string>& _coordinates, std::shared_ptr<lcm::LCM> _lcm)
          : CoordinateFrame(_name,_coordinates), lcm(_lcm), channel(_name) {};

  void setChannel(const std::string& channel_name) {
    channel = channel_name;
  }

  void publish(const double t, const Eigen::VectorXd& x) {
    MessageType msg;
    if (!encode(this,t,x,msg))
      throw std::runtime_error(std::string("failed to encode")+msg->getTypeName());
    lcm->publish(channel,msg);
  }

protected:
  std::shared_ptr<lcm::LCM> lcm;
  std::string channel;
};

/*
class LCMInput : public DrakeSystem {

};
*/

/*
class LCMOutput : public DrakeSystem {
public:
  LCMOutput(std::shared_ptr<LCMCoordinateFrame> _lcm_coordinate_frame)
          : DrakeSystem(channel_name,nullptr,nullptr,_lcm_coordinate_frame,nullptr),
            lcm_coordinate_frame(_lcm_coordinate_frame) {}
  virtual ~LCMOutput(void) {};

  virtual VectorXs output(double t, const VectorXs& x, const VectorXs& u) {
    lcm_coordinate_frame->publish(t,u);
  }

protected:
  std::shared_ptr<LCMCoordinateFrame> lcm_coordinate_frame;
};
*/


#endif //DRAKE_LCMCOORDINATEFRAME_H
