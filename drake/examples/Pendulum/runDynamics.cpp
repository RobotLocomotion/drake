
#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"
#include "BotVisualizer.h"

using namespace std;

class PendulumWithBotVis : public Pendulum {
public:
  PendulumWithBotVis(const shared_ptr<lcm::LCM>& lcm) : botvis(lcm,"Pendulum.urdf",DrakeJoint::FIXED) {}

  virtual VectorXs output(double t, const VectorXs& x, const VectorXs& u) const override {
    botvis.output(t,Eigen::VectorXd::Zero(0),x);
    return Pendulum::output(t,x,u);
  }

  BotVisualizer botvis;
};

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
   return 1;

  DrakeSystemPtr p = make_shared<PendulumWithBotVis>(lcm);

  p->input_frame = make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("PendulumInput",p->input_frame->getCoordinateNames(),lcm);
  p->output_frame = make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("PendulumState",p->output_frame->getCoordinateNames(),lcm);

  runLCM(p,lcm,0,50,p->getRandomState());

  cout << "output frame: " << p->getOutputFrame() << endl;
}
