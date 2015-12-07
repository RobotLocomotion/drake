#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"

using namespace std;

int main(int argc, char* argv[]) {
/*
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  Pendulum p(lcm);
  DrakeSystemPtr controller = make_shared<PendulumEnergyShaping>(p);

  runLCM(controller,*lcm,0,5,Eigen::VectorXd::Zero(0));
*/

  // todo: move these to core/test
  Eigen::Vector3d abc;  abc << 1,2,3;
  {
    Drake::CombinedVector<double, PendulumState, PendulumInput> test(abc);
    cout << test << endl;
    test=2*abc;
    cout << test << endl;
  }
  {
    Drake::CombinedVectorBuilder<PendulumState,PendulumInput>::VecType<double> test(abc);
    cout << test << endl;
    test=2*abc;
    cout << test << endl;
  }

  auto p = std::make_shared<Pendulum>();
  auto c = std::make_shared<PendulumEnergyShaping>(*p);

  cout << "p has " << p->num_states << " states" << endl;
  cout << "c has " << c->num_states << " states" << endl;

  // todo: make this syntax less painful!
  Drake::FeedbackSystem<Pendulum,PendulumState,PendulumEnergyShaping,Drake::UnusedVector,
          PendulumInput,PendulumState,false,false,false,true> sys(p,c);

  Eigen::Vector2d x0; x0 << 0.1, 0.2;
  simulate(sys,0,.1,x0);

}

