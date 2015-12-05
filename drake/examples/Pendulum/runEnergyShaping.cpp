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
  Pendulum p;
  PendulumEnergyShaping c(p);

  Drake::CombinedVector<double,PendulumState,PendulumInput> test;
  Drake::CombinedVectorBuilder<PendulumState,PendulumInput>::VecType<double> test2;
  Drake::VectorBuilder<2>::VecType<double> test3;

  cout << "test: " << test3.transpose() << endl;

//  auto sys = feedback(p,c);
}

