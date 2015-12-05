
#include <iostream>
#include "Pendulum.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {
/*
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
   return 1;

  DrakeSystemPtr p(new PendulumWithBotVis(lcm));
  runLCM(p,*lcm,0,50,p->getRandomState());
  cout << "output frame: " << p->getOutputFrame() << endl;
*/
  Pendulum p;
  PendulumState<double> x0;
  x0.theta = 1;
  x0.thetadot = 0;
  cout << "PendulumState::x0 = " << x0 << endl;
  Eigen::Vector2d e2x0(x0);
  cout << "Vector2d::x0 = " << e2x0.transpose() << endl;

//  Eigen::Vector3<double> test;
//  cout << test.transpose() << endl;

  simulate(p,0,.1,e2x0);

  // todo: move these to core/test
  cout << "rows at compile time: " << VectorTraits<PendulumState<double> >::RowsAtCompileTime << endl;
  cout << "rows at compile time: " << VectorTraits<Eigen::Matrix<double,2,1> >::RowsAtCompileTime << endl;
}
