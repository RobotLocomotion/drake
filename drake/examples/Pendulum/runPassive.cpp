
#include <iostream>
#include "Pendulum.h"

using namespace std;

int main(int argc, char* argv[]) {
  Pendulum p;

  VectorXd x = VectorXd::Zero(2);
  VectorXd u = VectorXd::Zero(1);
  cout << "xdot = " << p.dynamics(0,x,u).transpose() << endl;
}