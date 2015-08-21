
#include <iostream>
#include "Pendulum.h"

using namespace std;

int main(int argc, char* argv[]) {
  Pendulum p;

  p.simulate(0,5,p.getRandomState());
}
