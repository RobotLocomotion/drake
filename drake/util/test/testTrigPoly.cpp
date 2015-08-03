#include "TrigPoly.h"
#include <iostream>

using namespace Eigen;
using namespace std;

int main(int argc, char **argv) {

  Polynomiald q("q");
  Polynomiald s("s");
  Polynomiald c("c");

  TrigPolyd p(q,s,c);

  cout << p << endl;
  cout << sin(p) << endl;
  cout << cos(p) << endl;
  cout << (sin(p)*p*p + cos(p)) << endl;

  cout << "sin(p + p) = " << sin(p+p) << endl;
}
