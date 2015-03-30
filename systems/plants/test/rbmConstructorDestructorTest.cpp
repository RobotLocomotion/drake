

#include <iostream>
#include <cstdlib>
#include "RigidBodyManipulator.h"

using namespace std;

int main(int argc, char* argv[])
{
	if (argc<2) {
		cerr << "Usage: rbmConstructorDestructorTest urdf_filename" << endl;
		exit(-1);
	}
  RigidBodyManipulator* model = new RigidBodyManipulator(argv[1]);
  if (!model) {
  	cerr << "ERROR: Failed to load model from " << argv[1] << endl;
  	return -1;
  }
  delete model;
  return 0;
}
