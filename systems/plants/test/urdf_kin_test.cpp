
#include <fstream>
#include <iostream>
#include <cstdlib>
#include "../urdf.h"

using namespace std;

int main(int argc, char* argv[])
{
  // todo: pull urdf filename off the command line
  std::string filename("../../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
  }
  else
  {
    std::cerr << "Could not open file ["<<filename.c_str()<<"] for parsing."<< std::endl;
    return -1;
  }
  
  // parse URDF to get model
  RigidBodyManipulator* model = parseURDFModel(xml_string);
  
  // run kinematics with second derivatives 100 times
  double q[34];
  int i;
  
  for (int n=0; n<100; n++) {
    for (i=0; i<34; i++) q[i]=(double)rand();
    model->doKinematics(q,true);
  }
  
  Vector4d zero;
  zero << 0,0,0,1; 
  
  for (int i=0; i<=model->NB; i++) {
    cout << "forward kin: " << model->bodies[i].linkname << " is at " << model->forwardKin(i,zero,true).transpose() << endl;
  } 
  
  delete model;
  return 0;
}