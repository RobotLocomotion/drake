
#ifndef URDF_H
#define URDF_H

#include "RigidBodyManipulator.h"

void ROS_ERROR(const char* format, ...);
RigidBodyManipulator* parseURDFModel(const std::string &xml_string);
RigidBodyManipulator* loadURDF(const std::string &urdf_filename);

#endif
