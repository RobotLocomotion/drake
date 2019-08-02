#pragma once

#include <map>
#include <string>
#include <utility>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using drake::multibody::MultibodyPlant;

/**
 * Welds each finger's base frame to the world. The planar-gripper is made up of
 * three 2-DOF fingers, whose bases are fixed equidistant along the perimeter of
 * a circle. All "fingertips" point inwards (towards the world origin) when all
 * joint angles are zero, and all motion lies in the Y-Z plane.
 * @tparam T The scalar type. Currently only supports double.
 * @param plant The plant containing the planar-gripper.
 */
template <typename T>
void WeldGripperFrames(MultibodyPlant<T>* plant);

/**
 * Parses a text file containing keyframe joint positions for the planar gripper
 * and the planar brick (the object being manipulated).
 * @param[in] name The file name to parse.
 * @param[out] brick_initial_pose A vector containing the initial brick pose.
 * @return A std::pair containing a matrix of finger joint position keyframes
 * (each matrix row represents a single keyframe containing values for all joint
 * positions) and a std::map containing the mapping between each finger joint
 * name and the corresponding column index in the keyframe matrix containing the
 * data for that joint.
 * @pre The file should begin with a header row that indicates the joint
 * ordering for keyframes. Header names should consist of three finger base
 * joints, three finger mid joints, and three brick joints (9 total):
 * {finger1_BaseJoint, finger2_BaseJoint, finger3_BaseJoint, finger1_MidJoint,
 * finger2_MidJoint, finger3_MindJoint, brick_translate_y_joint,
 * brick_translate_z_joint, brick_revolute_x_joint}. Names may appear in any
 * order. Each row (keyframe) following the header should contain the same
 * number of values as indicated in the header. All entries should be white
 * space delimited and the file should end in a newline character. The behavior
 * of parsing is undefined if these conditions are not met.
 */
std::pair<MatrixX<double>, std::map<std::string, int>> ParseKeyframes(
    const std::string& name, EigenPtr<Vector3<double>> brick_initial_pose);

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
