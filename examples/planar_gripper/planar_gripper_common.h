#pragma once

#include <map>
#include <string>
#include <utility>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace planar_gripper {

using drake::multibody::MultibodyPlant;
using Eigen::Vector3d;

// The planar-gripper coordinate frame G (with origin Go) and finger layout are
// defined as follows (assuming all finger joint angles are set to zero):
//
//       F1_base         F3_base
//              \   +Gz    /
//               \   |   /
//                \  |  /
//                 ● | ●
//                   Go----+Gy
//                   ●
//                   |
//                   |
//                   |
//                 F2_base
//
// The gripper frame's Y and Z axes are denote Gy and Gz, respectively. When the
// planar-gripper is welded via WeldGripperFrames(), the coordinate frame G
// perfectly coincides with the world coordinate frame W.

/**
 * Welds each finger's base frame to the world. The planar-gripper is made up of
 * three 2-DOF fingers, whose bases are fixed equidistant along the perimeter of
 * a circle. The origin (Go) of the planar gripper is located at the center of
 * the workspace, i.e., if all joints are set to zero and all fingers are
 * pointing inwards, the origin is the point that is equidistant to all
 * fingertips. This method welds the planar-gripper such that all motion lies in
 * the Y-Z plane (in frame G). Note: The planar gripper frame G perfectly
 * coincides with the world coordinate frame W when welded via this method.
 * @tparam T The scalar type. Currently only supports double.
 * @param plant The plant containing the planar-gripper.
 */
template <typename T>
void WeldGripperFrames(MultibodyPlant<T>* plant);

/**
 * Parses a text file containing keyframe joint positions for the planar gripper
 * and the planar brick (the object being manipulated).
 * @param[in] name The file name to parse.
 * @param[out] brick_initial_pose A vector containing the initial brick pose,
 * expressed in the gripper frame G.
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
 * brick_translate_z_joint, brick_revolute_x_joint}. Note that brick
 * translations should be expressed in the planar-gripper frame G. Names may
 * appear in any order. Each row (keyframe) following the header should contain
 * the same number of values as indicated in the header. All entries should be
 * white space delimited and the file should end in a newline character. The
 * behavior of parsing is undefined if these conditions are not met.
 */
std::pair<MatrixX<double>, std::map<std::string, int>> ParseKeyframes(
    const std::string& name, EigenPtr<Vector3<double>> brick_initial_pose);

/// Returns the planar gripper frame G's transform w.r.t. the world frame W.
const math::RigidTransformd X_WGripper();

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
