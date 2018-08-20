#include "drake/examples/allegro_hand/allegro_set_joint_pose.h"
#include "drake/examples/allegro_hand/allegro_common.h"

namespace drake {
namespace examples {
namespace allegro_hand {

AllegroConstantJointValue::AllegroConstantJointValue(
            multibody::multibody_plant::MultibodyPlant<double>& hand_plant){
    GetControlPortMapping(hand_plant, Px, Py);

    joint_name_mapping = SetJointNameMapping();

    AllegroNumJoints = kAllegroNumJoints;  FingerNum = AllegroNumJoints / 4;
    TargetPose = Eigen::VectorXd::Zero(AllegroNumJoints * 2); 
}

Eigen::VectorXd AllegroConstantJointValue::set_joint_pose(
    const std::string& joint_name, const double target_angle){

    if (joint_name_mapping.find(joint_name) != joint_name_mapping.end())
        TargetPose(joint_name_mapping[joint_name]) = target_angle;
    return TargetPose;
}

Eigen::VectorXd AllegroConstantJointValue::set_finger_pose(const int finger_index, 
                                              VectorX<double>& target_angles){
    DRAKE_DEMAND(finger_index<FingerNum);
    TargetPose.segment(finger_index * 4, 4) << target_angles;
    return TargetPose;
}

Eigen::VectorXd AllegroConstantJointValue::set_open_hand(){
    TargetPose.setZero();
    set_joint_pose("joint_12", 0.263);
    set_joint_pose("joint_13", 1.1);
    return TargetPose;
}

Eigen::VectorXd AllegroConstantJointValue::set_close_hand(){
    set_finger_pose(0,  1.1,   1.5,  1.5, 1.);   //Thumb
    set_finger_pose(1, -0.1,   1.6,  1.7, 1.);   //Index
    set_finger_pose(2,    0,   1.6,  1.7, 1.);   //Middle
    set_finger_pose(3,  0.1,   1.6,  1.7, 1.);   //end
    return TargetPose;
}

Eigen::VectorXd AllegroConstantJointValue::set_test_position(){
    TargetPose.setZero();
    set_finger_pose(0,  0.5,   0.1,  0.5, 0.2);
    set_joint_pose(1, 0, 0.4);
    set_joint_pose(14, 0.4);
    return TargetPose;

}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake