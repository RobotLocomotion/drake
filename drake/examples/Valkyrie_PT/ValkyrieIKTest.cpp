//
// Created by Pang Tao on 21/10/16.
//
#include <iostream>
#include <limits>
#include "drake/examples/Valkyrie_PT/Valkyrie_plant.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"

// Includes for the planner.
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"

using drake::BotVisualizer;
using drake::ValkyriePlant;
using namespace std;
using namespace Eigen;

std::vector<int> GetJointPositionVectorIndices(const RigidBodyTree* tree,
                                               const std::string& name);
void findJointAndInsert(const RigidBodyTree* model, const std::string& name,
                        std::vector<int>& position_list);

int main() {
    std::shared_ptr<ValkyriePlant> val_sys = std::make_shared<ValkyriePlant>();
    auto const& tree = val_sys->get_rigid_body_tree();

    // tests
    std::cout << "Number of positions: "<< tree->get_num_positions() << std::endl;
    for(int i=0;i<tree->get_num_positions();i++) {
        cout << i << ":" << tree->get_position_name(i) << endl;
    }

    std::cout << "Number of velocities: "<< tree->get_num_velocities() << std::endl;
    for(int i=0;i<tree->get_num_velocities();i++) {
        cout << i << ":" << tree->get_velocity_name(i) << " " << tree->get_position_name(i) << endl;
    }

    std::cout << "Number of bodies: " << tree->get_num_bodies() << endl;
    for(int i=0;i<tree->get_num_bodies();i++) {
        cout << i << ":" << tree->getBodyOrFrameName(i) << endl;
    }

    auto leftPalmPtr = tree->FindBody("leftPalm");
    Eigen::Matrix3Xd leftPalmContactPts = leftPalmPtr->get_contact_points();
    cout << "LeftPalmContactPts:" << endl;
    cout << leftPalmContactPts << endl;

    // copies testIKMoreConstraints.cpp
    Vector2d tspan;
    tspan << 0, 1;

    VectorXd reach_start(tree->get_num_positions());
    reach_start <<  0.0, //0
    0.0,
    1.025,
    0.0,
    0.0,
    0.0, //5
    0.0,
    0.0,
    0.0,
    0.0, //9:lowerNeckPitch
    0.0, //10:neckYaw
    0.0, //11:upperNeckPitch
    0.30019663134302466,
    1.25,
    0.0,
    0.7853981633974483, //15
    1.571,
    0.0,
    0.0,
    0.30019663134302466,
    -1.25, //20
    0.0,
    -0.7853981633974483,
    1.571,
    0.0,
    0.0, //25
    0.0, //26
    0.0,
    -0.49,
    1.205, //29 rightKneePitch
    -0.71, //30
    0.0, //31
    0.0,
    0.0,
    -0.49,
    1.205,//35 leftKneePitch
    -0.71,//36
    0.0;//37

    KinematicsCache<double> cache = tree->doKinematics(reach_start);

    // 1 Neck Posture Constraint, posture constraints are imposed on q
    PostureConstraint kc_posture_neck(tree.get(), tspan);
    vector<int> neck_idx;
    findJointAndInsert(tree.get(), "lowerNeckPitch", neck_idx);
    findJointAndInsert(tree.get(), "neckYaw", neck_idx);
    findJointAndInsert(tree.get(), "upperNeckPitch", neck_idx);
    VectorXd neck_lb = VectorXd::Zero(3);
    VectorXd neck_ub = VectorXd::Zero(3);
    kc_posture_neck.setJointLimits(3, neck_idx.data(), neck_lb, neck_ub);

    // 2 left foot position and orientation constraint, position and orientation constraints are imposed on frames/bodies
    int l_foot = tree->FindBodyIndex("leftFoot");
    Vector4d lfoot_quat = tree->relativeQuaternion(cache, l_foot, 0);
    cout << "Relative quaternion between left foot and world" << endl << lfoot_quat << endl;

    Vector3d origin(0,0,0);
    cout << "origin is" << endl << origin << endl;
    auto lfoot_pos0 = tree->transformPoints(cache, origin, l_foot, 0);
    cout << "Relative position between left foot and world" << endl << lfoot_pos0 << endl;
    Vector3d lfoot_pos_lb =lfoot_pos0;
    lfoot_pos_lb(0) -= 0.001;
    lfoot_pos_lb(1) -= 0.001;
    lfoot_pos_lb(2) -= 0.001;
    Vector3d lfoot_pos_ub =lfoot_pos0;
    lfoot_pos_ub(0) += 0.001;
    lfoot_pos_ub(1) += 0.001;
    lfoot_pos_ub(2) += 0.001;
    WorldPositionConstraint kc_lfoot_pos(tree.get(), l_foot, origin, lfoot_pos_lb,
                                         lfoot_pos_ub, tspan);
    double tol = 0.0017453292519943296;
    WorldQuatConstraint kc_lfoot_quat(tree.get(), l_foot, lfoot_quat, tol, tspan);

    // 3 right foot position and orientation constraint
    int r_foot = tree->FindBodyIndex("rightFoot");
    Vector4d rfoot_quat = tree->relativeQuaternion(cache, r_foot, 0);
    cout << "Relative quaternion between right foot and world" << endl << rfoot_quat << endl;

    auto rfoot_pos0 = tree->transformPoints(cache, origin, r_foot, 0);
    cout << "Relative position between right foot and world" << endl << rfoot_pos0 << endl;
    Vector3d rfoot_pos_lb =rfoot_pos0;
    rfoot_pos_lb(0) -= 0.001;
    rfoot_pos_lb(1) -= 0.001;
    rfoot_pos_lb(2) -= 0.001;
    Vector3d rfoot_pos_ub =rfoot_pos0;
    rfoot_pos_ub(0) += 0.001;
    rfoot_pos_ub(1) += 0.001;
    rfoot_pos_ub(2) += 0.001;
    WorldPositionConstraint kc_rfoot_pos(tree.get(), r_foot, origin, rfoot_pos_lb,
                                         rfoot_pos_ub, tspan);
    WorldQuatConstraint kc_rfoot_quat(tree.get(), r_foot, rfoot_quat, tol, tspan);


    // 4 torso posture constraint
    double inf = std::numeric_limits<double>::infinity();
    PostureConstraint kc_posture_torso(tree.get(), tspan);
    vector<int> torso_idx;
    findJointAndInsert(tree.get(), "torsoYaw", torso_idx);
    findJointAndInsert(tree.get(), "torsoPitch", torso_idx);
    findJointAndInsert(tree.get(), "torsoRoll", torso_idx);
    cout << "torso indices " << torso_idx[0] << torso_idx[1] << torso_idx[2] << endl;
    Vector3d torso_nominal = Vector3d::Zero(3);
    Vector3d torso_half_range(0.2617993877991494, 0.4363323129985824, inf);
    Vector3d torso_lb = torso_nominal - torso_half_range;
    Vector3d torso_ub = torso_nominal + torso_half_range;
    torso_lb(1) = -0.08726646259971647;
    kc_posture_torso.setJointLimits(3, torso_idx.data(), torso_lb, torso_ub);
    //cout << "inf+1" << inf + 1.0 << endl;
    //cout << "-inf+1" << -inf + 1.0 << endl;

    // 5 knee posture constraint
    PostureConstraint kc_posture_knee(tree.get(), tspan);
    vector<int> knee_idx;
    findJointAndInsert(tree.get(), "leftKneePitch", knee_idx);
    findJointAndInsert(tree.get(), "rightKneePitch", knee_idx);
    Vector2d knee_nominal(reach_start(knee_idx[0]), reach_start(knee_idx[1]) );
    Vector2d knee_lb = knee_nominal;
    //knee_lb(0) += 0.60;
    //knee_lb(0) += 0.60;
    Vector2d knee_ub = knee_nominal;
    knee_ub(0) += 1.90;
    knee_ub(1) += 1.90;
    kc_posture_knee.setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);
    //cout << "knee_idx " << knee_idx[0] << knee_idx[1] << endl;
    //cout << "knee_nominal_posture" << knee_nominal << endl;

    // 6 left arm posture constraint
    PostureConstraint kc_posture_larm(tree.get(), tspan);
    vector<int> larm_idx;
    findJointAndInsert(tree.get(), "leftShoulderPitch", larm_idx);
    findJointAndInsert(tree.get(), "leftShoulderRoll", larm_idx);
    findJointAndInsert(tree.get(), "leftShoulderYaw", larm_idx);
    findJointAndInsert(tree.get(), "leftElbowPitch", larm_idx);
    findJointAndInsert(tree.get(), "leftForearmYaw", larm_idx);
    findJointAndInsert(tree.get(), "leftWristRoll", larm_idx);
    findJointAndInsert(tree.get(), "leftWristPitch", larm_idx);
    cout << "Number of elements in larm_idx " << larm_idx.size() << endl;
    VectorXd larm_lb = VectorXd::Zero(7);
    for(int i=0;i<7;i++)
        larm_lb(i) = reach_start(larm_idx[i]);
    VectorXd larm_ub = larm_lb;
    kc_posture_larm.setJointLimits(7, larm_idx.data(), larm_lb, larm_ub);

    // 7 right arm posture constraint
    PostureConstraint kc_posture_rarm(tree.get(), tspan);
    vector<int> rarm_idx;
    findJointAndInsert(tree.get(), "rightShoulderPitch", rarm_idx);
    findJointAndInsert(tree.get(), "rightShoulderRoll", rarm_idx);
    findJointAndInsert(tree.get(), "rightShoulderYaw", rarm_idx);
    findJointAndInsert(tree.get(), "rightElbowPitch", rarm_idx);
    findJointAndInsert(tree.get(), "rightForearmYaw", rarm_idx);
    findJointAndInsert(tree.get(), "rightWristRoll", rarm_idx);
    findJointAndInsert(tree.get(), "rightWristPitch", rarm_idx);
    cout << "Number of elements in rarm_idx " << rarm_idx.size() << endl;
    VectorXd rarm_lb = VectorXd::Zero(7);
    for(int i=0;i<7;i++)
        rarm_lb(i) = reach_start(rarm_idx[i]);
    VectorXd rarm_ub = rarm_lb;
    kc_posture_rarm.setJointLimits(7, rarm_idx.data(), rarm_lb, rarm_ub);

    // 8 quasistatic constraint
    QuasiStaticConstraint kc_quasi(tree.get(), tspan);
    kc_quasi.setShrinkFactor(0.2);
    kc_quasi.setActive(true);

    auto leftFootPtr = tree->FindBody("leftFoot");
    Matrix3Xd leftFootContactPts = leftFootPtr->get_contact_points();
    Matrix3Xd l_foot_pts = leftFootContactPts.rightCols(8);
    kc_quasi.addContact(1, &l_foot, &l_foot_pts);

    auto rightFootPtr = tree->FindBody("rightFoot");
    Matrix3Xd rightFootContactPts = rightFootPtr->get_contact_points();
    Matrix3Xd r_foot_pts = rightFootContactPts.rightCols(8);
    kc_quasi.addContact(1, &r_foot, &r_foot_pts);

    cout << "LeftFootContactPts:" << endl;
    cout << leftFootContactPts << endl;
    cout << "last 8 lfoot contact pts" << endl;
    cout << l_foot_pts << endl;


    //------------------solve-----------------------------------------------------
    std::vector<RigidBodyConstraint*> constraint_array;
    constraint_array.push_back(&kc_posture_neck);
    constraint_array.push_back(&kc_lfoot_pos);
    constraint_array.push_back(&kc_lfoot_quat);
    constraint_array.push_back(&kc_rfoot_pos);
    constraint_array.push_back(&kc_lfoot_quat);
    constraint_array.push_back(&kc_posture_torso);
    constraint_array.push_back(&kc_posture_knee);
    constraint_array.push_back(&kc_posture_larm);
    constraint_array.push_back(&kc_posture_rarm);
    constraint_array.push_back(&kc_quasi);

    IKoptions ikoptions(tree.get());
    VectorXd q_sol(tree->get_num_positions());
    int info;
    std::vector<std::string> infeasible_constraint;
    inverseKin(tree.get(), reach_start, reach_start, constraint_array.size(),
               constraint_array.data(), ikoptions,
               &q_sol, &info, &infeasible_constraint);
    printf("info = %d\n", info);

    /////////////////////////////////////////
    cache = tree->doKinematics(q_sol);
    Vector3d com = tree->centerOfMass(cache);
    printf("%5.6f\n%5.6f\n%5.6f\n", com(0), com(1), com(2));



    //show it in drake visualizer!
    /*
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) return 1;

    auto visualizer =
    std::make_shared<BotVisualizer<ValkyriePlant::StateVector>>(lcm, tree);
    auto sys_with_vis = cascade(val_sys, visualizer);

    drake::SimulationOptions options;
    options.initial_step_size = 5e-5;
    options.realtime_factor = 0.0;

    double final_time = std::numeric_limits<double>::infinity();
    runLCM(sys_with_vis, lcm, 0, final_time, val_sys->get_initial_state(), options);
    */


    return 0;
}



/* Finds and returns the indices within the state vector of @p tree that contain
 * the position states of a joint named @p name. The model instance ID is
 * ignored in this search (joints belonging to all model instances are
 * searched).
 */
std::vector<int> GetJointPositionVectorIndices(const RigidBodyTree* tree,
                                               const std::string& name) {
    RigidBody* joint_child_body = tree->FindChildBodyOfJoint(name);
    int num_positions = joint_child_body->getJoint().get_num_positions();
    std::vector<int> ret(static_cast<size_t>(num_positions));

    // Since the joint position states are located in a contiguous region of the
    // the rigid body tree's state vector, fill the return vector with
    // sequentially increasing indices starting at
    // `joint_child_body->get_position_start_index()`.
    iota(ret.begin(), ret.end(), joint_child_body->get_position_start_index());
    return ret;
}

void findJointAndInsert(const RigidBodyTree* model, const std::string& name,
                        std::vector<int>& position_list) {
    auto position_indices = GetJointPositionVectorIndices(model, name);

    position_list.insert(position_list.end(), position_indices.begin(),
                         position_indices.end());
}
