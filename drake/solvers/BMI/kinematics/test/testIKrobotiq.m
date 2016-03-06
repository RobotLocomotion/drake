function testIKrobotiq()
% This function checks the closed-loop kinematics chain
checkDependency('spotless');
hand = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/robotiq.urdf'],struct('floating',true));
finger_1_link_3 = hand.findLinkId('finger_1_link_3');
finger_2_link_3 = hand.findLinkId('finger_2_link_3');
finger_3_link_3 = hand.findLinkId('finger_middle_link_3');
finger_1_link_3_pts = hand.getBody(finger_1_link_3).getTerrainContactPoints();
finger_2_link_3_pts = hand.getBody(finger_2_link_3).getTerrainContactPoints();
finger_3_link_3_pts = hand.getBody(finger_3_link_3).getTerrainContactPoints();
p = InverseKinematicsBMI(hand);
p = p.addPositionConstraint(finger_1_link_3,finger_1_link_3_pts(:,1),[0 0 1],0,[],[]);
p = p.addPositionConstraint(finger_2_link_3,finger_2_link_3_pts(:,1),[0 0 1],0,[],[]);
p = p.addPositionConstraint(finger_3_link_3,finger_3_link_3_pts(:,1),[0 0 1],0,[],[]);
solver_sol = p.optimize();
[sol,sol_bilinear] = p.retrieveSolution(solver_sol);
p.plotSolution(sol,sol_bilinear);
[joint_lb,joint_ub] = hand.getJointLimits();
if(any(sol.q>joint_ub+1e-3) || any(sol.q<joint_lb-1e-3))
  error('The joint limits is not satisfied');
end
kinsol = hand.doKinematics(sol.q,false,false);
for i = 2:hand.getNumBodies()
  if(norm(kinsol.T{i} - [quat2rotmat(sol.body_quat(:,i)) sol.body_pos(:,i);0 0 0 1]) > 1e-3)
    error('The kinsol does not match with the body quat computed using BMI');
  end
end
finger_1_link_3_pt_pos = hand.forwardKin(kinsol,finger_1_link_3,finger_1_link_3_pts(:,1));
finger_2_link_3_pt_pos = hand.forwardKin(kinsol,finger_2_link_3,finger_2_link_3_pts(:,1));
finger_3_link_3_pt_pos = hand.forwardKin(kinsol,finger_3_link_3,finger_3_link_3_pts(:,1));
if(abs(finger_1_link_3_pt_pos(3))>1e-2 || abs(finger_2_link_3_pt_pos(3))>1e-2 || abs(finger_3_link_3_pt_pos(3))>1e-2)
  error('The kinematics constraints are not satisfied');
end
for i = 1:length(hand.loop)
  frameA_origin = hand.forwardKin(kinsol,hand.loop(i).frameA,zeros(3,1));
  frameB_origin = hand.forwardKin(kinsol,hand.loop(i).frameB,zeros(3,1));
  if(norm(frameA_origin - frameB_origin) > 1e-2)
    error('In the loop constraint, the origins in the two frames do not coincide');
  end
  frameA_axis = hand.forwardKin(kinsol,hand.loop(i).frameA,hand.loop(i).axis);
  frameB_axis = hand.forwardKin(kinsol,hand.loop(i).frameB,hand.loop(i).axis);
  if(norm(frameA_axis - frameB_axis) > 1e-2)
    error('In the loop constraint, the axes in the two frames do not coincide');
  end
end
end
