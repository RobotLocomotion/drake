function testIKABB
% test ik on ABB
checkDependency('spotless');
p = RigidBodyManipulator([getDrakePath,'/examples/IRB140/urdf/irb_140_robotiq_simple_ati.urdf'],struct('floating',false));
q0 = 0.2*randn(p.getNumPositions(),1);
q0(1:6) = 0;
kinsol0 = p.doKinematics(q0,0*q0,struct('use_mex',false));

link_6 = p.findLinkId('link_6');
link_6_pt = [0;0;0];
link_6_pos0 = p.forwardKin(kinsol0,link_6,link_6_pt);
hand_tip1 = p.findLinkId('finger_1_link_3');
hand_tip1_pt = [0;0;0];
base = p.findLinkId('base_link');

ik = InverseKinematicsBMI(p);
% ik = ik.fixLinkPosture(base,kinsol0.T{base}(1:3,4),rotmat2quat(kinsol0.T{base}(1:3,1:3)));
ik = ik.addPositionConstraint(link_6,link_6_pt,eye(3),link_6_pos0,[],[]);
solver_sol = ik.optimize();
[sol,sol_bilinear] = ik.retrieveSolution(solver_sol);
[joint_lb,joint_ub] = p.getJointLimits();
if(any(sol.q>joint_ub+1e-3) || any(sol.q<joint_lb-1e-3))
	error('The joint limit is not satisfied');
end
kinsol = p.doKinematics(sol.q);
if(norm(p.forwardKin(kinsol,link_6,link_6_pt)-link_6_pos0)>1e-3)
  error('The end effector is not at the desired position');
end
ik.robot_visualizer.draw(0,sol.q);
end
