function testPositionConstraintsmex

%create a robot with positions constraints
urdf = fullfile('../../../examples/Atlas/urdf/robotiq.urdf');
%urdf = fullfile('../../../examples/SimpleFourBar/FourBar.urdf');
robot = RigidBodyManipulator(urdf);
robot_new = RigidBodyManipulator(urdf, struct('use_new_kinsol', true));

%random initial pose
q = getRandomConfiguration(robot);

robot.doKinematics(q);
robot_new.doKinematics(q);

%mex, old
[phi_mex, J_mex] = positionConstraintsmex(robot.mex_model_ptr, q); 
%mex, new
[phi_mex_new, J_mex_new] = positionConstraintsmex(robot_new.mex_model_ptr, q); 

%matlab, old
[phi, J] = robot.positionConstraints(q);
%matlab, new
[phi_new, J_new] = robot_new.positionConstraints(q);

%make sure every pair of implementations matches
valuecheck(phi_mex, phi_mex_new);
valuecheck(phi_mex, phi);
valuecheck(phi_mex, phi_new);
valuecheck(phi_mex_new, phi);
valuecheck(phi_mex_new, phi_new);
valuecheck(phi, phi_new);
valuecheck(J_mex, J_mex_new);
valuecheck(J_mex, J);
valuecheck(J_mex, J_new);
valuecheck(J_mex_new, J);
valuecheck(J_mex_new, J_new);
valuecheck(J, J_new);
end

