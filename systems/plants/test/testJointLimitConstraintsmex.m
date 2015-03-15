function testJointLimitConstraintsmex

%build an atlas model 
robot = createAtlas('rpy');
if robot.mex_model_ptr == 0
  disp('testJointLimitConstraintsmex: no mex model pointer... nothing to test');
  return;
end

%random initial pose
q = getRandomConfiguration(robot);

%get c++ jointLimitConstraints
[phi_mex, J_mex] = jointLimitConstraintsmex(robot.mex_model_ptr, q);


%get matlab jointLimitConstraints
[phi, J] = robot.jointLimitConstraints(q);

valuecheck(phi_mex, phi);
valuecheck(J_mex, J);

%also test the case where only some joints have limits
robot = RigidBodyManipulator(fullfile('../../../examples/Atlas/urdf/robotiq_tendons.urdf'));

x0 = robot.getInitialState;
q = x0(1:robot.num_positions);

%get c++ jointLimitConstraints
[phi_mex, J_mex] = jointLimitConstraintsmex(robot.mex_model_ptr, q);


%get matlab jointLimitConstraints
[phi, J] = robot.jointLimitConstraints(q);

valuecheck(phi_mex, phi);
valuecheck(J_mex, J);


end

