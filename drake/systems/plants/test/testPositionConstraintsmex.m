function testPositionConstraintsmex(urdf)

%create a robot with a lot of positions constraints
if nargin<1 || isempty(urdf)
  urdf = fullfile('../../../examples/Atlas/urdf/robotiq.urdf');
end
robot = RigidBodyManipulator(urdf);

if robot.mex_model_ptr == 0 || robot_new.mex_model_ptr == 0 
  disp('No mex model pointer was found.  Aborting test')
  return
end

%random initial pose
q = getRandomConfiguration(robot);

robot.doKinematics(q);

%mex
[phi_mex, J_mex] = positionConstraintsmex(robot.mex_model_ptr, q); 

%matlab
[phi, J] = robot.positionConstraints(q);

%compare
valuecheck(phi_mex, phi);
valuecheck(J_mex, J);
end

