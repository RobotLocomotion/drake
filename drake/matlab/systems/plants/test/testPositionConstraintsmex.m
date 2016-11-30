function testPositionConstraintsmex(urdf)

%create a robot with a lot of positions constraints
if nargin<1 || isempty(urdf)
  urdf = fullfile('../../../../examples/Atlas/urdf/robotiq.urdf');
end
robot = RigidBodyManipulator(urdf);

if robot.mex_model_ptr == 0
  disp('No mex model pointer was found.  Aborting test')
  return
end

%random initial pose
q = getRandomConfiguration(robot);

%% test frame jacobians
kinsol_mat = robot.doKinematics(q,false,false);
kinsol_mex = robot.doKinematics(q,false,true);
options.in_terms_of_qdot = false;
for i = 1:numel(robot.loop)
%  loop = robot.loop(i)
%  options.base_or_frame_id = robot.loop(i).frameB;
%  robot.frame(-robot.loop(i).frameB)
%  robot.body(robot.frame(-robot.loop(i).frameB).body_ind)
%  robot.frame(-robot.loop(i).frameB).T
  [x_mat,J_mat] = robot.forwardKin(kinsol_mat,robot.loop(i).frameA,0*[1;0;0],options);
  [x_mex,J_mex] = robot.forwardKin(kinsol_mex,robot.loop(i).frameA,0*[1;0;0],options);
  valuecheck(x_mex,x_mat);
  valuecheck(J_mex,J_mat);
end

%% test position constraints

%matlab
[phi, J] = robot.positionConstraints(q);

% taylorvar
[phi_tv,J_tv]=geval(@robot.positionConstraints,q,struct('grad_method','taylorvar'));

%mex
[phi_mex, J_mex] = positionConstraintsmex(robot.mex_model_ptr, q); 

valuecheck(phi,phi_tv);
valuecheck(J,J_tv);


%compare
valuecheck(phi_mex, phi);
valuecheck(J_mex, J);
end

