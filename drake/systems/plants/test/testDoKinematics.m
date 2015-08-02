function testDoKinematics

testAtlas();

testFloatingObject();
end


function testAtlas()

rng(23415, 'twister');
robot = createAtlas('rpy');
robot_new_kinsol = setNewKinsolFlag(robot,true);

nq = robot.getNumStates() / 2;
nv = robot.getNumStates() / 2;

nTests = 5;
for i = 1 : nTests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, true, false, v);
  kinsol_new = robot_new_kinsol.doKinematics(q,true,false,v);
  
  for index = 1 : length(robot.body)
    valuecheck(kinsol.T{index},kinsol_new.T{index});
%    valuecheck(kinsol.Tdot{index},kinsol_new.Tdot{index});
%    valuecheck(kinsol.dTdq{index},kinsol_new.dTdq{index});
%    valuecheck(kinsol.dTdqdot{index},kinsol_new.dTdqdot{index});

% todo: fill these in with Jdotv, etc.
  end
end

rng('default');
end

function out = varname(~)
  out = inputname(1);
end

function testFloatingObject()
robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
robot = robot.addRobotFromURDF([getDrakePath,'/solvers/trajectoryOptimization/dev/block.urdf'],[],[],struct('floating',true));
nq = robot.getNumPositions();
q = randn(nq,1);
kinsol = robot.doKinematics(q,false,false);
end