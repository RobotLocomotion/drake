function testDoKinematics

testTwistsVersusTdotAtlas('rpy');

end

function testTwistsVersusTdotAtlas(floatingJointType)
options.floating = floatingJointType;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
robot = RigidBodyManipulator(fullfile('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'),options);
warning(w);

% robot = RigidBodyManipulator(fullfile('../../../examples/FurutaPendulum/FurutaPendulum.urdf'));

nq = robot.getNumStates() / 2; % TODO
nv = robot.getNumStates() / 2; % TODO

nBodies = length(robot.body);

nTests = 50;
for j = 1 : nTests
  q = zeros(nq, 1); %randn(nq, 1);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, false, false, v);
  
  for i = 1 : nBodies
    TdotFromTwist = kinsol.T{i} * twistToTildeForm(kinsol.twist{i});
    valuecheck(TdotFromTwist, kinsol.Tdot{i}, 1e-12);
  end
end

end