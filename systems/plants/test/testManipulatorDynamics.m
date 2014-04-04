function testManipulatorDynamics

testBrickQuaternion();
testActuatedPendulum();

end

function testBrickQuaternion()
options.floating = 'quat';
m = RigidBodyManipulator('FallingBrick.urdf',options);
nq = m.getNumPositions();
nv = m.getNumVelocities();
q = randn(nq, 1);
v = randn(nv, 1);
[H,C,B] = manipulatorDynamics(m,q,v,false);

valuecheck(H, m.body(2).I, 1e-12);
%TODO: check C
valuecheck([nv 0], size(B));
end

function testActuatedPendulum()
m = RigidBodyManipulator('ActuatedPendulum.urdf');
nq = m.getNumPositions();
nv = m.getNumVelocities();
q = randn(nq, 1);
v = randn(nv, 1);
[H,C,B] = manipulatorDynamics(m,q,v,false);

body = m.body(2);
axis = body.joint_axis;
I = body.I;
H_expected = axis' * I(1:3, 1:3) * axis;
valuecheck(H_expected, H, 1e-12);
mass = m.body(2).mass;
gravity = m.gravity(3);
% valuecheck(C, mass * gravity * cos(q));
valuecheck(1, B);
end