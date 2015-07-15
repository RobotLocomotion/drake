function testManipulatorDynamics
testActuatedPendulum();
testAtlasRPY();
checkGradients(createAtlas('rpy'));
checkHdotMinus2CoriolisMatrixSkewSymmetricMatrix(createAtlas('rpy'))

testBrickQuaternion();
testAtlasQuat();
checkGradients(createFallingBrick('quat'));
checkGradients(createAtlas('quat'));

checkMex(RigidBodyManipulator('MassSpringDamper.urdf'));
% checkGradients(RigidBodyManipulator('MassSpringDamper.urdf')); currently causes a Matlab crash due to https://github.com/RobotLocomotion/drake/issues/997
end

function robot = createFallingBrick(floating_type)
options.floating = floating_type;
robot = RigidBodyManipulator('FallingBrick.urdf',options);
end

function testBrickQuaternion()
options.floating = 'quat';
r = RigidBodyManipulator('FallingBrick.urdf',options);

nv = r.getNumVelocities();
q = getRandomConfiguration(r);
v = randn(nv, 1);
[H,C,B] = manipulatorDynamics(r,q,v,false);

valuecheck(H, r.body(2).I, 1e-12);
% C: Newton-Euler equations:
twist_omega = v(1:3);
twist_v = v(4:6);
m = r.body(2).mass;
I = r.body(2).inertia;
gravity_body = quatRotateVec(quatConjugate(q(4:7)), r.gravity);
C_check = [...
  cross(twist_omega, I * twist_omega);
  cross(twist_omega, m * twist_v) - m * gravity_body];
valuecheck(C, C_check);
valuecheck([nv 0], size(B));

checkMex(r);
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
% TODO: check C
valuecheck(1, B);

checkMex(m);
end

function testAtlasRPY()

r = createAtlas('rpy');
r_newkinsol = setNewKinsolFlag(r,true);
nq = r.getNumPositions();
nv = r.getNumVelocities();

nTests = 5;
for i = 1 : nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  [H, C, B] = manipulatorDynamics(r, q, v, false);
  [H_new, C_new, B_new] = manipulatorDynamics(r_newkinsol, q, v, false);

  valuecheck(H_new,H);
  valuecheck(C_new,C);
  valuecheck(B_new,B);
end

checkMex(r);

end

function testAtlasQuat()
r = createAtlas('quat');
nv = r.getNumVelocities();
checkMex(r);

nTests = 5;
for i = 1 : nTests
  q = getRandomConfiguration(r);
  v = randn(nv, 1);
  [H, C, B] = manipulatorDynamics(r, q, v, false);
  kinetic_energy = 1/2 * v' * H * v;

  kinsol = r.doKinematics(q, false, false, v);
  kinetic_energy_via_kinsol = computeKineticEnergy(r, kinsol);
  valuecheck(kinetic_energy_via_kinsol, kinetic_energy, 1e-10);
end
end

function out = varname(~)
out = inputname(1);
end

function ret = computeKineticEnergy(manipulator, kinsol)
NB = manipulator.getNumBodies();
ret = 0;

for i = 2 : NB
  twistInBody = relativeTwist(kinsol, 1, i, i);
  I = manipulator.body(i).I;
  ret = ret + 1/2 * twistInBody' * I * twistInBody;
end
end

function checkGradients(robot)
q = getRandomConfiguration(robot);
v = randn(robot.getNumVelocities(), 1);
[~,~,~,dH,dC,dB] = manipulatorDynamics(robot, q, v, false);

option.grad_method = 'taylorvar';
[~, ~, ~,dH_geval, dC_geval, dB_geval] = geval(3, @(q, v) manipulatorDynamics(robot, q, v, false), q, v, option);

valuecheck(dH_geval, dH, 1e-10);
valuecheck(dC_geval, dC, 1e-10);
valuecheck(dB_geval, dB, 1e-10);
end

function checkMex(robot)
q = getRandomConfiguration(robot);
v = randn(robot.getNumVelocities(), 1);
[H, C, B, dH, dC, dB] = manipulatorDynamics(robot, q, v, false);
[H_mex, C_mex, B_mex, dH_mex, dC_mex, dB_mex] = manipulatorDynamics(robot, q, v, true);

valuecheck(H_mex, H);
valuecheck(C_mex, C);
valuecheck(B_mex, B);
valuecheck(dH_mex, dH);
valuecheck(dC_mex, dC);
valuecheck(dB_mex, dB);
end

function checkHdotMinus2CoriolisMatrixSkewSymmetricMatrix(robot)
% Checks that \dot{H} - 2Q(q, v) is skew symmetric, where Q(q, v) is the
% Coriolis matrix. See Lemma 4.2 in Murray, Li, Sastry - A Mathematical
% Introduction to Robotic Manipulation.
% Note that CBar(q, v) = C(q, v) - friction(v) is quadratic in v, i.e. we
% can write the ith entry of Cbar as
% Cbar_i(q, v) = v' * A_i(q) v + N_i(q)
% so dCbar_i(q, v)/dv = 2 * v' * A_i(q)
% We can also write CBar(q, v) as
% CBar_i(q, v) = Q_(i,:)(q, v) * v + N_i(q)
% which shows that Q_(i,:)(q, v) = 1/2 * dCbar_i(q, v)/dv

nq = robot.getNumPositions();
nv = robot.getNumVelocities();
q = getRandomConfiguration(robot);
v = randn(nv, 1);

vToqdot = robot.vToqdot(q);
if nq ~= nv || any(any(vToqdot - eye(nv)))
  error('Hdot - 2Q is only skew symmetric if v = qdot')
end

[~,~,~,dH,dC] = manipulatorDynamics(robot, q, v, false);
qdot = vToqdot * v;
dHdq = dH(:, 1:nq);
Hdot = reshape(dHdq * qdot, [nv, nv]);
[~, dfrictiondv] = robot.computeFrictionForce(v);
dCdv = dC(:, nq + (1:nv)) - dfrictiondv;
coriolis_matrix = 1/2 * dCdv;

Hdot_minus_2_coriolis_matrix = Hdot - 2 * coriolis_matrix;
valuecheck(zeros(nv, nv), Hdot_minus_2_coriolis_matrix + Hdot_minus_2_coriolis_matrix', 1e-10);
end
