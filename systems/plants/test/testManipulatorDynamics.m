function testManipulatorDynamics

testBrickQuaternion();
testActuatedPendulum();
regressionTestAtlasRPY();

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

function regressionTestAtlasRPY()
replaceMatFile = false;

rng(23415, 'twister');

r = createAtlas('rpy');
nq = r.getNumPositions();
nv = r.getNumVelocities();

nTests = 5;
Hs = cell(nTests, 1);
Cs = cell(nTests, 1);

for i = 1 : nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  [H, C, B] = manipulatorDynamics(r, q, v, false);
  Hs{i} = H;
  Cs{i} = C;
end

filename = 'regressionTestAtlasManipulatorDynamics.mat';
if replaceMatFile
  save(filename, varname(Hs), varname(Cs), varname(B));
else
  data = load(filename);
  for i = 1 : nTests
    valuecheck(data.Hs{i}, Hs{i}, 1e-10);
    valuecheck(data.Cs{i}, Cs{i}, 1e-10);
  end
  valuecheck(data.B, B);
end

end

function out = varname(~)
out = inputname(1);
end