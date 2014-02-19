function testMotionSubspaceDotV()

testRPYVersusSymbolic();
testQuatVersusSymbolic();

end

function testRPYVersusSymbolic()
syms x y z roll pitch yaw real;
syms xd yd zd rolld pitchd yawd real;

qBody = [x; y; z; roll; pitch; yaw];
vBody = [xd; yd; zd; rolld; pitchd; yawd];

body = RigidBody();
body.floating = 1;

S = motionSubspace(body, qBody);
SDot = ...
  diff(S, x) * xd + ...
  diff(S, y) * yd + ...
  diff(S, z) * zd + ...
  diff(S, roll) * rolld + ...
  diff(S, pitch) * pitchd + ...
  diff(S, yaw) * yawd;
SdotVCheck = SDot * vBody;

SdotV = motionSubspaceDotV(body, qBody, vBody);

valuecheck(simplify(SdotV - SdotVCheck), zeros(6, 1), 0);
end

function testQuatVersusSymbolic()
syms x y z q_s q_x q_y q_z real;
syms xd yd zd q_sd q_xd q_yd q_zd real;

qBody = [x; y; z; q_s; q_x; q_y; q_z];
vBody = [xd; yd; zd; q_sd; q_xd; q_yd; q_zd];

body = RigidBody();
body.floating = 2;

S = motionSubspace(body, qBody);

SDot = ...
  diff(S, x) * xd + ...
  diff(S, y) * yd + ...
  diff(S, z) * zd + ...
  diff(S, q_s) * q_sd + ...
  diff(S, q_x) * q_xd + ...
  diff(S, q_y) * q_yd + ...
  diff(S, q_z) * q_zd;
SdotVSym = SDot * vBody;


for i = 1 : 100
  quat = randn(4, 1);
  quat = quat / norm(quat);
  omega = randn(3, 1);
  quatd = 1/2 * quatmultiply([0; omega]', quat')';
  qBodyTest = [randn(3, 1); quat];
  vBodyTest = [randn(3, 1); quatd];

  SdotV = motionSubspaceDotV(body, qBodyTest, vBodyTest);
  SdotVCheck = double(subs(SdotVSym, num2cell([qBody; vBody]'), num2cell([qBodyTest; vBodyTest]')));
  valuecheck(SdotV, SdotVCheck, 1e-8);
end



end