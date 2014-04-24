function testMotionSubspaceDotV()

testRPYVersusSymbolic();

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