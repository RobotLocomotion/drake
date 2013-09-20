function paramExtractFromURDF

r1 = RigidBodyManipulator('AcrobotWParams.urdf');
r2 = RigidBodyManipulator('../Acrobot.urdf');

p = getParams(r2)

%q=sym('q',[2,1]); q=sym(q,'real');
%qdot=sym('q',[2,1]); qdot=sym(qdot,'real');
%H1 = manipulatorDynamics(r1,q,qdot)
%H2 = manipulatorDynamics(r2,q,qdot)

for i=1:20
  t = randn;
  x = randn(4,1);
  u = randn;
  valuecheck(dynamics(r1,t,x,u),dynamics(r2,t,x,u));
end

% NOTEST (not ready yet!)