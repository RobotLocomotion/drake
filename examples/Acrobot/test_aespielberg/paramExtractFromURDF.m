function paramExtractFromURDF

oldpath = addpath(fullfile(pwd,'..'));

r1 = RigidBodyManipulator('AcrobotWParams.urdf');
r2 = AcrobotPlant;
addProjectionTransformByCoordinateNames(getParamFrame(r1),getParamFrame(r2));

p1 = getParams(r1);
p2 = getParams(r2);

valuecheck(p1,p2);
p1.b1 = .5; p1.Ic2 = 1; 
p2.b1 = .5; p2.Ic2 = 1;

r1 = setParams(r1,p1);
r2 = setParams(r2,p2);

valuecheck(getParams(r1),getParams(r2));

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


path(oldpath);

