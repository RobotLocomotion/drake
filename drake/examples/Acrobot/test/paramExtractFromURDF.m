function paramExtractFromURDF

tmp = addpathTemporary(fullfile(pwd,'..'));

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


% confirm that Ttree gets updated
kinsol = doKinematics(r1,zeros(2,1),zeros(2,1));
hand = forwardKin(r1,kinsol,3,zeros(3,1));
valuecheck(hand(3),-p1.l1);
p1.l1 = .8;
r1 = setParams(r1,p1);
kinsol = doKinematics(r1,zeros(2,1),zeros(2,1));
hand = forwardKin(r1,kinsol,3,zeros(3,1));
valuecheck(hand(3),-p1.l1);


% test planar visualizer
r = PlanarRigidBodyManipulator('AcrobotWParams.urdf');
p = getParams(r);
v = r.constructVisualizer();
v.xlim = [-4;4];
v.ylim = [-4;4];

for l1=.8:.05:1.5
  p.l1 = l1;
  r = setParams(r,p);
  v = updateManipulator(v,r);
  drawWrapper(v,0,[.3;.5]);
end


% test 3d visualizer
r = RigidBodyManipulator('AcrobotWParams.urdf');
p = getParams(r);
try 
  v = BotVisualizer(r);
catch ex
  if strcmp(ex.identifier,'Drake:MissingDependency:BotVisualizerDisabled')
    warning(ex.identifier,ex.message);
    return;  % ok to skip this part of the test test
  else
    rethrow(ex);
  end
end

for l1=.8:.05:1.5
  p.l1 = l1;
  r = setParams(r,p);
  v = updateManipulator(v,r);
  drawWrapper(v,0,[.3;.5]);
end

