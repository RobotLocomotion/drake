function testJointLimits()
% Simulate the passive acrobot

%r = PlanarRigidBodyManipulator('../Acrobot.urdf');
%r = HybridPlanarRigidBodyManipulator('../Acrobot.urdf');
m = PlanarRigidBodyManipulator('../Acrobot.urdf');
b = getBody(m,3);
b.joint_limit_min=-1.5;
b.joint_limit_max=1.5;
m = setBody(m,3,b);
m = compile(m);
r = TimeSteppingRigidBodyManipulator(m,.01);
v = r.constructVisualizer;

%traj = simulate(r,[0 5],[1;pi/4;pi/6;0;4]);
traj = simulate(r,[0 5],[pi/4;pi/6;0;7]);
playback(v,traj);

x=traj.eval(traj.getBreaks());
if min(x(2,:))<m.body(3).joint_limit_min || max(x(2,:))>m.body(3).joint_limit_max
  error('joint limits violated');
end

end
