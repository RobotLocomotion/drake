function testFrictionModel()
% runs the passive system
r = RigidBodyManipulator('PendulumWithFriction.urdf');
%v = constructVisualizer(r);

traj = simulate(r,[0 5],[pi/2; 0]);
%playback(v,traj,struct('slider',true));

xf = traj.eval(traj.tspan(end));
if xf(1) > pi/2 || xf(1) < pi/2-0.15
  error('Friction simulation not working.');
end
end

