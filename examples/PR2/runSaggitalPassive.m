function runSaggitalPassive

options.ignore_self_collisions = true;
options.floating = true;
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();

w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:BodyHasZeroInertia');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyGeometry:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
r = TimeSteppingRigidBodyManipulator('pr2.urdf',0.01,options);
warning(w);
r = setSimulinkParam(r,'MinStep','0.001');
v = r.constructVisualizer;
v.display_dt = .05;
v.axis = [-1.5,1.5,-1,2];

x0 = Point(r.getStateFrame);
v.draw(0,double(x0));

if (0)
  % Run animation while it is simulating (as fast as possible, but probably
  % slower than realtime)
  s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
  sys = cascade(r,v);
  warning(s);
  simulate(sys,[0 10],x0);
else
  % Run simulation, then play it back at realtime speed
  xtraj = simulate(r,[0 5],x0);
  v.playback(xtraj);
end
