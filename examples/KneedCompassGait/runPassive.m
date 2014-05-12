options = [];
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
m = PlanarRigidBodyManipulator('KneedCompassGait.urdf', options);
r = TimeSteppingRigidBodyManipulator(m,.001);
%q = zeros(3,1);
%qd = zeros(3,1);
%[H,C,B,dH,dC,dB] = manipulatorDynamics(r,q,qd);
v = r.constructVisualizer;
v.axis = [-1.7 1.7 -0.1 1.1];

v.display_dt = .05;

x0 = Point(r.getStateFrame());
x0.base_z = 1.1;
x0.hip_pin = 0.2;

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 5],x0);
v.playback(xtraj);
