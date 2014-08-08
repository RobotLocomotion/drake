function andresFallingBlockTest

% test script from Andres - the initial failure mode was that this passive
% block would swing back and forth like a pendulum when falling with
% gravity.

options = [];
options.floating = true; % 'quat'
r = PlanarRigidBodyManipulator('block_offset.urdf', options);
q = zeros(r.getNumPositions(),1);
v = zeros(r.getNumVelocities(),1);
[H,C] = manipulatorDynamics(r,q,v);
vd0 = inv(H)*C;
valuecheck(vd0(3),0);  % there should be no rotational acceleration

v = r.constructVisualizer;

v.display_dt = .05;

x0 = Point(r.getStateFrame);

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 5],x0);

xf = xtraj.eval(5);
valuecheck(xf(3),0);  % the block should still be at zero orientation at the end.
