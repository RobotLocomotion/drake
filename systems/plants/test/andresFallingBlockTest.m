function andresFallingBlockTest

% test script from Andres - the initial failure mode was that this passive
% block would swing back and forth like a pendulum when falling with
% gravity.

options = [];
options.floating = true;
r = PlanarRigidBodyManipulator('block_offset.urdf', options);
q = zeros(3,1);
qd = zeros(3,1);
[H,C] = manipulatorDynamics(r,q,qd);
qddot0 = inv(H)*C;
valuecheck(qddot0(3),0);  % there should be no rotational acceleration

v = r.constructVisualizer;

v.display_dt = .05;

x0 = Point(r.getStateFrame);

% Run simulation, then play it back at realtime speed
xtraj = simulate(r,[0 5],x0);

xf = xtraj.eval(5);
valuecheck(xf(3),0);  % the block should still be at zero orientation at the end.
