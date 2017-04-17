function testSimpleCTExample

sys = SimpleCTExample;
traj = simulate(sys, [0 10], .99);
fnplt(traj);

