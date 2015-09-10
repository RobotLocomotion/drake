function testSimpleDTExample

sys = SimpleDTExample;
traj = simulate(sys, [0 10], .99);
fnplt(traj);

