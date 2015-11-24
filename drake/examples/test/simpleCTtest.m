function simpleCTtest

tmppath = addpathTemporary(fullfile(pwd,'..'));

sys = SimpleCTExample();

x0 = 2*rand-1;  % unstable for |x|>1
xtraj = simulate(sys,[0 10],x0);
fnplt(xtraj);

