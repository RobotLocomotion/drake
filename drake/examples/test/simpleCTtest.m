function simpleCTtest

oldpath = addpath(fullfile(pwd,'..'));

try 

  sys = SimpleCTExample();

  x0 = 2*rand-1;  % unstable for |x|>1
  xtraj = simulate(sys,[0 10],x0);
  fnplt(xtraj);

 catch err
  path(oldpath);
  rethrow(err)
end

path(oldpath);
