function trajOptTest()
% Test optimal stapilizer based on SNOPT with the one 
% analytically derived

oldpath = addpath(fullfile(pwd,'..'));

x0 = [1 1]';
xf = [0 0]';
tf = 3;
dt = .1;

[xtraj,utraj,exitflag] = runTrajOpt(x0,xf,tf,dt);

if exitflag~=1
  path(oldpath);
  error('SNOPT did not find a solution!');
end

path(oldpath);

end