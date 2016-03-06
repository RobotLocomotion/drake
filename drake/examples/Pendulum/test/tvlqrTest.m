function tvlqrTest

oldpath = addpath(fullfile(pwd,'..'));

try 
  p = PendulumPlant();

  [utraj,xtraj]=swingUpTrajectory(p);

  p = setInputLimits(p,-inf,inf);
  Q=diag([10,1]);R=.1;
  [c,V] = tvlqr(p,xtraj,utraj,Q,R,Q);

  p_feedback=feedback(p,c);
  simulate(p_feedback,[0 1],p_feedback.getInitialState);
catch ex
  path(oldpath);
  rethrow(ex);
end

path(oldpath);
