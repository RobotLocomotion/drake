function tvlqrTest

oldpath = addpath('..');

try 
  p = PendulumPlant();
  p = setInputLimits(p,-inf,inf);

  % OKTOFAIL
  [utraj,xtraj]=runDircol(p);

  Q=diag([10,1]);R=.1;
  [c,V] = tvlqr(p,xtraj,utraj,Q,R,Q);

  p_feedback=feedback(p,c);
  simulate(p_feedback,[0 1],p_feedback.getInitialState);
catch ex
  path(oldpath);
  rethrow(ex);
end

path(oldpath);
