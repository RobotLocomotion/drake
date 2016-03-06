function satFeedback
% test to make sure that the dynamics are the same if do the saturated
% feedback combination via simulink or via the feedback (polytopic) forms.

oldpath = addpath(fullfile(pwd,'..'));

try 
  pd = PendulumPlant;

  c=PendulumLQR(pd);
  sys1 = feedback(pd,c);
  sys2 = feedback(pd,c,struct('try_to_be_robotlibsystem',false));

  for i=1:1000
    t=rand(); x=randn(2,1); u=[];
    xdot1 = sys1.dynamics(t,x,u);
    xdot2 = sys2.dynamics(t,x,u);
    if (any(abs(xdot1-xdot2)>1e-6))
      i
      x
      xdot1
      xdot2
      error('saturated dynamics don''t match');
    end
  end
  
  % now test the taylor approximated version possibility

  x0=[pi;0];
  pp = pd.taylorApprox(0,x0,0,3);
  sys1 = feedback(pp,c);
  sys2 = feedback(pp,c,struct('try_to_be_robotlibsystem',false));
  
  for i=1:1000
    t=rand(); x=randn(2,1); u=[];
    xdot1 = sys1.dynamics(t,x,u);
    xdot2 = sys2.dynamics(t,x,u);
    if (any(abs(xdot1-xdot2)>1e-6))
      error('saturated dynamics don''t match');
    end
  end
  
catch ex
  path(oldpath);
  rethrow(ex);
end
path(oldpath);
