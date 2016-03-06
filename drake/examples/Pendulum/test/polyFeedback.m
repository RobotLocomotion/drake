function polyFeedback
% test to make sure that the dynamics are the same if I taylor approx then
% to LQR or feedback with LQR then do taylorApprox.  Sanity check for the
% feedback codes (especially in PolynomialSystem).

oldpath = addpath(fullfile(pwd,'..'));

try 
  pd = PendulumPlant;
  pd = pd.setInputLimits(-inf,inf);  
  c = pd.balanceLQR;

  x0=Point(pd.getStateFrame,[pi;0]);
  pp = pd.taylorApprox(0,x0,[],3);
  sys1 = feedback(pp,c); 
  sys2 = taylorApprox(feedback(pd,c),0,x0,[],3);
  
  for i=1:100
    t=rand(1); x=2*pi*rand(2,1); u=randn(1);  % only matches for thetas that don't wrap
    valuecheck(sys1.dynamics(t,x,u),sys2.dynamics(t,x,u));
    valuecheck(sys1.output(t,x,u),sys2.output(t,x,u));
  end
  
catch ex
  path(oldpath);
  rethrow(ex);
end
path(oldpath);
