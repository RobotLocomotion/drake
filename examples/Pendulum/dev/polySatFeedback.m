function polySatFeedback
% test to make sure that the dynamics are the same if I taylor approx then
% to LQR or feedback with LQR then do taylorApprox.  Sanity check for the
% feedback codes (especially in PolynomialSystem).

oldpath = path();
addpath(fullfile(pwd,'..'));

try 
  pd = PendulumPlant;

  x0=[pi;0];
  pp = pd.taylorApprox(0,x0,0,3);
  sys1 = feedback(pp,PendulumLQR(pp));
  sys2 = feedback(pd,PendulumLQR(pd));
  
  if (any(any(abs(sys1.A - sys2.A)>1e-6)) || any(abs(sys1.b - sys2.b)>1e-6))
    error('saturated dynamics don''t match'); 
  end
  
  % now test the other feedback possibility
  sys1 = feedback(PendulumLQR(pp),pp);
  sys2 = feedback(PendulumLQR(pd),pd);
  
  if (any(any(abs(sys1.A - sys2.A)>1e-6)) || any(abs(sys1.b - sys2.b)>1e-6))
    error('saturated dynamics don''t match'); 
  end
  
catch ex
  path(oldpath);
  rethrow(ex);
end
path(oldpath);
