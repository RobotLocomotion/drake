function lqrTest()

%% run the lqr controller from a handful of initial conditions on the
%% boundary of the estimated ROA and verify that it gets to the top

oldpath = addpath(fullfile(pwd,'..'));

p = AcrobotPlant;
p = p.setInputLimits(-inf,inf);  % for now
[c,V] = balanceLQR(p);
sys = feedback(p,c);
n=10;
x0=[pi;0;0;0];
V = V.inFrame(p.getStateFrame);
y=getLevelSet(V,0,struct('num_samples',n));
for i=1:n
  xtraj=simulate(sys,[0 4],.01*x0 + .99*y(:,i));
  if (V.eval(4,xtraj.eval(4))>V.eval(0,xtraj.eval(0)))
    path(oldpath);
    error('simulation appears to be going uphill on the Lyapunov function');
  end
end

path(oldpath);

end
