function lqrTest()

%% run the lqr controller from a handful of initial conditions on the
%% boundary of the estimated ROA and verify that it gets to the top

oldpath = path;
addpath('..');

p = AcrobotPlant;
p = p.setInputLimits(-inf,inf);  % for now
[c,V] = balanceLQR(p);
sys = feedback(p,c);
n=10;
x0=[pi;0;0;0];
y=getLevelSet(V,x0,struct('num_samples',n));
for i=1:n
  xtraj=simulate(sys,[0 4],x0 + .99*y(:,i));
  if (norm(xtraj.eval(4)-x0)>1e-2)
    path(oldpath);
    error('initial condition from verified ROA didn''t get to the top (in 4 seconds)');
  end
end

path(oldpath);

end
