function runLQRtest()
% run the lqr controller from a handful of initial conditions on the
% boundary of the estimated ROA and verify that it gets to the top

pd = PendulumPlant;
pd = pd.setInputLimits(-inf,inf);  % for now
[c,V] = balanceLQR(pd);

sys = feedback(pd,c);
n=10;
x0=[pi;0];
V = V.inFrame(pd.getStateFrame);
y=getLevelSet(V,[],struct('num_samples',n));
for i=1:n
  xtraj=simulate(sys,[0 4],.01*x0 + .99*y(:,i));
  if (V.eval(4,xtraj.eval(4))>V.eval(0,xtraj.eval(0)))
    xtraj.eval(4)-x0
    error('simulation appears to be going uphill on the Lyapunov function');
  end
end

end
