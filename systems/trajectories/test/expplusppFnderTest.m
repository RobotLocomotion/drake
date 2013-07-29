function expplusppFnderTest

for i=1:20
  nbreaks = 10;
  T = 20;
  dim = 1+randi(10);
  pporder = randi(4);
  breaks = linspace(0,10,nbreaks);
  A = -0.01*rand(5);
  K = 0.1*randn(dim,5);
  alpha = 0.5*randn(5,nbreaks-1);
  gamma = 0.1*randn(dim,nbreaks-1,pporder);
	traj = ExpPlusPPTrajectory(breaks,K,A,alpha,gamma);
	dtraj = fnder(traj);
	dtraj2 = fnder(traj,2);
 	dtraj3 = fnder(traj,3);
  
  for t=linspace(0,T,13)
    der = dtraj.eval(t);
    der2 = dtraj2.eval(t);
    der3 = dtraj3.eval(t);
    [~,nder] = geval(@traj.eval,t,struct('grad_method','numerical'));
    [~,nder2] = geval(@dtraj.eval,t,struct('grad_method','numerical'));
    [~,nder3] = geval(@dtraj2.eval,t,struct('grad_method','numerical'));
    
    valuecheck(der,nder,1e-5);
    valuecheck(der2,nder2,1e-5);
    valuecheck(der3,nder3,1e-5);
  end
  
end
end
