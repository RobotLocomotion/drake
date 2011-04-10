function c=TrajectorySwingup

p = PendulumPlant;
p = p.setInputLimits(-inf,inf);  % note: take out artificial pruning below if I remove this

disp('constructing LQR balance controller');
[ti,Vf] = PendulumLQR(p);

disp('estimating ROA for balance controller');
sys = feedback(p,ti);
psys = taylorApprox(sys,0,ti.x0,[],3);
Vf = regionOfAttraction(psys,[],Vf,struct('monom_order',3));

Vf = Vf/5;  % artificially prune, since ROA is solved without input limits

x=msspoly('x',2); Vf=x'*diag([10,1])*x;  % for debugging

% create LQR Tree
c = LQRTree(ti,Vf);

disp('construct swingup trajectory');
[utraj,xtraj]=runDircol(p);

disp('stabilizing swingup trajectory');
Q = diag([10 1]);  R=1;
[tv,sys,xtraj,utraj,Vtraj,Vf] = tvlqrClosedLoop(p,xtraj,utraj,Q,R,Vf);

disp('estimating swingup funnel');
psys = taylorApprox(sys,xtraj,[],3);


V=sampledFiniteTimeInvariance(psys,Vf,Vtraj,xtraj.getBreaks());

% add swingup controller to the tree
c = c.addTrajectory(tv,V);

disp('done');


if (nargout<1)
  sys = feedback(p,c);
  pv = PendulumVisualizer;

  for i=1:5
    xtraj = simulate(sys,[0 6],zeros(3,1));
    playback(pv,xtraj);
  end
end
  
