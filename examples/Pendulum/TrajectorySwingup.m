function c=TrajectorySwingup

p = PendulumPlant;
p = p.setInputLimits(-inf,inf);  % note: take out artificial pruning below if I remove this

disp('constructing LQR balance controller');
[ti,Vf] = PendulumLQR(p);

disp('estimating ROA for balance controller');
sys = feedback(p,ti);
psys = taylorApprox(sys,0,ti.x0,[],3);
Vf = regionOfAttraction(psys,ti.x0,Vf,struct('monom_order',3));

Vf = Vf*5;  % artificially prune, since ROA is solved without input limits

figure(1); clf; hold on;
plotFunnel(Vf,ti.x0); drawnow;


%x=msspoly('x',2); Vf=x'*diag([10,1])*x;  % for debugging

% create LQR Tree
c = LQRTree(ti,Vf);

disp('construct swingup trajectory');
[utraj,xtraj]=runDircol(p);

figure(1); 
fnplt(xtraj); drawnow;


disp('stabilizing swingup trajectory');
Q = diag([10 1]);  R=1;
[tv,sys,xtraj2,utraj2,Vtraj,Vf] = tvlqrClosedLoop(p,xtraj,utraj,Q,R,Vf);

disp('estimating swingup funnel');
psys = taylorApprox(sys,xtraj2,[],3);

options.monom_order=2;
V=sampledFiniteTimeVerification(psys,Vf,Vtraj,xtraj2.getBreaks(),xtraj2,utraj2,options);

figure(1); 
plotFunnel(V,xtraj); drawnow;


% add swingup controller to the tree
c = c.addTrajectory(tv,V);

disp('done');


if (nargout<1)
  sys = feedback(p,c);
  pv = PendulumVisualizer;

  for i=1:5
    xtraj = simulate(sys,[0 6]);
    playback(pv,xtraj);
  end
end
  
