function c=runTransverseCycle

tmp = addpathTemporary({fullfile(pwd,'..'),fullfile(getDrakePath(),'systems','controllers','dev')});

[p,utraj,xtraj] = runDircolCycle;

p=setInputLimits(p,-inf,inf);

figure(1); clf; hold on;
fnplt(xtraj,[2 4]);
fnplt(xtraj,[3 5]);
  
w= randn(2,1);
Nsteps = 50;


Q=diag([10,5,1,.5]);  R=.1;  Qjump=zeros(4);
[ctrans, transSurf, V] = transverseLQR(p,w,xtraj,utraj,{Q,Q},{R,R},Q,{Qjump},false);

for i=1:length(transSurf)
  xt=xtraj.traj{i}.subTrajectory(2:5);
  plotSurface(transSurf{i},xt,.4,[1 3]); 
  plotSurface(transSurf{i},xt,.4,[2 4]); 
end


%% this stuff should be in a hybrid version of transverseLQRClosedLoop
for i=1:length(ctrans)
  xtraji=xtraj.traj{i};
  x0=xtraji.eval(xtraji.tspan(1)); mode=x0(1);
  xtraji=xtraji.subTrajectory(2:5);
  sys = feedback(p.modes{mode},ctrans{i});

  N = p.modes{mode}.getNumStates();
  m = ctrans{i}.getNumContStates();
  if (ctrans{i}.getNumDiscStates()~=0) error('not implemented yet'); end

  if (m>0)
    xtraji2 = MixedTrajectory({xtraji, PPTrajectory(foh(xtraji.tspan,xtraji.tspan))},{1:N,N+(1:m)}); % add controller state
    xtraji2 = setOutputFrame(xtraji2,sys.getStateFrame);
  end
  utraji = FunctionHandleTrajectory(@(t)zeros(0),[0 0],xtraji.tspan);  % utraj is now empty

  Pi = getPi(transSurf{i},xtraji.tspan(end));
  Vf = Pi*Q*Pi';

  if (0) % need to finish nursing this one back into working order
    disp('Estimating funnel..');
    psys = taylorApprox(sys,xtraji2,[],3,5);  %ignore var 5 (tau)
    options=struct();
    options.rho0_tau=8;
    V{i}=sampledTransverseVerification(psys,V{i}.eval(V{i}.tspan(end)),V{i},V{i}.getBreaks(),xtraji,utraji,transSurf{i},options);
    
    transSurf{i}.plotFunnel(V{i},xtraji,[1 3]);
  end
end

