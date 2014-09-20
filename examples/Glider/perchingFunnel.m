function PerchingFunnel

close all
megaclear;

p = GliderPlant;
%p = p.setInputLimits(-inf,inf);

%create mss state variables
x=msspoly('x',7);

% obtain nomial trajectories via dircol with end constraints
try
  trajs=load('glider_trajs.mat');

  xtraj=trajs.xtraj;
  utraj=trajs.utraj;

  utraj = setOutputFrame(utraj,p.getInputFrame);
  xtraj = setOutputFrame(xtraj,p.getStateFrame);
catch
  [utraj,xtraj]=runDircolPerching(p);
end

% plot the trajectory positions
figure(1);
fnplt(xtraj); drawnow;

% plot the trajectory velocities
figure(2);
fnplt(xtraj,[5,6]); drawnow;

%xG=[0 0 1.57 -0.5 1 -2 -1]';
xG=xtraj.eval(xtraj.tspan(end));
% create the final time goal ellipse for the funnel
Vf=(x-xG)'*diag([(1/0.05)^2 (1/0.05)^2 (1/3)^2 (1/3)^2 1 1 (1/3)^2])*(x-xG);
Qf=diag([(1/0.05)^2 (1/0.05)^2 (1/3)^2 (1/3)^2 1 1 (1/3)^2]);
p = p.setInputLimits(-inf,inf);

% stabilize the open loop trajectory with LQR
disp('stabilizing swingup trajectory');
Q = diag([10 10 10 1 1 1 1]);  R=0.1; % LQR Cost Matrices
options.sqrt_method=0; % do not use square root method
options.affine_dynamics=0; % do not use affine dynamics

%create closed loop system using LQR
%[tv,sys,xtraj2,utraj2,Vtraj,Vf] = tvlqrClosedLoop(p,xtraj,utraj,Q,R,Vf,@ode15s,options);

[tv,Vtraj] = tvlqr(p,xtraj,utraj,Q,R,Qf);

disp('create polynomial system');
psys = taylorApprox(feedback(p,tv),xtraj,[],3); %generate 3rd tayalor approximation of feedback system

% begin funnel growing process
disp('estimating swingup funnel');
options.rho0_tau=15.5; %initial exponential coefficient
options.rho0_tau_alpha=1.0;
options.degL1=2; %order of the lagrange multiplier
options.max_iterations=2;
options.lyap_parameterization='rhoS';
options.polysys_saved=0;
% begin finite time verification process
V0=sampledFiniteTimeVerification(psys,xtraj.getBreaks(),Qf,Vtraj,options);

% plot glider funnels

V=V0.inFrame(getStateFrame(p));

save('Funnel_Rho2','xtraj','V','Vf')

optionsV.plotdims=[1,2];
optionsV.x0=xtraj;

figure(8);
hold on
plotFunnel(V,optionsV); drawnow;
fnplt(xtraj); drawnow;
axis([-4 1 -1 1])
xlabel('X-Z Position Funnel','FontSize',20)
xlabel('x-position (m)','FontSize',20)
ylabel('z-position (m)','FontSize',20)
hold off

optionsV.plotdims=[5,6];
optionsV.x0=xtraj;

figure(9);
hold on
plotFunnel(V,optionsV); drawnow;
fnplt(xtraj,[5,6]); drawnow;
xlabel('X-Z Velocity Funnel','FontSize',20)
xlabel('x-velocity (m/s)','FontSize',20)
ylabel('z-velocity (m/s)','FontSize',20)
hold off

end

% NOTEST  - sadly, it takes too long to run to be used on the build server
