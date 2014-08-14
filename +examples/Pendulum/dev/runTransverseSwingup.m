function ctrans=runTransverseSwingup

p = PendulumPlant();

[utraj,xtraj]=runDircol(p);
% OKTOFAIL

% artificially truncate trajectory to test
%tspan = .6*utraj.tspan; breaks=utraj.getBreaks();  breaks=breaks(breaks<tspan(end));
%utraj = FunctionHandleTrajectory(@(t)utraj.eval(t),utraj.dim,breaks);
%xtraj = FunctionHandleTrajectory(@(t)xtraj.eval(t),xtraj.dim,breaks,@(t)xtraj.deriv(t));
% end artificial truncation

p=setInputLimits(p,-inf,inf);

figure(1); clf; hold on;
fnplt(xtraj);

w= randn(2,1);
Nsteps = 50;

% Use orthogonal initial and final surface normals (not essential)
fs0 = p.dynamics(0,xtraj.eval(0),utraj.eval(0));
fsend = p.dynamics(utraj.tspan(end),xtraj.eval(utraj.tspan(end)),utraj.eval(utraj.tspan(end)));
init_surf_normal = fs0/norm(fs0);
final_surf_normal = fsend/norm(fsend);

disp('Designing transversal surfaces...')
transSurf = TransversalSurface.design(p,w, xtraj, utraj, Nsteps,1,init_surf_normal, final_surf_normal);

plotSurface(transSurf,xtraj,.4); drawnow;

disp('Computing transverse lqr...')
[ctrans,sys,xtraj2,utraj2,Vtraj,Vf] = transverseLQRClosedLoop(p,xtraj,utraj,10*eye(2),1,10*eye(2),transSurf);

disp('Estimating funnel...')
psys = taylorApprox(sys,xtraj2,[],3,3);  %ignore var 3 (tau)
options=struct();
options.rho0_tau=10;
options.max_iterations=25;
V=sampledTransverseVerification(psys,Vf,Vtraj,Vtraj.getBreaks(),xtraj,utraj2,transSurf,options);

figure(3); clf;
transSurf.plotFunnel(V,xtraj);
fnplt(xtraj);

disp('Simulating...');
v=PendulumVisualizer;

%keyboard; 

for i=1:5
%  x0=[randn(2,1);0];
  y=simulate(sys,[0 1]);%,x0);
  fnplt(y);
end

v.playback(y);

end


