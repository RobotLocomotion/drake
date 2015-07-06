[p,v,xtraj,utraj,z,ltraj,F,info,traj_opt] = testConstrainedDircol(13);
% info
t = linspace(0,xtraj.tspan(2),1000);
traj=utraj.cascade(p).simulate(xtraj.tspan,xtraj.eval(0));
x = xtraj.eval(t);
xsim = traj.eval(t);
figure(1)
plot(t,x-xsim)
N =length(xtraj.pp.breaks);
display(sprintf('info: %d, N: %d, F: %e, F/N: %e',info,N,F, F/N));
% F/length(xtraj.pp.breaks)

% %%
% clear phi phidot
% for i=1:length(t),
%   qi = x(1:3,i);
%   vi = x(4:6,i);
%   [phii,Ji] = p.positionConstraints(qi);
%   phi(:,i) = phii;
%   phidot(:,i) = Ji*vi;
% end