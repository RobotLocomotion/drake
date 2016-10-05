[p,v,xtraj,utraj,z,ltraj,F,info,traj_opt] = testConstrainedDircol(10);

% run lqr
Q = 100*eye(6);
R = 1;
options.alpha_1 = 100; % regularization term. Larger means more accurate but stiffer ODE to integrate.
[c,Ktraj,Straj,Ptraj,Btraj,Ftraj,Straj_full] = constrainedtvlqr(p,xtraj,utraj,Q,R,Q,options);

% build closed loop plant
sys_cl = p.feedback(c);
%%

% get perturbed initial state
x0 = xtraj.eval(0) + 0.1*randn(6,1);
x0 = p.resolveConstraints(x0);

traj_cl = sys_cl.simulate(xtraj.tspan,x0);

t = linspace(xtraj.tspan(1),xtraj.tspan(2),1e3);

%%
figure
plot(t,xtraj.eval(t),t,traj_cl.eval(t),'--');