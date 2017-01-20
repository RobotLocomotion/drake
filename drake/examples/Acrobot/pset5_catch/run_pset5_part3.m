% RUN THIS to generate your solution
megaclear

[p,xtraj,utraj,v,x0] = pset5_catch;

% if you want to display the trajectory again
%v.playback(xtraj);

% ********YOUR CODE HERE ********
% Set Q, R, and Qf for time varying LQR
% See problem statement for instructions here
Q = 
R = 
Qf =
% *******************************

c = p.tvlqr(xtraj,utraj,Q,R,Qf);
sys_cl = p.feedback(c);

%%
x0_test = x0;
x0_test(3) = x0(3) + .1;
traj_test_1 = sys_cl.simulate(xtraj.tspan,x0_test);
v.drawWrapper(traj_test_1.tspan(2),traj_test_1.eval(traj_test_1.tspan(2)));

x0_test = x0 + .02*(rand(10,1) - 1);
traj_test_2 = sys_cl.simulate(xtraj.tspan,x0_test);
v.drawWrapper(traj_test_2.tspan(2),traj_test_2.eval(traj_test_2.tspan(2)));

% submit x_grade below
x_grade = [traj_test_1.eval(xtraj.pp.breaks) traj_test_2.eval(xtraj.pp.breaks) Qf repmat(xtraj.tspan(2),10,1)]';

format short
display(x_grade)
format long