clear all
% Get x0 from end of previous trajectory
data = load('data/longer_step_smooth.mat');
x0 = data.xtraj{3}.eval(data.xtraj{3}.tspan(2));

[p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = trajOptStopping(x0);

%%
% [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = trajOptStopping(x0,z);

% THIS SCRIPT GENERATED data/stop_3mode_traj.mat

