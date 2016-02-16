% A self-contained, end-to-end, example from the paper 
% "Optimization and stabilization of trajectories for constrained
%  dynamical systems" by Posa, Kuindersma and Tedrake 2016.
%  This uses a humanoid model loosely based on the Atlas robot, but
%  features a passive spring-damper ankle joint. The arms have been removed
%  and this is a planar (2D) example.
% 
% Trajectory optimization and LQR scripts can be relatively slow, and a
% user may desire to run them one at a time, inspecting the results
%
% All components are included here, in one script, for clarity

%% DIRCON trajectory optimization
display('**********************************************************')
display('********* Running DIRCON Trajectory Optimization *********')
display('**********************************************************')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodySupportState:NoSupportSurface');
[p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = passiveAnkleTrajectoryOptimization();
traj=xtraj{1}.append(xtraj{2}).append(xtraj{3});
v.playback(traj,struct('slider',true));
contact_seq = traj_opt.contact_sequence;
display('Inspect trajectory and press any key to continue')
pause

%% Constrained time varying LQR
display('**********************************************************')
display('**************** Running Constrained LQR *****************')
display('**********************************************************')
% Cost functions
Q = diag([100*ones(p.getNumPositions,1);1*ones(p.getNumVelocities,1)]);
R = .01*eye(getNumInputs(p));
Qf = 2*Q;

% Periodic jump matrix with mirroring
R_periodic = zeros(p.getNumStates);
R_periodic(1:3,1:3) = eye(3); %x,z,pitch
R_periodic(4:6,8:10) = eye(3); %leg joints w/symmetry
R_periodic(8:10,4:6) = eye(3); %leg joints w/symmetry
R_periodic(7,7) = 1; % back joint
R_periodic(11:13,11:13) = eye(3); %x,z,pitch velocities
R_periodic(14:16,18:20) = eye(3); %leg joints w/symmetry
R_periodic(18:20,14:16) = eye(3); %leg joints w/symmetry
R_periodic(17,17) = 1; % back joint
options.periodic_jump = R_periodic;
options.periodic = true;

[c,Ktraj,Straj,Ptraj,Btraj,Straj_full,Ftraj,xtraj,utraj] = hybriddircolconstrainedtvlqr(p,xtraj,utraj,contact_seq,Q,R,Qf,options);

%% Run QP controller
display('**********************************************************')
display('**************** Running QP Stabilization ****************')
display('**********************************************************')
simulated_traj=passiveAnkleExampleStabilization(xtraj,utraj,Btraj,Straj_full,R);