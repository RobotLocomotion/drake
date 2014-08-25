function [ltvsys,Vtraj] = GliderLQR(plant,xtraj,utraj)

Qf=diag([(1/0.05)^2 (1/0.05)^2 (1/3)^2 (1/3)^2 1 1 (1/3)^2]);
Q = diag([10 10 10 1 1 1 1]);  R=0.1; % LQR Cost Matrices

[ltvsys,Vtraj] = tvlqr(plant,xtraj,utraj,Q,R,Qf);

% NOTEST
