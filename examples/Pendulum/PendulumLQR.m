function [c,V] = PendulumLQR(plant)

x0=[pi;0];
u0=0;
Q = diag([10 1]);
R = 1;

[c,V] = tilqr(plant,x0,u0,Q,R);

% NOTEST
