function [sys,V] = CartPoleHinf(plant)

if (nargin<1) plant = CartPolePlant(); end

typecheck(plant,'CartPolePlant');

Bw = [zeros(2,2); eye(2)];  %uncertainty affects velocities only
Q = diag([1 50 1 50]);  
R = .1;
xG = [0;pi;0;0]; uG = 0;
gamma = 20;
[sys,V] = tiHinf(plant,xG,uG,Q,R,Bw,gamma);

