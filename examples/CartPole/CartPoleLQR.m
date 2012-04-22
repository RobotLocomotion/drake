function [sys,V] = CartPoleLQR(plant)

typecheck(plant,'DynamicalSystem');

%Q = diag([100,10,1,1]); R = 10;
Q = diag([1 50 1 50]);  
R = .2;

xG = [0;pi;0;0]; uG = 0;
[sys,V] = tilqr(plant,xG,uG,Q,R);

% NOTEST
