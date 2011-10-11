function [sys,V] = AcrobotLQR(plant)

Q = diag([10,10,1,1]); R = 1;
[sys,V] = tilqr(plant,[pi;0;0;0],0,Q,R);

% NOTEST
