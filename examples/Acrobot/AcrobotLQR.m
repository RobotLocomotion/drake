function sys = AcrobotLQR(plant)

Q = diag([10,10,1,1]); R = 1;
sys = tilqr(plant,[pi;0;0;0],0,Q,R);

% NOTEST
