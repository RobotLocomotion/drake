function sys = cgLQR(stance_plant)

Q = diag([10 10 1 1]); 
R = 1;

sys = tilqr(stance_plant,[0;0;0;0],0,Q,R);

% NOTEST