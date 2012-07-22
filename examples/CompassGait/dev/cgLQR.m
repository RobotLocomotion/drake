function sys = cgLQR(stance_plant)

Q = diag([10 10 1 1]); 
R = 1;

sys = tilqr(stance_plant,Point(stance_plant.getStateFrame,zeros(4,1)),Point(stance_plant.getInputFrame,0),Q,R);

% NOTEST
