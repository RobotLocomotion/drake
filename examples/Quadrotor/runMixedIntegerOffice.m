function [r] = runMixedIntegerOffice
% NOTEST
% Run the mixed-integer SOS trajectory planner in the office environment

r = Quadrotor();

degree = 3;
n_segments = 7;
start = [-2.5;9;1.5];
goal = [0;2;.5];

r = addRobotFromURDF(r, 'office.urdf');

lb = [-5;0;0.01];
ub = [6;9.5;2.01];

seeds = [...
        [-2.25,8,1.5];
         start';
         goal';
         ]';
n_regions = 7;

runMixedIntegerEnvironment(r, start, goal, lb, ub, seeds, degree, n_segments, n_regions);


return;

