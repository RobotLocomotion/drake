function [r] = runMixedIntegerOffice
% NOTEST
% Run the mixed-integer SOS trajectory planner in the office environment

r = Quadrotor();

degree = 3;
n_segments = 7;
start = [5;-1;1.5];
% start = [-2;0;1];
% goal = [0;0;.5];
goal = [-1;0;0.5];
% goal = [5;-1;1.5];
% goal = [6;-1;1.5];

r = addRobotFromURDF(r, 'office.urdf');

lb = [-5;0;0.01];
ub = [6;9.5;2.01];

seeds = [...
         [4, -1, 1.5];
         [2.5, .75, 1];
         [3, 0, 1];
         % start';
         goal';
         ]';
n_regions = 7;

runMixedIntegerEnvironment(r, start, goal, lb, ub, seeds, degree, n_segments, n_regions);


return;

