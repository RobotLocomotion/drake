function runMixedIntegerSimpleForest()
% Run the mixed-integer SOS trajectory planner in a very simple forest.

r = Quadrotor();
r = addTree(r, [.8,.45,1.25], [.20;2.5], pi/4);

start = [0;-1.5;.5];
goal = [1; 4; 0.5];
seeds = [...
         start';
         goal';
         ]';
        

lb = [-3;-2;0];
ub = [3;5;2];
degree = 3;
n_segments = 4;
n_regions = 3;

runMixedIntegerEnvironment(r, start, goal, lb, ub, seeds, degree, n_segments, n_regions);
