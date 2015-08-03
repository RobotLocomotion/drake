function runMixedIntegerForest
% NOTEST
% Run the mixed-integer SOS trajectory planner in the forest environment. 

r = Quadrotor();
r = addTree(r, [.8,.45,1.25], [.20;2.5], pi/4);
r = addTree(r, [.5,.35,1.65], [-.25;5], -pi/6);
r = addTree(r, [.55,.65,1.5], [.25;7.5], pi/4);
r = addTree(r, [.55,.85,1.6], [-1.35;8.5], pi/3.7);
r = addTree(r, [.85,.95,1.65], [-1.85;5.2], -pi/3.7);
r = addTree(r, [.75,.9,1.75], [2;4.4], -pi/5);

start = [0;-1.5;.5];
goal = [1; 10; 0.5];
seeds = [...
         start';
         goal';
         [0.5, 5, 0.5]; % the middle of the forest
         ]';
        

lb = [-3;-2;0.01];
ub = [3;13;2];
degree = 3;
n_segments = 7;
n_regions = 8;

runMixedIntegerEnvironment(r, start, goal, lb, ub, seeds, degree, n_segments, n_regions);
