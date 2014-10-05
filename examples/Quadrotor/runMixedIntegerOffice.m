function [r, xtraj, utraj, prog] = runMixedIntegerOffice
% NOTEST
% Run the mixed-integer SOS trajectory planner in the office environment

r = Quadrotor();

degree = 3;
n_segments = 7;
% start = [5;-1;1.5];
start = [-2;0;1];
% goal = [0;0;.5];
goal = [5;-1;1.5];
% goal = [6;-1;1.5];

wall_height = 3.0;


%room walls
r = addBox(r, [10.0,.1,wall_height], [0;3.15],0);
r = addBox(r, [10.0,.1,wall_height], [0;-3.15],0);
r = addBox(r, [.1,8.0,1.0], [4.0;0],0);
r = addBox(r, [.1,8.0,wall_height], [-4.0;0],0);

%room internal wall
r = addBox(r, [.1,3.7,wall_height], [2.5;-1.65],0);
r = addBox(r, [.1,1.5,wall_height], [2.5;2.5],0);

%room window
r = addBox(r, [.1,4.0,wall_height], [4.0;1.5],0);
r = addBox(r, [.1,1.75,wall_height], [4.0;-2.5],0);
r = addFloatingBox(r, [.28,1.5,wall_height-2], [4.0,-.95,2.5], 0, [191,120,21]/255);

%roof
%r = addBox(r, [10,8,2.0], [0;0],0);

%cabinet
r = addFloatingBox(r, [.8,1.7,1.7],[-3.5,-1.8,0.8],0);

r = addFloatingBox(r, [.6,1.5,.4],[-3.3,-1.8,1.3],0);
r = addFloatingBox(r, [.6,1.5,.4],[-3.3,-1.8,0.8],0);
r = addFloatingBox(r, [.6,1.5,.4],[-3.3,-1.8,0.3],0);

r = addTable(r, [.2,.2,1.0], [-1,-1,.5],2);


lb = [-5;-3.5;0.01];
ub = [6;3.5;3.01];

seeds = [...
         [4, -1, 1.5];
         [2.5, .75, 1];
         [3, 0, 1];
         start';
         ]';
n_regions = 7;

runMixedIntegerEnvironment(r, start, goal, lb, ub, seeds, degree, n_segments, n_regions);
