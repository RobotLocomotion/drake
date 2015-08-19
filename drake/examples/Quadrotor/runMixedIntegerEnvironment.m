function runMixedIntegerEnvironment(r, start, goal, lb, ub, seeds, traj_degree, num_traj_segments, n_regions, dt)
% NOTEST
% Run the mixed-integer SOS trajectory planner on a simulated 3D environment, using IRIS to seed
% convex regions of safe space. For example usage, see runMixedIntegerForest and runMixedIntegerOffice.
% @param r the Quadrotor, with obstacle geometry added
% @param start 3x1 the initial pose
% @param goal 3x1 the final pose
% @param lb 3x1 the lower bound of the bounding box for the trajectory and safe regions
% @param ub 3x1 the upper bound
% @param seeds 3xm seeds for the IRIS algorithm. Can be empty.
% @param traj_degree degree of the trajectory pieces
% @param num_traj_segments number of polynomial pieces
% @param n_regions number of IRIS regions
% @param dt the time (in seconds) duration of each trajectory piece

checkDependency('lcmgl');
checkDependency('iris');
checkDependency('mosek');

if nargin < 10
  dt = 0.5;
end

AUTOSAVE = false;

bot_radius = 0.3;

can_draw_lcm_polytopes = logical(exist('drawLCMPolytope', 'file')) && logical(exist('drc.lin_con_t'));

lc = lcm.lcm.LCM.getSingleton();
lcmgl = LCMGLClient('quad_goal');
lcmgl.glColor3f(.8,.2,.2);
lcmgl.sphere(goal, .04, 20, 20);
lcmgl.switchBuffers();

v = constructVisualizer(r);
x0 = double(Point(getStateFrame(r)));  % initial conditions: all-zeros
x0(1:3) = start;
v.draw(0,double(x0));

b = r.getBody(1);
obstacles = cell(1, length(b.getCollisionGeometry()));
for j = 1:length(b.getCollisionGeometry())
  obstacles{j} = b.getCollisionGeometry{j}.getPoints();
end

if can_draw_lcm_polytopes
  lcmgl = LCMGLClient('iris_seeds');
  % Clear the displayed polytopes
  drawLCMPolytope(0, 0, 0, 0, lc);
end

dim = length(lb);
A_bounds = [eye(dim);-eye(dim)];
b_bounds = [ub; -lb];

if can_draw_lcm_polytopes
  % Draw the bounding box
  drawLCMPolytope(A_bounds, b_bounds, 100, true, lc);

  % Clear the displayed polytopes
  drawLCMPolytope(0, 0, 0, 0, lc);
end

region_idx = 1;
function done = drawRegion(r, seed)
  if can_draw_lcm_polytopes
    drawLCMPolytope(r.A, r.b, region_idx, false, lc);
    region_idx = region_idx + 1;
    lcmgl.glColor3f(.8,.8,.2);
    lcmgl.sphere(seed, 0.06, 20, 20);
    lcmgl.switchBuffers();
  end
  done = false;
end

% Automatically generate IRIS regions as necessary
num_steps = 10;
safe_regions = iris.util.auto_seed_regions(obstacles, lb, ub, seeds, n_regions, num_steps, @drawRegion);

% Set up and solve the problem
prob = MISOSTrajectoryProblem();
prob.num_traj_segments = num_traj_segments;
prob.traj_degree = traj_degree;
prob.bot_radius = bot_radius;
prob.dt = dt;

% Add initial and final velocities and accelerations
start = [start, [0;0;0], [0;0;0]];
goal = [goal, [0;0;0], [0;0;0]];

% Find a piecewise 3rd-degree polynomial through the convex regions from start to goal
disp('Running mixed-integer convex program for 3rd-degree polynomial...');
[~, ~, ~, safe_region_assignments] = prob.solveTrajectory(start, goal, safe_regions);
disp('done!');

% Run the program again with the region assignments fixed, for a piecewise 5th-degree polynomial
disp('Running semidefinite program for 5th-degree polynomial...');
prob.traj_degree = 5;
ytraj = prob.solveTrajectory(start, goal, safe_regions, safe_region_assignments);
disp('done!');

% Add an all-zeros yaw trajectory
ytraj = ytraj.vertcat(ConstantTrajectory(0));

% % Invert differentially flat outputs to find the state traj
disp('Inverting differentially flat system...')
ytraj = ytraj.setOutputFrame(DifferentiallyFlatOutputFrame);
[xtraj, utraj] = invertFlatOutputs(r,ytraj);
disp('done!');

figure(83);
clf
hold on
ts = utraj.getBreaks();
ts = linspace(ts(1), ts(end), 100);
u = utraj.eval(ts);
plot(ts, u(1,:), ts, u(2,:), ts, u(3,:), ts, u(4,:))
drawnow()
% v.playback(xtraj, struct('slider', true));

% Stabilize the trajectory with TVLQR
x0 = xtraj.eval(0);
tf = utraj.tspan(2);
Q = 10*eye(12);
R = eye(4);
Qf = 10*eye(12);
disp('Computing stabilizing controller with TVLQR...');
c = tvlqr(r,xtraj,utraj,Q,R,Qf);
sys = feedback(r,c);
disp('done!');
% sys = cascade(utraj, r);

% Simulate the result
disp('Simulating the system...');
xtraj_sim = simulate(sys,[0 tf],x0);
disp('done!');

if AUTOSAVE
  folder = fullfile('../data', datestr(now,'yyyy-mm-dd_HH.MM.SS'));
  system(sprintf('mkdir -p %s', folder));
  save(fullfile(folder, 'results.mat'), 'xtraj', 'ytraj', 'utraj', 'r', 'v', 'safe_region_assignments', 'prob', 'safe_regions', 'xtraj_sim', 'start', 'goal', 'sys');
end

% Draw the result
xtraj_sim = xtraj_sim.setOutputFrame(r.getStateFrame());
v.playback(xtraj_sim, struct('slider', true));

lc = lcm.lcm.LCM.getSingleton();
lcmgl = drake.util.BotLCMGLClient(lc, 'quad_trajectory');
lcmgl.glBegin(lcmgl.LCMGL_LINES);
lcmgl.glColor3f(0.0,0.0,1.0);

breaks = ytraj.getBreaks();
ts = linspace(breaks(1), breaks(end));
Y = squeeze(ytraj.eval(ts));
for i = 1:size(Y, 2)-1
  lcmgl.glVertex3f(Y(1,i), Y(2,i), Y(3,i));
  lcmgl.glVertex3f(Y(1,i+1), Y(2,i+1), Y(3,i+1));
end

figure(123)
clf
hold on
Ysnap = fnder(ytraj, 4);
Ysn = squeeze(Ysnap.eval(ts));
plot(ts, sum(Y.^2, 1), ts, sum(Ysn.^2, 1))

lcmgl.glEnd();
lcmgl.switchBuffers();

end

