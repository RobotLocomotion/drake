function runMixedIntegerEnvironment(r, start, goal, lb, ub, seeds, traj_degree, num_traj_segments, n_regions)
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

checkDependency('lcmgl');
checkDependency('iris');

bot_radius = 0.3;

can_draw_lcm_polytopes = logical(exist('drawLCMPolytope', 'file'));

lc = lcm.lcm.LCM.getSingleton();
lcmgl = LCMGLClient('quad_goal');
lcmgl.glColor3f(.8,.2,.2);
lcmgl.sphere(goal, .1, 20, 20);
lcmgl.switchBuffers();

v = constructVisualizer(r);
x0 = double(Point(getStateFrame(r)));  % initial conditions: all-zeros
x0(1:3) = start;
v.draw(0,double(x0));

b = r.getBody(1);
obstacles = cell(1, length(b.getContactShapes()));
for j = 1:length(b.getContactShapes())
  obstacles{j} = b.getContactShapes{j}.getPoints();
end

if can_draw_lcm_polytopes
  lcmgl = LCMGLClient('iris_seeds');
  % Clear the displayed polytopes
  drawLCMPolytope(0, 0, 0, 0, lc);
  % pause
end

dim = length(lb);
A_bounds = [eye(dim);-eye(dim)];
b_bounds = [ub; -lb];

if can_draw_lcm_polytopes
  % Draw the bounding box
  drawLCMPolytope(A_bounds, b_bounds, 100, true, lc);
  % pause

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

start = [start, [0;0;0], [0;0;0]];
goal = [goal, [0;0;0], [0;0;0]];
[ytraj, ~, ~, safe_region_assignments] = prob.solveTrajectory(start, goal, safe_regions);

% Add an all-zeros yaw trajectory
ytraj = ytraj.vertcat(ConstantTrajectory(0));

% Invert differentially flat outputs to find the state traj
ytraj = ytraj.setOutputFrame(DifferentiallyFlatOutputFrame);
xtraj = invertFlatOutputs(r,ytraj);
v.playback(xtraj, struct('slider', true));

% Draw the result
lc = lcm.lcm.LCM.getSingleton();
lcmgl = drake.util.BotLCMGLClient(lc, 'quad_trajectory');
lcmgl.glBegin(lcmgl.LCMGL_LINES);
lcmgl.glColor3f(0.0,0.0,1.0);

breaks = ytraj.getBreaks();
ts = linspace(breaks(1), breaks(end));
Y = ytraj.eval(ts);
for i = 1:size(Y, 2)-1
  lcmgl.glVertex3f(Y(1,i), Y(2,i), Y(3,i));
  lcmgl.glVertex3f(Y(1,i+1), Y(2,i+1), Y(3,i+1));
end

lcmgl.glEnd();
lcmgl.switchBuffers();


figure(1)
for j = 1:5
  subplot(5, 1, j);
  fnplt(fnder(ytraj(1), j));
end
end

