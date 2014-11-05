function testMISOCP()

checkDependency('yalmip');
checkDependency('gurobi');
options.floating = true;
options.dt = 0.001;

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
r = Atlas(fullfile(getDrakePath(), 'examples', 'Atlas', 'urdf', 'atlas_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);


fp = load(fullfile(getDrakePath, 'examples', 'Atlas', 'data', 'atlas_fp.mat'));
fp.xstar(3) = fp.xstar(3) + 0.50; % make sure we're not assuming z = 0

% r = r.setTerrain(KinematicTerrainMap(r, fp.xstar(1:r.getNumPositions), true));

foot_orig = struct('right', [0;-0.15;0;0;0;0], 'left', [0;0.15;0;0;0;0]);

safe_regions = struct('A', {}, 'b', {}, 'point', {}, 'normal', {});
n_regions = 10;
lb = [0;-.2;-0.05];
ub = [2;2.2;0.05];
stone_scale = 0.3;
if 1
  stones = [0;-0.15;0];
  [Ai, bi] = poly2lincon(stones(1) + stone_scale*[-1, -1, 1, 1],...
                         stones(2) + stone_scale*[-1, 1, 1, -1]);
  Ai = [Ai, zeros(size(Ai, 1), 1)];
  safe_regions(end+1) = struct('A', Ai, 'b', bi, 'point', [0;0;stones(3)], 'normal', [0;0;1]);
  for j = 2:n_regions
    stones(:,end+1) = rand(3,1) .* (ub - lb) + lb;
    [Ai, bi] = poly2lincon(stones(1,end) + stone_scale*[-1, -1, 1, 1],...
                           stones(2,end) + stone_scale*[-1, 1, 1, -1]);
    Ai = [Ai, zeros(size(Ai, 1), 1)];
    safe_regions(end+1) = struct('A', Ai, 'b', bi, 'point', [0;0;stones(3,end)], 'normal', [0;0;1]);
  end
else
  stones = [];
  [Ai, bi] = poly2lincon([-10, -10, 10,10], [-10, 10, 10, -10]);
  Ai = [Ai, zeros(size(Ai, 1), 1)];
  safe_regions(1) = struct('A', Ai, 'b', bi, 'point', [0;0;0], 'normal', [0;0;1]);
end

goal = [rand(2,1) * 2;0;0;0;rand(1,1)*pi - pi/2];
R = rotmat(goal(6));
goal_pos = struct('right', [goal(1:2) + R * [0;-0.15]; 0;0;0;goal(6)],...
                  'left', [goal(1:2) + R * [0;0.15]; 0;0;0;goal(6)]);
% goal_pos = struct('right', [2;2-0.15;0.1;0;0;0],...
%                   'left',  [2;2+0.15;0.1;0;0;0]);

params = r.default_footstep_params;
params.max_num_steps = 12;
params.min_num_steps = 0;
params.min_step_width = 0.25;
params.nom_step_width = 0.26;
params.max_step_width = 0.27;
params.nom_forward_step = 0.25;
params.max_forward_step = 0.4;
params.nom_upward_step = 0.25;
params.nom_downward_step = 0.15;
params.ignore_terrain = true;
params.map_command = 0;
params.leading_foot = 0;

weights = struct('relative', [10;10;10;0;0;.5],...
                 'relative_final', [100;100;100;0;0;100],...
                 'goal', [100;100;0;0;0;1000]);

tic
nsteps = params.max_num_steps + 2;
seed_plan = FootstepPlan.blank_plan(r, nsteps, [r.foot_frame_id.right, r.foot_frame_id.left], params, safe_regions);
seed_plan.footsteps(1).pos = foot_orig.right;
seed_plan.footsteps(2).pos = foot_orig.left;
plan = footstepMISOCP(r, seed_plan, weights, goal_pos);
toc

steps_rel = plan.relative_step_offsets()

figure(1);
clf
nsteps = length(plan.footsteps);
r_ndx = 1:2:nsteps;
l_ndx = 2:2:nsteps;
steps = plan.step_matrix();
k = 0.25;
arrow_props = {'AutoScale', 'off', 'LineWidth', 2, 'MaxHeadSize', 0.8};
quiver(steps(1,r_ndx), steps(2, r_ndx), k*cos(steps(6,r_ndx)), k*sin(steps(6,r_ndx)), 'Color', [0.2,0.8,0.2], arrow_props{:})
hold on
quiver(steps(1,l_ndx), steps(2,l_ndx), k*cos(steps(6,l_ndx)), k*sin(steps(6,l_ndx)), 'Color', [1, 204/255,0], arrow_props{:})
plot(steps(1,r_ndx), steps(2,r_ndx), 'ko', 'MarkerFaceColor', [0.2,0.8,0.2], 'MarkerSize', 10)
plot(steps(1,l_ndx), steps(2,l_ndx), 'ko', 'MarkerFaceColor', [1, 204/255,0], 'MarkerSize', 10)
goal_pos.center = mean([goal_pos.right, goal_pos.left], 2);
k = 0.15;
quiver(goal_pos.center(1), goal_pos.center(2), k*cos(goal_pos.center(6)), k*sin(goal_pos.center(6)), 'Color', [0,0,0], arrow_props{:}, 'ShowArrowHead', 'off', 'Marker', 'o', 'MarkerSize', 30)
plot(steps(1,:), steps(2,:), 'k:')
for j = 1:length(stones)
  pts = [stones(1,j) + stone_scale*[-1, -1, 1, 1];
           stones(2,j) + stone_scale*[-1, 1, 1, -1]];
  patch(pts(1,:), pts(2,:), 'k', 'FaceAlpha', 0.2);
end
xlim([-0.5,2.5])
ylim([-0.5,2.5])
axis equal
set(gca, 'XTick', []);
set(gca, 'YTick', []);
