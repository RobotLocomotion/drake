function testFootstepSolvers()

checkDependency('gurobi');
addpath(fullfile(getDrakePath, 'examples', 'Atlas'));
options.floating = true;
options.dt = 0.001;

% rng(118)

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
r = Atlas(fullfile(getDrakePath(), 'examples', 'Atlas', 'urdf', 'atlas_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);


fp = load(fullfile(getDrakePath, 'examples', 'Atlas', 'data', 'atlas_fp.mat'));
fp.xstar(3) = fp.xstar(3) + 0.50; % make sure we're not assuming z = 0

z = (rand() - 0.5) * 2 * 0.05;
foot_orig = struct('right', [0;-0.15;z;0;0;0], 'left', [0;0.15;z;0;0;0]);

safe_regions = struct('A', {}, 'b', {}, 'point', {}, 'normal', {});
n_regions = 10;
lb = [0;-.2;z];
ub = [2;2.2;z];
stone_scale = .3;
stones = [0;-0.15;z];
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

goal_pos = struct('right', [1+0.13;1;z;0;0;pi/2],...
                  'left',  [1-0.13;1;z;0;0;pi/2]);
% goal_pos = struct('right', [1; 1-0.13; 0.1; 0; 0; 0],...
%                   'left', [1; 1+0.13; 0.1; 0; 0; 0]);

params = r.default_footstep_params;
params.max_num_steps = 10;

weights = r.getFootstepOptimizationWeights();

function test_solver(solver, h, t)
  nsteps = params.max_num_steps + 2;
  seed_plan = FootstepPlan.blank_plan(r, nsteps, [r.foot_frame_id.right, r.foot_frame_id.left], params, safe_regions);
  seed_plan.footsteps(1).pos = foot_orig.right;
  seed_plan.footsteps(2).pos = foot_orig.left;

  if checkDependency('yalmip')
    use_symbolic = 2;
  else
    use_symbolic = 0;
  end
  [plan, solvertime] = solver(r, seed_plan, weights, goal_pos, use_symbolic);

  axes(h);
  cla(h);
  nsteps = length(plan.footsteps);
  r_ndx = 2:2:nsteps;
  l_ndx = 1:2:nsteps;
  steps = plan.step_matrix();
  quiver(h, steps(1,r_ndx), steps(2, r_ndx), cos(steps(6,r_ndx)), sin(steps(6,r_ndx)), 'b', 'AutoScaleFactor', 0.2)
  hold on
  quiver(h, steps(1,l_ndx), steps(2,l_ndx), cos(steps(6,l_ndx)), sin(steps(6,l_ndx)), 'r', 'AutoScaleFactor', 0.2)
  plot(steps(1,:), steps(2,:), 'k:')
  for j = 1:size(stones, 2)
    pts = [stones(1,j) + stone_scale*[-1, -1, 1, 1];
             stones(2,j) + stone_scale*[-1, 1, 1, -1]];
    patch(pts(1,:), pts(2,:), 'k', 'FaceAlpha', 0.2);
  end
  axis equal
  xlim([-.25, 2.25])
  ylim([-.25, 2.25])
  title(sprintf('%s: %fs', t, solvertime))
end

figure(1)
clf
h = subplot(2, 3, 1);
test_solver(@footstepPlanner.footstepMIQP, h, 'miqp');
drawnow()
h = subplot(2, 3, 2);
test_solver(@footstepPlanner.fixedRotation, h, 'fixedRotation');
drawnow()
h = subplot(2, 3, 3);
test_solver(@footstepPlanner.alternatingMIQP, h, 'alternating miqp');
drawnow()
h = subplot(2, 3, 4);
test_solver(@footstepPlanner.linearUnitCircle, h, 'linear unit circle');
drawnow()
h = subplot(2, 3, 5);
test_solver(@footstepPlanner.humanoids2014, h, 'humanoids2014');
drawnow();
h = subplot(2, 3, 6);
test_solver(@footstepPlanner.dev.relaxedMISOCP, h, 'relaxedMISOCP');
drawnow();
end
