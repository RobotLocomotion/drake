function testFootstepSolversNoRegions()
% Test all the footstep planners with no mixed-integer safe region selection.

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

n = rpy2rotmat(foot_orig.right(4:6)) * [0;0;1];
pt = foot_orig.right(1:3);
safe_regions = struct('A', [1,0,0], 'b', 30, 'point', pt, 'normal', n);
% safe_regions = struct('A', zeros(0,3), 'b', zeros(0,1), 'point', pt, 'normal', n);

goal_pos = struct('right', [1+0.13;1;z;0;0;pi/2],...
                  'left',  [1-0.13;1;z;0;0;pi/2]);

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
  axis equal
  xlim([-.25, 2.25])
  ylim([-.25, 2.25])
  title(sprintf('%s: %fs', t, solvertime))
end

figure(1)
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
