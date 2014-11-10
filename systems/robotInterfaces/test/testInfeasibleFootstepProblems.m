function testInfeasibleFootstepProblems()
% Test the footstep planners on a problem which is known to be infeasible,
% and make sure they throw the appropriate exception. 

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
stone_scale = .3;
stones = [2;-0.15;z];
[Ai, bi] = poly2lincon(stones(1) + stone_scale*[-1, -1, 1, 1],...
                       stones(2) + stone_scale*[-1, 1, 1, -1]);
Ai = [Ai, zeros(size(Ai, 1), 1)];
safe_regions(end+1) = struct('A', Ai, 'b', bi, 'point', [0;0;stones(3)], 'normal', [0;0;1]);


goal_pos = struct('right', [1+0.13;1;z;0;0;pi/2],...
                  'left',  [1-0.13;1;z;0;0;pi/2]);

params = r.default_footstep_params;
params.max_num_steps = 10;

weights = r.getFootstepOptimizationWeights();

function test_solver(solver)
  nsteps = params.max_num_steps + 2;
  seed_plan = FootstepPlan.blank_plan(r, nsteps, [r.foot_frame_id.right, r.foot_frame_id.left], params, safe_regions);
  seed_plan.footsteps(1).pos = foot_orig.right;
  seed_plan.footsteps(2).pos = foot_orig.left;

  try
    [plan, solvertime] = solver(r, seed_plan, weights, goal_pos, 0);
    error('running the solver on an infeasible plan should throw Drake:MixedIntegerConvexProgram:InfeasibleProblem');
  catch e
    if ~strcmp(e.identifier, 'Drake:MixedIntegerConvexProgram:InfeasibleProblem')
      e.getReport()
      error('running the solver on an infeasible plan should throw Drake:MixedIntegerConvexProgram:InfeasibleProblem');
    end
  end
  if checkDependency('yalmip')
    try
      [plan, solvertime] = solver(r, seed_plan, weights, goal_pos, 1);
      error('running the solver on an infeasible plan should throw Drake:MixedIntegerConvexProgram:InfeasibleProblem');
    catch e
      if ~strcmp(e.identifier, 'Drake:MixedIntegerConvexProgram:InfeasibleProblem')
        e.getReport()
        error('running the solver on an infeasible plan should throw Drake:MixedIntegerConvexProgram:InfeasibleProblem');
      end
    end
  end
end

test_solver(@footstepPlanner.footstepMIQP);
test_solver(@footstepPlanner.fixedRotation);
test_solver(@footstepPlanner.alternatingMIQP);
test_solver(@footstepPlanner.linearUnitCircle);
test_solver(@footstepPlanner.humanoids2014);
if checkDependency('yalmip')
  test_solver(@footstepPlanner.dev.relaxedMISOCP);
end

disp('ok');
end
