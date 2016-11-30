function [plan, solvertime] = fixedRotation(biped, seed_plan, weights, goal_pos, use_symbolic)
% Run the Mixed Integer Quadratic Program form of the footstep planning problem.
% This form can efficiently choose the assignment of individual foot positions to
% safe (obstacle-free) regions, but always keeps the yaw value of every foot
% fixed.
%
% The structure of the desired footstep plan is indicated by the seed plan.
% @param seed_plan a FootstepPlan object which specifies the structure of the
%                  desired footstep plan. This seed_plan must contain a
%                  list of Footsteps and the region_order of matching length,
%                  but undetermined footstep positions and region assignments
%                  can be NaN. The first two footstep poses must not be NaN,
%                  since they correspond to the current positions of the feet
% @param weights a struct with fields 'goal', 'relative', and
%                'relative_final' describing the various contributions to
%                the cost function. These are described in detail in
%                Biped.getFootstepOptimizationWeights()
% @param goal_pos a struct with fields 'right' and 'left'.
%                 goal_pos.right is the desired 6 DOF pose
%                 of the right foot sole, and likewise for
%                 goal_pos.left
% @option use_symbolic (default: false) whether to use the symbolic yalmip
%                     version of the solver, which is slower to set up
%                     but easier to modify.
% @retval plan a FootstepPlan matching the number of footsteps, frame_id, etc.
%              of the seed plan, but with the footstep positions and region_order
%              replaced by the results of the MIQP

if nargin < 5
  use_symbolic = 0;
end

nsteps = length(seed_plan.footsteps);

if use_symbolic
  t0 = tic();
  p1 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
  p1.weights = weights;
  p1 = p1.fixRotation();
  p1 = p1.addReachabilityLinearConstraints();
  p1 = p1.addQuadraticRelativeObjective(true);
  p1 = p1.addTrimToFinalPoses(true);
  p1 = p1.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], true);
  p1 = p1.addTerrainRegions([], true);
  if use_symbolic
    fprintf(1, 'yalmip setup: %f\n', toc(t0));
  end
  [p1, solvertime, objval_symb] = p1.solve();
  if use_symbolic
    fprintf(1, 'yalmip total: %f\n', toc(t0));
  end
  plan = p1.getFootstepPlan();
end

if (use_symbolic == 0 || use_symbolic == 2)
  t0 = tic();
  p2 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, false);
  p2.weights = weights;
  p2 = p2.fixRotation();
  p2 = p2.addReachabilityLinearConstraints();
  p2 = p2.addQuadraticRelativeObjective(false);
  p2 = p2.addTrimToFinalPoses(false);
  p2 = p2.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], false);
  p2 = p2.addTerrainRegions([], false);
  if use_symbolic == 2
    fprintf(1, 'gurobi setup: %f\n', toc(t0));
  end
  [p2, solvertime, objval_nosymb] = p2.solve();
  if use_symbolic == 2
    fprintf(1, 'gurobi total: %f\n', toc(t0));
  end
  plan = p2.getFootstepPlan();
end

if use_symbolic == 2
  try
    valuecheck(p2.vars.footsteps.value(1:3,:), p1.vars.footsteps.value(1:3,:), 1e-2);
    valuecheck(p2.vars.footsteps.value(4,:), p1.vars.footsteps.value(4,:), pi/64);
  catch e
    % sometimes there are multiple solutions with very similar objective values
    rangecheck(abs(objval_symb - objval_nosymb) / objval_symb, 0, 1e-3);
  end

  figure(21)
  clf
  hold on
  plot(p1.vars.cos_yaw.value, p1.vars.sin_yaw.value, 'bo')
  plot(cos(p1.vars.footsteps.value(4,:)), sin(p1.vars.footsteps.value(4,:)), 'ro')
  plot(cos(linspace(0, 2*pi)), sin(linspace(0, 2*pi)), 'k-')
  ylim([-1.1,1.1])
  axis equal
end

