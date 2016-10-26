function [plan, solvertime] = humanoids2014(biped, seed_plan, weights, goal_pos, use_symbolic)
% Footstep planner based on the approach presented in "Footstep Planning on
% Uneven Terrain with Mixed-Integer Convex Optimization" by Robin Deits and
% Russ Tedrake. This implementation uses a mixed-integer
% quadratically-constrained program to plan the number of footsteps to take,
% the position and yaw of those steps, and the assignments of footsteps to
% convex regions of obstacle-free terrain. 
%
% This planner should be used by passing the 'method_handle', @footstepPlanner.humanoids2014 
% option to planFootsteps.m.
% 
% @param seed_plan a blank footstep plan, provinding the structure of the
%                  desired plan. Probably generated with
%                  FootstepPlan.blank_plan()
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
% @retval plan a FootstepPlan with the results of the optimization


if nargin < 5
  use_symbolic = 0;
end

nsteps = length(seed_plan.footsteps);

if use_symbolic
  t0 = tic();
  p1 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
  p1.weights = weights;
  p1 = p1.addSinCosLinearEquality(true);
  % p1 = p1.addOuterUnitCircleEquality(8, true);
  p1 = p1.addQuadraticRelativeObjective(true);
  p1 = p1.addZAndYawReachability(true);
  p1 = p1.addTrimToFinalPoses(true);
  p1 = p1.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], true);
  p1 = p1.addTerrainRegions([], true);
  p1 = p1.addXYReachabilityCircles(true);
  if use_symbolic == 2
    fprintf(1, 'yalmip setup: %f\n', toc(t0));
  end
  [p1, solvertime, objval_symb] = p1.solve();
  if use_symbolic == 2
    fprintf(1, 'yalmip total: %f\n', toc(t0));
  end
  plan = p1.getFootstepPlan();
end

if (use_symbolic == 0 || use_symbolic == 2)
  t0 = tic();
  p2 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, false);
  p2.weights = weights;
  p2 = p2.addSinCosLinearEquality(false);
  % p2 = p2.addOuterUnitCircleEquality(8, false);
  p2 = p2.addQuadraticRelativeObjective(false);
  p2 = p2.addZAndYawReachability(false);
  p2 = p2.addTrimToFinalPoses(false);
  p2 = p2.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], false);
  p2 = p2.addTerrainRegions([], false);
  p2 = p2.addXYReachabilityCircles(false);
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