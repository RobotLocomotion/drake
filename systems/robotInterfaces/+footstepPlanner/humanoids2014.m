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
% @param v_seed (optional) an advanced starting point for the mixed-integer
%               solver. This should be a structure of the same form as 'v'
%               returned by this function, and may be useful for re-running
%               the planner after making a small change to the problem. If
%               the values contained in this seed lead to a feasible
%               solution, then this can dramatically improve the
%               performance of the optimization. 
% @retval plan a FootstepPlan with the results of the optimization
% @retval v a data structure describing all of the variables in the
%           optimization. This is only useful to you if you intend to pass
%           it in as 'v_seed' to another run of this planner.


if nargin < 5
  use_symbolic = 0;
end

if use_symbolic
  nsteps = length(seed_plan.footsteps);
  t0 = tic();
  p1 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
  p1.weights = weights;
  p1 = p1.addSinCosLinearEquality(true);
  p1 = p1.addQuadraticRelativeObjective(true);
  p1 = p1.addZAndYawReachability(true);
  p1 = p1.addTrimToFinalPoses(true);
  p1 = p1.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], true);
  p1 = p1.addTerrainRegions([], true);
  p1 = p1.addXYReachabilityCircles(true);
  fprintf(1, 'yalmip setup: %f\n', toc(t0));
  [p1, ok, solvertime] = p1.solve();
  fprintf(1, 'yalmip total: %f\n', toc(t0));
  plan = p1.getFootstepPlan();
end

if (use_symbolic == 0 || use_symbolic == 2)
  t0 = tic();
  p2 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, false);
  p2.weights = weights;
  p2 = p2.addSinCosLinearEquality(false);
  p2 = p2.addQuadraticRelativeObjective(false);
  p2 = p2.addZAndYawReachability(false);
  p2 = p2.addTrimToFinalPoses(false);
  p2 = p2.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], false);
  p2 = p2.addTerrainRegions([], false);
  p2 = p2.addXYReachabilityCircles(false);
  fprintf(1, 'gurobi setup: %f\n', toc(t0));
  [p2, ok, solvertime] = p2.solve();
  fprintf(1, 'gurobi total: %f\n', toc(t0));
  plan = p2.getFootstepPlan();
end

if use_symbolic == 2
  valuecheck(p2.vars.footsteps.value(1:3,:), p1.vars.footsteps.value(1:3,:), 1e-2);
  valuecheck(p2.vars.footsteps.value(4,:), p1.vars.footsteps.value(4,:), pi/64);
  figure(21)
  clf
  hold on
  plot(p1.vars.cos_yaw.value, p1.vars.sin_yaw.value, 'bo')
  plot(cos(p1.vars.footsteps.value(4,:)), sin(p1.vars.footsteps.value(4,:)), 'ro')
  plot(cos(linspace(0, 2*pi)), sin(linspace(0, 2*pi)), 'k-')
  ylim([-1.1,1.1])
  axis equal
end