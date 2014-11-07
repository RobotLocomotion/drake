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
  fprintf(1, 'yalmip setup: %f\n', toc(t0));
  [p1, solvertime] = p1.solve();
  fprintf(1, 'yalmip total: %f\n', toc(t0));
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
  fprintf(1, 'gurobi setup: %f\n', toc(t0));
  [p2, solvertime] = p2.solve();
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

  plan3 = footstepPlanner.footstepMIQP(biped, seed_plan, weights, goal_pos);
  plan3steps = plan3.step_matrix();
  plansteps = plan.step_matrix();
  valuecheck(plansteps(1:3,:), plan3steps(1:3,:), 1e-2);
  valuecheck(plansteps(6,:), plan3steps(6,:), pi/64);
end

