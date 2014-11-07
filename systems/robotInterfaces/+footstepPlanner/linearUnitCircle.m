function [plan, solvertime] = linearUnitCircle(biped, seed_plan, weights, goal_pos, use_symbolic)

if nargin < 5
  use_symbolic = 0;
end

nsteps = length(seed_plan.footsteps);

if use_symbolic
  t0 = tic();
  p1 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
  p1.weights = weights;
  p1 = p1.addOuterUnitCircleEquality(8, true);
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
  p2 = p2.addOuterUnitCircleEquality(8, false);
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
  valuecheck(p2.vars.footsteps.value, p1.vars.footsteps.value, 1e-2);
  figure(21)
  clf
  hold on
  plot(p1.vars.cos_yaw.value, p1.vars.sin_yaw.value, 'bo')
  plot(cos(p1.vars.footsteps.value(4,:)), sin(p1.vars.footsteps.value(4,:)), 'ro')
  plot(cos(linspace(0, 2*pi)), sin(linspace(0, 2*pi)), 'k-')
  ylim([-1.1,1.1])
  axis equal
end
