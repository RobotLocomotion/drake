function [plan, seed, solvertime] = footstepProgram(biped, seed_plan, weights, goal_pos)

nsteps = length(seed_plan.footsteps);
p = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
p.weights = weights;
p = p.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], true);

p = p.addOuterUnitCircleEquality(8, true);
% p = p.addSinCosLinearEquality(true);
p = p.addXYReachabilityCircles(true);

% p = p.addInnerUnitCircleInequality(8, true);
% p = p.addOuterUnitCircleCone(true);
% p = p.addXYReachabilityEllipse(true);

p = p.addQuadraticRelativeObjective(true);
p = p.addZAndYawReachability(true);
p = p.addTrimToFinalPoses(true);
p1 = p.addTerrainRegions([], true);
p2 = p.addTerrainRegions([], false);

[p1, ok, solvertime] = p1.solve();
[p2, ok, solvertime] = p2.solve();

valuecheck(p1.vars.footsteps.value, p2.vars.footsteps.value, 1e-2);

plan = p1.getFootstepPlan();

figure(21)
clf
hold on
plot(p1.vars.cos_yaw.value, p1.vars.sin_yaw.value, 'bo')
plot(cos(p1.vars.footsteps.value(4,:)), sin(p1.vars.footsteps.value(4,:)), 'ro')
plot(cos(linspace(0, 2*pi)), sin(linspace(0, 2*pi)), 'k-')
ylim([-1.1,1.1])
axis equal

plan.step_matrix()
plan.relative_step_offsets()

seed = [];
