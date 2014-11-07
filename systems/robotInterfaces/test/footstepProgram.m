function [plan, seed, solvertime] = footstepProgram(biped, seed_plan, weights, goal_pos)

nsteps = length(seed_plan.footsteps);
p1 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
p2 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
p1.weights = weights;
p2.weights = weights;

p1 = p1.addOuterUnitCircleEquality(8, true);
p2 = p2.addOuterUnitCircleEquality(8, false);
% p = p.addSinCosLinearEquality(true);

% p1 = p1.addInnerUnitCircleInequality(8, true);
% p2 = p2.addInnerUnitCircleInequality(8, true);

% p1 = p1.addOuterUnitCircleCone(true);
% p2 = p2.addOuterUnitCircleCone(false);


p1 = p1.addQuadraticRelativeObjective(true);
p2 = p2.addQuadraticRelativeObjective(true);

p1 = p1.addZAndYawReachability(true);
p2 = p2.addZAndYawReachability(false);

p1 = p1.addTrimToFinalPoses(true);
p2 = p2.addTrimToFinalPoses(false);

p1 = p1.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], true);
p2 = p2.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], false);

p1 = p1.addTerrainRegions([], true);
p2 = p2.addTerrainRegions([], false);

% p1 = p1.addXYReachabilityEllipse(true);
% p2 = p2.addXYReachabilityEllipse(true);

p1 = p1.addXYReachabilityCircles(true);
p2 = p2.addXYReachabilityCircles(false);

[p1, ok, solvertime] = p1.solve();
[p2, ok, solvertime] = p2.solve();

valuecheck(p2.vars.footsteps.value, p1.vars.footsteps.value, 1e-2);

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
