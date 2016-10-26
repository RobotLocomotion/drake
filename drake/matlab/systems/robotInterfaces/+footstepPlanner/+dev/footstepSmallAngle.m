function [plan, seed, solvertime] = footstepSmallAngle(biped, seed_plan, weights, goal_pos)

nsteps = length(seed_plan.footsteps);
p1 = MixedIntegerFootstepPlanningProblem(biped, seed_plan, true);
p1.weights = weights;

% p1 = p1.addOuterUnitCircleEquality(8, true);
p1 = p1.addSmallRelativeAngleRotation(true);
p1 = p1.addXYReachabilityCircles(true);

p1 = p1.addQuadraticRelativeObjective(true);

p1 = p1.addZAndYawReachability(true);

p1 = p1.addTrimToFinalPoses(true);

p1 = p1.addQuadraticGoalObjective(goal_pos, nsteps-1:nsteps, [1,1], true);

p1 = p1.addTerrainRegions([], true);

[p1, ok, solvertime] = p1.solve();

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
keyboard()

seed = [];
