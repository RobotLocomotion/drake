function plan = footstepMISOCPAndNLP(biped, seed_plan, weights, goal_pos)
% Run the MISOCP to find the step positions and region assignments, then
% run the NLP to pretty up the output.

plan = footstepMISOCP(biped, seed_plan, weights, goal_pos);
lc = lcm.lcm.LCM.getSingleton();
lc.publish('FOOTSTEP_PLAN_RESPONSE', plan.toLCM());
plan = footstepNLP(biped, plan, weights, goal_pos);
lc.publish('FOOTSTEP_PLAN_RESPONSE', plan.toLCM());