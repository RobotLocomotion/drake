function [plan, solvertime] = alternatingMIQP(obj, plan, weights, goal_pos, use_symbolic)
% footstepPlanner.alternatingMIQP plan footsteps by alternating between an MIQP (to
% assign steps to safe regions and determine total number of steps) and an
% NLP (to find the orientations of the steps)
% @param plan a blank footstep plan, provinding the structure of the
%             desired plan. Probably generated with
%             FootstepPlan.blank_plan()
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

if nargin < 5
  use_symbolic = false;
end

ANGLE_CHANGE_THRESHOLD = pi/32;

num_outer_iterations = 2;
solvertime = 0;
for j = 1:num_outer_iterations
  [miqp_plan, t] = footstepPlanner.footstepMIQP(obj, plan, weights, goal_pos, use_symbolic);
  solvertime = solvertime + t;
  if length(miqp_plan.footsteps) <= 2
    % No feasible solution was found
    plan = miqp_plan;
    break
  end

  [plan, t] = footstepPlanner.nonlinearCollocation(obj, miqp_plan, weights, goal_pos);
  solvertime = solvertime + t;
  miqp_steps = [miqp_plan.footsteps.pos];
  nlp_steps = [plan.footsteps.pos];
  if all(abs(miqp_steps(6,:) - nlp_steps(6,:)) <= ANGLE_CHANGE_THRESHOLD)
    break
  end
end

end



