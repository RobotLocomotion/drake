function plan = footstepAlternatingMIQP(obj, plan, weights, goal_pos)
%footstepAlternatingMIQP plan footsteps by alternating between an MIQP (to
% assign steps to safe regions and determine total number of steps) and an
% NLP (to find the orientations of the steps)
% @param plan a blank footstep plan, provinding the structure of the
%             desired plan. Probably generated with
%             FootstepPlan.blank_plan()
% @param goal_pos a struct with fields 'right' and 'left'.
%                 goal_pos.right is the desired 6 DOF pose
%                 of the right foot sole, and likewise for
%                 goal_pos.left


ANGLE_CHANGE_THRESHOLD = pi/32;

num_outer_iterations = 2;
for j = 1:num_outer_iterations
  miqp_plan = footstepMIQP(obj, plan, weights, goal_pos);
  if length(miqp_plan.footsteps) <= 2
    % No feasible solution was found
    plan = miqp_plan;
    break
  end

  plan = footstepNLP(obj, miqp_plan, weights, goal_pos);
  miqp_steps = [miqp_plan.footsteps.pos];
  nlp_steps = [plan.footsteps.pos];
  if all(abs(miqp_steps(6,:) - nlp_steps(6,:)) <= ANGLE_CHANGE_THRESHOLD)
    break
  end
end

end



