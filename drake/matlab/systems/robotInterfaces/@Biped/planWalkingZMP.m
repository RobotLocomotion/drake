function walking_plan_data = planWalkingZMP(obj, x0, footstep_plan)
% Construct a dynamic walking plan based on the ZMP formulation.
% @param x0 the initial robot state vector
% @param footstep_plan a FootstepPlan or DynamicFootstepPlan
% @param qp_link_names the robot specific link names for certain leg links

settings = QPLocomotionPlanSettings.fromBipedFootstepPlan(footstep_plan, obj, x0);

% if (nargin>3)
%   qp_link_names_fields = fieldnames(qp_link_names);
%   for n = 1:length(qp_link_names_fields)
%     settings.(qp_link_names_fields{n}) = qp_link_names.(qp_link_names_fields{n})
%   end
% end

walking_plan_data = QPLocomotionPlanCPPWrapper(settings);
% walking_plan_data = QPLocomotionPlan.from_biped_footstep_plan(footstep_plan, obj, x0);
