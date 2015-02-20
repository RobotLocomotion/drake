classdef PlanEvalBlock < DrakeSystem
  properties
    controller_data;
    plan_data;
  end

  methods
    function obj = PlanEvalBlock(r,controller_data,plan_data,options)
      input_frame = r.getStateFrame();
      output_frame = input_frame;
      obj = obj@DrakeSystem(0,0,input_frame,output_frame,true,true);
      obj.controller_data = controller_data;
      obj.plan_data = plan_data;
    end

    function x = output(obj, t, ~, x)
      plan_data = obj.plan_data;
      ctrl_data = obj.controller_data;

      for f = {'x0', 'u0', 'y0', 'S', 's1'}
        field = f{1};
        if isa(plan_data.(field), 'Trajectory')
          ctrl_data.(field) = fasteval(plan_data.(field), t);
        else
          ctrl_data.(field) = plan_data.(field);
        end
      end

      for j = 1:length(plan_data.link_constraints)
        ctrl_data.body_motion_desired(j).body_id = plan_data.link_constraints(j).lind_idx;
        body_traj_ind = find(ctrl_data.link_constraints(link_con_ind).ts<=t,1,'last');
        ctrl_data.body_motion_desired(j).ts = plan_data.link_constraints(j).ts(body_traj_ind:min(body_traj_ind+1, length(plan_data.link_constraints(j).ts)));
        ctrl_data.body_motion_desired(j).coefs = ctrl_data.link_constraints(j).coefs(:,body_traj_ind,:);
      end

      ctrl_data.constrained_dofs = plan_data.constrained_dofs;
      



