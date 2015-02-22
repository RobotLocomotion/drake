classdef AtlasPlanEval
  properties
    plan_data;

    robot
  end

  methods
    function obj = AtlasPlanEval(r, walking_plan)
      obj.robot = r;
      obj.plan_data = walking_plan;
    end

    function qp_input = tick(obj, t, x)
      pdata = obj.plan_data;

      T = pdata.comtraj.tspan(2)-0.001;

      qp_input = PlanlessQPInput2D();
      qp_input.zmp_data.x0 = [pdata.zmptraj.eval(T); 0;0];
      qp_input.zmp_data.S = pdata.V.S;
      qp_input.zmp_data.s1 = pdata.V.s1.eval(t);

      supp_idx = find(pdata.support_times<=t,1,'last');
      supp = pdata.supports(supp_idx);

      if supp_idx < length(pdata.support_times) 
        t0 = pdata.support_times(supp_idx);
        t1 = pdata.support_times(supp_idx+1);
        if ~isempty(pdata.link_constraints) && isfield(pdata.link_constraints(1),'pelvis_reference_height')
          pelvis_ref = pdata.link_constraints(1).pelvis_reference_height(supp_idx);
          pelvis_ref_next = pdata.link_constraints(1).pelvis_reference_height(supp_idx+1);
          eta = double((t1-t)/(t1-t0));
          obj.controller_data.pelvis_foot_height_reference = eta * pelvis_ref + (1-eta)*pelvis_ref_next - pdata.plan_shift(3); 
        end
      end

      if any(supp.bodies==obj.robot.foot_body_id.right)
        warning('hard-coded for heel+toe support')
        qp_input.support_data(qp_input.support_body_ids == obj.robot.foot_body_id.right,1:2) = true;
      end
      if any(supp.bodies==obj.robot.foot_body_id.left)
        warning('hard-coded for heel+toe support')
        qp_input.support_data(qp_input.support_body_ids == obj.robot.foot_body_id.left,1:2) = true;
      end

      qp_input.whole_body_data.q_des = pdata.qstar;
      qp_input.whole_body_data.w_qdd(obj.robot.findPositionIndices('back_bkx')) = 0.01;
      qp_input.whole_body_data.Kp(obj.robot.findPositionIndices('back_bkx')) = 50;
      qp_input.whole_body_data.Kd = getDampingGain(qp_input.whole_body_data.Kp, 0.0);
      qp_input.whole_body_data.constrained_dof_mask(obj.robot.findPositionIndices('neck')) = true;

      for j = 1:length(pdata.link_constraints)
        qp_input.bodies_data(j).body_id = pdata.link_constraints(j).link_idx;
        body_t_ind = find(pdata.link_constraints(j).ts<=t,1,'last');
        if body_t_ind < length(pdata.link_constraints(j).ts)
          qp_input.bodies_data(j).ts = pdata.link_constraints(j).ts(body_t_ind,body_t_ind+1);
        else
          qp_input.bodies_data(j).ts = pdata.link_constraints(j).ts(body_t_ind,body_t_ind);
        end
        qp_input.bodies_data(j).coefs = pdata.link_constraints(j).coefs(:,body_t_ind,:);
        qp_input.bodies_data(j).Kp = [12; 12; 12; 12; 12; 12];
        qp_input.bodies_data(j).Kd = getDampingGain(qp_input.bodies_data(j).Kp, 0.7);
      end
    end
  end
end




