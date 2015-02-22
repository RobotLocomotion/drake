classdef AtlasPlanEval
  properties
    plan_data;
    body_accel_input_weights = [0.15 0.15, 0.07]
    pelvis_body_id;
    robot
  end

  methods
    function obj = AtlasPlanEval(r, walking_plan)
      obj.robot = r;
      obj.plan_data = walking_plan;
      if isempty(obj.plan_data.qstar)
        load(obj.robot.fixed_point_file, 'xstar')
        obj.plan_data.qstar = xstar(1:obj.robot.getNumPositions());
      end
      if isa(obj.plan_data.V.S, 'ConstantTrajectory')
        obj.plan_data.V.S = obj.plan_data.V.S.eval(0);
      end
      obj.pelvis_body_id = obj.robot.findLinkId('pelvis');
    end

    function qp_input = tick(obj, t, x)
      pdata = obj.plan_data;
      r = obj.robot;

      T = pdata.comtraj.tspan(2)-0.001;

      import atlasControllers.PlanlessQPInput2D;
      qp_input = PlanlessQPInput2D();
      qp_input.zmp_data.x0 = [pdata.zmptraj.eval(T); 0;0];
      qp_input.zmp_data.S = pdata.V.S;
      qp_input.zmp_data.s1 = pdata.V.s1.eval(t);

      supp_idx = find(pdata.support_times<=t,1,'last');
      supp = pdata.supports(supp_idx);

      if any(supp.bodies==r.foot_body_id.right)
        warning('hard-coded for heel+toe support')
        qp_input.support_data.group_mask(qp_input.support_body_ids == r.foot_body_id.right,1:2) = true;
      end
      if any(supp.bodies==r.foot_body_id.left)
        warning('hard-coded for heel+toe support')
        qp_input.support_data.group_mask(qp_input.support_body_ids == r.foot_body_id.left,1:2) = true;
      end

      qp_input.whole_body_data.q_des = pdata.qstar;
      qp_input.whole_body_data.w_qdd(findPositionIndices(r, 'arm')) = .0001;
      qp_input.whole_body_data.w_qdd(findPositionIndices(r, 'back')) = .0001;
      qp_input.whole_body_data.w_qdd(findPositionIndices(r, 'back_bkx')) = 0.1;
      qp_input.whole_body_data.Kp = 150 + zeros(r.getNumPositions(), 1);
      qp_input.whole_body_data.Kp(r.findPositionIndices('back_bkx')) = 50;
      qp_input.whole_body_data.Kd = getDampingGain(qp_input.whole_body_data.Kp, 0.0);
      qp_input.whole_body_data.constrained_dof_mask(r.findPositionIndices('neck')) = true;

      feet_poses = zeros(6,2);
      i = 1;
      tracked_bodies = 0;
      for j = 1:length(pdata.link_constraints)
        qp_input.bodies_data(j).body_id = pdata.link_constraints(j).link_ndx;
        body_t_ind = find(pdata.link_constraints(j).ts<=t,1,'last');
        if body_t_ind < length(pdata.link_constraints(j).ts)
          qp_input.bodies_data(j).ts = pdata.link_constraints(j).ts(body_t_ind:body_t_ind+1);
        else
          qp_input.bodies_data(j).ts = pdata.link_constraints(j).ts([body_t_ind,body_t_ind]);
        end
        qp_input.bodies_data(j).coefs = pdata.link_constraints(j).coefs(:,body_t_ind,:);
        if any(qp_input.bodies_data(j).body_id == [r.foot_body_id.right, r.foot_body_id.left])
          feet_poses(:,i) = evalCubicSplineSegment(t - qp_input.bodies_data(j).ts(1), qp_input.bodies_data(j).coefs);
          i = i + 1;
        end
        qp_input.bodies_data(j).Kp = [12; 12; 12; 12; 12; 12];
        qp_input.bodies_data(j).Kd = getDampingGain(qp_input.bodies_data(j).Kp, 0.7);
        warning('using same weight for all bodies')
        qp_input.bodies_data(j).weight = obj.body_accel_input_weights(j);
        tracked_bodies = tracked_bodies + 1;
      end

      r.warning_manager.warnOnce('Drake:HardCodedPelvisheight', 'hard-coding pelvis height');
      r.warning_manager.warnOnce('Drake:NoPelvisHeightLogic', 'not using pelvis height logic');
      mean_feet_pose = [mean(feet_poses(1:3,:), 2); angleAverage(feet_poses(4:6,1), feet_poses(4:6,2))];
      pelvis_target = [mean_feet_pose(1:2); mean_feet_pose(3) + 0.74; mean_feet_pose(4:6)];
      coefs = reshape(pelvis_target, [6, 1, 1]);
      coefs = cat(3, zeros(6, 1, 3), coefs);
      qp_input.bodies_data(tracked_bodies+1).body_id = obj.pelvis_body_id;
      qp_input.bodies_data(tracked_bodies+1).ts = [t, t];
      qp_input.bodies_data(tracked_bodies+1).coefs = coefs;
      warning('hard-coding pelvis weight')
      qp_input.bodies_data(tracked_bodies+1).Kp = [nan; nan; 20; 20; 20; 20];
      qp_input.bodies_data(tracked_bodies+1).Kd = getDampingGain(qp_input.bodies_data(tracked_bodies+1).Kp, 0.5);
      qp_input.bodies_data(tracked_bodies+1).weight = obj.body_accel_input_weights(tracked_bodies+1);

    end
  end
end




