classdef QPLocomotionPlan < QPControllerPlan
  properties
    % The support logic maps make it possible for the planner
    % to specify the support state used by the controller, based
    % on the controller's instantaneous force and kinematic input. 
    % See QPInputConstantHeight.m for a more complete description. 
    support_logic_maps = struct('require_support', ones(4,1),...
                                'only_if_force_sensed', [0;0;1;1],...
                                'only_if_kinematic', [0;1;0;1],...
                                'kinematic_or_sensed', [0;1;1;1],...
                                'prevent_support', zeros(4,1));
    duration = inf;
    start_time = 0;
    default_qp_input = atlasControllers.QPInputConstantHeight;
    robot;
    x0;
    support_times
    supports;
    link_constraints;
    zmptraj;
    zmp_final;
    LIP_height;
    V;
    qtraj;
    comtraj;
    mu = 1;
    plan_shift_data = PlanShiftData();
    g = 9.81; % gravity m/s^2
  end

  methods
    function obj = QPLocomotionPlan(robot)
      obj.robot = robot;
      S = load(obj.robot.fixed_point_file);
      obj.qtraj = S.xstar(1:obj.robot.getNumPositions());
      obj.default_qp_input = atlasControllers.QPInputConstantHeight();
      obj.default_qp_input.whole_body_data.q_des = zeros(obj.robot.getNumPositions(), 1);
      obj.default_qp_input.whole_body_data.constrained_dofs = [findPositionIndices(obj.robot,'arm');findPositionIndices(obj.robot,'neck');findPositionIndices(obj.robot,'back_bkz');findPositionIndices(obj.robot,'back_bky')];
    end

    function next_plan = getSuccessor(obj, t, x)
      next_plan = obj;
      next_plan.duration = inf;
    end

    function is_finished = isFinished(obj, t, x)
      is_finished = t - obj.start_time >= obj.duration;
    end

    function qp_input = getQPControllerInput(obj, t_global, x, rpc, contact_force_detected)
      % Get the input structure which can be passed to the stateless QP control loop
      % @param t the current time
      % @param x the current robot state
      % @param rpc the robot property cache, which lets us quickly look up info about
      % @param contact_force_detected num_bodies vector indicating whether contact force
      %                               was detected on that body. Default: zeros(num_bodies,1)
      % the robot which would be expensive to compute (such as terrain contact points)

      if nargin < 5
        contact_force_detected = zeros(rpc.num_bodies, 1);
      end

      r = obj.robot;
      t_plan = t_global - obj.start_time;
      t_plan = double(t_plan);
      
      T = obj.duration;
      t_plan = min([t_plan, T]);

      qp_input = obj.default_qp_input;
      qp_input.zmp_data.D = -obj.LIP_height/obj.g * eye(2);
      qp_input.zmp_data.x0 = [obj.zmp_final; 0;0];
      if isnumeric(obj.zmptraj)
        qp_input.zmp_data.y0 = obj.zmptraj;
      else
        qp_input.zmp_data.y0 = fasteval(obj.zmptraj, t_plan);
      end
      qp_input.zmp_data.S = obj.V.S;
      if isnumeric(obj.V.s1)
        qp_input.zmp_data.s1 = obj.V.s1;
      else
        qp_input.zmp_data.s1 = fasteval(obj.V.s1,t_plan);
      end

      supp_idx = find(obj.support_times<=t_plan,1,'last');
      supp = obj.supports(supp_idx);

      qp_input.support_data = struct('body_id', {r.foot_body_id.right, r.foot_body_id.left},...
                                     'contact_pts', {...
                               [rpc.contact_groups{r.foot_body_id.right}.heel, ...
                                rpc.contact_groups{r.foot_body_id.right}.toe],...
                               [rpc.contact_groups{r.foot_body_id.left}.heel,...
                                rpc.contact_groups{r.foot_body_id.left}.toe]},...
                                     'support_logic_map', {zeros(4,1), zeros(4,1)},...
                                     'mu', {obj.mu, obj.mu},...
                                     'contact_surfaces', {0,0});

      if ~isempty(supp.bodies) && any(supp.bodies==r.foot_body_id.right)
        r.warning_manager.warnOnce('Drake:HardCodedSupport', 'hard-coded for heel+toe support');
        qp_input.support_data(1).support_logic_map = obj.support_logic_maps.kinematic_or_sensed;
      else
        qp_input.support_data(1).support_logic_map = obj.support_logic_maps.prevent_support;
      end
      if ~isempty(supp.bodies) && any(supp.bodies==r.foot_body_id.left)
        r.warning_manager.warnOnce('Drake:HardCodedSupport', 'hard-coded for heel+toe support');
        qp_input.support_data(2).support_logic_map = obj.support_logic_maps.kinematic_or_sensed;
      else
        qp_input.support_data(2).support_logic_map = obj.support_logic_maps.prevent_support;
      end

      if isnumeric(obj.qtraj)
        qp_input.whole_body_data.q_des = obj.qtraj;
      else
        qp_input.whole_body_data.q_des = fasteval(obj.qtraj, t_plan);
      end

      pelvis_has_tracking = false;
      for j = 1:length(obj.link_constraints)
        qp_input.body_motion_data(j).body_id = obj.link_constraints(j).link_ndx;
        if qp_input.body_motion_data(j).body_id == rpc.body_ids.pelvis
          pelvis_has_tracking = true;
        end
        body_t_ind = find(obj.link_constraints(j).ts<=t_plan,1,'last');
        if body_t_ind < length(obj.link_constraints(j).ts)
          qp_input.body_motion_data(j).ts = obj.link_constraints(j).ts(body_t_ind:body_t_ind+1) + obj.start_time;
        else
          qp_input.body_motion_data(j).ts = obj.link_constraints(j).ts([body_t_ind,body_t_ind]) + obj.start_time;
        end
        qp_input.body_motion_data(j).coefs = obj.link_constraints(j).coefs(:,body_t_ind,:);
      end
      assert(pelvis_has_tracking, 'Expecting a link_constraints block for the pelvis');

      qp_input.param_set_name = obj.gain_set;

      obj = obj.updatePlanShift(t_global, x, qp_input, contact_force_detected);
      qp_input = obj.applyPlanShift(qp_input);
    end
  end
end