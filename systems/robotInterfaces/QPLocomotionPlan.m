classdef QPLocomotionPlan < QPControllerPlan
  properties
    robot;
    x0;
    support_times
    supports;
    link_constraints;
    zmptraj = [];
    zmp_final = [];
    LIP_height;
    V;
    qtraj;
    comtraj = [];
    mu = 0.5;
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
      if isnumeric(obj.qtraj)
        qf = obj.qtraj;
      else
        qf = fasteval(obj.qtraj, obj.qtraj.tspan(end));
      end
      next_plan = QPLocomotionPlan.from_standing_state([x(1:6); qf(7:end); x(obj.robot.getNumPositions+1:end)], obj.robot, obj.supports(end));
      % next_plan = QPLocomotionPlan.from_standing_state(x, obj.robot, obj.supports(end));

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

      if ~isempty(supp) && any(supp.bodies==r.foot_body_id.right)
        r.warning_manager.warnOnce('Drake:HardCodedSupport', 'hard-coded for heel+toe support');
        qp_input.support_data(1).support_logic_map = obj.support_logic_maps.require_support;
      else
        qp_input.support_data(1).support_logic_map = obj.support_logic_maps.prevent_support;
      end
      if ~isempty(supp) && any(supp.bodies==r.foot_body_id.left)
        r.warning_manager.warnOnce('Drake:HardCodedSupport', 'hard-coded for heel+toe support');
        qp_input.support_data(2).support_logic_map = obj.support_logic_maps.require_support;
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

      if supp_idx < length(obj.supports)
        next_support = obj.supports(supp_idx + 1);
      else
        next_support = obj.supports(supp_idx);
      end
      obj = obj.updatePlanShift(t_global, x, qp_input, contact_force_detected, next_support);
      qp_input = obj.applyPlanShift(qp_input);
    end

    function obj = updatePlanShift(obj, t_global, x, qp_input, contact_force_detected, next_support)
      active_support_bodies = next_support.bodies;
      if any(active_support_bodies == obj.robot.foot_body_id.right) && contact_force_detected(obj.robot.foot_body_id.right)
        loading_foot = obj.robot.foot_body_id.right;
      elseif any(active_support_bodies == obj.robot.foot_body_id.left) && contact_force_detected(obj.robot.foot_body_id.left)
        loading_foot = obj.robot.foot_body_id.left;
      else
        return;
      end

      for j = 1:length(qp_input.body_motion_data)
        if qp_input.body_motion_data(j).body_id == loading_foot;
	  kinsol = obj.robot.doKinematics(x(1:obj.robot.getNumPositions()));
          foot_actual = obj.robot.forwardKin(kinsol, loading_foot, [0;0;0], 1);
          foot_des = evalCubicSplineSegment(t_global - qp_input.body_motion_data(j).ts(1), qp_input.body_motion_data(j).coefs);
          obj.plan_shift_data.plan_shift(1:3) = foot_des(1:3) - foot_actual(1:3);
          break
        end
      end
      disp('plan shift: ')
      obj.plan_shift_data.plan_shift
    end

    function qp_input = applyPlanShift(obj, qp_input)
      qp_input.zmp_data.x0(1:2) = qp_input.zmp_data.x0(1:2) - obj.plan_shift_data.plan_shift(1:2);
      qp_input.zmp_data.y0 = qp_input.zmp_data.y0 - obj.plan_shift_data.plan_shift(1:2);
      for j = 1:length(qp_input.body_motion_data)
        qp_input.body_motion_data(j).coefs(1:3,:,end) = bsxfun(@minus, qp_input.body_motion_data(j).coefs(1:3,:,end), obj.plan_shift_data.plan_shift(1:3));
      end
      qp_input.whole_body_data.q_des(1:3) = qp_input.whole_body_data.q_des(1:3) - obj.plan_shift_data.plan_shift(1:3);
    end

    function [ytraj, v] = simulatePointMassBiped(obj, r)
      typecheck(r, 'PointMassBiped');
      typecheck(obj.robot, 'Biped');

      obj = obj.buildLinkTrajectories();

      r_ind = [];
      l_ind = [];
      for j = 1:length(obj.link_constraints)
        if obj.link_constraints(j).link_ndx == obj.robot.getFrame(obj.robot.foot_frame_id.right).body_ind
          r_ind = j;
        elseif obj.link_constraints(j).link_ndx == obj.robot.getFrame(obj.robot.foot_frame_id.left).body_ind
          l_ind = j;
        end
      end

      breaks = obj.zmptraj.getBreaks();
      traj = PPTrajectory(foh(breaks, obj.zmptraj.eval(breaks)));
      rtraj = PPTrajectory(foh(breaks, obj.link_constraints(r_ind).traj.eval(breaks)));
      ltraj = PPTrajectory(foh(breaks, obj.link_constraints(l_ind).traj.eval(breaks)));
      contact = false(2, length(obj.support_times));
      for j = 1:length(obj.support_times)
        if any(obj.supports(j).bodies == obj.robot.foot_body_id.right)
          contact(1,j) = true;
        end
        if any(obj.supports(j).bodies == obj.robot.foot_body_id.left)
          contact(2,j) = true;
        end
      end
      ctraj = PPTrajectory(zoh(obj.support_times, contact));
      comtraj = obj.comtraj;
      dcomtraj = fnder(obj.comtraj, 1);

      utraj = traj.vertcat(rtraj(1:2));
      utraj = utraj.vertcat(ltraj(1:2));
      utraj = utraj.vertcat(ctraj);
      utraj = utraj.vertcat(comtraj(1:2));
      utraj = utraj.setOutputFrame(r.getInputFrame());

      sys = cascade(utraj, r);
      com0 = comtraj.eval(breaks(1));
      comdot0 = dcomtraj.eval(breaks(1));
      ytraj = sys.simulate([breaks(1), breaks(end)], [com0(1:2); comdot0(1:2)]);

      if nargout > 1
        v = r.constructVisualizer();
      end
    end

    function draw_lcmgl(obj, lcmgl)
      function plot_traj_foh(traj, color)
        ts = traj.getBreaks();
        pts = traj.eval(ts);
        if size(pts,1) == 2
          pts = [pts; zeros(1,size(pts,2))];
        end
        lcmgl.glColor3f(color(1), color(2), color(3));
        lcmgl.glBegin(lcmgl.LCMGL_LINES);
        for j = 1:length(ts)-1
          lcmgl.glVertex3f(pts(1,j), pts(2,j),pts(3,j));
          lcmgl.glVertex3f(pts(1,j+1), pts(2,j+1), pts(3,j+1));
        end
        lcmgl.glEnd();
      end

      if ~isfield(obj.link_constraints(1), 'traj')
        for j = 1:length(obj.link_constraints)
          obj.link_constraints(j).traj = PPTrajectory(mkpp(obj.link_constraints(j).ts, obj.link_constraints(j).coefs, 6));
        end
      end
      for j = 1:length(obj.link_constraints)
        if ~isempty(obj.link_constraints(j).traj)
          plot_traj_foh(obj.link_constraints(j).traj, [0.8, 0.8, 0.2]);
        else
          plot_traj_foh(obj.link_constraints(j).traj_min, [0.8, 0.8, 0.2]);
          plot_traj_foh(obj.link_constraints(j).traj_max, [0.2, 0.8, 0.8]);
        end
      end
      plot_traj_foh(obj.comtraj, [0,1,0]);
      plot_traj_foh(obj.zmptraj, [0,0,1]);
    end
  end

  methods(Static)
    function obj = from_standing_state(x0, biped, support_state)

      if nargin < 3
        support_state = RigidBodySupportState(biped, [biped.foot_body_id.right, biped.foot_body_id.left]);
      end

      obj = QPLocomotionPlan(biped);
      obj.x0 = x0;
      obj.support_times = [0, inf];
      obj.duration = inf;
      obj.supports = [support_state, support_state];

      nq = obj.robot.getNumPositions();
      q0 = x0(1:nq);
      kinsol = doKinematics(obj.robot, q0);

      foot_pos = [obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.right, [0;0;0]),...
                  obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.left, [0;0;0])];
      comgoal = mean(foot_pos(1:2,:), 2);

      obj.zmptraj = comgoal;
      [~, obj.V, obj.comtraj, obj.LIP_height] = obj.robot.planZMPController(comgoal, q0);

      link_constraints(1).link_ndx = obj.robot.foot_body_id.right;
      link_constraints(1).pt = [0;0;0];
      link_constraints(1).ts = [0, inf];
      link_constraints(1).coefs = cat(3, zeros(6,1,3), reshape(forwardKin(obj.robot,kinsol,obj.robot.foot_body_id.right,[0;0;0],1),[6,1,1]));
      link_constraints(2).link_ndx = obj.robot.foot_body_id.left;
      link_constraints(2).pt = [0;0;0];
      link_constraints(2).ts = [0, inf];
      link_constraints(2).coefs = cat(3, zeros(6,1,3),reshape(forwardKin(obj.robot,kinsol,obj.robot.foot_body_id.left,[0;0;0],1),[6,1,1]));
      pelvis_id = obj.robot.findLinkId('pelvis');
      link_constraints(3).link_ndx = pelvis_id;
      link_constraints(3).pt = [0;0;0];
      link_constraints(3).ts = [0, inf];
      pelvis_current = forwardKin(obj.robot,kinsol,pelvis_id,[0;0;0],1);
      pelvis_target = [mean(foot_pos(1:2,:), 2); pelvis_current(3:6)];
      link_constraints(3).coefs = cat(3, zeros(6,1,3),reshape(pelvis_target,[6,1,1,]));
      obj.link_constraints = link_constraints;

      obj.zmp_final = comgoal;
      obj.qtraj = x0(1:nq);
      obj.comtraj = comgoal;
      obj.gain_set = 'standing';
    end

    function obj = from_biped_footstep_plan(footstep_plan, biped, x0, zmp_options)
      if nargin < 4
        zmp_options = struct();
      end
      for j = 1:length(footstep_plan.footsteps)
        footstep_plan.footsteps(j).walking_params = applyDefaults(struct(footstep_plan.footsteps(j).walking_params),...
          biped.default_walking_params);
      end
      [zmp_knots, foot_origin_knots] = biped.planZMPTraj(x0(1:biped.getNumPositions()), footstep_plan.footsteps, zmp_options);
      obj = QPLocomotionPlan.from_biped_foot_and_zmp_knots(foot_origin_knots, zmp_knots, biped, x0);
    end

    function obj = from_biped_foot_and_zmp_knots(foot_origin_knots, zmp_knots, biped, x0, options)
      if nargin < 5
        options = struct();
      end
      options = applyDefaults(options, struct('pelvis_height_above_sole', biped.default_walking_params.pelvis_height_above_foot_sole));
      if isempty(options.pelvis_height_above_sole)
        kinsol = doKinematics(biped, x0(1:biped.getNumPositions()));
        pelvis_pos = forwardKin(biped, kinsol, biped.findLinkId('pelvis'), [0;0;0]);
        feetPosition = biped.feetPosition(x0(1:biped.getNumPositions()));
        options.pelvis_height_above_sole = pelvis_pos(3) - mean([feetPosition.right(3), feetPosition.left(3)]);
      end

      obj = QPLocomotionPlan(biped);
      obj.x0 = x0;
      arm_inds = biped.findPositionIndices('arm');
      obj.qtraj(arm_inds) = x0(arm_inds);
      % obj.qtraj = x0(1:biped.getNumPositions());

      [obj.supports, obj.support_times] = QPLocomotionPlan.getSupports(zmp_knots);
      obj.zmptraj = QPLocomotionPlan.getZMPTraj(zmp_knots);
      [~, obj.V, obj.comtraj, ~] = biped.planZMPController(obj.zmptraj, obj.x0, options);
      obj.link_constraints = biped.getLinkConstraints(foot_origin_knots, obj.zmptraj, obj.supports, obj.support_times, options);

      obj.duration = obj.support_times(end)-obj.support_times(1)-0.001;
      obj.zmp_final = obj.zmptraj.eval(obj.zmptraj.tspan(end));
      if isa(obj.V.S, 'ConstantTrajectory')
        obj.V.S = fasteval(obj.V.S, 0);
      end
      obj.LIP_height = biped.default_walking_params.nominal_LIP_COM_height;
      obj.gain_set = 'walking';
    end

    function obj = from_point_mass_biped_plan(plan, biped, x0, param_set_name)
      if nargin < 4
        param_set_name = 'recovery';
      end
      typecheck(biped, 'Biped');
      typecheck(plan, 'PointMassBipedPlan');

      foot_start = biped.feetPosition(x0(1:biped.getNumPositions()));
      body_ind = struct('right', biped.getFrame(biped.foot_frame_id.right).body_ind,...
                        'left', biped.getFrame(biped.foot_frame_id.left).body_ind);
      body_ind_list = [body_ind.right, body_ind.left];
      initial_supports = RigidBodySupportState(biped, body_ind_list(plan.support(:,1)));
      zmp_knots = struct('t', 0, 'zmp', plan.qcop(:,1), 'supp', initial_supports);

      offset = [-0.048; 0; 0.0811; 0;0;0];
      foot_origin_knots = struct('t', plan.ts(1),...
                                 'right', foot_start.right + offset,...
                                 'left', foot_start.left + offset,...
                                 'is_liftoff', false,...
                                 'is_landing', false,...
                                 'toe_off_allowed', struct('right', false, 'left', false));
      motion = [any(abs(diff(plan.qr, 1, 2)) >= 0.005), false;
                any(abs(diff(plan.ql, 1, 2)) >= 0.005), false];
      warning('ignoring roll and pitch')
      for j = 2:length(plan.ts)
        foot_origin_knots(end+1).t = plan.ts(j);
        if motion(1,j) || motion(1,j-1)
          zr = 0.025;
        else
          zr = 0;
        end
        if motion(2,j) || motion(2, j-1)
          zl = 0.025;
        else
          zl = 0;
        end
        foot_origin_knots(end).right = [plan.qr(:,j); zr; 0; 0; foot_start.right(6)] + offset;
        foot_origin_knots(end).left = [plan.ql(:,j); zl; 0; 0; foot_start.left(6)] + offset;
        foot_origin_knots(end).is_liftoff = any(plan.support(:,j) < plan.support(:,j-1));
        if j > 2
          foot_origin_knots(end).is_landing = any(plan.support(:,j) > plan.support(:,j-1));
        else
          foot_origin_knots(end).is_landing = false;
        end
        foot_origin_knots(end).toe_off_allowed = struct('right', false, 'left', false);

        zmp_knots(end+1).t = plan.ts(j);
        zmp_knots(end).zmp = plan.qcop(:,j);
        zmp_knots(end).supp = RigidBodySupportState(biped, body_ind_list(plan.support(:,j)));
      end

      foot_origin_knots(end+1) = foot_origin_knots(end);
      foot_origin_knots(end).t = foot_origin_knots(end-1).t + (plan.ts(end)-plan.ts(end-1));

      zmp_knots(end+1) = zmp_knots(end);
      zmp_knots(end).t = zmp_knots(end).t + (plan.ts(end)-plan.ts(end-1));

      obj = QPLocomotionPlan.from_biped_foot_and_zmp_knots(foot_origin_knots, zmp_knots, biped, x0, struct('pelvis_height_above_sole', []));
      obj.default_qp_input.whole_body_data.constrained_dofs = biped.findPositionIndices('neck');
      obj.gain_set = param_set_name;
    end

    function obj = from_configuration_traj(biped, qtraj_pp, link_constraints)
      breaks = unmkpp(qtraj_pp);
      x0 = [ppval(qtraj_pp, breaks(1)); zeros(biped.getNumVelocities(), 1)];
      obj = QPLocomotionPlan.from_standing_state(x0, biped);
      obj.qtraj = PPTrajectory(qtraj_pp);
      obj.start_time = obj.qtraj.tspan(1);
      obj.duration = obj.qtraj.tspan(end) - obj.start_time;
      obj.support_times = [obj.qtraj.tspan(1); inf];
      obj.link_constraints = link_constraints;
    end

    function [supports, support_times] = getSupports(zmp_knots)
      supports = [zmp_knots.supp];
      support_times = [zmp_knots.t];
    end

    function zmptraj = getZMPTraj(zmp_knots)
      zmptraj = PPTrajectory(foh([zmp_knots.t], [zmp_knots.zmp]));
      zmptraj = setOutputFrame(zmptraj, SingletonCoordinateFrame('desiredZMP',2,'z',{'x_zmp','y_zmp'}));
    end

  end
end
