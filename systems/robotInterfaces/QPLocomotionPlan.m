classdef QPLocomotionPlan < QPControllerPlan
  properties
    robot;
    x0;
    support_times
    supports;
    body_motions;
    zmptraj = [];
    zmp_final = [];
    LIP_height;
    V;
    qtraj;
    comtraj = [];
    mu = 0.5;
    plan_shift = zeros(6,1);
    plan_shift_z_only = false;
    g = 9.81; % gravity m/s^2
    is_quasistatic = false;
    constrained_dofs = [];

    planned_support_command = QPControllerPlan.support_logic_maps.require_support; % when the plan says a given body is in support, require the controller to use that support. To allow the controller to use that support only if it thinks the body is in contact with the terrain, try QPControllerPlan.support_logic_maps.kinematic_or_sensed; 

    last_qp_input;

    lcmgl = LCMGLClient('locomotion_plan');
    joint_pd_override_data

    MIN_KNEE_ANGLE = 0.7;
    KNEE_KP = 40;
    KNEE_KD = 4;
    KNEE_WEIGHT = 1;

  end

  properties(Access=protected)
    toe_off_active = struct('right', false, 'left', false);
  end


  methods
    function obj = QPLocomotionPlan(robot)
      obj.robot = robot;
      S = load(obj.robot.fixed_point_file);
      obj.qtraj = S.xstar(1:obj.robot.getNumPositions());
      obj.default_qp_input = atlasControllers.QPInputConstantHeight();
      obj.default_qp_input.whole_body_data.q_des = zeros(obj.robot.getNumPositions(), 1);
      obj.constrained_dofs = [findPositionIndices(obj.robot,'arm');findPositionIndices(obj.robot,'neck');findPositionIndices(obj.robot,'back_bkz');findPositionIndices(obj.robot,'back_bky')];
      obj.joint_pd_override_data = struct('position_ind',{},'kp',{},'kd',{},'weight',{});
    end

    function next_plan = getSuccessor(obj, t, x)
      next_plan = FrozenPlan(obj.last_qp_input);
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

      if isempty(obj.start_time)
        obj.start_time = t_global;
      end
      r = obj.robot;
      t_plan = t_global - obj.start_time;
      t_plan = double(t_plan);
      if t_plan < 0
        qp_input = [];
        return;
      end
      
      T = obj.duration;
      t_plan = min([t_plan, T]);

      q = x(1:rpc.nq);
      qd = x(rpc.nq+(1:rpc.nv));

      qp_input = obj.default_qp_input;
      qp_input.zmp_data.D = -obj.LIP_height/obj.g * eye(2);

      if isnumeric(obj.qtraj)
        qp_input.whole_body_data.q_des = obj.qtraj;
      else
        qp_input.whole_body_data.q_des = fasteval(obj.qtraj, t_plan);
      end
      qp_input.whole_body_data.constrained_dofs = obj.constrained_dofs;

      if obj.is_quasistatic
        if isnumeric(obj.comtraj)
          com_state = obj.comtraj;          
        else
          com_state = fasteval(obj.comtraj,t_plan);
        end

        if size(com_state,1) == 2;
            com_state = [com_state;0*com_state];
        end

        qp_input.zmp_data.x0 = com_state;
        qp_input.zmp_data.y0 = com_state(1:2);
      else
        qp_input.zmp_data.x0 = [obj.zmp_final; 0;0];
        if isnumeric(obj.zmptraj)
          qp_input.zmp_data.y0 = obj.zmptraj;
        else
          qp_input.zmp_data.y0 = fasteval(obj.zmptraj, t_plan);
        end
      end

      qp_input.zmp_data.S = obj.V.S;
      
      if isnumeric(obj.V.s1)
        qp_input.zmp_data.s1 = obj.V.s1;
      else
        qp_input.zmp_data.s1 = fasteval(obj.V.s1, t_plan);
      end

      kinsol = doKinematics(obj.robot, q);

      if t_plan < obj.support_times(1)
        supp_idx = 1;
      elseif t_plan > obj.support_times(end)
        supp_idx = length(obj.support_times);
      else
        supp_idx = find(obj.support_times<=t_plan,1,'last');
      end

      pelvis_has_tracking = false;

      for j = 1:length(obj.body_motions)
        body_id = obj.body_motions(j).body_id;
        if body_id < 0
          body_id = obj.robot.getFrame(body_id).body_ind;
        end
        if body_id == obj.robot.foot_body_id.right
          kny_ind = rpc.position_indices.r_leg_kny;
          foot_name = 'right';
          other_foot = obj.robot.foot_body_id.left;

        elseif body_id == obj.robot.foot_body_id.left
          kny_ind = rpc.position_indices.l_leg_kny;
          foot_name = 'left';
          other_foot = obj.robot.foot_body_id.right;
        else
          kny_ind = [];
          other_foot = [];
        end

        body_t_ind = obj.body_motions(j).findTInd(t_plan);
        if ~isempty(kny_ind)
          if ~obj.toe_off_active.(foot_name)
            if any(obj.supports(supp_idx).bodies == body_id) && q(kny_ind) < obj.MIN_KNEE_ANGLE % && any(obj.supports(supp_idx).bodies == other_foot) 
              obj.toe_off_active.(foot_name) = (supp_idx < length(obj.supports)) && ~any(obj.supports(supp_idx+1).bodies == body_id);
            end
          else
            if ~any(obj.supports(supp_idx).bodies == body_id)
              obj.toe_off_active.(foot_name) = false;
              obj = obj.updateSwingTrajectory(t_plan, j, body_t_ind-1, kinsol, qd);
            end
          end

          if obj.toe_off_active.(foot_name)
            body_mask = obj.supports(supp_idx).bodies == body_id;
            if ~isempty(obj.supports(supp_idx).contact_groups{body_mask})
              obj.supports(supp_idx) = obj.supports(supp_idx).setContactPts(body_mask, rpc.contact_groups{body_id}.toe, {'toe'});
            end
            qp_input.joint_pd_override(end+1) = struct('position_ind', kny_ind,...
                                                       'qi_des', obj.MIN_KNEE_ANGLE,...
                                                       'qdi_des', 0,...
                                                       'kp', obj.KNEE_KP,...
                                                       'kd', obj.KNEE_KD,...
                                                       'weight', obj.KNEE_WEIGHT);
            if obj.body_motions(j).toe_off_allowed(body_t_ind)
              obj = obj.updateSwingTrajectory(t_plan, j, body_t_ind, kinsol, qd);
            end

          end

          % if ~isempty(kny_ind) && q(kny_ind) < obj.MIN_KNEE_ANGLE
          %   qp_input.joint_pd_override(end+1) = struct('position_ind', kny_ind,...
          %                                              'qi_des', obj.MIN_KNEE_ANGLE,...
          %                                              'qdi_des', 0,...
          %                                              'kp', obj.KNEE_KP,...
          %                                              'kd', obj.KNEE_KD,...
          %                                              'weight', 1e-2);
          % end

        end

        qp_input.body_motion_data(j) = obj.body_motions(j).slice(body_t_ind);

        qp_input.body_motion_data(j).ts = qp_input.body_motion_data(j).ts + obj.start_time;

        if qp_input.body_motion_data(j).body_id == rpc.body_ids.pelvis
          pelvis_has_tracking = true;
        end

      end

      assert(pelvis_has_tracking, 'Expecting a motion_motion_data element for the pelvis');

      supp = obj.supports(supp_idx);

      qp_input.support_data = struct('body_id', cell(1, length(supp.bodies)),...
                                     'contact_pts', cell(1, length(supp.bodies)),...
                                     'support_logic_map', cell(1, length(supp.bodies)),...
                                     'mu', cell(1, length(supp.bodies)),...
                                     'contact_surfaces', cell(1, length(supp.bodies)));
      for j = 1:length(supp.bodies)
        qp_input.support_data(j).body_id = supp.bodies(j);
        qp_input.support_data(j).contact_pts = supp.contact_pts{j};
        qp_input.support_data(j).support_logic_map = obj.planned_support_command;
        qp_input.support_data(j).mu = obj.mu;
        qp_input.support_data(j).contact_surfaces = 0;
      end

      qp_input.param_set_name = obj.gain_set;

      if supp_idx < length(obj.supports)
        next_support = obj.supports(supp_idx + 1);
      else
        next_support = obj.supports(supp_idx);
      end
      obj = obj.updatePlanShift(t_global, kinsol, qp_input, contact_force_detected, next_support);
      qp_input = obj.applyPlanShift(qp_input);
      
      if(~isempty(obj.joint_pd_override_data))
        for j = 1:length(obj.joint_pd_override_data.position_ind)
          qp_input.joint_pd_override(end+1) = struct('position_ind',obj.joint_pd_override_data.position_ind(j),...
                                                     'qi_des',qp_input.whole_body_data.q_des(obj.joint_pd_override_data.position_ind(j)),...
                                                     'qdi_des',0,...
                                                     'kp',obj.joint_pd_override_data.kp(j),...
                                                     'kd',obj.joint_pd_override_data.kd(j),...
                                                     'weight',obj.joint_pd_override_data.weight(j));
        end
      end
      
      obj.last_qp_input = qp_input;
    end

    function obj = updateSwingTrajectory(obj, t_plan, body_motion_ind, body_t_ind, kinsol, qd)
      body_motion_data = obj.body_motions(body_motion_ind);

      [x0, J] = obj.robot.forwardKin(kinsol, body_motion_data.body_id, [0;0;0], 2);
      xd0 = J * qd;
      
      
      x1_exp = body_motion_data.coefs(:,body_t_ind+2, end);
      xd1_exp = body_motion_data.coefs(:,body_t_ind+2, end);
      [quat1, dquat1_dexp] = expmap2quat(x1_exp(4:6));
      quatdot1 = dquat1_dexp * xd1_exp(4:6);
      
      flip = sign(x0(4:7)' * quat1);
      x0(4:7) = x0(4:7) * flip;
      xd0(4:7) = xd0(4:7) * flip;

      xd0 = [zeros(3,1); quatdot2expmapdot(x0(4:7), xd0(4:7))];

      xs = [[x0(1:3); quat2expmap(x0(4:7))], body_motion_data.coefs(:, body_t_ind+(2:4), end)];
      
      % If the current pose is pitched down more than the first aerial knot point, adjust the knot point to match the current pose
      T_x0_to_world_in_world = poseQuat2tform([xs(1:3,1); expmap2quat(xs(4:6,1))]);
      T_x1_to_world_in_world = poseQuat2tform([xs(1:3,2); expmap2quat(xs(4:6,2))]);
      xprime_x0 = T_x0_to_world_in_world * [1;0;0;0];
      xprime_x1 = T_x1_to_world_in_world * [1;0;0;0];
      if xprime_x0(3) < xprime_x1(3)
        xs(4:6,2) = quat2expmap(slerp(expmap2quat(xs(4:6,1)), expmap2quat(xs(4:6,3)), 0.1));
      end

      xdf = body_motion_data.coefs(:,body_t_ind+4,end-1);

      ts = body_motion_data.ts(body_t_ind+(1:4));
      qpSpline_options = struct('optimize_knot_times', false);
      [coefs, ts] = qpSpline(ts, xs, xd0, xdf, qpSpline_options);

      % tt = linspace(ts(1), ts(end));
      % pp = mkpp(ts, coefs, 6);
      % xs = ppval(pp, tt);
      % xds = ppval(fnder(pp, 1), tt);
      % obj.lcmgl.glPointSize(5);
      % obj.lcmgl.points(xs(1,:), xs(2,:), xs(3,:));

      % vscale = 0.1;
      % for j = 1:size(xs,2)
      %   obj.lcmgl.line3(xs(1,j), xs(2,j), xs(3,j), ...
      %               xs(1,j) + vscale*xds(1,j),...
      %               xs(2,j) + vscale*xds(2,j),...
      %               xs(3,j) + vscale*xds(3,j));
      % end
      % obj.lcmgl.switchBuffers();

      obj.body_motions(body_motion_ind).coefs(:,body_t_ind+(1:3),:) = coefs;
      obj.body_motions(body_motion_ind).ts(body_t_ind+(1:3)) = ts(1:3);
    end

    function obj = updatePlanShift(obj, t_global, kinsol, qp_input, contact_force_detected, next_support)
      active_support_bodies = next_support.bodies;
      if any(active_support_bodies == obj.robot.foot_body_id.right) && contact_force_detected(obj.robot.foot_body_id.right)
        loading_foot = obj.robot.foot_body_id.right;
      elseif any(active_support_bodies == obj.robot.foot_body_id.left) && contact_force_detected(obj.robot.foot_body_id.left)
        loading_foot = obj.robot.foot_body_id.left;
      else
        return;
      end

      for j = 1:length(qp_input.body_motion_data)
        body_id = qp_input.body_motion_data(j).body_id;
        if body_id < 0
          body_id = obj.robot.getFrame(body_id).body_ind;
        end
        if body_id == loading_foot;
          foot_actual = obj.robot.forwardKin(kinsol, qp_input.body_motion_data(j).body_id, [0;0;0], 0);
          foot_des = evalCubicSplineSegment(t_global - qp_input.body_motion_data(j).ts(1), qp_input.body_motion_data(j).coefs);
          obj.plan_shift(1:3) = foot_des(1:3) - foot_actual(1:3);
          break
        end
      end
    end

    function qp_input = applyPlanShift(obj, qp_input)
      if ~obj.plan_shift_z_only
        qp_input.zmp_data.x0(1:2) = qp_input.zmp_data.x0(1:2) - obj.plan_shift(1:2);
        qp_input.zmp_data.y0 = qp_input.zmp_data.y0 - obj.plan_shift(1:2);
        inds = 1:3;
      else
        inds = 3;
      end
      for j = 1:length(qp_input.body_motion_data)
        qp_input.body_motion_data(j).coefs(inds,:,end) = qp_input.body_motion_data(j).coefs(inds,:,end) - obj.plan_shift(inds);
      end
      qp_input.whole_body_data.q_des(inds) = qp_input.whole_body_data.q_des(inds) - obj.plan_shift(inds);
    end

    function [ytraj, v] = simulatePointMassBiped(obj, r)
      typecheck(r, 'PointMassBiped');
      typecheck(obj.robot, 'Biped');

      link_trajectories = obj.getLinkTrajectories();

      r_ind = [];
      l_ind = [];
      for j = 1:length(link_trajectories)
        if link_trajectories(j).link_ndx == obj.robot.getFrame(obj.robot.foot_frame_id.right).body_ind
          r_ind = j;
        elseif link_trajectories(j).link_ndx == obj.robot.getFrame(obj.robot.foot_frame_id.left).body_ind
          l_ind = j;
        end
      end

      breaks = obj.zmptraj.getBreaks();
      traj = PPTrajectory(foh(breaks, obj.zmptraj.eval(breaks)));
      rtraj = PPTrajectory(foh(breaks, link_trajectories(r_ind).traj.eval(breaks)));
      ltraj = PPTrajectory(foh(breaks, link_trajectories(l_ind).traj.eval(breaks)));
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

      link_trajectories = obj.getLinkTrajectories();
      for j = 1:length(link_trajectories)
        if ~isempty(link_trajectories(j).traj)
          plot_traj_foh(link_trajectories(j).traj, [0.8, 0.8, 0.2]);
        else
          plot_traj_foh(link_trajectories(j).traj_min, [0.8, 0.8, 0.2]);
          plot_traj_foh(link_trajectories(j).traj_max, [0.2, 0.8, 0.8]);
        end
      end
      if ~isa(obj.comtraj, 'Trajectory')
        obj.comtraj = ExpPlusPPTrajectory(obj.comtraj.breaks,...
                                          obj.comtraj.K,...
                                          obj.comtraj.A,...
                                          obj.comtraj.alpha,...
                                          obj.comtraj.gamma);
      end
      plot_traj_foh(obj.comtraj, [0,1,0]);
      plot_traj_foh(obj.zmptraj, [0,0,1]);
    end

    function link_trajectories = getLinkTrajectories(obj)
      link_trajectories = struct('link_ndx', {}, 'traj', {}, 'min_traj', {}, 'max_traj', {});
      for j = 1:length(obj.body_motions)
        link_trajectories(j).link_ndx = obj.body_motions(j).body_id;
        link_trajectories(j).traj = PPTrajectory(mkpp(obj.body_motions(j).ts, obj.body_motions(j).coefs, size(obj.body_motions(j).coefs, 1)));
      end
    end
    
    function obj = setCOMTraj(obj)
      ts = obj.qtraj.getBreaks();
      % this deals with the case of a constant trajectory
      com_poses = zeros(2,length(ts));
      for j = 1:numel(ts)
        kinsol = obj.robot.doKinematics(obj.qtraj.eval(ts(j)));
        com_position = obj.robot.getCOM(kinsol);
        com_poses(:,j) = com_position(1:2);
      end
      obj.comtraj = PPTrajectory(pchip(ts,com_poses));      
    end

    function obj = setLQR_for_COM(obj)
      Q = diag([10 10 1 1]);
      R = 0.0001*eye(2);
      A = [zeros(2),eye(2); zeros(2,4)];
      B = [zeros(2); eye(2)];
      [~,S,~] = lqr(A,B,Q,R);
      % set the Qy to zero since we only want to stabilize COM
      obj.default_qp_input.zmp_data.Qy = 0*obj.default_qp_input.zmp_data.Qy;
      obj.default_qp_input.zmp_data.A = A;
      obj.default_qp_input.zmp_data.B = B;
      obj.default_qp_input.zmp_data.R = R;
      obj.default_qp_input.zmp_data.S = S;
    end
    
  end

  methods(Static)
    function obj = from_standing_state(x0, biped, support_state, options)

      if nargin < 3
        support_state = RigidBodySupportState(biped, [biped.foot_body_id.right, biped.foot_body_id.left]);
      end
      if nargin < 4
        options = struct();
      end
      options = applyDefaults(options, struct('center_pelvis', true));

      obj = QPLocomotionPlan(biped);
      obj.x0 = x0;
      obj.support_times = [0, inf];
      obj.duration = inf;
      obj.supports = [support_state, support_state];
      obj.is_quasistatic = true;

      nq = obj.robot.getNumPositions();
      q0 = x0(1:nq);
      kinsol = doKinematics(obj.robot, q0);


      pelvis_id = obj.robot.findLinkId('pelvis');
      pelvis_current_xyzquat = forwardKin(obj.robot,kinsol,pelvis_id,[0;0;0],2);
      if options.center_pelvis
        foot_pos = [obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.right, [0;0;0]),...
                    obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.left, [0;0;0])];
        comgoal = mean(foot_pos(1:2,:), 2);
        pelvis_target_xyzquat = [mean(foot_pos(1:2,:), 2); pelvis_current_xyzquat(3:end)];
      else
        comgoal = obj.robot.getCOM(kinsol);
        comgoal = comgoal(1:2);
        pelvis_target_xyzquat = pelvis_current_xyzquat;
      end

      obj.zmptraj = comgoal;
      [~, obj.V, obj.comtraj, obj.LIP_height] = obj.robot.planZMPController(comgoal, q0);

      obj.body_motions = [BodyMotionData(obj.robot.foot_body_id.right, [0, inf]),...
                          BodyMotionData(obj.robot.foot_body_id.left, [0, inf]),...
                          BodyMotionData(pelvis_id, [0, inf])];
      rfoot_xyzquat = forwardKin(obj.robot,kinsol,obj.robot.foot_body_id.right,[0;0;0],2);
      rfoot_xyzexpmap = [rfoot_xyzquat(1:3);quat2expmap(rfoot_xyzquat(4:7))];
      obj.body_motions(1).coefs = cat(3, zeros(6,1,3), reshape(rfoot_xyzexpmap,[6,1,1]));
      obj.body_motions(1).in_floating_base_nullspace = true(1, 2);

      lfoot_xyzquat = forwardKin(obj.robot,kinsol,obj.robot.foot_body_id.left,[0;0;0],2);
      lfoot_xyzexpmap = [lfoot_xyzquat(1:3);quat2expmap(lfoot_xyzquat(4:7))];
      obj.body_motions(2).coefs = cat(3, zeros(6,1,3),reshape(lfoot_xyzexpmap,[6,1,1]));
      obj.body_motions(2).in_floating_base_nullspace = true(1, 2);

      pelvis_target_xyzexpmap = [pelvis_target_xyzquat(1:3);quat2expmap(pelvis_target_xyzquat(4:7))];
      obj.body_motions(3).coefs = cat(3, zeros(6,1,3),reshape(pelvis_target_xyzexpmap,[6,1,1,]));
      obj.body_motions(3).in_floating_base_nullspace = false(1, 2);

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
      [zmp_knots, foot_motion_data] = biped.planZMPTraj(x0(1:biped.getNumPositions()), footstep_plan.footsteps, zmp_options);
      obj = QPLocomotionPlan.from_biped_foot_and_zmp_knots(foot_motion_data, zmp_knots, biped, x0);
    end

    function obj = from_biped_foot_and_zmp_knots(foot_motion_data, zmp_knots, biped, x0, options)
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
      pelvis_motion_data = biped.getPelvisMotionForWalking(foot_motion_data, obj.supports, obj.support_times, options);
      obj.body_motions = [foot_motion_data, pelvis_motion_data];

      obj.duration = obj.support_times(end)-obj.support_times(1)-0.001;
      obj.zmp_final = obj.zmptraj.eval(obj.zmptraj.tspan(end));
      if isa(obj.V.S, 'ConstantTrajectory')
        obj.V.S = fasteval(obj.V.S, 0);
      end
      obj.LIP_height = biped.default_walking_params.nominal_LIP_COM_height;
      obj.gain_set = 'walking';
    end

    function obj = from_point_mass_biped_plan(plan, biped, x0, param_set_name)
      error('this needs to be updated to use exponential map rotations');
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

    function obj = from_quasistatic_qtraj(biped, qtraj, options)
      if nargin < 3
        options = struct();
      end
      options = applyDefaults(options, struct('bodies_to_track', [biped.findLinkId('pelvis'),...
                                                                  biped.foot_body_id.right,...
                                                                  biped.foot_body_id.left],...
                                              'bodies_to_control_when_in_contact', biped.findLinkId('pelvis')));

      if(~isfield(options,'zero_final_acceleration'))
        options.zero_final_acceleration = false;
      end
      % handle the case where qtraj is a constant trajectory
      if isa(qtraj,'ConstantTrajectory')
        q0 = qtraj.eval(0);
        qtraj = PPTrajectory(zoh([0, inf], [q0, q0]));
      end
      ts = qtraj.getBreaks();
      if(options.zero_final_acceleration)
        % Append a zero-order-hold trajectory at the end
        qtraj_pp = qtraj.pp;
        
        ts = [ts ts(end)+0.05];
        qtraj_coefs = reshape(qtraj_pp.coefs,[qtraj_pp.dim,qtraj_pp.pieces,qtraj_pp.order]);
        qtraj_coefs = cat(2,qtraj_coefs,reshape([zeros(qtraj_pp.dim,qtraj_pp.order-1) qtraj_coefs(:,end,end)],[qtraj_pp.dim,1,qtraj_pp.order]));
        qtraj_pp = mkpp(ts,reshape(qtraj_coefs,[],qtraj_pp.order),qtraj_pp.dim);
        qtraj = PPTrajectory(qtraj_pp);
      end
      q0 = qtraj.eval(qtraj.tspan(1));
      x0 = [q0; zeros(biped.getNumVelocities(), 1)];
      obj = QPLocomotionPlan.from_standing_state(x0, biped);
      obj.qtraj = qtraj;
      obj.duration = obj.qtraj.tspan(end) - obj.qtraj.tspan(1);
      obj.support_times = [obj.qtraj.tspan(1); inf];

      if isfield(options,'supports') && isfield(options,'support_times')
        obj.supports = options.supports;
        obj.support_times = options.support_times;
      end

      for j = 1:length(options.bodies_to_track)
        if options.bodies_to_track(j) == biped.findLinkId('r_hand')
          obj.constrained_dofs = setdiff(obj.constrained_dofs, findPositionIndices(obj.robot,'r_arm'));
        elseif options.bodies_to_track(j) == biped.findLinkId('l_hand')
          obj.constrained_dofs = setdiff(obj.constrained_dofs, findPositionIndices(obj.robot,'l_arm'));
        end
      end

      
      body_poses = zeros([7, length(ts), length(options.bodies_to_track)]);
      body_velocity = zeros([7,length(ts), length(options.bodies_to_track)]);
      body_xyzexpmap = zeros([6, length(ts), length(options.bodies_to_track)]);
      body_xyzexpmap_dot = zeros([6, length(ts), length(options.bodies_to_track)]);
      for i = 1:numel(ts)
        qi = qtraj.eval(ts(i));
        vi = qtraj.deriv(ts(i));
        kinsol = doKinematics(obj.robot,qi);
        for j = 1:numel(options.bodies_to_track)
          [body_poses(:,i,j),Jij] = obj.robot.forwardKin(kinsol, options.bodies_to_track(j), [0;0;0], 2);
          body_velocity(:,i,j) = Jij*vi;
          body_xyzexpmap(1:3,i,j) = body_poses(1:3,i,j);
          body_xyzexpmap_dot(1:3,i,j) = body_velocity(1:3,i,j);
        end
      end
      for j = 1:numel(options.bodies_to_track)
        [body_xyzexpmap(4:6,:,j),body_xyzexpmap_dot(4:6,:,j)] = quat2expmapSequence(body_poses(4:7,:,j),body_velocity(4:7,:,j));
      end

      obj.body_motions = BodyMotionData.empty();
      for j = 1:numel(options.bodies_to_track)
        obj.body_motions(j) = BodyMotionData.from_body_xyzexp_and_xyzexpdot(options.bodies_to_track(j), ts, body_xyzexpmap(:,:,j), body_xyzexpmap_dot(:,:,j));
        if ismember(options.bodies_to_track(j), options.bodies_to_control_when_in_contact)
          obj.body_motions(j).control_pose_when_in_contact = true(1, numel(obj.body_motions(j).ts));
        end
      end

      obj.gain_set = 'manip';
      obj = obj.setCOMTraj();
      obj = obj.setLQR_for_COM();
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
