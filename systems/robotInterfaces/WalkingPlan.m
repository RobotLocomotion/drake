classdef WalkingPlan < QPControllerPlan
% Container for the results of the ZMP walking planning, which can be consumed by planWalkingStateTraj
% to generate the full walking motion.
  properties
    robot
    x0
    support_times
    supports
    link_constraints
    zmptraj
    zmp_final
    V
    qstar = [];
    c = [];
    comtraj
    mu=1;
    t_offset=0;
    ignore_terrain=false;
    gain_set='walking';
    xyz_shift=[0;0;0]; 
    breaking_contact_max_time = 0.25; % for how long after leaving support should we tell the controller to break contact
  end

  methods(Static)
    function obj = from_standing_state(x0, biped, support_state)

      if nargin < 3
        support_state = RigidBodySupportState(biped, [biped.foot_body_id.right, biped.foot_body_id.left]);
      end

      obj = WalkingPlan(biped);
      obj.x0 = x0;
      obj.support_times = [0, inf];
      obj.supports = [support_state, support_state];

      nq = obj.robot.getNumPositions();
      q0 = x0(1:nq);
      kinsol = doKinematics(obj.robot, q0);

      foot_pos = [obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.right, [0;0;0]),...
                  obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.left, [0;0;0])];
      comgoal = mean(foot_pos(1:2,:), 2);

      obj.zmptraj = comgoal;
      [~, obj.V, obj.comtraj] = obj.robot.planZMPController(comgoal, q0);
      assert(isa(obj.V.S, 'ConstantTrajectory'))

      obj.V.S = fasteval(obj.V.S, 0);

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
      obj.qstar = x0(1:nq);
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
      obj = WalkingPlan.from_biped_foot_and_zmp_knots(foot_origin_knots, zmp_knots, biped, x0);
    end

    function obj = from_biped_foot_and_zmp_knots(foot_origin_knots, zmp_knots, biped, x0)
      [supports, support_times] = WalkingPlan.getSupports(zmp_knots);
      zmptraj = WalkingPlan.getZMPTraj(zmp_knots);
      link_constraints = biped.getLinkConstraints(foot_origin_knots, zmptraj, supports, support_times);
      [c, V, comtraj] = biped.planZMPController(zmptraj, x0);

      obj = WalkingPlan(biped);
      obj.x0 = x0;
      obj.support_times = support_times;
      obj.supports = supports;
      obj.link_constraints = link_constraints;
      obj.zmptraj = zmptraj;
      obj.zmp_final = zmptraj.eval(zmptraj.tspan(end));
      if isa(V.S, 'ConstantTrajectory')
        V.S = fasteval(V.S, 0);
      end
      obj.V = V;
      obj.c = c;
      obj.comtraj = comtraj;
    end

    function obj = from_point_mass_biped_plan(plan, biped, x0)
      typecheck(biped, 'Biped');
      typecheck(plan, 'PointMassBipedPlan');

      foot_start = biped.feetPosition(x0(1:biped.getNumPositions()));
      % contact = plan.getContactSequence();
      % contained = plan.contained;
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

      obj = WalkingPlan.from_biped_foot_and_zmp_knots(foot_origin_knots, zmp_knots, biped, x0);
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


  methods
    function obj = WalkingPlan(robot)
      obj = obj@QPControllerPlan();
      obj.robot = robot;
      S = load(obj.robot.fixed_point_file);
      obj.qstar = S.xstar(1:obj.robot.getNumPositions());
      obj.default_qp_input = atlasControllers.QPInput2D();
      obj.default_qp_input.whole_body_data.q_des = zeros(obj.robot.getNumPositions(), 1);
      obj.default_qp_input.whole_body_data.constrained_dof_mask = false(obj.robot.getNumPositions(), 1);
    end

    function obj = buildLinkTrajectories(obj)
      % By default, link_constraints just stores polynomial coefficients, rather than full trajectories.
      % This function returns a new WalkingPlan with link_constraints augmented to contain a traj
      % field, which holds a PPTrajectory of that link's motion.
      for j = 1:length(obj.link_constraints)
        obj.link_constraints(j).traj = PPTrajectory(mkpp(obj.link_constraints(j).ts, obj.link_constraints(j).coefs, size(obj.link_constraints(j).coefs, 1)));
      end
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

    function next_plan = getSuccessor(obj, t, x)
      next_plan = StandingPlan.from_standing_state(x, obj.robot);
    end

    function qp_input = getQPControllerInput(obj, t, x, rpc)
      % Get the input structure which can be passed to the stateless QP control loop
      % @param t the current time
      % @param x the current robot state
      % @param rpc the robot property cache, which lets us quickly look up info about
      % the robot which would be expensive to compute (such as terrain contact points)

      r = obj.robot;
      t = t - obj.start_time;
      
      T = obj.support_times(end);
      t = min([t, T]);

      qp_input = obj.default_qp_input;
      qp_input.zmp_data.x0 = [obj.zmp_final + obj.xyz_shift(1:2); 0;0];
      if isnumeric(obj.zmptraj)
        qp_input.zmp_data.y0 = obj.zmptraj + obj.xyz_shift(1:2);
      else
        qp_input.zmp_data.y0 = fasteval(obj.zmptraj, t) + obj.xyz_shift(1:2);
      end
      qp_input.zmp_data.S = obj.V.S;
      if isnumeric(obj.V.s1)
        qp_input.zmp_data.s1 = obj.V.s1;
      else
        qp_input.zmp_data.s1 = fasteval(obj.V.s1,t);
      end

      supp_idx = find(obj.support_times<=t,1,'last');
      supp = obj.supports(supp_idx);

      qp_input.support_data = struct('body_id', {r.foot_body_id.right, r.foot_body_id.left},...
                                     'contact_pts', {...
                               [rpc.contact_groups{r.foot_body_id.right}.heel, ...
                                rpc.contact_groups{r.foot_body_id.right}.toe],...
                               [rpc.contact_groups{r.foot_body_id.left}.heel,...
                                rpc.contact_groups{r.foot_body_id.left}.toe]},...
                                     'support_logic_map', {false(4,1), false(4,1)},...
                                     'mu', {obj.mu, obj.mu},...
                                     'contact_surfaces', {0,0});

      if any(supp.bodies==r.foot_body_id.right)
        r.warning_manager.warnOnce('Drake:HardCodedSupport', 'hard-coded for heel+toe support');
        qp_input.support_data(1).support_logic_map = obj.support_logic_maps.kinematic_or_sensed;
      else
        if supp_idx > 1 && any(obj.supports(supp_idx-1).bodies==r.foot_body_id.right)
          if t - obj.support_times(supp_idx) <= obj.breaking_contact_max_time
            qp_input.support_data(1).support_logic_map = obj.support_logic_maps.prevent_support;
          end
        end
      end
      if any(supp.bodies==r.foot_body_id.left)
        r.warning_manager.warnOnce('Drake:HardCodedSupport', 'hard-coded for heel+toe support');
        qp_input.support_data(2).support_logic_map = obj.support_logic_maps.kinematic_or_sensed;
      else
        if supp_idx > 1 && any(obj.supports(supp_idx-1).bodies==r.foot_body_id.left)
          if t - obj.support_times(supp_idx) <= obj.breaking_contact_max_time
            qp_input.support_data(2).support_logic_map = obj.support_logic_maps.prevent_support;
          end
        end
      end

      qp_input.whole_body_data.q_des = obj.qstar;
      qp_input.whole_body_data.constrained_dof_mask(rpc.neck_id) = true;

      feet_poses = zeros(6,2);
      i = 1;
      tracked_bodies = 0;
      for j = 1:length(obj.link_constraints)
        qp_input.body_motion_data(j).body_id = obj.link_constraints(j).link_ndx;
        body_t_ind = find(obj.link_constraints(j).ts<=t,1,'last');
        if body_t_ind < length(obj.link_constraints(j).ts)
          qp_input.body_motion_data(j).ts = obj.link_constraints(j).ts(body_t_ind:body_t_ind+1);
        else
          qp_input.body_motion_data(j).ts = obj.link_constraints(j).ts([body_t_ind,body_t_ind]);
        end
        qp_input.body_motion_data(j).coefs = obj.link_constraints(j).coefs(:,body_t_ind,:);
        qp_input.body_motion_data(j).coefs(1:3,1,end) = qp_input.body_motion_data(j).coefs(1:3,1,end) + obj.xyz_shift;
        if any(qp_input.body_motion_data(j).body_id == [r.foot_body_id.right, r.foot_body_id.left])
          feet_poses(:,i) = evalCubicSplineSegment(t - qp_input.body_motion_data(j).ts(1), qp_input.body_motion_data(j).coefs);
          i = i + 1;
        end
        tracked_bodies = tracked_bodies + 1;
      end

      if tracked_bodies == 2
        r.warning_manager.warnOnce('Drake:HardCodedPelvisheight', 'hard-coding pelvis height');
        r.warning_manager.warnOnce('Drake:NoPelvisHeightLogic', 'not using pelvis height logic');
        pelvis_target = [mean(feet_poses(1:2,:), 2); min(feet_poses(3,:)) + 0.74; 0; 0; angleAverage(feet_poses(6,1), feet_poses(6,2))];
        coefs = reshape(pelvis_target, [6, 1, 1]);
        coefs = cat(3, zeros(6, 1, 3), coefs);
        qp_input.body_motion_data(tracked_bodies+1).body_id = rpc.pelvis_body_id;
        qp_input.body_motion_data(tracked_bodies+1).ts = [t, t];
        qp_input.body_motion_data(tracked_bodies+1).coefs = coefs;
      else
        assert(tracked_bodies == 3, 'expecting 2 or 3 tracked bodies here');
      end
      qp_input.param_set_name = obj.gain_set;

    end

    function obj = fix_link(obj, biped, kinsol, link, pt, tolerance_xyz, tolerance_rpy)
      % Add a new link constraint which fixes the point pt on the given link to its
      % pose as given in kinsol, within the tolerance specified.
      pos = biped.forwardKin(kinsol, link, pt, 1);
      pos_min = pos - [tolerance_xyz; tolerance_rpy];
      pos_max = pos + [tolerance_xyz; tolerance_rpy];
      obj.link_constraints(end+1) = struct('link_ndx', link, 'pt', pt, 'min_traj', ConstantTrajectory(pos_min), 'max_traj', ConstantTrajectory(pos_max));
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
          obj.link_constraints(j).traj = PPTrajectory(mkpp(obj.link_constraints(j).ts, cat(3, obj.link_constraints(j).a3, obj.link_constraints(j).a2, obj.link_constraints(j).a1, obj.link_constraints(j).a0), 6));
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
end


