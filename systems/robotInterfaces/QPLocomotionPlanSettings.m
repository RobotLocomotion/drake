classdef QPLocomotionPlanSettings
  properties
    robot;
    contact_groups;
    support_times
    supports;
    body_motions;
    zmp_data;
    zmptraj = [];
    V;
    qtraj;
    comtraj = [];
    mu = 0.7;
    D_control;
    use_plan_shift = false;
    plan_shift_body_motion_inds = 3;
    g = 9.81; % gravity m/s^2
    is_quasistatic = false;
    constrained_dofs = [];
    untracked_joint_inds;
    x0;

    planned_support_command = QPControllerPlan.support_logic_maps.require_support; % when the plan says a given body is in support, require the controller to use that support. To allow the controller to use that support only if it thinks the body is in contact with the terrain, try QPControllerPlan.support_logic_maps.kinematic_or_sensed; 

    min_knee_angle = 0.7;
    ankle_limits_tolerance = 0.2;
    knee_kp = 40;
    knee_kd = 4;
    knee_weight = 1;
    zmp_safety_margin = 0.005;

    % If a body is about to come into contact, then, after this fraction of the
    % duration of the current support, add an optional support that the
    % controller can use if it senses force on that body
    early_contact_allowed_fraction = 0.75; 

    pelvis_name = 'pelvis';
    r_foot_name = 'r_foot';
    l_foot_name = 'l_foot';
    r_knee_name = 'r_leg_kny';
    l_knee_name = 'l_leg_kny';
    l_akx_name = 'l_leg_akx';
    r_akx_name = 'r_leg_akx';
    r_aky_name = 'r_leg_aky';
    l_aky_name = 'l_leg_aky';

    duration = inf;
    start_time = 0;
    default_qp_input = atlasControllers.QPInputConstantHeight;
    gain_set = 'standing';
  end

  methods
    function obj = QPLocomotionPlanSettings(robot)
      obj.robot = robot;
      S = load(obj.robot.fixed_point_file);
      rpc = atlasUtil.propertyCache(obj.robot);
      obj.contact_groups = rpc.contact_groups;
      obj.qtraj = S.xstar(1:obj.robot.getNumPositions());
      obj.default_qp_input = atlasControllers.QPInputConstantHeight();
      obj.default_qp_input.whole_body_data.q_des = zeros(obj.robot.getNumPositions(), 1);
      obj.constrained_dofs = [findPositionIndices(obj.robot,'arm');findPositionIndices(obj.robot,'neck');findPositionIndices(obj.robot,'back_bkz');findPositionIndices(obj.robot,'back_bky')];
      obj.untracked_joint_inds = [];
      obj.zmp_data = QPLocomotionPlanSettings.defaultZMPData();
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

    function obj = setLQRForCoM(obj)
      error('this has never been properly tuned on the robot and should not be used yet')
      Q = diag([10 10 1 1]);
      R = 0.0001*eye(2);
      A = [zeros(2),eye(2); zeros(2,4)];
      B = [zeros(2); eye(2)];
      [~,S,~] = lqr(A,B,Q,R);
      % set the Qy to zero since we only want to stabilize COM
      obj.zmp_data.Qy = 0*obj.zmp_data.Qy;
      obj.zmp_data.A = A;
      obj.zmp_data.B = B;
      obj.zmp_data.R = R;
      obj.zmp_data.S = S;
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
    
  end

  methods(Static)
    function obj = fromStandingState(x0, biped, support_state, options)

      if nargin < 3 || isempty(support_state)
        support_state = RigidBodySupportState(biped, [biped.foot_body_id.right, biped.foot_body_id.left], struct('contact_groups', {{{'heel', 'toe'}, {'heel', 'toe'}}}));
      end
      if nargin < 4
        options = struct();
      end
      options = applyDefaults(options, struct('center_pelvis', true));

      obj = QPLocomotionPlanSettings(biped);
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
        active_contact_pts = zeros(3,0);
        for j = 1:length(support_state.bodies)
          active_contact_pts = [active_contact_pts, obj.robot.forwardKin(kinsol, support_state.bodies(j), support_state.contact_pts{j}, 0)];
        end
        comgoal = mean(active_contact_pts(1:2,:), 2);
        pelvis_target_xyzquat = [comgoal; pelvis_current_xyzquat(3:end)];
      else
        comgoal = obj.robot.getCOM(kinsol);
        comgoal = comgoal(1:2);
        pelvis_target_xyzquat = pelvis_current_xyzquat;
      end

      obj.zmptraj = comgoal;
      [~, obj.V, ~, LIP_height] = obj.robot.planZMPController(comgoal, q0);
      obj.zmp_data.D = -LIP_height / obj.g * eye(2);
      obj.D_control = -obj.robot.default_walking_params.nominal_LIP_COM_height / obj.g * eye(2);

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

      obj.qtraj = x0(1:nq);
      obj.comtraj = comgoal;
      obj.gain_set = 'standing';
    end

    function obj = fromBipedFootstepPlan(footstep_plan, biped, x0, zmp_options)
      if nargin < 4
        zmp_options = struct();
      end
      for j = 1:length(footstep_plan.footsteps)
        footstep_plan.footsteps(j).walking_params = applyDefaults(struct(footstep_plan.footsteps(j).walking_params),...
          biped.default_walking_params);
      end
      [zmp_knots, foot_motion_data] = biped.planZMPTraj(x0(1:biped.getNumPositions()), footstep_plan.footsteps, zmp_options);
      obj = QPLocomotionPlanSettings.fromBipedFootAndZMPKnots(foot_motion_data, zmp_knots, biped, x0);
    end

    function obj = fromBipedFootAndZMPKnots(foot_motion_data, zmp_knots, biped, x0, options)
      if nargin < 5
        options = struct();
      end
      options = applyDefaults(options, struct('pelvis_height_above_sole', biped.default_walking_params.pelvis_height_above_foot_sole,...
        'pelvis_height_transition_knot',1));

      obj = QPLocomotionPlanSettings(biped);
      obj.x0 = x0;
      arm_inds = biped.findPositionIndices('arm');
      obj.qtraj(arm_inds) = x0(arm_inds);
      % obj.qtraj = x0(1:biped.getNumPositions());

      [obj.supports, obj.support_times] = QPLocomotionPlanSettings.getSupports(zmp_knots);
      obj.zmptraj = QPLocomotionPlanSettings.getZMPTraj(zmp_knots);
      [~, obj.V, obj.comtraj, LIP_height] = biped.planZMPController(obj.zmptraj, obj.x0, options);
      obj.D_control = -biped.default_walking_params.nominal_LIP_COM_height / obj.g * eye(2);
      obj.zmp_data.D = -LIP_height / obj.g * eye(2);
      pelvis_motion_data = biped.getPelvisMotionForWalking(x0, foot_motion_data, obj.supports, obj.support_times, options);
      obj.body_motions = [foot_motion_data, pelvis_motion_data];

      obj.duration = obj.support_times(end)-obj.support_times(1)-0.001;
      if isa(obj.V.S, 'ConstantTrajectory')
        obj.V.S = fasteval(obj.V.S, 0);
      end
      obj.gain_set = 'walking';
      obj.use_plan_shift = true;
    end

    function obj = fromQuasistaticQTraj(biped, qtraj, options)
      if nargin < 3
        options = struct();
      end
      options = applyDefaults(options, struct('bodies_to_track', [biped.findLinkId('pelvis'),...
                                                                  biped.foot_body_id.right,...
                                                                  biped.foot_body_id.left],...
                                              'quat_task_to_world',repmat([1;0;0;0],1,3),...
                                              'translation_task_to_world',zeros(3,3),...
                                              'bodies_to_control_when_in_contact', biped.findLinkId('pelvis'),...
                                              'is_quasistatic',false,...
                                              'supports', RigidBodySupportState(biped, [biped.foot_body_id.right, biped.foot_body_id.left]),...
                                              'support_times',[0, inf],...
                                              'gain_set', 'manip'));

      num_bodies_to_track = length(options.bodies_to_track);
      sizecheck(options.quat_task_to_world,[4,num_bodies_to_track]);
      sizecheck(options.translation_task_to_world,[3,num_bodies_to_track]);
      
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
      obj = QPLocomotionPlanSettings(biped);
      obj.x0 = x0;
      obj.qtraj = qtraj;
      obj.duration = obj.qtraj.tspan(end) - obj.qtraj.tspan(1);
      obj.support_times = options.support_times;
      obj.supports = options.supports;
      obj.is_quasistatic = options.is_quasistatic;
      obj.gain_set = options.gain_set;

      obj = obj.setCOMTraj();
      if ~obj.is_quasistatic
        support_state = obj.supports(1);
        for j = 1:length(obj.supports)
          if any(obj.supports(j).bodies ~= support_state.bodies)
            error('We don''t know how to construct non-quasistatic manipulation plans in which the active supports change. Try going slowly and setting options.is_quasistatic to true, or just make a walking plan.')
          end
        end
        active_contact_pts = zeros(3,0);
        kinsol = obj.robot.doKinematics(q0);
        for j = 1:length(support_state.bodies)
          active_contact_pts = [active_contact_pts, obj.robot.forwardKin(kinsol, support_state.bodies(j), support_state.contact_pts{j}, 0)];
        end
        comgoal = mean(active_contact_pts(1:2,:), 2);
        obj.zmptraj = comgoal;
        [~, obj.V, ~, LIP_height] = obj.robot.planZMPController(comgoal, x0(1:obj.robot.getNumPositions()));
        obj.zmp_data.D = -LIP_height / obj.g * eye(2);
        obj.D_control = -obj.robot.default_walking_params.nominal_LIP_COM_height / obj.g * eye(2);
      else
        % We can just use a comgoal of [0;0] here because, for the quasistatic solution, it has no effect on V
        [~, obj.V, ~, LIP_height] = obj.robot.planZMPController([0;0], x0(1:obj.robot.getNumPositions()));
        obj.zmptraj = obj.comtraj;
        obj.zmp_data.D = -LIP_height / obj.g * eye(2);
        obj.D_control = -obj.robot.default_walking_params.nominal_LIP_COM_height / obj.g * eye(2);
      end

      for j = 1:num_bodies_to_track
        if options.bodies_to_track(j) == biped.findLinkId('r_hand')
          obj.constrained_dofs = setdiff(obj.constrained_dofs, findPositionIndices(obj.robot,'r_arm'));
        elseif options.bodies_to_track(j) == biped.findLinkId('l_hand')
          obj.constrained_dofs = setdiff(obj.constrained_dofs, findPositionIndices(obj.robot,'l_arm'));
        end
      end
      
      body_poses = zeros([7, length(ts), num_bodies_to_track]);
      body_velocity = zeros([7,length(ts), num_bodies_to_track]);
      body_xyzexpmap = zeros([6, length(ts), num_bodies_to_track]);
      body_xyzexpmap_dot = zeros([6, length(ts), num_bodies_to_track]);
      body_R_world_to_task = zeros(3, 3, num_bodies_to_track);
      body_translation_world_to_task = zeros(3, num_bodies_to_track);
      for i = 1:length(options.bodies_to_track)
        body_R_world_to_task(:,:,i) = quat2rotmat(quatConjugate(options.quat_task_to_world(:,i)));
        body_translation_world_to_task(:,i) = -body_R_world_to_task(:,:,i)*options.translation_task_to_world(:,i);
      end
      
      for i = 1:numel(ts)
        qi = qtraj.eval(ts(i));
        vi = qtraj.deriv(ts(i));
        kinsol = doKinematics(obj.robot,qi);
        for j = 1:num_bodies_to_track
          [body_poses(:,i,j),Jij] = obj.robot.forwardKin(kinsol, options.bodies_to_track(j), [0;0;0], 2);
          body_velocity(:,i,j) = Jij*vi;
          body_poses(1:3,i,j) = body_R_world_to_task(:,:,j)*body_poses(1:3,i,j)+body_translation_world_to_task(:,j);
          body_poses(4:7,i,j) = quatProduct(quatConjugate(options.quat_task_to_world(:,j)),body_poses(4:7,i,j));
          body_velocity(1:3,i,j) = body_R_world_to_task(:,:,j)*body_velocity(1:3,i,j);
          body_velocity(4:7,i,j) = quatProduct(quatConjugate(options.quat_task_to_world(:,j)),body_velocity(4:7,i,j));
          body_xyzexpmap(1:3,i,j) = body_poses(1:3,i,j);
          body_xyzexpmap_dot(1:3,i,j) = body_velocity(1:3,i,j);
        end
      end
      for j = 1:num_bodies_to_track
        [body_xyzexpmap(4:6,:,j),body_xyzexpmap_dot(4:6,:,j)] = quat2expmapSequence(body_poses(4:7,:,j),body_velocity(4:7,:,j));
      end

      obj.body_motions = BodyMotionData.empty();
      for j = 1:numel(options.bodies_to_track)
        obj.body_motions(j) = BodyMotionData.from_body_xyzexp_and_xyzexpdot(options.bodies_to_track(j), ts, body_xyzexpmap(:,:,j), body_xyzexpmap_dot(:,:,j));
        obj.body_motions(j).quat_task_to_world = options.quat_task_to_world(:,j);
        obj.body_motions(j).translation_task_to_world = options.translation_task_to_world(:,j);
        if ismember(options.bodies_to_track(j), options.bodies_to_control_when_in_contact)
          obj.body_motions(j).control_pose_when_in_contact = true(1, numel(obj.body_motions(j).ts));
        end
      end

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
  
  methods (Static, Access=private)
    function zmp_data = defaultZMPData()
      zmp_data = struct('A',  [zeros(2),eye(2); zeros(2,4)],... % COM state map 4x4
        'B', [zeros(2); eye(2)],... % COM input map 4x2
        'C', [eye(2),zeros(2)],... % ZMP state-output map 2x4
        'u0', zeros(2,1),... % nominal input 2x1
        'R', zeros(2),... % input LQR cost 2x2
        'Qy', eye(2));
    end
  end
end
