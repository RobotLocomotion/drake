classdef QPLocomotionPlanSettings
  properties
    robot;
    contact_groups;
    support_times
    supports;
    body_motions;
    zmp_data;
    zmptraj = [];
    zmp_final = [];
    LIP_height;
    V;
    qtraj;
    comtraj = [];
    mu = 0.5;
    plan_shift = zeros(6,1);
    plan_shift_zmp_inds = 1:2;
    plan_shift_body_motion_inds = 3;
    g = 9.81; % gravity m/s^2
    is_quasistatic = false;
    constrained_dofs = [];
    untracked_joint_inds;

    planned_support_command = QPControllerPlan.support_logic_maps.require_support; % when the plan says a given body is in support, require the controller to use that support. To allow the controller to use that support only if it thinks the body is in contact with the terrain, try QPControllerPlan.support_logic_maps.kinematic_or_sensed; 

    min_knee_angle = 0.7;
    knee_kp = 40;
    knee_kd = 4;
    knee_weight = 1;

    pelvis_name = 'pelvis';
    r_foot_name = 'r_foot';
    l_foot_name = 'l_foot';

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

    function obj = setLQRForCoM(obj)
      Q = diag([10 10 1 1]);
      R = 0.0001*eye(2);
      A = [zeros(2),eye(2); zeros(2,4)];
      B = [zeros(2); eye(2)];
      [~,S,~] = lqr(A,B,Q,R);
      % set the Qy to zero since we only want to stabilize COM
      obj.zmp_data.Qy = 0*obj.default_qp_input.zmp_data.Qy;
      obj.zmp_data.A = A;
      obj.zmp_data.B = B;
      obj.zmp_data.R = R;
      obj.zmp_data.S = S;
    end
    
  end

  methods(Static)
    function obj = fromStandingState(x0, biped, support_state, options)

      if nargin < 3 || isempty(support_state)
        support_state = RigidBodySupportState(biped, [biped.foot_body_id.right, biped.foot_body_id.left]);
      end
      if nargin < 4
        options = struct();
      end
      options = applyDefaults(options, struct('center_pelvis', true));

      obj = QPLocomotionPlanSettings(biped);
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
      options = applyDefaults(options, struct('pelvis_height_above_sole', biped.default_walking_params.pelvis_height_above_foot_sole));
      if isempty(options.pelvis_height_above_sole)
        kinsol = doKinematics(biped, x0(1:biped.getNumPositions()));
        pelvis_pos = forwardKin(biped, kinsol, biped.findLinkId('pelvis'), [0;0;0]);
        feetPosition = biped.feetPosition(x0(1:biped.getNumPositions()));
        options.pelvis_height_above_sole = pelvis_pos(3) - mean([feetPosition.right(3), feetPosition.left(3)]);
      end

      obj = QPLocomotionPlanSettings(biped);
      arm_inds = biped.findPositionIndices('arm');
      obj.qtraj(arm_inds) = x0(arm_inds);
      % obj.qtraj = x0(1:biped.getNumPositions());

      [obj.supports, obj.support_times] = QPLocomotionPlanSettings.getSupports(zmp_knots);
      obj.zmptraj = QPLocomotionPlanSettings.getZMPTraj(zmp_knots);
      [~, obj.V, obj.comtraj, ~] = biped.planZMPController(obj.zmptraj, x0, options);
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
                                              'track_com_traj',false));

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
      obj = QPLocomotionPlanSettings.fromStandingState(x0, biped);
      obj.qtraj = qtraj;
      obj.duration = obj.qtraj.tspan(end) - obj.qtraj.tspan(1);
      obj.support_times = [obj.qtraj.tspan(1); inf];

      if isfield(options,'supports') && isfield(options,'support_times')
        obj.supports = options.supports;
        obj.support_times = options.support_times;
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

      obj.gain_set = 'manip';
      if(options.track_com_traj)
        obj = obj.setCOMTraj();
        obj = obj.setLQRForCoM();
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
        'D', -0.89/9.81*eye(2),... % ZMP input-output map 2x2
        'x0', zeros(4,1),... % nominal state 4x1
        'y0', zeros(2,1),... % nominal output 2x1
        'u0', zeros(2,1),... % nominal input 2x1
        'R', zeros(2),... % input LQR cost 2x2
        'Qy', 0.8*eye(2),... % output LQR cost 2x2
        'S', zeros(4),... % cost-to-go terms: x'Sx + x's1 + s2 [4x4]
        's1', zeros(4,1),... % 4x1
        's1dot', zeros(4,1),... % 4x1
        's2', 0,... % 1x1
        's2dot', 0); % 1x1
    end
  end
end
