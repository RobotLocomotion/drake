classdef Valkyrie < TimeSteppingRigidBodyManipulator & Biped
  methods

    function obj=Valkyrie(urdf,options)
      typecheck(urdf,'char');

      if nargin < 2
        options = struct();
      end
      if ~isfield(options,'dt')
        options.dt = 0.001;
      end
      if ~isfield(options,'floating')
        options.floating = true;
      end
      if ~isfield(options,'terrain')
        options.terrain = RigidBodyFlatTerrain;
      end
      if ~isfield(options,'external_force')
        options.external_force = [];
      end

      w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped('r_foot_sole', 'l_foot_sole');
      warning(w);

      % Add a force on a specified link if we want!
      if ~isempty(options.external_force)
        % For compile purposes, record that we have external force applied to a link
        % (this affects input frames)
        obj.external_force = findLinkId(obj,options.external_force);
        options_ef.weld_to_link = obj.external_force;
        obj = obj.addRobotFromURDF(fullfile(getDrakePath,'matlab','util','three_dof_force.urdf'), ...
          [0; 0; 0], [0; 0; 0], options_ef);
      end

      if options.floating
        % could also do fixed point search here
        obj = obj.setInitialState(obj.resolveConstraints(zeros(obj.getNumStates(),1)));
      else
        % TEMP HACK to get by resolveConstraints
        for i=1:length(obj.manip.body), obj.manip.body(i).contact_pts=[]; end
        obj.manip = compile(obj.manip);
        obj = obj.setInitialState(zeros(obj.getNumStates(),1));
      end

      lastwarn = warning('off', 'Drake:RigidBodySupportState:NoSupportSurface');
      obj.left_full_support = RigidBodySupportState(obj,obj.foot_body_id.left);
      obj.left_toe_support = RigidBodySupportState(obj,obj.foot_body_id.left,struct('contact_groups',{{'toe'}}));
      obj.right_full_support = RigidBodySupportState(obj,obj.foot_body_id.right);
      obj.right_toe_support = RigidBodySupportState(obj,obj.foot_body_id.right,struct('contact_groups',{{'toe'}}));
      obj.left_full_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right]);
      obj.left_toe_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],struct('contact_groups',{{{'toe'},{'heel','toe'}}}));
      obj.left_full_right_toe_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],struct('contact_groups',{{{'heel','toe'},{'toe'}}}));
      warning(lastwarn);
    end

    function obj = compile(obj)
      obj = compile@TimeSteppingRigidBodyManipulator(obj);

      % Construct state vector itself -- start by replacing the
      % valkyriePosition and valkyrieVelocity frames with a single
      % larger state frame
      if (strcmp(obj.manip.getStateFrame().getFrameByNum(1).name, 'valkyriePosition'))
        valkyrie_state_frame = valkyrieFrames.ValkyrieState(obj);
      else
        valkyrie_state_frame = obj.manip.getStateFrame();
        valkyrie_state_frame = replaceFrameNum(valkyrie_state_frame,1,valkyrieFrames.ValkyrieState(obj));
      end

      tsmanip_state_frame = obj.getStateFrame();
      if tsmanip_state_frame.dim>valkyrie_state_frame.dim
        tsmanip_state_frame.frame{1} = valkyrie_state_frame;
        state_frame = tsmanip_state_frame;
      else
        state_frame = valkyrie_state_frame;
      end
      obj.manip = obj.manip.setStateFrame(valkyrie_state_frame);
      obj = obj.setStateFrame(state_frame);

      % Same bit of complexity for input frame to get hand inputs
      if (obj.external_force > 0)
        input_frame = getInputFrame(obj);
        input_frame  = replaceFrameNum(input_frame,1,valkyrieFrames.ValkyrieInput(obj));
      else
        input_frame = valkyrieFrames.ValkyrieInput(obj);
      end

      obj = obj.setInputFrame(input_frame);
      obj.manip = obj.manip.setInputFrame(input_frame);

      % Construct output frame, which comes from state plus sensor
      % info
      valkyrie_output_frame = valkyrie_state_frame;
      if (~isempty(obj.manip.sensor))
        for i=1:length(obj.manip.sensor)
          % If it's not a full state feedback sensor (we have already
          % got the state for that above in the state frame
          if (~isa(obj.manip.sensor{i}, 'FullStateFeedbackSensor'))
            if (isa(valkyrie_output_frame, 'MultiCoordinateFrame'))
              valkyrie_output_frame = valkyrie_output_frame.appendFrame(obj.manip.sensor{i}.constructFrame(obj.manip));
            else
              valkyrie_output_frame = MultiCoordinateFrame({valkyrie_output_frame, obj.manip.sensor{i}.constructFrame(obj.manip)});
            end
          end
        end
      end
      % The output function of a TSRBM appends the TS sensors to the
      % output of the RBM. So get ready for that:
      output_frame = valkyrie_output_frame;
      if (~isempty(obj.sensor))
        for i=1:length(obj.sensor)
          if (~isa(obj.sensor{i}, 'FullStateFeedbackSensor'))
            if (isa(output_frame, 'MultiCoordinateFrame'))
              output_frame = output_frame.appendFrame(obj.sensor{i}.constructFrame(obj));
            else
              output_frame = MultiCoordinateFrame({output_frame, obj.sensor{i}.constructFrame(obj)});
            end
          end
        end
      end

      if ~isequal_modulo_transforms(valkyrie_output_frame,getOutputFrame(obj.manip))
        obj.manip = obj.manip.setNumOutputs(valkyrie_output_frame.dim);
        obj.manip = obj.manip.setOutputFrame(valkyrie_output_frame);
      end

      if ~isequal_modulo_transforms(output_frame,getOutputFrame(obj))
        obj = obj.setNumOutputs(output_frame.dim);
        obj = obj.setOutputFrame(output_frame);
      end
    end

    function obj = setInitialState(obj,x0)
      if isa(x0,'Point')
        obj.x0 = double(x0); %.inFrame(obj.getStateFrame));
      else
        typecheck(x0,'double');
        sizecheck(x0,obj.getNumStates());
        obj.x0 = x0;
      end
    end

    function x0 = getInitialState(obj)
      x0 = obj.x0;
    end

    function weights = getFootstepOptimizationWeights(obj)
      % Return a reasonable set of default weights for the footstep planner
      % optimization. The weights describe the following quantities:
      % 'relative': the contribution to the cost function of the
      %             displacement from one step to the next
      % 'relative_final': the cost contribution of the displacement of the
      %                   displacement of the very last step (this can be
      %                   larger than the normal 'relative' cost in
      %                   order to encourage the feet to be close together
      %                   at the end of a plan)
      % 'goal': the cost contribution on the distances from the last two
      %         footsteps to their respective goal poses.
      % Each weight is a 6 element vector, describing the weights on
      % [x, y, z, roll, pitch, yaw]

      weights = struct('relative', [1;1;1;0;0;0.5],...
                       'relative_final', [10;10;10;0;0;2],...
                       'goal', [100;100;0;0;0;10]);
    end

    function prop_cache = getRobotPropertyCache(obj)
      % Functions like findLinkId, getTerrainContactPoints, etc. can be too slow to call
      % in the inner loop of our controller or planner, so we cache some useful information
      % at setup time. 

      prop_cache = struct('contact_groups', [],...
                          'body_ids', struct(),...
                          'position_indices', struct(),...
                          'actuated_indices', [],...
                          'nq', 0,...
                          'nv', 0,...
                          'num_bodies', 0);

      % getTerrainContactPoints is pretty expensive, so we'll just call it
      % for all the bodies and cache the results
      nbod = length(obj.getManipulator().body);
      contact_group_cache = cell(1, nbod);
      for j = 1:nbod
        contact_group_cache{j} = struct();
        for f = 1:length(obj.getBody(j).collision_geometry_group_names)
          name = obj.getBody(j).collision_geometry_group_names{f};
          if obj.getBody(j).robotnum == 1
            contact_group_cache{j}.(name) = obj.getBody(j).getTerrainContactPoints(name);
          end
        end
      end

      prop_cache.contact_groups = contact_group_cache;

      prop_cache.nq = obj.getNumPositions();
      prop_cache.nv = obj.getNumVelocities();
      prop_cache.num_bodies = length(obj.getManipulator().body);

      prop_cache.body_ids.('pelvis') = obj.findLinkId('pelvis');
      prop_cache.body_ids.('r_foot') = obj.findLinkId('rightFoot');
      prop_cache.body_ids.('l_foot') = obj.findLinkId('leftFoot');
      prop_cache.body_ids.('rightFoot') = obj.findLinkId('rightFoot');
      prop_cache.body_ids.('leftFoot') = obj.findLinkId('leftFoot');

      prop_cache.position_indices.('neck') = obj.findPositionIndices('lowerNeckPitch');
      prop_cache.position_indices.('r_leg_ak') = [obj.findPositionIndices('rightAnklePitch'); obj.findPositionIndices('rightAnkleRoll')];
      prop_cache.position_indices.('l_leg_ak') = [obj.findPositionIndices('leftAnklePitch'); obj.findPositionIndices('leftAnkleRoll')];

      prop_cache.position_indices.('r_leg') = [obj.findPositionIndices('rightHipYaw'); obj.findPositionIndices('rightHipRoll'); obj.findPositionIndices('rightHipPitch'); obj.findPositionIndices('rightKneePitch'); obj.findPositionIndices('rightAnklePitch'); obj.findPositionIndices('rightAnkleRoll')];
      prop_cache.position_indices.('l_leg') = [obj.findPositionIndices('leftHipYaw'); obj.findPositionIndices('leftHipRoll'); obj.findPositionIndices('leftHipPitch'); obj.findPositionIndices('leftKneePitch'); obj.findPositionIndices('leftAnklePitch'); obj.findPositionIndices('leftAnkleRoll')];

      prop_cache.position_indices.('r_leg_kny') = obj.findPositionIndices('rightKneePitch');
      prop_cache.position_indices.('l_leg_kny') = obj.findPositionIndices('leftKneePitch');

      prop_cache.position_indices.('arm') = [obj.findPositionIndices('leftShoulderPitch'); obj.findPositionIndices('leftShoulderRoll'); obj.findPositionIndices('leftShoulderYaw'); obj.findPositionIndices('leftElbowPitch'); obj.findPositionIndices('leftForearm'); obj.findPositionIndices('leftWristRoll'); obj.findPositionIndices('leftWristPitch'); obj.findPositionIndices('rightShoulderPitch'); obj.findPositionIndices('rightShoulderRoll'); obj.findPositionIndices('rightShoulderYaw'); obj.findPositionIndices('rightElbowPitch'); obj.findPositionIndices('rightForearm'); obj.findPositionIndices('rightWristRoll'); obj.findPositionIndices('rightWristPitch')];

      prop_cache.position_indices.('back_bkz') = obj.findPositionIndices('torsoYaw');
      prop_cache.position_indices.('back_bky') = obj.findPositionIndices('torsoPitch');
      prop_cache.position_indices.('neck') = obj.findPositionIndices('lowerNeckPitch');

      prop_cache.actuated_indices = obj.getActuatedJoints();
    end
  end

  properties (SetAccess = protected, GetAccess = public)
    x0
    % preconstructing these for efficiency
    left_full_support
    left_toe_support
    right_full_support
    right_toe_support
    left_full_right_full_support
    left_toe_right_full_support
    left_full_right_toe_support
    external_force = 0; % if nonzero, body id where force is being exerted
  end

  properties
    fixed_point_file = fullfile(getDrakePath,'examples','Valkyrie','data','valkyrie_fp_june2015_30joints_one_neck.mat');
    default_footstep_params = struct('nom_forward_step', 0.25,... % m
                                      'max_forward_step', 0.35,...% m
                                      'max_backward_step', 0.2,...% m
                                      'max_step_width', 0.38,...% m
                                      'min_step_width', 0.18,...% m
                                      'nom_step_width', 0.26,...% m
                                      'max_outward_angle', pi/8,... % rad
                                      'max_inward_angle', 0.01,... % rad
                                      'nom_upward_step', 0.25,... % m
                                      'nom_downward_step', 0.25,...% m
                                      'max_num_steps', 10,...
                                      'min_num_steps', 1,...
                                      'leading_foot', 1); % 0: left, 1: right
    default_walking_params = struct('step_speed', 0.5,... % speed of the swing foot (m/s)
                                    'step_height', 0.05,... % approximate clearance over terrain (m)
                                    'drake_min_hold_time', 0.7,... % minimum time in double support (s)
                                    'drake_instep_shift', 0.0,... % Distance to shift ZMP trajectory inward toward the instep from the center of the foot (m)
                                    'mu', 1.0,... % friction coefficient
                                    'constrain_full_foot_pose', true,... % whether to constrain the swing foot roll and pitch
                                    'pelvis_height_above_foot_sole', 1.01,... % default pelvis height when walking
                                    'support_contact_groups', {{'heel', 'toe'}},... % which contact groups are available for support when walking
                                    'prevent_swing_undershoot', false,... % prevent the first phase of the swing from going backwards while moving to the first knot point
                                    'prevent_swing_overshoot', false,... % prevent the final phase of the swing from moving forward of the last knot point
                                    'nominal_LIP_COM_height', 1.01); % nominal height used to construct D_ls for our linear inverted pendulum model
    r_foot_name = 'rightFoot+rightLegSixAxis_Frame+rightCOP_Frame';
    l_foot_name = 'leftFoot+leftLegSixAxis_Frame+leftCOP_Frame';
    pelvis_name = 'pelvis+leftPelvisIMU_Frame+rightPelvisIMU_Frame';
    r_knee_name = 'rightKneePitch';
    l_knee_name = 'leftKneePitch';
    l_akx_name = 'leftAnkleRoll';
    r_akx_name = 'rightAnkleRoll';
    r_aky_name = 'rightAnklePitch';
    l_aky_name = 'leftAnklePitch';
    control_config_file = fullfile(fileparts(mfilename('fullpath')), '..', 'config', 'control_config_sim.yaml');
  end
end
