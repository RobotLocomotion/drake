classdef Atlas < TimeSteppingRigidBodyManipulator & Biped
  methods

    function obj=Atlas(urdf,options)
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
      if ~isfield(options,'hands')
        options.hands = 'none';
      end

      w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
      
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      obj = obj@Biped('r_foot_sole', 'l_foot_sole');
      warning(w);

      if (~strcmp(options.hands, 'none'))
        if (strcmp(options.hands, 'robotiq'))
          options_hand.weld_to_link = findLinkId(obj,'r_hand');
          obj.hands = 1;
          obj = obj.addRobotFromURDF(getFullPathFromRelativePath('urdf/robotiq.urdf'), [0; -0.195; -0.01], [0; -3.1415/2; 3.1415], options_hand);
        elseif (strcmp(options.hands, 'robotiq_weight_only'))
          % Adds a box with weight roughly approximating the hands, so that
          % the controllers know what's up
          options_hand.weld_to_link = findLinkId(obj,'r_hand');
          obj = obj.addRobotFromURDF(getFullPathFromRelativePath('urdf/robotiq_box.urdf'), [0; -0.195; -0.01], [0; -3.1415/2; 3.1415], options_hand);
        else
          error('unsupported hand type');
        end
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
      if isfield(options, 'atlas_version')
        obj.atlas_version = options.atlas_version;
      end
      
      obj.left_full_support = RigidBodySupportState(obj,obj.foot_body_id.left);
      obj.left_toe_support = RigidBodySupportState(obj,obj.foot_body_id.left,{{'toe'}});
      obj.right_full_support = RigidBodySupportState(obj,obj.foot_body_id.right);
      obj.right_toe_support = RigidBodySupportState(obj,obj.foot_body_id.right,{{'toe'}});
      obj.left_full_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right]);
      obj.left_toe_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],{{'toe'},{'heel','toe'}});
      obj.left_full_right_toe_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],{{'heel','toe'},{'toe'}});
    end

    function obj = compile(obj)
      obj = compile@TimeSteppingRigidBodyManipulator(obj);

      % Sanity check if we don't have hands.
      if (~isa(obj.manip.getStateFrame().getFrameByNum(1), 'MultiCoordinateFrame'))
        obj.hands = 0;
      end
      % Construct state vector itself
      if (obj.hands == 0 && ~isa(obj.manip.getStateFrame().getFrameByNum(1), 'MultiCoordinateFrame'))
        atlas_state_frame = atlasFrames.AtlasState(obj);
      else
        atlas_state_frame = getStateFrame(obj);
        atlas_state_frame = replaceFrameNum(atlas_state_frame,1,atlasFrames.AtlasState(obj));
      end
      if (obj.hands > 0)
        % Sub in handstates for the hand (curently assuming just 1)
        % TODO: by name?
        for i=2:2
          atlas_state_frame = replaceFrameNum(atlas_state_frame,i,atlasFrames.HandState(obj,i,'atlasFrames.HandState'));
        end
      end
      
      tsmanip_state_frame = obj.getStateFrame();
      if (obj.hands == 0 && ~isa(tsmanip_state_frame.getFrameByNum(1), 'MultiCoordinateFrame'))
        tsmanip_state_frame = atlasFrames.AtlasState(obj);
      else
        tsmanip_state_frame = replaceFrameNum(tsmanip_state_frame,1,atlasFrames.AtlasState(obj));
      end
      if tsmanip_state_frame.dim>atlas_state_frame.dim
        id = findSubFrameEquivalentModuloTransforms(tsmanip_state_frame,atlas_state_frame);
        tsmanip_state_frame.frame{id} = atlas_state_frame;
        state_frame = tsmanip_state_frame;
      else
        state_frame = atlas_state_frame;
      end
      obj.manip = obj.manip.setStateFrame(atlas_state_frame);
      obj = obj.setStateFrame(state_frame);
      
      % Same bit of complexity for input frame to get hand inputs
      if (obj.hands > 0)
        input_frame = getInputFrame(obj);
        input_frame  = replaceFrameNum(input_frame,1,atlasFrames.AtlasInput(obj));
        % Sub in handstates for each hand
        % TODO: by name?
        for i=2:2
          input_frame = replaceFrameNum(input_frame,i,atlasFrames.HandInput(obj,i,'atlasFrames.HandInput'));
        end
      else
        input_frame = atlasFrames.AtlasInput(obj);
      end
      obj = obj.setInputFrame(input_frame);
      obj.manip = obj.manip.setInputFrame(input_frame);
      
      % Construct output frame, which comes from state plus sensor
      % info
      atlas_output_frame = atlas_state_frame;
      if (~isempty(obj.manip.sensor))
        for i=1:length(obj.manip.sensor)
          % If it's not a full state feedback sensor (we have already
          % got the state for that above in the state frame
          if (~isa(obj.manip.sensor{i}, 'FullStateFeedbackSensor'))
            if (isa(atlas_output_frame, 'MultiCoordinateFrame'))
              atlas_output_frame = atlas_output_frame.appendFrame(obj.manip.sensor{i}.constructFrame(obj.manip));
            else
              atlas_output_frame = MultiCoordinateFrame({atlas_output_frame, obj.manip.sensor{i}.constructFrame(obj.manip)});
            end
          end
        end
      end
      output_frame = atlas_output_frame;
      % Continuing frame from above...
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
      
      if ~isequal_modulo_transforms(atlas_output_frame,getOutputFrame(obj.manip))
        obj.manip = obj.manip.setNumOutputs(atlas_output_frame.dim);
        obj.manip = obj.manip.setOutputFrame(atlas_output_frame);
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

    function [qp,lfoot_control_block,rfoot_control_block,pelvis_control_block,pd,options] = constructQPWalkingController(obj,controller_data,options)
      if nargin < 3
        options = struct();
      end
      options = applyDefaults(options, struct(...
        'w_qdd',zeros(obj.getNumVelocities(),1),...
        'input_foot_contacts',true,...
        'Kp_pelvis',[0; 0; 20; 20; 20; 20],...
        'use_walking_pelvis_block',true,...
        'pelvis_damping_ratio',0.5,...
        'Kp_accel',0.0,...
        'body_accel_input_weights',[0.15 0.15 0.075],...
        'use_foot_motion_block',true,...
        'Kp_foot',[12; 12; 12; 12; 12; 12],...
        'foot_damping_ratio',0.7,...
        'min_knee_angle',0.7,...
        'Kp_q',0.0*ones(obj.getNumPositions(),1),...
        'q_damping_ratio',0.5));

      options.w_qdd(findPositionIndices(obj,'back_bkx')) = 0.01;
      options.Kp_q(findPositionIndices(obj,'back_bkx')) = 50;

      acc_limit = [100;100;100;50;50;50];
      body_accel_bounds(1).body_idx = obj.foot_body_id.right;
      body_accel_bounds(1).min_acceleration = -acc_limit;
      body_accel_bounds(1).max_acceleration = acc_limit;
      body_accel_bounds(2).body_idx = obj.foot_body_id.left;
      body_accel_bounds(2).min_acceleration = -acc_limit;
      body_accel_bounds(2).max_acceleration = acc_limit;
      if ~isfield(options, 'body_accel_bounds'); options.body_accel_bounds = body_accel_bounds; end;
      
      [qp,lfoot_control_block,rfoot_control_block,pelvis_control_block,pd,options] = ...
        constructQPBalancingController(obj,controller_data,options);
    end

    function [qp,lfoot_control_block,rfoot_control_block,pelvis_control_block,pd,options] = constructQPBalancingController(obj,controller_data,options)
      import atlasControllers.*;
      if nargin < 3
        options = struct();
      end
      options = applyDefaults(options, struct(...
        'slack_limit',30,...
        'w_qdd',0.0*ones(obj.getNumVelocities(),1),...
        'W_kdot',0.0*eye(3),...
        'w_grf',0.0,...
        'w_slack',0.05,...
        'Kp_accel',1.0,...
        'debug',false,...
        'use_mex',true,...
        'contact_threshold',0.001,...
        'output_qdd',true,...
        'solver',0,...  % 0 fastqp, 1 gurobi
        'Kp_pelvis',20*[1; 1; 1; 0.6; 0.6; 0.6],...
        'pelvis_damping_ratio',0.7,...
        'body_accel_input_weights',0.01,...
        'use_ik',false,...
        'Kp_q',0.0*ones(obj.getNumPositions(),1),...
        'q_damping_ratio',0.0));
      
      options.w_qdd(findPositionIndices(obj,'back_bkx')) = 0.1;
      options.Kp_q(findPositionIndices(obj,'back_bkx')) = 50;

      options.Kp = options.Kp_pelvis;
      options.Kd = getDampingGain(options.Kp,options.pelvis_damping_ratio);
      if isfield(options,'use_walking_pelvis_block') && options.use_walking_pelvis_block
        pelvis_control_block = PelvisMotionControlBlock(obj,'pelvis',controller_data,options);
      else
        pelvis_control_block = BodyMotionControlBlock(obj,'pelvis',controller_data,options);
      end

      if isfield(options,'use_foot_motion_block') && options.use_foot_motion_block
        foot_options = struct('Kp', options.Kp_foot,...
                              'Kd', getDampingGain(options.Kp,options.foot_damping_ratio),...
                              'use_plan_shift', true);
        lfoot_control_block = BodyMotionControlBlock(obj,'l_foot',controller_data,foot_options);
        rfoot_control_block = BodyMotionControlBlock(obj,'r_foot',controller_data,foot_options);
        motion_frames = {lfoot_control_block.getOutputFrame,rfoot_control_block.getOutputFrame,...
          pelvis_control_block.getOutputFrame};
      else
        lfoot_control_block = [];
        rfoot_control_block = [];
        motion_frames = {pelvis_control_block.getOutputFrame};
      end
      qp = AtlasQPController(obj,motion_frames,controller_data,options);

      options.Kp = options.Kp_q;
      options.Kd = getDampingGain(options.Kp,options.q_damping_ratio);
      pd = IKPDBlock(obj,controller_data,options);
    end
  end

  properties (SetAccess = protected, GetAccess = public)
    x0
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
                                    'hold_frac', 0.4,... % fraction of the swing time spent in double support
                                    'drake_min_hold_time', 1.0,... % minimum time in double support (s)
                                    'drake_instep_shift', 0.0,... % Distance to shift ZMP trajectory inward toward the instep from the center of the foot (m)
                                    'mu', 1.0,... % friction coefficient
                                    'constrain_full_foot_pose', true); % whether to constrain the swing foot roll and pitch
    hands = 0; % 0, none; 1, Robotiq
    % preconstructing these for efficiency
    left_full_support
    left_toe_support
    right_full_support
    right_toe_support
    left_full_right_full_support
    left_toe_right_full_support
    left_full_right_toe_support
    atlas_version = [];
  end

  properties
    fixed_point_file = fullfile(getDrakePath(), 'examples', 'Atlas', 'data', 'atlas_fp.mat');
  end
end
