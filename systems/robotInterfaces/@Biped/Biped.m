classdef Biped < LeggedRobot
% Interface class for a bipedal robot. Being a Biped currently affords
% footstep planning and ZMP-based walking planning, but these capabilities
% will expand in the future.
% In order to function as a Biped, a robot's URDF must be tagged with two
% Drake-style <frame> tags indicating the centers of the soles of its feet,
% oriented with the x-axis pointing forward. Such a frame might look something
% like:
% <frame link="r_foot" name="r_foot_sole" rpy="0 0 0" xyz="0.0480 0 -0.081119"/>
%
% see examples/Atlas/Atlas.m for an example implementation

  properties
    foot_body_id
    foot_frame_id
    toe_frame_id
    inner_foot_shape % see getInnerFootShape() below
    default_body_collision_slices % see getBodyCollisionSlices() below
    body_collision_slice_heights % see getBodyCollisionSlices() below
  end

  properties(Abstract)
    default_footstep_params
    default_walking_params
  end

  methods
    function obj = Biped(r_foot_frame_id_or_name, l_foot_frame_id_or_name)
      % Construct a biped by identifying the Drake frame's corresponding
      % to the soles of its feet.
      if nargin < 1
        % use atlas defaults
        r_foot_frame_id_or_name = 'r_foot_sole';
        l_foot_frame_id_or_name = 'l_foot_sole';
      end

      obj = obj@LeggedRobot();

      r_frame_id = obj.parseBodyOrFrameID(r_foot_frame_id_or_name);
      r_body_id = obj.getFrame(r_frame_id).body_ind;
      l_frame_id = obj.parseBodyOrFrameID(l_foot_frame_id_or_name);
      l_body_id = obj.getFrame(l_frame_id).body_ind;
      obj.foot_body_id = struct('left', l_body_id, 'right', r_body_id);
      obj.foot_frame_id = struct('left', l_frame_id, 'right', r_frame_id);
      obj.toe_frame_id = struct('left', obj.parseBodyOrFrameID('l_foot_toe'),...
                                'right', obj.parseBodyOrFrameID('r_foot_toe'));
    end

    function [A, b] = getReachabilityPolytope(obj, stance_foot_frame, swing_foot_frame, params)
      % Get a polytope representing the reachable set of locations for the
      % swing foot during walking. This polytope is a constraint on the
      % [x,y,z,roll,pitch,yaw] position of the swing foot sole expressed in the
      % frame of the stance foot sole.
      % @param stance_foot_frame the ID of the frame of the stance foot
      % @param swing_foot_frame the ID of the frame of the moving foot
      % @param params parameters describing the footstep reachability:
      %
      %               Lateral extent of reachable set:
      %               min_step_width (m)
      %               nom_step_width (m)
      %               max_step_width (m)
      %
      %               Forward/backward extent of reachable set:
      %               max_forward_step (m)
      %               nom_forward_step (m)
      %
      %               Vertical extent of reachable set:
      %               nom_upward_step (m)
      %               nom_downward_step (m)
      %
      %               Amount of rotation allowed between adjacent steps
      %               max_outward_angle (rad)
      %               max_inward_angle (rad)
      %
      %               These quantities are expressed as if we are describing the
      %               pose of the left foot relative to the right, and signs will be
      %               automatically adjusted to describe the pose of the right foot
      %               relative to the left
      params = applyDefaults(params, obj.default_footstep_params);

      bodies = [stance_foot_frame, swing_foot_frame];

      if ~ (all(bodies == [obj.foot_frame_id.right, obj.foot_frame_id.left]) ||...
            all(bodies == [obj.foot_frame_id.left, obj.foot_frame_id.right]))
        error('Drake:Biped:BadBodyIdx', 'Feasibility polytope not defined for this pairing of body indices.');
      end
      if params.max_step_width <= params.nom_step_width
        warning('Drake:Biped:BadNominalStepWidth', 'Nominal step width should be less than max step width');
        params.max_step_width = params.nom_step_width * 1.01;
      end
      if params.min_step_width >= params.nom_step_width
        warning('Drake:Biped:BadNominalStepWidth', 'Nominal step width should be greater than min step width');
        params.min_step_width = params.nom_step_width * 0.99;
      end
      if params.nom_upward_step <= 0
        params.nom_upward_step = 0.01;
      end
      if params.nom_downward_step <= 0
        params.nom_downward_step = 0.01;
      end

      [Axy, bxy] = poly2lincon([0, params.max_forward_step, 0, -params.max_forward_step],...
                               [params.min_step_width, params.nom_step_width, params.max_step_width, params.nom_step_width]);
      [Axz, bxz] = poly2lincon([0, params.max_forward_step, params.max_forward_step, params.max_forward_step, 0, -params.max_forward_step], ...
                               [params.nom_upward_step, params.nom_upward_step, 0, -params.nom_downward_step, -params.nom_downward_step, 0]);
      A = [Axy, zeros(size(Axy, 1), 4);
           Axz(:,1), zeros(size(Axz, 1), 1), Axz(:,2), zeros(size(Axz, 1), 3);
           0 0 0 0 0 -1;
           0 0 0 0 0 1;
           0, 1/(params.max_step_width-params.nom_step_width), 0, 0, 0, 1/params.max_outward_angle;
           0, 1/(params.min_step_width-params.nom_step_width), 0, 0, 0, 1/params.max_outward_angle;
           params.max_outward_angle / (params.max_forward_step), 0, 0, 0, 0, 1;
           -params.max_outward_angle / (params.max_forward_step), 0, 0, 0, 0, 1;
           ];

      if bodies(1) == obj.foot_frame_id.left
        A(:,2) = -A(:,2);
        A(:,6) = -A(:,6);
      end
      b = [bxy;
           bxz;
           params.max_inward_angle;
           params.max_outward_angle;
           1 + params.nom_step_width/(params.max_step_width-params.nom_step_width);
           1 + params.nom_step_width/(params.min_step_width-params.nom_step_width)
           params.max_outward_angle;
           params.max_outward_angle;
           ];
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
      
      weights = struct('relative', [1;1;1;0;0;0.05],...
                       'relative_final', [10;10;10;0;0;1],...
                       'goal', [1000;1000;0;0;0;100]);
    end

    function params = applyDefaultFootstepParams(obj, params)
      params = applyDefaults(params, obj.default_footstep_params);
    end

    function foot_center = feetPosition(obj, q0)
      % Convenient way to find the poses of the center soles of the feet given a
      % configuration vector q0

      sizecheck(q0,[obj.getNumPositions,1]);

      kinsol = doKinematics(obj,q0);

      rfoot0 = forwardKin(obj,kinsol,obj.foot_frame_id.right,[0;0;0], 1);
      lfoot0 = forwardKin(obj,kinsol,obj.foot_frame_id.left,[0;0;0], 1);

      foot_center = struct('right', rfoot0, 'left', lfoot0);
    end

    function foot_min_z = getFootHeight(obj,q)
      % Get the height in world coordinates of the lower of the robot's foot soles
      foot_center = obj.feetPosition(q);
      foot_min_z = min(foot_center.right(3), foot_center.left(3));
    end

    function [centers, radii] = getReachabilityCircles(obj, params, fixed_foot_frame_id)
      % Compute the centers and radii of the circular regions which constrain the
      % next foot position in the frame of the fixed foot
      params = applyDefaults(params, obj.default_footstep_params);

      v1x = params.max_forward_step - params.max_backward_step;
      v2x = v1x;
      mean_width = mean([params.min_step_width, params.max_step_width]);
      r2 = -((params.min_step_width - mean_width)^2 + (params.max_forward_step - v1x)^2) / (2 * (params.min_step_width - mean_width));
      r1 = r2;
      v1y = params.max_step_width - r1;
      v2y = params.min_step_width + r2;
      v1 = [v1x; v1y];
      v2 = [v2x; v2y];
      centers = [v1, v2];
      radii = [r1, r2];

      if fixed_foot_frame_id == obj.foot_frame_id.right;
      elseif fixed_foot_frame_id == obj.foot_frame_id.left
        centers(2,:) = -centers(2,:);
      else
        error('Invalid foot frame ID: %d', fixed_foot_frame_id);
      end

    end

    function [foci, l] = getReachabilityEllipse(obj, params, fixed_foot_frame_id)
      params = applyDefaults(params, obj.default_footstep_params);

      f1y = (params.max_step_width + params.min_step_width) / 2;
      f2y = f1y;
      l = params.max_forward_step + params.max_backward_step;

      d = sqrt((params.max_forward_step + params.max_backward_step)^2 - (params.max_step_width - params.min_step_width)^2);
      f1x = (params.max_forward_step - params.max_backward_step)/2 - d/2;
      f2x = (params.max_forward_step - params.max_backward_step)/2 + d/2;

      foci = [f1x f2x; f1y f2y];

      if fixed_foot_frame_id == obj.foot_frame_id.right;
      % nothing needed
      elseif fixed_foot_frame_id == obj.foot_frame_id.left
        foci(2,:) = -foci(2,:); % flip left-right
      else
        error('Invalid foot frame ID: %d', fixed_foot_frame_id);
      end
    end
    
    function bool = isDoubleSupport(obj,rigid_body_support_state)
      bool = any(rigid_body_support_state.bodies==obj.robot.foot_body_id.left) && any(rigid_body_support_state.bodies==obj.robot.foot_body_id.right);
    end

    function bool = isLeftSupport(obj,rigid_body_support_state)
      bool = any(rigid_body_support_state.bodies==obj.robot.foot_body_id.left) && ~any(rigid_body_support_state.bodies==obj.robot.foot_body_id.right);
    end

    function bool = isRightSupport(obj,rigid_body_support_state)
      bool = ~any(rigid_body_support_state.bodies==obj.robot.foot_body_id.left) && any(rigid_body_support_state.bodies==obj.robot.foot_body_id.right);
    end

    function fc = getFootContacts(obj, q)
      % For a given configuration of the biped, determine whether each foot is
      % in contact with the terrain. 
      % @param q a robot configuration vector
      % @retval fc a logical vector of length 2. If fc(1) is true, then the right
      %            foot is in contact. If fc(2) is true, then the left foot is in
      %            contact.
      [phiC,~,~,~,idxA,idxB] = obj.collisionDetect(q,false);
      within_thresh = phiC < 0.002;
      contact_pairs = [idxA(within_thresh); idxB(within_thresh)];
      
      % The following would be faster but would require us to have
      % heightmaps in Bullet
      %[~,~,idxA,idxB] = obj.r_control.allCollisions(x(1:obj.nq_control));
      %contact_pairs = [idxA; idxB];
      foot_indices = [obj.foot_body_id.right, obj.foot_body_id.left];
      fc = any(bsxfun(@eq, contact_pairs(:), foot_indices),1)';
    end

    function collision_model = getFootstepPlanningCollisionModel(obj, varargin)
      % Get a simple collision model for the Atlas robot to enable
      % (limited) whole-body awareness during footstep planning. The
      % collision model is represented as a set of points which define
      % the shape of the foot and a set of slices which define the 
      % bounding boxes of the legs, torso, and arms. The collision model
      % of the foot used here may actually be smaller than the robot's
      % feet in order to allow the toes or heels to hang over edges. 
      % @param q (optional) if provided, use the given robot configuration 
      %          to compute the collision volumes. Otherwise use hard-coded
      %          values derived from a typical walking posture.
      % @retval collision_model an IRIS CollisionModel object with fields
      %         foot and body. The body field has subfields z and xy, where
      %         xy is of shape [3, N, length(z)]. Each page of xy(:,:,j)
      %         represents the bounds of the robot from z(j) to z(j+1) (or inf).
      checkDependency('iris');
      foot_shape = obj.getInnerFootShape();
      slices = obj.getBodyCollisionSlices(varargin{:});
      collision_model = iris.terrain_grid.CollisionModel(foot_shape, slices);
    end

    function shape = getInnerFootShape(obj)
      % Get an inner approximation of the foot shape as a set of points in xy. The convention here is that this entire shape must be supported by the terrain. So, making this shape smaller than the actual foot will allow the edge of the foot to hang over edges when the footstep planner is run. 
      % By convention, x points forward (from heel to toe), y points left, and the foot shape is symmetric about y=0.
      if ~isempty(obj.inner_foot_shape)
        shape = obj.inner_foot_shape;
      else
        obj.warnAboutAtlasDefault('inner_foot_shape');
        shape = [-0.12, -0.12, 0.13, 0.13;
                0.04, -0.04, 0.04, -0.04];
      end
    end

    function slices = getBodyCollisionSlices(obj, varargin)
      % Get a simple collision model for the biped robot to enable
      % (limited) whole-body awareness during footstep planning. The
      % collision model is represented as a set of slices which define the 
      % bounding boxes of the legs, torso, and arms. The slices are taken at
      % z values which can be specified in obj.body_collision_slice_heights
      % and which are measured in meters up from the sole of the robot's feet
      % @param q (optional) if provided, use the given robot configuration 
      %          to compute the collision volumes. Otherwise use hard-coded
      %          values derived from a typical walking posture.
      % @retval slices a struct with fields z and xy, where
      %         xy is of shape [3, N, length(z)]. Each page of xy(:,:,j)
      %         represents the bounds of the robot from z(j) to z(j+1) (or inf).
      p = inputParser();
      p.addOptional('q', []);
      p.addParamValue('bottom_slice_shift', 0.05, @isscalar); % amount to shift the lowest collision 
                                                              % slice upward (in m) to prevent false 
                                                              % positive obstacle detections from small 
                                                              % terrain height variations. 
      p.addParamValue('padding_margin', [0, 0.01, 0.01, 0.01], @isnumeric) % padding for each slice in all directions
      p.addParamValue('debug', false, @isnumeric);
      p.parse(varargin{:});
      q = p.Results.q;
      options = p.Results;

      if nargin < 2 || isempty(q) || ~any(ismember(obj.getCollisionGeometryGroupNames, 'default'))
        if ~any(ismember(obj.getCollisionGeometryGroupNames, 'default'))
          warning('Drake:Biped:MayBeMissingCollisionGeometry', 'This biped may not have any collision geometry, so computing an upper body collision model from a configuration may not be possible. Using the default model instead.')
        end
        if ~isempty(obj.default_body_collision_slices)
          slices = obj.default_body_collision_slices;
        else
          obj.warnAboutAtlasDefault('default_body_collision_slices');
          slices = struct('z', [0.05, 0.35, 0.75, 1.15],...
                     'xy', cat(3, ...
                         [-0.17, -0.17, 0.17, 0.17; 0.07, -0.07, 0.07, -0.07],...
                         [-0.17, -0.17, 0.25, 0.25; .25, -.25, .25, -.25],...
                         [-0.2, -0.2, 0.25, 0.25; .4, -.4, .4, -.4],...
                         [-0.4, -0.4, 0.3, 0.3; .45, -.45, .45, -.45]));
        end
      else
        if ~isempty(obj.body_collision_slice_heights)
          slice_heights = obj.body_collision_slice_heights;
        else
          obj.warnAboutAtlasDefault('body_collision_slice_heights');
          slice_heights = [0, 0.35, 0.75, 1.15, 2];
        end
        if isscalar(options.padding_margin)
          options.padding_margin = options.padding_margin + zeros(size(slice_heights));
        end
        slices = obj.collisionSlices(q, slice_heights, 'margin', options.padding_margin, 'debug', options.debug);
        % Shift the lowest slice up slightly (to prevent false positive obstacle
        % detections from small terrain height variations). 
        slices.z(1) = slices.z(1) + options.bottom_slice_shift;

        % Center and average the slices in y to simulate putting the center of mass above one foot
        for j = 1:length(slices.z)
          slices.xy(2,:,j) = slices.xy(2,:,j) - mean(slices.xy(2,:,j));
        end

        if options.debug
          checkDependency('iris');
          figure(105)
          for j = 1:length(slice_heights)-1
            verts = [slices.xy(:,:,j), slices.xy(:,:,j); repmat(slice_heights(j), 1, 4), repmat(slice_heights(j+1), 1, 4)];
            iris.drawing.drawPolyFromVertices(verts, 'b');
          end
          drawnow();
        end
      end
    end

    function warnAboutAtlasDefault(obj, field_name)
      if ~isa(obj, 'Atlas')
        warning('Drake:Biped:UsingAtlasDefaults', 'Using default value from Atlas for the following value: ''%s''. You may want to override that value in your particular Biped subclass', field_name)
      end
    end
  end
end
