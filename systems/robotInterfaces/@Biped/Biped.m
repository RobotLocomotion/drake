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
    foot_frame_id
  end

  properties(Abstract, SetAccess = protected, GetAccess = public)
    default_footstep_params
    default_walking_params
  end

  methods
    function obj = Biped(r_foot_id_or_name, l_foot_id_or_name)
      % Construct a biped by identifying the Drake frame's corresponding
      % to the soles of its feet.
      if nargin == 0
        l_foot_id_or_name = 'r_foot_sole';
        r_foot_id_or_name = 'l_foot_sole';
      end

      obj = obj@LeggedRobot();
      if isstr(r_foot_id_or_name)
        r_frame_id = findFrameId(obj, r_foot_id_or_name);
      else
        r_frame_id = r_foot_id_or_name;
      end
      if isstr(l_foot_id_or_name)
        l_frame_id = findFrameId(obj, l_foot_id_or_name);
      else
        l_frame_id = l_foot_id_or_name;
      end
      obj.foot_frame_id = struct('left', l_frame_id, 'right', r_frame_id);
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
      [Axz, bxz] = poly2lincon([0, params.nom_forward_step, params.max_forward_step, params.nom_forward_step, 0, -params.max_forward_step], ...
                               [params.nom_upward_step, params.nom_upward_step, 0, -params.nom_downward_step, -params.nom_downward_step, 0]);
      A = [Axy, zeros(size(Axy, 1), 4);
           Axz(:,1), zeros(size(Axz, 1), 1), Axz(:,2), zeros(size(Axz, 1), 3);
           0 0 0 0 0 -1;
           0 0 0 0 0 1;
           0, 1/(params.max_step_width-params.nom_step_width), 0, 0, 0, 1/params.max_outward_angle;
           0, 1/(params.min_step_width-params.nom_step_width), 0, 0, 0, 1/params.max_outward_angle];

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

      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumPositions,1]);

      kinsol = doKinematics(obj,q0);

      rfoot0 = forwardKin(obj,kinsol,obj.foot_frame_id.right,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,obj.foot_frame_id.left,[0;0;0],true);

      foot_center = struct('right', rfoot0, 'left', lfoot0);
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
  end


end

