classdef Biped < LeggedRobot

  properties
    foot_frame_id
  end

  properties(Abstract, SetAccess = protected, GetAccess = public)
    default_footstep_params
    default_walking_params
  end

  methods
    function obj = Biped(r_foot_id_or_name, l_foot_id_or_name)
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
      weights = struct('relative', [10;10;10;0;0;0.2],...
                       'relative_final', [1000;100;100;0;0;100],...
                       'goal', [100;100;0;0;0;10]);
    end

    function params = applyDefaultFootstepParams(obj, params)
      params = applyDefaults(params, obj.default_footstep_params);
    end

    function foot_center = feetPosition(obj, q0)
      % Convenient way to find the poses of the center soles of the feet given a
      % configuration vector q0

      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumDOF,1]);

      kinsol = doKinematics(obj,q0);

      rfoot0 = forwardKin(obj,kinsol,obj.foot_frame_id.right,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,obj.foot_frame_id.left,[0;0;0],true);

      foot_center = struct('right', rfoot0, 'left', lfoot0);
    end
  end


end

