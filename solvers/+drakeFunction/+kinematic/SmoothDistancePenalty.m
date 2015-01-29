classdef SmoothDistancePenalty < drakeFunction.kinematic.Kinematic
  % DrakeFunction penalizing all distances less than a given minimum between
  % bodies of a RigidBodyManipulator. The output of this function is zero if
  % and only if all pairs of bodies considered are separarated by at least the
  % given minimum distance.
  properties (SetAccess = immutable)
    min_distance              % Positive scalar representing the minimum
                              % unpenalized distance
    active_collision_options  % Options struct as described in
                              % RigidBodyManipulator/collisionDetect
  end

  methods
    function obj = SmoothDistancePenalty(rbm,min_distance,active_collision_options)
      % obj = SmoothDistancePenalty(rbm,min_distance,active_collision_options)
      %
      % @param rbm                      -- RigidBodyManipulator object
      % @param min_distance             -- Positive scalar representing the
      %                                    minimum unpenalized distance
      % @param active_collision_options -- Options struct as described in
      %                                    RigidBodyManipulator/collisionDetect
      if nargin < 3, active_collision_options = struct(); end
      sizecheck(min_distance,[1,1]);
      assert(min_distance>0);
      output_frame = drakeFunction.frames.realCoordinateSpace(1);
      obj = obj@drakeFunction.kinematic.Kinematic(rbm,output_frame);
      obj.min_distance = min_distance;
      obj.active_collision_options = active_collision_options;
    end

    function [f,df] = eval(obj,q)
      obj.rbm.doKinematics(q);
      [f,df] = smoothDistancePenaltymex(obj.rbm.mex_model_ptr,obj.min_distance,false,obj.active_collision_options);
    end
  end

  methods (Access = private)
    function [scaled_dist,dscaled_dist_ddist] = scaleDistance(obj,dist)
      recip_min_dist = 1/obj.min_distance;
      scaled_dist = recip_min_dist*dist - 1;
      %scaled_dist = recip_min_dist*dist - 2;
      dscaled_dist_ddist = recip_min_dist*eye(size(dist,1));
    end

    function [cost, dcost_ddist] = penalty(obj,dist)
      % [cost, dcost_ddist] = penalty(obj,dist) applies a smooth hinge loss
      % element-wise to dist. This hinge loss is given by
      %
      % \f[
      % c = 
      % \begin{cases}
      %   -de^{\frac{1}{d}}, & d <   0  \\
      %   0,                & d \ge 0.
      % \end{cases}
      % \f]
      %           
      
      idx_neg = find(dist < 0);
      cost = zeros(size(dist));
      dcost_ddist = zeros(numel(dist));
      exp_recip_dist = exp(dist(idx_neg).^(-1));
      cost(idx_neg) = -dist(idx_neg).*exp_recip_dist;
      dcost_ddist(sub2ind(size(dcost_ddist),idx_neg,idx_neg)) = ...
        exp_recip_dist.*(dist(idx_neg).^(-1) - 1);
    end
  end
end
