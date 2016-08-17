classdef CartesianMotionPlanningTree < VertexArrayMotionPlanningTree
  % Concrete implementation of the MotionPlanningTree interface for planning on
  % $R^n$ with a Euclidean distance metric
  methods
    function obj = CartesianMotionPlanningTree(num_vars)
      % @param num_vars   Numeric scalar indicating the dimension of the space
      %                   on which the tree is grown
      obj = obj@VertexArrayMotionPlanningTree(num_vars);
    end

    function q = interpolate(obj, q1, q2, interpolation_parameter)
      % q = interpolate(obj, q1, q2, interpolation_parameter) returns points
      % along the line between q1 and q2 at the locations specified by
      % interpolation_parameter.
      %
      % @param q1   Initial point
      % @param q2   Final point
      % @param interpolation_parameter  Vector of doubles between 0 and 1,
      %                                 inclusive.
      %
      % @retval q   num_vars-by-numel(interpolation_parameter) array. Each
      %             column of q is an interpolated point whose position along
      %             the line between q1 and q2 is given by the corresponding
      %             element of interpolation_paramter:
      %
      %               interpolation_parameter(i) == 0 implies q(:,i) == q1
      %               interpolation_parameter(i) == 1 implies q(:,i) == q2
      q = bsxfun(@times,1-interpolation_parameter,q1) + ...
          bsxfun(@times,interpolation_parameter,q2);
    end

    function d = distanceMetric(obj, q1, q_array)
      d = sqrt(sum(bsxfun(@minus, q1, q_array).^2,1));
    end

    function obj = drawTree(obj, n_at_last_draw)
      line([obj.V(1,(n_at_last_draw+1):obj.n); ...
            obj.V(1,obj.parent((n_at_last_draw+1):obj.n))], ...
           [obj.V(2,n_at_last_draw+1:obj.n); ...
            obj.V(2,obj.parent(n_at_last_draw+1:obj.n))], ...
           'Color',obj.line_color,'Marker','.');
      axis equal
      drawnow
    end

    function obj = drawPath(obj, varargin)
      hold on
      q_path = extractPath(obj, varargin{:});
      plot(q_path(1,:), q_path(2,:), 'g');
      hold off
      axis equal
      drawnow
    end
  end
end
