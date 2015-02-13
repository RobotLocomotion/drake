classdef CartesianMotionPlanningTree < VertexArrayMotionPlanningTree
  methods
    function obj = CartesianMotionPlanningTree(num_vars)
      obj = obj@VertexArrayMotionPlanningTree(num_vars);
    end

    function q = interpolate(obj, q1, q2, f)
      q = bsxfun(@times,1-f,q1) + bsxfun(@times,f,q2);
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
