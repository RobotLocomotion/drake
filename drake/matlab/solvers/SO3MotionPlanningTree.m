classdef SO3MotionPlanningTree < VertexArrayMotionPlanningTree
  properties
    visualization_origin = [0; 0; 0] % world point about which the body rotates
    visualization_point  = [1; 0; 0] % point relative to the body origin whose
                                     % position will be drawn to visualize the tree
  end

  methods
    function obj = SO3MotionPlanningTree()
      obj = obj@VertexArrayMotionPlanningTree(4);
    end

    function q = randomSample(obj)
      q = uniformlyRandomQuat();
    end

    function d = distanceMetric(obj, quat1, quat2)
      d = quaternionDistance(quat1, quat2);
    end

    function q = interpolate(obj, q1, q2, interpolation_parameter)
      q = slerp(q1, q2, interpolation_parameter);
    end

    function obj = drawTree(obj, ~, draw_now)
      if nargin < 3, draw_now = true; end
      obj.lcmgl.glColor3f(obj.line_color(1), obj.line_color(2), ...
                          obj.line_color(3));
      for jj = 2:obj.n
        xyz = [quatRotateVec(obj.V(:,jj), obj.visualization_point), ...
               quatRotateVec(obj.V(:,obj.parent(jj)), obj.visualization_point)];
        xyz = bsxfun(@plus, xyz, obj.visualization_origin);
        obj.lcmgl.plot3(xyz(1,:), xyz(2,:), xyz(3,:));
      end
      if draw_now
        obj.lcmgl.switchBuffers();
      end
    end

    function drawPath(obj, varargin)
      obj.drawTree([], false);
      q_path = extractPath(obj, varargin{:});
      path_length = size(q_path, 2);
      xyz = NaN(3, path_length);
      for i = 1:path_length
        xyz(:, i) = quatRotateVec(q_path(:,i), obj.visualization_point) ...
                    + obj.visualization_origin;
      end
      obj.lcmgl.glLineWidth(2);
      obj.lcmgl.glColor3f(0,1,0);
      obj.lcmgl.plot3(xyz(1,:), xyz(2,:), xyz(3,:));
      obj.lcmgl.switchBuffers();
    end
  end
end
