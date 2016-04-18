classdef HolonomicDriveVisualizer < Visualizer
  properties
    wheels
    outline
    radius


    fade_percent = 0;
  end

  methods
    function obj = HolonomicDriveVisualizer(plant)
      typecheck(plant, 'HolonomicDrivePlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.wheels = plant.wheels;

      vertices = [obj.wheels.pos];
      try
        obj.outline = vertices(:, convhull(vertices(1,:), vertices(2,:)));
      catch e
        warning(['Couldn''t get convex hull of body: ' e.message]);
        obj.outline = vertices;
      end

      obj.radius = sum(cellfun(@(x) norm(x), num2cell(vertices, 1))) / length(obj.wheels);
    end

    function draw(obj, ~, x)
      axis 'equal';
      grid on;

      if length(x) == 6
        theta = x(3);
        rotation = rotmat(theta);
      else
        % for trigPolySystem
        rotation = [[x(3); x(4)] [-x(4); x(3)]];
      end

      % draw the body
      translation = repmat(x(1:2), 1, 4);
      body = translation + (rotation * obj.outline);
      patch(body(1,:), body(2,:), 'r', 'FaceAlpha', 1-obj.fade_percent);

      corners = [1 1 -1 -1; 0.1 -0.1 -0.1 0.1];

      % draw the wheels
      for wheel = obj.wheels
        % corners of wheels in wheel space
        w_corners = wheel.r * [wheel.driveDir wheel.slipDir] * corners;

        % corners of wheels in body space
        b_corners = repmat(wheel.pos, 1, 4) + w_corners;

        % corners of wheels in global space
        g_corners = translation + (rotation * b_corners);

        patch(g_corners(1,:), g_corners(2,:), [0.5 0.5 0.5], 'FaceAlpha', 1-obj.fade_percent);
      end
      forward = rotation(:,1)*obj.radius;
      q = quiver(x(1), x(2), forward(1), forward(2));
      set(q, 'Color', [0.5 0 0]);
    end
  end
end

