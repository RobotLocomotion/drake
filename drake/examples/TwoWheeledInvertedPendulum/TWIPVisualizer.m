classdef TWIPVisualizer < Visualizer
  methods
    function obj = TWIPVisualizer(plant)
      
      typecheck(plant,'TWIP');
      obj = obj@Visualizer(plant.getStateFrame);
      obj.R = plant.R;
      obj.L = plant.L;
    end
    
    function draw(obj,t,x)
      L=2*obj.L; R=obj.R;
      alpha = x(1);
      beta = x(2);
      body_comx = -sin(alpha)*L-beta*R;
      body_comy = cos(alpha)*L + R;
      wheel_comx = -beta*R;
      wheel_comy = R;
      hold on
      line([-5, 5], [0, 0], 'Color', [.3 .5 1], 'LineWidth', 1);
      axis([-0.1 0.1 -0.3 0.3]);
      line([wheel_comx, body_comx], [wheel_comy, body_comy], 'Color', MITgray, 'LineWidth', 15);
      circle(wheel_comx, wheel_comy, R, MITred);
      line([wheel_comx, wheel_comx + R*cos(beta+pi/2)], [wheel_comy,  wheel_comy + R*sin(beta+pi/2)], 'Color', [0 0 0], 'LineWidth', 5);
      axis equal
    end
  end

  properties
    L=0.190;
    R=0.040;
  end
  
end

function h = circle(x,y,r, color)
    th = 0:pi/50:2*pi;
    hold on;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  fill(xunit, yunit, color);
  hold off;
end
