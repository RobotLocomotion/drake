classdef DoublePendVisualizer < Visualizer

  methods
    function obj = DoublePendVisualizer(plant)
      % Construct visualizer, and set l1 and l2 parameters
      
      typecheck(plant,'DoublePendPlant');
      
      obj = obj@Visualizer(plant.getStateFrame());
      
      obj.l1 = plant.l1;
      obj.l2 = plant.l2;
      obj.m1 = plant.m1;
      obj.m2 = plant.m2;
    end
    
    function draw(obj,t,x)
      l1=obj.l1; l2=obj.l2; m1=obj.m1; m2=obj.m2;
      scale=0.1;
      
      x1 = l1*[sin(x(1)); -cos(x(1))];
      x2 = x1 + l2*[sin(x(1)+x(2)); -cos(x(1)+x(2))];

      line([0, x1(1)], [0, x1(2)],'LineWidth',2.0);
      line([x1(1), x2(1)], [x1(2), x2(2)],'LineWidth',2.0);

      th = 0:0.1:2*pi;
      fill(x1(1) + scale*m1*sin(th), x1(2) + scale*m1*cos(th),[0 0 1]);
      fill(x2(1) + scale*m2*sin(th), x2(2) + scale*m2*cos(th),[0 0 1]);
      
      l = 1.2*(l1+l2);
      axis equal;
      axis([-l l -l l])

      title(['t = ', num2str(t,'%0.2f')]);
    end
  end

  properties
    l1=1;    % length of link 1
    l2=2;    % length of link 2
    m1=1;
    m2=1;
  end
  
end
