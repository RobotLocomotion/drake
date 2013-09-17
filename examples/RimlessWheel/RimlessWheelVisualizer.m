classdef RimlessWheelVisualizer < Visualizer
  
  properties
    l, alpha, gamma
  end
  
  methods 
    function obj = RimlessWheelVisualizer(plant)
      typecheck(plant,'RimlessWheelPlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.l = plant.l;
      obj.alpha = plant.alpha;
      obj.gamma = plant.gamma;
    end
    
    function draw(obj,t,x)
      l = obj.l;  alpha = obj.alpha; gamma = obj.gamma;
      
      hip = x(3)*[cos(gamma);-sin(gamma)] + l*[sin(x(1));cos(x(1))];
      hold on;
      for t=x(1)+[0:2*alpha:pi]
        line(hip(1)+l*sin(t)*[1;-1], hip(2)+l*cos(t)*[1;-1],'Color',[0 0 0],'LineWidth',2);
      end
      t = 0:0.1:2*pi;
      line(hip(1)+0.15*sin(t),hip(2)+0.15*cos(t),'Color',[0 0 0]);
      fill(hip(1)+0.15*sin(t),hip(2)+0.15*cos(t),MITlightgray); %[ 0.502 1.000 1.000 ]);
      axis equal;
      if (isempty(obj.axis))
        line(hip(1)+[-2,2],tan(-gamma)*(hip(1)+[-4,4]),'LineWidth',1.5,'Color',MITred);
      else
        line(obj.axis(1:2),tan(-gamma)*obj.axis(1:2),'LineWidth',1.5,'Color',MITred);
        axis(obj.axis);
      end
    end
  end
end
