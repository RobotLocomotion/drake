classdef RimlessWheelVisualizer < Visualizer
  
  properties
    l, alpha, gamma
  end
  
  methods 
    function obj = RimlessWheelVisualizer(l,alpha,gamma)
      obj = obj@Visualizer(4);
      if (isa(l,'RimlessWheelPlant'))
        obj.l = l.l;
        obj.alpha = l.alpha;
        obj.gamma = l.gamma;
      else
        obj.l = l;
        obj.alpha = alpha;
        obj.gamma = gamma;
      end
    end
    
    function draw(obj,t,x)
      x = x(2:end);  % first variable is the mode, which is always 1.
      l = obj.l;  alpha = obj.alpha; gamma = obj.gamma;
      
      persistent hFig;

      if (isempty(hFig))
        hFig = figure(25);
        set(hFig,'DoubleBuffer','on');
      end
      
      figure(hFig);
      cla;
      hip = x(3)*[cos(gamma);-sin(gamma)] + l*[sin(x(1));cos(x(1))];
      hold on;
      for t=x(1)+[0:2*alpha:pi]
        line(hip(1)+l*sin(t)*[1;-1], hip(2)+l*cos(t)*[1;-1],'Color',[0 0 0],'LineWidth',2);
      end
      t = 0:0.1:2*pi;
      line(hip(1)+0.15*sin(t),hip(2)+0.15*cos(t),'Color',[0 0 0]);
      fill(hip(1)+0.15*sin(t),hip(2)+0.15*cos(t),[ 0.502 1.000 1.000 ]);
      line(hip(1)+[-2,2],tan(-gamma)*(hip(1)+[-4,4]));
      axis equal;
      
      drawnow;
    end
  end
end
