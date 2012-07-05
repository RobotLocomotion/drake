classdef CompassGaitVisualizer < Visualizer
  
  properties
    a,b,l,gamma
  end
  
  methods 
    function obj = CompassGaitVisualizer(plant)
      typecheck(plant,'CompassGaitPlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.a = plant.a;
      obj.b = plant.b;
      obj.l = plant.l;
      obj.gamma = plant.gamma;
    end
    
    function draw(obj,t,x)
%      m=mod(x(1),2); % left or right stance

      q = x(1:2);
      qd = x(3:4);

      a = obj.a;  b = obj.b; l = obj.l; 
      
      h = sfigure(25);
      set(h,'DoubleBuffer','on');
      clf;
      hold on;
      hip = l*[-sin(q(2)); cos(q(2))];
      swfoot = hip - l*[-sin(q(1)); cos(q(1))];
      
      x = hip(1) + 1*[-1,1];
      line(x, tan(-obj.gamma)*x,'Color',MITgray,'LineWidth',1.5);
      
      line([0, hip(1)], [0, hip(2)],'LineWidth',2.0,'Color','k');%MITgray);%[(1-m) 0 m]);
      line([hip(1), swfoot(1)], [hip(2), swfoot(2)],'LineWidth',2.0,'Color','k');%MITgray);%[m 0 1-m]);
      
      x = 0:0.1:2*pi;
      fill(hip(1) + 0.05*sin(x), hip(2) + 0.05*cos(x),MITred);%[0 1 0]);
      fill(-a*sin(q(2)) + 0.05*sin(x), a*cos(q(2)) + 0.05*cos(x),MITred);%[1-m 0 m]);
      fill(hip(1)+b*sin(q(1)) + 0.05*sin(x), hip(2)-b*cos(q(1)) + 0.05*cos(x),MITred);%[m 0 1-m]);
      
      axis equal;
      %  axis off;
      hold off;
      
      drawnow;
    end
  end
end
