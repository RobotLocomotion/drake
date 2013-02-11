classdef AcrobotVisualizer < Visualizer
% Implements the draw function for the Acrobot 

  methods
    function obj = AcrobotVisualizer(plant)
      % Construct visualizer
      %   AcrobotVisualizer(AcrobotPlant) will take the necessary
      %   parameters from the plant class
      
      typecheck(plant,'AcrobotPlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.l1 = plant.l1;
      obj.l2 = plant.l2;
    end
    
    function draw(obj,t,x)
      % draw the acrobot
      persistent L1r L1a L2r L2a;
      l1=obj.l1; l2=obj.l2;
  
      if (isempty(L1r))
        av = pi/2*[1:.05:3];
        r = .04*min([l1 l2]);
        L1x = [r*cos(av) l1+r*cos(av+pi)];
        L1y = [r*sin(av) r*sin(av+pi)];
        L1r = (L1x.^2+L1y.^2).^.5;
        L1a = atan2(L1y,L1x);
        L2x = [r*cos(av) l2+r*cos(av+pi)];
        L2y = [r*sin(av) r*sin(av+pi)];
        L2r = (L2x.^2+L2y.^2).^.5;
        L2a = atan2(L2y,L2x);
      end
  
      patch(L1r.*sin(L1a+x(1)),-L1r.*cos(L1a+x(1)),0*L1a,'r');
      hold on
      patch(l1*sin(x(1))+L2r.*sin(L2a+x(1)+x(2)),-l1*cos(x(1))-L2r.*cos(L2a+x(1)+x(2)),1+0*L2a,'b');
      plot3(0,0,2,'k+');
      axis image
      view(0,90)
      set(gca,'XTick',[],'YTick',[])
      axis((l1+l2)*1.1*[-1 1 -1 1 -1 1000]);
    end
  end

  properties
    l1=1;    % length of link 1
    l2=2;    % length of link 2
  end
  
end
