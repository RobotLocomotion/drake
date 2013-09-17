classdef SLIPVisualizer < Visualizer
  
  methods 
    function obj = SLIPVisualizer(slip)
      typecheck(slip,'SLIP');
      obj = obj@Visualizer(getOutputFrame(slip));
    end
    
    function draw(obj,t,x)
      hip = x(1:2);
      r = x(3); 
      theta = x(4);

      lx=linspace(0,1,51);
      ly=.075*sin(8*pi*lx);
      lx = [0,.3,.5*lx+.3,.8,1];
      ly = [0,0,ly,0,0];
      R = rotmat(theta-pi/2);
      lpts = repmat(hip,1,numel(lx))+R*[r*lx;ly];
      plot(lpts(1,:),lpts(2,:),'Color',[0 0 0],'LineWidth',2);
      
      hold on
      t = 0:0.1:2*pi;
      line(hip(1)+0.15*sin(t),hip(2)+0.15*cos(t),'Color',[0 0 0]);
      fill(hip(1)+0.15*sin(t),hip(2)+0.15*cos(t),MITlightgray); %[ 0.502 1.000 1.000 ]);

      axis equal;
      if (isempty(obj.axis))
        v(1) = 4*floor(hip(1)/4)-.2;
        v(2) = v(1)+4.4;
        v(3) = -.1;
        v(4) = 3;
        line([v(1)-1 v(2)+1],[0 0],'LineWidth',1.5,'Color',MITred);
        axis(v)
      else
        line(obj.axis(1:2),[0 0],'LineWidth',1.5,'Color',MITred);
        axis(obj.axis);
      end

      drawnow;
    end
  end
end
