classdef GliderVisualizer < Visualizer

  properties
    perch_location = [0;0];
    talon_b = [0 -.043]'; % talon coordinates, about cg in body coords (meters).
  end

  methods
    function obj = GliderVisualizer(plant)
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.playback_speed = .2;
      obj.display_dt = 0;
    end
    
    function draw(obj,t,x)
      persistent hFig;

      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
      end
      
      % Draws the glider.  
      % x: [x z theta phi dot(x,z,theta,phi)]
      sc = 2;
      lw = -0.15*sc;
      lh = 0.45*sc;
      le = 0.04*sc;
      mac = .1145*sc;
      
      ct = cos(x(3)); st = sin(x(3));
      ctp = cos(x(3)+x(4)); stp = sin(x(3)+x(4));
      
      sfigure(25); clf; hold on;
      
      %cg
      plot(x(1),x(2),'ro','MarkerSize',4,'MarkerFaceColor',[1 0 0]);
      
      % fuselage
      if (lw < 0)
        line([x(1) x(1)-lh*ct],[x(2) x(2)-lh*st],...
          'LineWidth',1,'Color',[0 0 0]);
      else
        line([x(1)+(lw+mac/2)*ct x(1)-lh*ct],...
          [x(2)+(lw+mac/2)*st x(2)-lh*st],...
          'LineWidth',1,'Color',[0 0 0]);
      end
      
      % wing
      line([x(1)+(lw+mac/2)*ct x(1)+(lw-mac/2)*ct],...
        [x(2)+(lw+mac/2)*st x(2)+(lw-mac/2)*st],...
        'LineWidth',3,'Color',[0 .2 1]);
      
      % elevator
      line([x(1)-lh*ct x(1)-lh*ct-2*le*ctp],...
        [x(2)-lh*st x(2)-lh*st-2*le*stp],...
        'LineWidth',3,'Color',[0 .2 1]);
      
      axis equal; axis([-4.5 1 -1.0 1.0]);
      title(['time: ',num2str(t,2),' s']);
      xlabel('x (m)'); ylabel('z (m)');
      h = gcf;
      scrsz = get(0,'ScreenSize');
      
      % the perch
      plot(obj.perch_location(1),obj.perch_location(2),'ko','MarkerSize',5,'MarkerFaceColor',[0 0 0]);
      
      % the talon
      pitch = x(3); pitchdot = x(7);
      R = [cos(pitch), -sin(pitch); sin(pitch), cos(pitch)];
      talon_pos = x(1:2) + R*obj.talon_b;
      plot(talon_pos(1),talon_pos(2),'bo','MarkerSize',4,'MarkerFaceColor',[0 1 0]);
    end    
  end
  
end
