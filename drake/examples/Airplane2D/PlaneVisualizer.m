classdef PlaneVisualizer < Visualizer
% Implements the draw function for the Plane 

  properties
    obstaclefield=[];
  end

  methods
    function obj = PlaneVisualizer(plant,obstaclefield)
      typecheck(plant,'PlanePlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.playback_speed = .2;
      obj.display_dt = 0;
      if (nargin>1)
        obj.obstaclefield = obstaclefield;
      end
    end
    
    function draw(obj,t,x)
        persistent hFig;

        if (isempty(hFig))
            hFig = sfigure(25);
            set(hFig,'DoubleBuffer', 'on');
        end
        sfigure(hFig);
      
        %Airplane definition
        hull_scale=0.04;
        air_hull=[1 3; -1 3; -1 1; -6 1; -6 -1; -1 -1; -0.8 -4.8; -3 -5; -3 -6;...
            3 -6; 3 -5; 0.8 -4.8; 1 -1; 6 -1; 6 1; 1 1; 1 3]';
        air_hull=hull_scale*air_hull;
        x_coll = air_hull;

        %Size of the arena
        X_LOW=-3.5 + x(1);
        X_HIGH=3.5 + x(1);
        Z_LOW=0;
        Z_END=100;

        Z_SCREEN=12; %10 %Screen height
        Z_OFFSET=-1.5;

        z_av = max(0,x(2)-8.5);

        curr_window=[X_LOW,X_HIGH,z_av+Z_OFFSET,z_av+Z_SCREEN+Z_OFFSET];

        axis_size = curr_window;
    
      %function plot_sim(x,obs_map,axis_size,x_coll,scanner,vis_obs,do_clear)
% 
%       %%Plot the obstacles
%       obs_color=[0,0.2,1];
%       scan0_color=[0.7,0.9,0.7];
%       scan1_color=[0.9,0.7,0.7];
% 
%       if(nargin < 6 || isempty(vis_obs))
%           vis_obs=1:obs_map.num_obs;
%       else
%           if(sum(vis_obs < 1) > 0 || sum(vis_obs > obs_map.num_obs) > 0)
%               error('obstacle index outside of allowed range');
%           end
%       end
% 
%       if(nargin < 7 || do_clear ~= 0)
%           figure(1); clf; hold on;
%       end
%       %Plot the scanner
%       if(~isempty(scanner))
%           scanval=get_scanner(x,obs_map,scanner,axis_size,vis_obs);
%           for k=1:size(scanner,3)
%               [scan_x,scan_y]=plane_coll_points(x,scanner(:,:,k));
%               if(scanval(k)==0)
%                   fill(scan_x,scan_y,scan0_color);
%               else
%                   fill(scan_x,scan_y,scan1_color);
%               end
%           end
%       end
% 
%       %%Plot obstacles
%       for k=vis_obs
%           fill(obs_map.coords{k}(1,:),obs_map.coords{k}(2,:),obs_color);
%       end

      %%Plot the airplane
      if(~isempty(x))
          plane_color=[0, 0, 0];
          [x_points,y_points]=plane_coll_points(x,x_coll);
          fill(x_points,y_points,plane_color);
      end
% 
%       %%Plot borders
%       plot([axis_size(1) axis_size(2); axis_size(1) axis_size(2)],...
%           [axis_size(3) axis_size(3); axis_size(4) axis_size(4)],'r','LineWidth',2);

      if (~isempty(obj.obstaclefield))
        obj.obstaclefield.draw();
      end
      
      title(strcat('t = ', num2str(t,'%.5f')));
      axis(axis_size);
      set(gca,'DataAspectRatio',[1 1 1]);
    end    
  end
  
end


function [x_out,y_out]=plane_coll_points(x,x_coll)

%x_points=(x_coll(1,:))*cos(x(4)); %Adjust by roll
x_points=(x_coll(1,:));
y_points=x_coll(2,:);
x_out=x_points*cos(x(3))-y_points*sin(x(3))+x(1);
y_out=x_points*sin(x(3))+y_points*cos(x(3))+x(2);

end
