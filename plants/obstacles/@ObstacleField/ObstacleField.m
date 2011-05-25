classdef ObstacleField < handle
   properties
       obstacles;
       number_of_obstacles = 0
   end
   
   methods
       function GenerateRandomObstacles(obj)
           
           % from Michael's code
           %function obs_map=gen_rand_obstacles(x_low,x_high,z_low,z_high,seedval)
           x_low = 0;
           x_high = 10;
           z_low = 0;
           z_high = 10;
           
           seedval = 5;
           
           
            %Obstacle parameters
            OBSTACLE_DENSITY=1.2; %obstacles per 1 unit in z
            NUM_OBSTACLES=floor((z_high-z_low)*OBSTACLE_DENSITY);
            OBS_MIN_EDGE=4;
            OBS_MAX_EDGE=6;
            OBS_MIN_DIM=0.4;
            OBS_MAX_DIM=0.7;

            % Generate random obstacles
            S = RandStream('mt19937ar','Seed',seedval);
            obs_pos=[S.rand(1,NUM_OBSTACLES)*(x_high-x_low)+x_low; S.rand(1,NUM_OBSTACLES)*(z_high-z_low)+z_low];
            obs_num_edges=S.randi(OBS_MAX_EDGE-OBS_MIN_EDGE+1,1,NUM_OBSTACLES)+OBS_MIN_EDGE-1;
            obs_map.coords={};
            obs_map.A={}; % A*x <= b
            obs_map.b={};
            obs_map.nump=zeros(1,NUM_OBSTACLES);

            for k=1:NUM_OBSTACLES
                obs_angles=S.rand(1,obs_num_edges(1,k)); %Random angles between rays
                obs_angles=obs_angles*2*pi/sum(obs_angles);
                for n=2:length(obs_angles)
                    obs_angles(n)=obs_angles(n-1)+obs_angles(n);
                end
                obs_angles=obs_angles+2*pi*S.rand; %Randomize the absolute orientation
                obs_lengths=S.rand(1,obs_num_edges(1,k))*(OBS_MAX_DIM-OBS_MIN_DIM)+OBS_MIN_DIM;
                obs_coords=[cos(obs_angles).*obs_lengths + obs_pos(1,k); sin(obs_angles).*obs_lengths + obs_pos(2,k)];
                obs_coords=obs_coords(:,convhull(obs_coords(1,:)',obs_coords(2,:)'));
                obs_map.coords=[obs_map.coords; {obs_coords}];
                obs_map.nump(k)=size(obs_coords,2)-1;
                
                % create the obstacle object
                
                thisOb = PolygonalObstacle2D(obs_coords(1,:), obs_coords(2,:));
                obj.obstacles{k} = thisOb;
                
                %[A,b]=vert2con(obs_coords');
                %obs_map.A=[obs_map.A; {A}];
                %obs_map.b=[obs_map.b; {b}];
            end

            obj.number_of_obstacles=NUM_OBSTACLES;
           
       end
       
       function draw(obj)
           
           % to draw, we simply call draw() on each of the obstacles in
           % turn
           
           for (i=1:obj.number_of_obstacles)
               obj.obstacles{i}.draw();
           end
           
       end
       
       function [c,dc] = obstacleConstraint(obj,x)
         c=repmat(x(1),obj.number_of_obstacles,1);  % preallocate f (to overcome temporary limitation of TaylorVar)
         dc=zeros(obj.number_of_obstacles,length(x));
         for i=1:obj.number_of_obstacles
           [c(i),dc(i,1:2)] = obj.obstacles{i}.polyconstraint(x(1),x(2));
         end
       end
       
       function con = AddConstraints(obj, con)
         con.x.c = @(x)obj.obstacleConstraint(x);
       end
       
       function [xarray yarray] = GetObstacleCoordinates(obj)
         for (i=1:obj.number_of_obstacles)
           
           obj.obstacles{i}.xarray
           
         end
       end
       
       function [startArray endArray] = GetLineSegments(obj)
         startArray = [];
         endArray = [];
         
         for (i=1:obj.number_of_obstacles)
           [thisStartArray thisEndArray] = obj.obstacles{i}.GetLineSegments();
           
           % TODO preallocate for speed
           
           startArray = [ startArray thisStartArray ];
           endArray = [ endArray thisEndArray ];
           
         end
       end
       
   end
    
    
end
