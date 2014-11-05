classdef ObstacleField
  % Holds many Obstacles, and can operate on them.
  % At the moment, is useful for PolygonalObstacle2D's.

   properties
       obstacles; % cell array of obstacle instances
       number_of_obstacles = 0 % number of obstacles currently in the field
   end
   
   methods
       function obj = GenerateRandomObstacles(obj)
           % NOTE: currently locked into a seed value, so will generate the SAME FIELD every time.
           % Generates a field of random PolygonalObstacle2D's.  
           %
           % @retval obj ObstacleField generated
           
           % from Michael's code
           %function obs_map=gen_rand_obstacles(x_low,x_high,z_low,z_high,seedval)
           x_low = 0;
           x_high = 10;
           z_low = 0;
           z_high = 10;
           
           %seedval = 11;
           seedval = 5;
           
           
            %Obstacle parameters
            OBSTACLE_DENSITY=1.2; %obstacles per 1 unit in z
            NUM_OBSTACLES=floor((z_high-z_low)*OBSTACLE_DENSITY);
            %NUM_OBSTACLES=4;
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
           % Draws the obstacle field.  Implemented by calling draw() on each Obstacle.
           
           % to draw, we simply call draw() on each of the obstacles in
           % turn
           
           for (i=1:obj.number_of_obstacles)
               obj.obstacles{i}.draw();
           end
           
       end
       
       function [fieldCost fieldG] = GetCost(obj, x)
         % Returns the cost and gradients at the point x.
         %
         % @param x x = [ x y ] for the point (x, y)
         %
         % @retval fieldCost cost at the point specified by the vector x
         % @retval fieldG gradients at that point.
         
         totalVal = 0;
         totalDeriv = [0 0 0 0];
         
         for (i=1:obj.number_of_obstacles)
            thisfunc = obj.obstacles{i}.GetConstraintFunction();
           
            if (nargout > 1)
                [thisVal, thisDeriv] = thisfunc(x);
                totalVal = totalVal + 1 + thisVal;
                totalDeriv = totalDeriv + thisDeriv;
            else
                thisVal = thisfunc(x);
                totalVal = totalVal + 1 + thisVal;
            end
            
          end
         %[fieldCost fieldG] = ConstraintAdder(obj, x);
         fieldCost = totalVal;
         
         if (nargout > 1)
            fieldG = totalDeriv;
         end
       end
       
       %{
       function [totalVal totalDeriv] = ConstraintAdder(obj, x)
          totalVal = 0;
          totalDeriv = [0 0 0 0];
         
           
          for (i=1:obj.number_of_obstacles)
            thisfunc = obj.obstacles{i}.GetConstraintFunction();
           
            [thisVal, thisDeriv] = thisfunc(x);
            totalVal = totalVal + 1 + thisVal;
            
%             if (thisVal > totalVal)
%               totalVal = thisVal;
%               totalDeriv = thisDeriv;
%             end
            
            %totalVal = max(totalVal, thisVal);
           
            totalDeriv = totalDeriv + thisDeriv; 
          end
          totalVal = totalVal - 1;
          
        end
       %}
       %{
       function con = AddConstraints(obj, con)
         % create a function which adds all the constraints of the
         % individual obstacles
         
        for (i=1:obj.number_of_obstacles)
          con.x.cn{i} = obj.obstacles{i}.GetConstraintFunction();
        end
         
         %%con.x.c = obj.obstacles{10}.GetConstraintFunction();
         %allConstraints = @(x) ConstraintAdder(obj, x);
         
         %con.x.c = allConstraints;
       end
       %}
       
       function [c,dc] = obstacleConstraint(obj,x)
           % A second implementation of GetCost.  This function and GetCost
           % should be merged into one.
           
           c=repmat(x(1),obj.number_of_obstacles,1);  % preallocate f (to overcome temporary limitation of TaylorVar)
           dc=zeros(obj.number_of_obstacles,length(x));
           for i=1:obj.number_of_obstacles
               [c(i),dc(i,1:2)] = obj.obstacles{i}.polyconstraint(x(1),x(2));
           end
       end
       
       function prog = AddConstraints(obj, prog)
           % Adds the non-convex constraints for the ObstacleField to the 
           % given NonlinearProgram
           %
           % @param prog a DirectTrajectoryOptimization
           % @retval prog the updated NonlinearProgram
           
           prog = prog.addStateConstraint(FunctionHandleConstraint(-inf(obj.number_of_obstacles,1),zeros(obj.number_of_obstacles,1),2,@(x)obj.obstacleConstraint(x)),1:prog.N,1:2);
       end
       
       
       function [startArray endArray] = GetLineSegments(obj)
         % Collects all line segements from the entire array (of assumed PolygonalObstacle2D's
         % See PolygonalObstacle2D.GetLineSegments(...) for more details.
         % 
         % @retval startArray [ startArray (obstacle1), startArray (obstacle2), ..etc.. ]
         % @retval endArray [ endArray (obstacle1), endArray (obstacle2), ..etc.. ]
         startArray = [];
         endArray = [];
         
         for (i=1:obj.number_of_obstacles)
           [thisStartArray thisEndArray] = obj.obstacles{i}.GetLineSegments();
           
           % TODO preallocate for speed
           
           startArray = [ startArray thisStartArray ];
           endArray = [ endArray thisEndArray ];
           
         end
       end
       
       function field2 = GetRelativeField(obj, x, y, theta)
         % Get an ObstacleField that is relative to the x, y, and rotation (theta) value given
         % This effectively puts your point at the orgin and translates/rotates the obstacle field around
         % it.
         %
         % @param x x-coordinate for the new orgin
         % @param y y-coordinate for the new orgin
         % @param theta rotation amount
         %
         % @retval field2 new translated and rotated obstacle field

         field2 = obj;
         theta = -theta;
         for i=1:field2.number_of_obstacles
           
           % translate the obstacle 
           field2.obstacles{i}.xvector = field2.obstacles{i}.xvector - x;
           field2.obstacles{i}.yvector = field2.obstacles{i}.yvector - y;
           
           % rotate the obstacle field by the angle of the plane
           % we can use a standard rotation matrix because we've already
           % centered everything around the plane
           
           
           
           rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
           
           % rotate each obstacle line
           for j=1:length(field2.obstacles{i}.xvector)
             
             linevector = [field2.obstacles{i}.xvector(j); field2.obstacles{i}.yvector(j)];
             
             linevector = rotationMatrix * linevector;
             
             field2.obstacles{i}.xvector(j) = linevector(1);
             field2.obstacles{i}.yvector(j) = linevector(2);
             
           end
             
           
         end
         
       end
       
       function obj = AddObstacle(obj, obstacle)
         % Adds an obstacle to the field and updates the number of obstacles appropriately.
         %
         % @param obstacle new obstacle to add
         %
         % @retval obj new ObstacleField object with the added obstacle
         
         obj.number_of_obstacles = obj.number_of_obstacles + 1;
         obj.obstacles{obj.number_of_obstacles} = obstacle;
       end
       
   end
    
    
end
