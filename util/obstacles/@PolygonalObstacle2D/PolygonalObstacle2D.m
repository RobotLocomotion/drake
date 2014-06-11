classdef PolygonalObstacle2D < Obstacle
    
    methods
        
        % constructor
        % x and y are vectors of points in clockwise order
        function obj = PolygonalObstacle2D(xPoints, yPoints)
            obj = obj@Obstacle();
            
            obj.xvector = xPoints;
            obj.yvector = yPoints;
            
            % get the centroid
            %[geom, iner, cpmo] = polygeom(xPoints, yPoints);
            %obj.centroid = geom(2:3);
            %obj.area = geom(1);
            
            % set up the polygonal constraint function
            obj.func = @polyconstraint;
            
            
            % set up the polygonal gradiant function
            % todo
            
            
        end
        
        function con = getConstraints(obj)
           con.x.c = @(x,y) obj.func(obj, x, y);
        end
        
        function draw(obj)
            
            persistent hFig;

            if (isempty(hFig))
                hFig = sfigure(25);
                set(hFig,'DoubleBuffer', 'on');
            end
            
            % plot outline
            xplot = [obj.xvector obj.xvector(1) ];
            yplot = [obj.yvector obj.yvector(1) ];
            
            line(xplot, yplot,'LineWidth',2);
            
            
        end
        
        function [phi,dphi] = polyconstraint(obj, x, y)
            
    
            
            % See if we are actually in a non-generic configuration
            % (i.g. closest point is not a vertex, but a point on an edge)
            
            % Find distance to all edges
            firstRunFlag = 1;
          
            % find the distance to the closest edge of the obstacle
            minDist = inf;
            dminDist = zeros(1,2);
            xy = [x;y];
            
            for (i=1:length(obj.xvector))
                if (i ~= length(obj.xvector))
                  iPlusOne = i+1;
                else
                  iPlusOne = 1; % wrap around to the first point on the polygon
                end
              
                % for each line in the polygon...
                % compute the distance to that line

                % find the direction vector of the line
                pt1 = [obj.xvector(i); obj.yvector(i)];
                pt2 = [obj.xvector(iPlusOne); obj.yvector(iPlusOne)];
                u = pt2 - pt1;
                u = u / norm(u);

                if all(pt2 == pt1) % special case for edges of zero length
                  distance = sum(([x;y] - pt1).^2);
                  ddistance = 2*([x;y]-pt1);
                else
                  % find a and b such that a' * xy = b describes the line
                  a = [-u(2); u(1)];
                  b = a' * pt1;

                  % find distance g such that the point (xy - g*a) is on the line:
                  % a'*(xy - g*a) = b
                  % a'*xy - g*a'*a = b
                  % g = (a'*xy - b) / (a'*a) = (a'*xy - b) since a is unit vector by construction
                  g = (a' * xy - b);

                  % Check if the intersection is on the line
                  delta1 = u' * xy - u' * pt1;
                  delta2 = u' * xy - u' * pt2;
                  if sign(delta1) == sign(delta2)
                      % not on the line segment
                      % distance is the distance to the closest edge point
                      if delta2 > 0
                          distance = sum((xy - pt2).^2);
                          ddistance = 2*(xy - pt2)';
                      else
                          distance = sum((xy - pt1).^2);
                          ddistance = 2*(xy - pt1)';
                      end
                  else
                      % on the line segment
                      % distance is the distance from the point on the line
                      % segment to the point in question
                      distance = g^2;
                      if g >= 0
                          ddistance = 2*a';
                      else
                          ddistance = -2*a';
                      end
                  end
                end

                if (distance < minDist || firstRunFlag == 1)
                    minDist = distance;
                    dminDist = ddistance;
                    firstRunFlag = 0;
                end
            end

            insidePoly = inpolygon(x, y, obj.xvector, obj.yvector);
            
            if (insidePoly == 1)
                signInside = 1;
            else
                signInside = -1;
            end
            
            dminDist = .5/sqrt(minDist)*dminDist;
            minDist = sqrt(minDist);

            phi_edges = tanh(signInside * minDist);
            dphi_edges = (1-tanh(signInside * minDist)^2)*signInside*dminDist;
            minDist_edges = minDist;
            
            % Now see if we were actually in generic configuration or not,
            % and then return distance accordingly
            
             % Assume that we have a "generic" configuration
            % (i.e., closest point is a vertex)
            % Find distance to all vertices
            dists = Inf*ones(length(obj.xvector),1);
            for k = 1:length(obj.xvector)
                dists(k) = norm([obj.xvector(k);obj.yvector(k)] - [x;y]);
            end
            
            [minDist,minInd] = min(dists);
            
            % Figure out if point is in polygon
            insidePoly = inpolygon(x, y, obj.xvector, obj.yvector);
            
            if (insidePoly == 1)
                signInside = 1;
            else
                signInside = -1;
            end
            
            dminDist = [(x-obj.xvector(minInd))/minDist; ...
                        (y-obj.yvector(minInd))/minDist];        

            phi_verts = tanh(signInside * minDist);
            dphi_verts = (1-tanh(signInside * minDist)^2)*signInside*dminDist;
            minDist_verts = minDist;
                   
            if minDist_edges < minDist_verts 
                phi = phi_edges;
                dphi = dphi_edges;
            else % generic
                phi = phi_verts;
                dphi = dphi_verts;
            end
                
             
  
        end

%         function [phi,dphi] = polyconstraint(obj, x, y)
%             firstRunFlag = 1;
%           
%             % first find the distance to the closest edge of the obstacle
%             minDist = inf;
%             dminDist = zeros(1,2);
%             xy = [x;y];
%             
%             for (i=1:length(obj.xvector))
%                 if (i ~= length(obj.xvector))
%                   iPlusOne = i+1;
%                 else
%                   iPlusOne = 1; % wrap around to the first point on the polygon
%                 end
%               
%                 % for each line in the polygon...
%                 % compute the distance to that line
% 
%                 % find the direction vector of the line
%                 pt1 = [obj.xvector(i); obj.yvector(i)];
%                 pt2 = [obj.xvector(iPlusOne); obj.yvector(iPlusOne)];
%                 u = pt2 - pt1;
%                 u = u / norm(u);
% 
%                 if all(pt2 == pt1) % special case for edges of zero length
%                   distance = sum(([x;y] - pt1).^2);
%                   ddistance = 2*([x;y]-pt1);
%                 else
%                   % find a and b such that a' * xy = b describes the line
%                   a = [-u(2); u(1)];
%                   b = a' * pt1;
% 
%                   % find distance g such that the point (xy - g*a) is on the line:
%                   % a'*(xy - g*a) = b
%                   % a'*xy - g*a'*a = b
%                   % g = (a'*xy - b) / (a'*a) = (a'*xy - b) since a is unit vector by construction
%                   g = (a' * xy - b);
% 
%                   % Check if the intersection is on the line
%                   delta1 = u' * xy - u' * pt1;
%                   delta2 = u' * xy - u' * pt2;
%                   if sign(delta1) == sign(delta2)
%                       % not on the line segment
%                       % distance is the distance to the closest edge point
%                       if delta2 > 0
%                           distance = sum((xy - pt2).^2);
%                           ddistance = 2*(xy - pt2)';
%                       else
%                           distance = sum((xy - pt1).^2);
%                           ddistance = 2*(xy - pt1)';
%                       end
%                   else
%                       % on the line segment
%                       % distance is the distance from the point on the line
%                       % segment to the point in question
%                       distance = g^2;
%                       if g >= 0
%                           ddistance = a';
%                       else
%                           ddistance = -a';
%                       end
%                   end
%                 end
% 
%                 if (distance < minDist || firstRunFlag == 1)
%                     minDist = distance;
%                     dminDist = ddistance;
%                     firstRunFlag = 0;
%                 end
%             end
% 
%             insidePoly = inpolygon(x, y, obj.xvector, obj.yvector);
%             
%             if (insidePoly == 1)
%                 signInside = 1;
%             else
%                 signInside = -1;
%             end
%             
%             dminDist = .5/sqrt(minDist)*dminDist;
%             minDist = sqrt(minDist);
% 
%             phi = tanh(signInside * minDist);
%             dphi = (1-tanh(signInside * minDist)^2)*signInside*dminDist;
%         end
        
        function [startArray endArray] = GetLineSegments(obj)
          
          % have:
          % xvector:
          %   [ x1 x2 x3 x4]
          % yvector
          %   [ y1 y2 y3 y4]
          
          % want:
          % start:
          %   [ x1    x2    x3    x4   ]
          %     y1    y2    y3    y4
          %
          % end:
          %   [ x2    x3    x4    x1   ]
          %     y2    y3    y4    y1
          % 
          
          startArray(1,:) = obj.xvector;
          startArray(2,:) = obj.yvector;
          
          endArray(1,:) = [ obj.xvector(2:end) obj.xvector(1) ];
          endArray(2,:) = [ obj.yvector(2:end) obj.yvector(2) ];

          
          
        end
        
    end
    
    properties
        %centroid
        %area
        xvector
        yvector
        
    end
    
    
end
