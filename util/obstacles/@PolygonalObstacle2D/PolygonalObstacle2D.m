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
            
            % note: could make this much cleaner using point to a line 
          
            firstRunFlag = 1;
          
            % first find the distance to the closest edge of the obstacle
            minDist = inf;
            dminDist = zeros(1,2);
            
            for (i=1:length(obj.xvector))
              
              
                if (i ~= length(obj.xvector))
                  iPlusOne = i+1;
                else
                  iPlusOne = 1; % wrap around to the first point on the polygon
                end
              
                % for each line in the polygon...
                
                % compute the distance to that line

                
                % check for a vertical line
                if (obj.xvector(iPlusOne) == obj.xvector(i))
                    % vertical line
                    
                    % check to see if we're going to land on the line
                    % segment
                    
                    delta1 = y - obj.yvector(i);
                    delta2 = y - obj.yvector(iPlusOne);
                    
                    if (sign(delta1) == sign(delta2))
                        % not on the line segment
                        
                        % distance is the distance to the closest edge point
                        dist1 = (x-obj.xvector(i))^2 + (y-obj.yvector(i))^2;
                        dist2 = (x-obj.xvector(iPlusOne))^2 + (y - obj.yvector(iPlusOne))^2;

%                        distance = min(dist1, dist2);
                        if (dist1<dist2) 
                          distance = dist1;
                          ddistance = [2*(x-obj.xvector(i)), 2*(y-obj.yvector(i))];
                        else
                          distance = dist2;
                          ddistance = [2*(x-obj.xvector(iPlusOne)), 2*(y-obj.yvector(iPlusOne))];
                        end
%                        distance = sqrt(distance);
% note: moved all sqrts to the end (so I have less gradients to write!) -Russ
                        
                        
                    else
                        % on the line segment
                        
                        % distance is simply the distance in the x
                        % direction
                        
%                        distance = abs(x - obj.xvector(i));
                        distance = (x-obj.xvector(i))^2;
                        ddistance = 2*(x-obj.xvector(i));
                    end
                    
                else % not a vertical line
                    

                    % compute slope of the line
                    m = (obj.yvector(iPlusOne) - obj.yvector(i)) / (obj.xvector(iPlusOne) - obj.xvector(i));

                    b = obj.yvector(i) - m*obj.xvector(i);

                    % compute y-intercept of orthognal line through the point
                    % in question

                    c = y + 1/m*x;

                    % compute where the two lines intersect
                    xIntersect = (c - b) / (m + 1/m);


                    % check to see if the intersection is on the line segment
                    delta1 = xIntersect - obj.xvector(iPlusOne);
                    delta2 = xIntersect - obj.xvector(i);

                    if (sign(delta1) == sign(delta2))
                        % not on the line segment

                        % distance is the distance to the closest edge point
                        dist1 = (x-obj.xvector(i))^2 + (y-obj.yvector(i))^2;
                        dist2 = (x-obj.xvector(iPlusOne))^2 + (y - obj.yvector(iPlusOne))^2;

%                        distance = min(dist1, dist2);
                        if (dist1<dist2) 
                          distance = dist1;
                          ddistance = [2*(x-obj.xvector(i)), 2*(y-obj.yvector(i))];
                        else
                          distance = dist2;
                          ddistance = [2*(x-obj.xvector(iPlusOne)), 2*(y-obj.yvector(iPlusOne))];
                        end
%                        distance = sqrt(distance);

                    else
                        % on the line segment

                        % distance is the distance from the point on the line
                        % segment to the point in question

                        yIntersect = m * xIntersect + b;

%                        distance = sqrt( (yIntersect - y)^2 + (xIntersect - x)^2 );
                        distance = (yIntersect - y)^2 + (xIntersect - x)^2 ;
                        ddistance = [-2*(xIntersect - x), -2*(yIntersect - y)];

                    end
                end
                if (distance < minDist || firstRunFlag == 1)
                    minDist = distance;
                    dminDist = ddistance;
                    firstRunFlag = 0;
                end
            end
            
            
            insidePoly = inpolygon(x, y, obj.xvector, obj.yvector);
            %distance = (x - obj.centroid(1)) ^2 + (y - obj.centroid(2))^2;
            
            
            if (insidePoly == 1)
                signInside = 1;
            else
                signInside = -1;
            end
            
            dminDist = .5/sqrt(minDist)*dminDist;
            minDist = sqrt(minDist);

            phi = tanh(signInside * minDist);
            dphi = (1-tanh(signInside * minDist)^2)*signInside*dminDist;
            
        end
        
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
