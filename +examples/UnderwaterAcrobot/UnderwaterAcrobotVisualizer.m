classdef UnderwaterAcrobotVisualizer < Visualizer
    % Implements the draw function for the UnderwaterAcrobot
    
    properties
        L1; %Length from base to first pivot
        X1;    % array of circumference points on link1
        X2;    % array of circumference points on link2
        mr1;   % mass ratio link1 (use for coloring)
        mr2;   % mass ratio link2 (use for coloring)
    end
    
    methods
        function obj = UnderwaterAcrobotVisualizer(plant)
            % Construct visualizer
            
            typecheck(plant,'UnderwaterAcrobotPlant');
            obj = obj@Visualizer(plant.getOutputFrame);
            
            obj.L1 = plant.L1;
            obj.mr1 = plant.mr1;
            obj.mr2 = plant.mr2;
            %npts = 30; %Must be EVEN, same number of points on top and bottom
            
            %Points around ellipse
%             obj.X1 = ellipseCircumference(obj,plant.semiX1,plant.semiY1,npts);
%             obj.X2 = ellipseCircumference(obj,plant.semiX2,plant.semiY2,npts);
            obj.X1 = boxCircumference(obj,plant.L1,plant.d1);
            obj.X2 = boxCircumference(obj,plant.L2,plant.d2);
            
            %Translate so that rotation is about the top, not the center
%             obj.X1 = obj.X1+repmat([0;-plant.semiY1],1,npts);
%             obj.X2 = obj.X2+repmat([0;-plant.semiY2],1,npts);
            obj.X1 = obj.X1+repmat([0;-plant.L1/2],1,4);
            obj.X2 = obj.X2+repmat([0;-plant.L2/2],1,4);
        end
        
        function res = ellipseCircumference(obj,a,b,npts)
            %Creates a set of column vectors of the border points of an ellipse
            x = linspace(-a,a,npts/2);
            y = b*sqrt(1-(x/a).^2);
            
            x = [x fliplr(x)]; y = [y -fliplr(y)];
            res = [x;y];
        end
        function res = boxCircumference(obj,L,d)
            %Creates a set of column vectors of the border points of an ellipse
            x = 0.5*[-d d];
            y = 0.5*[L L];
            
            x = [x fliplr(x)]; y = [y -fliplr(y)];
            res = [x;y];
        end
        
        function draw(obj,t,x)
            % draw the underwateracrobot
            
            R = @(th)[cos(th) -sin(th); sin(th) cos(th)]; %Define Rotation matrix function
            th1 = x(1); th2 = x(2);
            
            %Transform links into global fram
            link1 = R(th1)*obj.X1; %Rotate link1 to global frame
            link2 = R(th1+th2)*obj.X2; %Rotate link2 to global frame
            link2 = link2+repmat(R(th1)*[0;-obj.L1],1,size(link2,2)); %Add to end of other link
            
            %make white for float, black for sink
            patch(link1(1,:),link1(2,:),0*link1(1,:),'FaceColor',1/(1+obj.mr1)*[1,1,1],'Edgecolor','k');
            hold on;
            patch(link2(1,:),link2(2,:),0*link2(1,:),'FaceColor',1/(1+obj.mr2)*[1,1,1],'Edgecolor','k');
            plot3(0,0,2,'k+');
            
            axis image
            view(0,90)
            set(gca,'XTick',[],'YTick',[])
            
            maxlength = max(abs(obj.X1(2,:)))+max(abs(obj.X2(2,:)));
            axis(maxlength*1.5*[-1 1 -1 1 -1 1000]);
            
            xL = xlim(); yL = ylim();
            xwave = linspace(xL(1),xL(2),100);
            ywave = yL(2)-0.5*obj.L1 + 0.1*obj.L1*sin(xwave/(xL(2)-xL(1))*2*pi*5);
            plot3(xwave,ywave,-1+0*xwave,'Color',[0,0.75,1],'Linewidth',3);
        end
        

    end
    
    
    
end
