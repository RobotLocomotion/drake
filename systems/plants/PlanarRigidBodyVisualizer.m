classdef PlanarRigidBodyVisualizer < Visualizer
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  % This is the planar version of the visualizer.  The y-axis of the 
  % URDF file is essentially ignored.  Joints that act out of
  % plane are not supported.  All geometry out of plane is projected onto
  % the plane.
  
  methods
    function obj = PlanarRigidBodyVisualizer(manip,options)
      typecheck(manip,'PlanarRigidBodyManipulator');
      obj=obj@Visualizer(manip.getStateFrame);
      obj.model = manip;
    end
    
    function draw(obj,t,x)

      n = obj.model.num_q;
      q = x(1:n); %qd=x(n+(1:n));
      kinsol = obj.model.doKinematics(q);
      
      % for debugging:
      %co = get(gca,'ColorOrder');
      %h = [];
      % end debugging

      for i=1:length(obj.model.body)
        body = obj.model.body(i);
        for j=1:length(body.geometry)
          c = (1-obj.fade_percent)*body.geometry{j}.c + obj.fade_percent*obj.fade_color;
          pts = obj.model.T_2D_to_3D'*forwardKin(obj.model,kinsol,i,body.geometry{j}.xyz);
          patch(pts(1,:)',pts(2,:)',c,'LineWidth',.01,'EdgeColor',obj.fade_percent*obj.fade_color); %0*xpts,'FaceColor','flat','FaceVertexCData',body.geometry.c);
          % patch(xpts,ypts,body.geometry{j}.c,'EdgeColor','none','FaceAlpha',1); %0*xpts,'FaceColor','flat','FaceVertexCData',body.geometry.c);

          % for debugging:
          %h(i)=patch(xpts,ypts,co(mod(i-2,size(co,1))+1,:));
          % end debugging
        end
        if (obj.debug) % draw extra debugging info
          origin = forwardKin(obj.model,kinsol,i,[0;0]);
          plot(origin(1),origin(2),'k+');
          if (body.mass~=0)
            com = forwardKin(obj.model,kinsol,i,body.com);
            plot(com(1),com(2),'ro');
            line([origin(1),com(1)],[origin(2),com(2)],'Color','r');
          end
          if body.parent>0
            parent_origin = forwardKin(obj.model,kinsol,body.parent,[0;0]);
            line([origin(1),parent_origin(1)],[origin(2),parent_origin(2)],'Color','k');
          end
          if ~isempty(body.contact_pts)
            pts = obj.model.T_2D_to_3D'*forwardKin(obj.model,kinsol,i,body.contact_pts);
            plot(pts(1,:),pts(2,:),'g*');
          end
        end
      end

      if (obj.debug)
        com = getCOM(obj.model,q);
        plot(com(1),com(2),'bo','MarkerSize',15,'LineWidth',3);
        plot(com(1),com(2),'b+','MarkerSize',15,'LineWidth',3);
        plot([com(1) com(1)], [0 com(2)],'b--');
      end

      % for debugging
      %legend(h,{obj.model.body.linkname},'interpreter','none');
      % end debugging
      
      axis equal;
      if ~isempty(obj.xlim)
        xlim(obj.xlim);
      end
      if ~isempty(obj.ylim)
        ylim(obj.ylim);
      end
      if ~isempty(obj.axis)
        axis(obj.axis);
      end
      
      xlabel(obj.model.x_axis_label);
      ylabel(obj.model.y_axis_label);
      
      if (~isempty([obj.model.body.contact_pts]))
        v=axis;
        line([v(1)-.1*(v(2)-v(1)),v(2)+.1*(v(2)-v(1))],[0 0],'Color','k');
      end
      
      title(['t = ', num2str(t,'%.2f') ' sec']);
    end
  end

  properties (Access=protected)
    model;
  end
  
  properties
    xlim=[]
    ylim=[];
    debug = false;  % if true, draws extras, like the coordinate frames and COMs for each link
    fade_percent = 0;     % 0 to 1 
    fade_color = [1 1 1];
  end
end
