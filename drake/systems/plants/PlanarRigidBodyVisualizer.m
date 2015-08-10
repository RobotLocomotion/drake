classdef PlanarRigidBodyVisualizer < RigidBodyVisualizer
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  % This is the planar version of the visualizer.  The y-axis of the 
  % URDF file is essentially ignored.  Joints that act out of
  % plane are not supported.  All geometry out of plane is projected onto
  % the plane.
  
  methods
    function obj = PlanarRigidBodyVisualizer(manip,options)
      if (nargin<2) options = struct(); end
      
      obj=obj@RigidBodyVisualizer(manip);

      if ~isfield(options,'Tview')
        typecheck(manip,'PlanarRigidBodyManipulator');
        obj.Tview = [manip.x_axis, manip.y_axis, manip.view_axis]';
      else
        obj.Tview = options.Tview;
        typecheck(manip,'RigidBodyManipulator');
      end
      
      obj = updateManipulator(obj,manip);
      
      obj.preserve_view = false;
    end
    
    function obj = updateManipulator(obj,manip)
      obj = updateManipulator@RigidBodyVisualizer(obj,manip);
      
      if isa(manip,'PlanarRigidBodyManipulator')
        obj.x_axis_label = manip.x_axis_label;
        obj.y_axis_label = manip.y_axis_label;
      end
            
      w = warning('off','Drake:RigidBodyGeometry:SimplifiedCollisionGeometry');
      for i=1:length(obj.model.body)
        b = obj.model.body(i);
        for j=1:length(b.visual_geometry)
          [obj.body(i).x{j},obj.body(i).y{j},obj.body(i).z{j},obj.body(i).c{j}] = getPatchData(b.visual_geometry{j},obj.Tview(:,1),obj.Tview(:,2), obj.Tview(:,3));
        end
      end
      warning(w);      
    end
    
    function draw(obj,t,x)

      n = obj.model.num_positions;
      q = x(1:n); %qd=x(n+(1:n));
      kinsol = obj.model.doKinematics(q);
      
      % for debugging:
      %co = get(gca,'ColorOrder');
      %h = [];
      % end debugging

      for i=1:length(obj.model.body)
        for j=1:length(obj.body(i).x)
          c = (1-obj.fade_percent)*obj.body(i).c{j} + obj.fade_percent*obj.fade_color;
          pts = obj.Tview*forwardKin(obj.model,kinsol,i,[obj.body(i).x{j}(:),obj.body(i).y{j}(:),obj.body(i).z{j}(:)]');
          x = reshape(pts(1,:),size(obj.body(i).x{j}));
          y = reshape(pts(2,:),size(obj.body(i).y{j}));
          z = reshape(pts(3,:),size(obj.body(i).z{j}));
          patch(x,y,z,c,'LineWidth',.01,'EdgeColor',obj.fade_percent*obj.fade_color); %0*xpts,'FaceColor','flat','FaceVertexCData',body.geometry.c);
          % patch(xpts,ypts,body.geometry{j}.c,'EdgeColor','none','FaceAlpha',1); %0*xpts,'FaceColor','flat','FaceVertexCData',body.geometry.c);

          % for debugging:
          %h(i)=patch(xpts,ypts,co(mod(i-2,size(co,1))+1,:));
          % end debugging
        end
        if (obj.debug) % draw extra debugging info
          body = obj.model.body(i);
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
        end
      end
      
      for j=1:length(obj.model.position_constraints)
        % todo: generalize this to all position constraints?
        if isa(obj.model.position_constraints{j},'DrakeFunctionConstraint') && isa(obj.model.position_constraints{j}.fcn,'drakeFunction.kinematic.CableLength')
          [pts,edges] = getSegments(obj.model.position_constraints{j}.fcn,q);
          pts = obj.Tview*pts; x=pts(1,:);y=pts(2,:);z=pts(3,:);
          line(x(edges),y(edges),z(edges),'Color','k','LineWidth',3);
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
      
      xlabel(obj.x_axis_label);
      ylabel(obj.y_axis_label);
      
      if isa(obj.model.terrain,'RigidBodyFlatTerrain')
        v=axis;
        line([v(1)-.1*(v(2)-v(1)),v(2)+.1*(v(2)-v(1))],[0 0],'Color','k');
      end
      
      title(['t = ', num2str(t,'%.2f') ' sec']);
    end
  end

  properties (Access=protected)
    Tview;
    body;  % body(i).x{j} and body(i).c{j} describe the jth geometry on body i
    x_axis_label = 'x';
    y_axis_label = 'y';
  end
  
  properties
    fade_percent = 0;     % 0 to 1 
    fade_color = [1 1 1];
  end
end
