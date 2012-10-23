classdef PlanarRigidBodyVisualizer < Visualizer
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  % This is the planar version of the visualizer.  The y-axis of the 
  % URDF file is essentially ignored.  Joints that act out of
  % plane are not supported.  All geometry out of plane is projected onto
  % the plane.
  
  methods
    function obj = PlanarRigidBodyVisualizer(frame,model,options)
      obj=obj@Visualizer(frame);
      
      options=struct('twoD',true);
      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        obj.model = PlanarRigidBodyModel(fullfile(pathname,filename),options);
      elseif ischar(model)
        obj.model = PlanarRigidBodyModel(model,options);
      elseif isa(model,'PlanarRigidBodyModel')
        obj.model = model;
      else
        error('model must be a PlanarRigidBodyModel or the name of a urdf file'); 
      end
    end
    
    function draw(obj,t,x)

      persistent hFig;
      
      if (isempty(hFig))
        hFig = sfigure(32);
        set(hFig,'DoubleBuffer','on');
      end

      sfigure(hFig);
      clf; hold on;
      
      n = obj.model.featherstone.NB;
      q = x(1:n); qd=x(n+(1:n));
      obj.model.doKinematics(q);
      
      % for debugging:
      %co = get(gca,'ColorOrder');
      %h = [];
      % end debugging

      for i=1:length(obj.model.body)
        body = obj.model.body(i);
        for j=1:length(body.geometry)
          s = size(body.geometry{j}.x); n=prod(s);
          pts = body.T*[reshape(body.geometry{j}.x,1,n); reshape(body.geometry{j}.y,1,n); ones(1,n)];
          xpts = reshape(pts(1,:),s); ypts = reshape(pts(2,:),s);

          patch(xpts,ypts,body.geometry{j}.c,'LineWidth',2); %0*xpts,'FaceColor','flat','FaceVertexCData',body.geometry.c);
          patch(xpts,ypts,body.geometry{j}.c,'EdgeColor','none'); %0*xpts,'FaceColor','flat','FaceVertexCData',body.geometry.c);

          % for debugging:
          %h(i)=patch(xpts,ypts,co(mod(i-2,size(co,1))+1,:));
          % end debugging
        end
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
      drawnow;

    end
  end

  properties
    model;
    xlim=[]
    ylim=[];
  end
end
