classdef PlanarRigidBodyVisualizer < Visualizer
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  % This is the planar version of the visualizer.  The y-axis of the 
  % URDF file is essentially ignored.  Joints that act out of
  % plane are not supported.  All geometry out of plane is projected onto
  % the plane.
  
  methods
    function obj = PlanarRigidBodyVisualizer(frame,model)
      obj=obj@Visualizer(frame);
      
      options=struct('twoD',true);
      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        obj.model = RigidBodyModel.parseURDF(fullfile(pathname,filename),options);
      elseif ischar(model)
        obj.model = RigidBodyModel.parseURDF(model,options);
      elseif isa(model,'RigidBodyModel')
        obj.model = model;
      else
        error('model must be a RigidBodyModel or the name of a urdf file'); 
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
      
      for i=1:length(obj.model.body)
        body = obj.model.body(i);
        if (isempty(body.parent))
          body.T = eye(3);
        else
          TJ = Tjcalcp(body.jcode,x(body.dofnum));
          body.T=body.parent.T*body.Ttree*TJ;
        end
        
        for i=1:length(body.geometry)
          s = size(body.geometry{i}.x); n=prod(s);
          pts = body.T*[reshape(body.geometry{i}.x,1,n); reshape(body.geometry{i}.z,1,n); ones(1,n)];
          xpts = reshape(pts(1,:),s); zpts = reshape(pts(2,:),s);
          patch(xpts,zpts,body.geometry{i}.c); %0*xpts,'FaceColor','flat','FaceVertexCData',body.geometry.c);
        end
      end

      axis equal;
      if (obj.xlim)
        xlim(obj.xlim);
      end
      if (obj.ylim)
        ylim(obj.ylim);
      end
      if (obj.axis)
        axis(obj.axis);
      end
      
      if (~isempty([obj.model.body.ground_contact]))
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
