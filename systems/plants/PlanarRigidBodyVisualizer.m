classdef PlanarRigidBodyVisualizer < Visualizer
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  % This is the planar version of the visualizer.  The y-axis of the 
  % URDF file is essentially ignored.  Joints that act out of
  % plane are not supported.  All geometry out of plane is projected onto
  % the plane.
  
  methods
    function obj = PlanarRigidBodyVisualizer(model,axis)
      obj=obj@Visualizer(0);
      
      if (nargin<1)
        urdf_filename=uigetfile('*.urdf');
      elseif ischar(model)
        obj.model = RigidBodyModel.parseURDF(model);
      elseif isa(model,'RigidBodyModel')
        obj.model = model;
      else
        error('model must be a RigidBodyModel or the name of a urdf file'); 
      end

      obj = obj.setNumInputs(2*obj.model.featherstone.NB);
      if (nargin>1)
        obj.axis = axis;
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
    axis=[];
    xlim=[]
    ylim=[];
  end
end
