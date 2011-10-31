classdef PlanarURDFVisualizer < Visualizer
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  % This is the planar version of the visualizer.  The y-axis of the 
  % URDF file is essentially ignored.  Joints that act out of
  % plane are not supported.  All geometry out of plane is projected onto
  % the plane.
  
  methods
    function obj = PlanarURDFVisualizer(urdf_filename,axis)
      obj=obj@Visualizer(0);
      
      checkDependency('featherstone_enabled');
      
      if (nargin<1)
        urdf_filename=uigetfile('*.urdf');
      end
      obj.model = parseURDF(urdf_filename);

      obj = obj.setNumInputs(2*obj.model.NB);
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
      clf;
      
      T{1} = eye(3);  % note inside this method, T{i} is for linknum i-1
      patch(obj.model.geometry(1).x,obj.model.geometry(1).z,obj.model.geometry(1).c);
      
      for i=1:obj.model.NB
        TJ = Tjcalcp(obj.model.jcode(i),x(i));
        T{i+1}=T{obj.model.parent(i)+1}*obj.model.Ttree{i}*TJ;

        xpts = obj.model.geometry(i+1).x;
        zpts = obj.model.geometry(i+1).z;
        c = obj.model.geometry(i+1).c;
        
        % transform into world coordinates and draw
        s = size(xpts); n = prod(s);
        pts = T{i+1}*[reshape(xpts,1,n);reshape(zpts,1,n);ones(1,n)];
        patch(reshape(pts(1,:),s),reshape(pts(2,:),s),c);
      end

      axis equal;
      if (obj.axis)
        axis(obj.axis);
      end
      title(['t = ', num2str(t,'%.2f') ' sec']);
      drawnow;

      
    end
  end

  properties
    model;
    axis;
  end
end
