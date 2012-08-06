classdef RigidBodyWRLVisualizer < Visualizer 
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  
  methods
    function obj = RigidBodyWRLVisualizer(frame,model)
      obj=obj@Visualizer(frame);
      
      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        obj.model = RigidBodyModel.parseURDF(fullfile(pathname,filename));
      elseif ischar(model)
        obj.model = RigidBodyModel.parseURDF(model);
      elseif isa(model,'RigidBodyModel')
        obj.model = model;
      else
        error('model must be a RigidBodyModel or the name of a urdf file'); 
      end
      
      wrlfile = ['/tmp/',obj.model.name,'.wrl'];
      obj.model.writeWRL(wrlfile);
      obj.wrl = vrworld(wrlfile);
      if ~strcmpi(get(obj.wrl,'Open'),'on')
        open(obj.wrl);
      end
      if get(obj.wrl,'Clients')<1
        view(obj.wrl);
      end
      obj.display_time=false;
    end
    
    function delete(obj)
      close(obj.wrl);
      delete(obj.wrl);
    end
    
    function draw(obj,t,x)
      for i=1:length(obj.model.body)
        b = obj.model.body(i);
        if ~isempty(b.parent)
          node=getfield(obj.wrl,b.jointname);
          switch (b.jcode)
            case 1 % pin joint
              node.rotation=[0 1 0 x(i-1)];
            case 2 % x-axis slider
              node.translation=[x(i-1) 0 0];
            case 3 % z-axis slider
              node.translation=[0 0 x(i-1)];
          end
        end
      end
      vrdrawnow;
    end
    
    function playbackAVI(varargin)
      error('AVI playback not implemented yet for VRML visualizers.  But should be possible');
    end
    
    function playbackSWF(varargin)
      error('SWF playback not available for VRML visualizers.  The vector graphics equivalent is playbackVRML.');
    end
    
    function playbackVRML(varargin)
      error('not implemented yet, but should be possible to record the sequence to a VRML movie');
    end
  end

  properties
    model;
    wrl;
  end
end
